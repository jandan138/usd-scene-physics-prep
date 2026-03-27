#!/usr/bin/env python3
"""Bounding-box verification: compare world-space vertex positions between
original and dedup'd USD datasets to detect asset displacement."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from multiprocessing import Pool
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# Suppress noisy USD diagnostics
os.environ.setdefault("TF_LOG", "0")

from pxr import Gf, Usd, UsdGeom  # noqa: E402


# ---------------------------------------------------------------------------
# Vertex extraction
# ---------------------------------------------------------------------------

def _get_world_vertices(stage: Usd.Stage, prim: Usd.Prim) -> List[Tuple[float, float, float]]:
    """Return world-space vertex positions for all meshes under *prim*."""
    xf_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    world_mat = xf_cache.GetLocalToWorldTransform(prim)
    all_pts: list[Tuple[float, float, float]] = []

    if prim.IsInstance():
        proto = prim.GetPrototype()
        if proto:
            proto_xf = UsdGeom.XformCache(Usd.TimeCode.Default())
            for child in Usd.PrimRange(proto):
                mesh = UsdGeom.Mesh(child)
                if not mesh:
                    continue
                pts = mesh.GetPointsAttr().Get()
                if not pts:
                    continue
                local_mat = proto_xf.GetLocalToWorldTransform(child)
                combined = local_mat * world_mat
                for pt in pts:
                    wp = combined.Transform(Gf.Vec3d(pt[0], pt[1], pt[2]))
                    all_pts.append((wp[0], wp[1], wp[2]))
    else:
        for child in Usd.PrimRange(prim):
            mesh = UsdGeom.Mesh(child)
            if not mesh:
                continue
            pts = mesh.GetPointsAttr().Get()
            if not pts:
                continue
            child_world = xf_cache.GetLocalToWorldTransform(child)
            for pt in pts:
                wp = child_world.Transform(Gf.Vec3d(pt[0], pt[1], pt[2]))
                all_pts.append((wp[0], wp[1], wp[2]))
    return all_pts


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _centroid(pts: List[Tuple[float, float, float]]) -> Tuple[float, float, float]:
    n = len(pts)
    sx = sum(p[0] for p in pts)
    sy = sum(p[1] for p in pts)
    sz = sum(p[2] for p in pts)
    return (sx / n, sy / n, sz / n)


def _bbox(pts: List[Tuple[float, float, float]]) -> Dict[str, List[float]]:
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    zs = [p[2] for p in pts]
    return {
        "min": [min(xs), min(ys), min(zs)],
        "max": [max(xs), max(ys), max(zs)],
    }


def _bbox_volume(bb: Dict[str, List[float]]) -> float:
    dx = max(bb["max"][0] - bb["min"][0], 0.0)
    dy = max(bb["max"][1] - bb["min"][1], 0.0)
    dz = max(bb["max"][2] - bb["min"][2], 0.0)
    return dx * dy * dz


def _bbox_iou(a: Dict[str, List[float]], b: Dict[str, List[float]]) -> float:
    ix_min = [max(a["min"][i], b["min"][i]) for i in range(3)]
    ix_max = [min(a["max"][i], b["max"][i]) for i in range(3)]
    ix_vol = 1.0
    for i in range(3):
        d = ix_max[i] - ix_min[i]
        if d <= 0:
            return 0.0
        ix_vol *= d
    va = _bbox_volume(a)
    vb = _bbox_volume(b)
    union = va + vb - ix_vol
    if union <= 0:
        return 0.0
    return ix_vol / union


def _dist(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return math.sqrt(sum((a[i] - b[i]) ** 2 for i in range(3)))


# ---------------------------------------------------------------------------
# Leaf prim collection
# ---------------------------------------------------------------------------

def _collect_leaf_prims(
    stage: Usd.Stage, category: Optional[str]
) -> Dict[str, Usd.Prim]:
    """Collect asset-level prims (model_*) under /Root/Meshes.

    Scene hierarchy: /Root/Meshes/<group>/<category>/model_<hash>_<idx>
    where <group> is Base/Furnitures/Animation and <category> is ground/desk/etc.
    Asset prims can be Mesh, Xform, or Scope typed, and may be instanceable.
    """
    root = stage.GetPrimAtPath("/Root/Meshes")
    if not root:
        return {}
    result: Dict[str, Usd.Prim] = {}
    # Walk: group -> category -> asset prims
    for group in root.GetChildren():
        for cat_prim in group.GetChildren():
            cat_name = cat_prim.GetName()
            if category is not None and cat_name != category:
                continue
            for asset_prim in cat_prim.GetChildren():
                result[str(asset_prim.GetPath())] = asset_prim
    return result


# ---------------------------------------------------------------------------
# Per-scene processing
# ---------------------------------------------------------------------------

def _process_scene(args: Tuple[str, str, str, Optional[str], float]) -> Dict[str, Any]:
    orig_layout, dedup_layout, scene_id, category, threshold = args

    scene_result: Dict[str, Any] = {
        "scene_id": scene_id,
        "total_prims": 0,
        "matched": 0,
        "missing_orig": 0,
        "missing_dedup": 0,
        "displaced_gt_threshold": 0,
        "max_displacement": 0.0,
        "prims": [],
    }

    try:
        stage_orig = Usd.Stage.Open(orig_layout)
        stage_dedup = Usd.Stage.Open(dedup_layout)
    except Exception as e:
        scene_result["error"] = str(e)
        return scene_result

    if not stage_orig or not stage_dedup:
        scene_result["error"] = "Failed to open one or both stages"
        return scene_result

    prims_orig = _collect_leaf_prims(stage_orig, category)
    prims_dedup = _collect_leaf_prims(stage_dedup, category)

    all_paths = sorted(set(prims_orig.keys()) | set(prims_dedup.keys()))
    scene_result["total_prims"] = len(all_paths)

    for path in all_paths:
        prim_info: Dict[str, Any] = {"path": path}

        has_orig = path in prims_orig
        has_dedup = path in prims_dedup

        verts_orig = _get_world_vertices(stage_orig, prims_orig[path]) if has_orig else []
        verts_dedup = _get_world_vertices(stage_dedup, prims_dedup[path]) if has_dedup else []

        if not verts_orig and not verts_dedup:
            # Both empty, skip
            prim_info["status"] = "both_empty"
            scene_result["prims"].append(prim_info)
            continue

        if not verts_orig:
            prim_info["status"] = "missing_orig"
            scene_result["missing_orig"] += 1
            if verts_dedup:
                c = _centroid(verts_dedup)
                bb = _bbox(verts_dedup)
                prim_info["centroid_dedup"] = list(c)
                prim_info["bbox_dedup"] = bb
            scene_result["prims"].append(prim_info)
            continue

        if not verts_dedup:
            prim_info["status"] = "missing_dedup"
            scene_result["missing_dedup"] += 1
            c = _centroid(verts_orig)
            bb = _bbox(verts_orig)
            prim_info["centroid_orig"] = list(c)
            prim_info["bbox_orig"] = bb
            scene_result["prims"].append(prim_info)
            continue

        # Both have vertices
        scene_result["matched"] += 1

        c_orig = _centroid(verts_orig)
        c_dedup = _centroid(verts_dedup)
        bb_orig = _bbox(verts_orig)
        bb_dedup = _bbox(verts_dedup)
        disp = _dist(c_orig, c_dedup)
        iou = _bbox_iou(bb_orig, bb_dedup)

        prim_info["centroid_orig"] = list(c_orig)
        prim_info["centroid_dedup"] = list(c_dedup)
        prim_info["displacement"] = round(disp, 6)
        prim_info["bbox_orig"] = bb_orig
        prim_info["bbox_dedup"] = bb_dedup
        prim_info["bbox_iou"] = round(iou, 6)

        if disp > threshold:
            prim_info["status"] = "displaced"
            scene_result["displaced_gt_threshold"] += 1
        else:
            prim_info["status"] = "ok"

        if disp > scene_result["max_displacement"]:
            scene_result["max_displacement"] = round(disp, 6)

        scene_result["prims"].append(prim_info)

    return scene_result


# ---------------------------------------------------------------------------
# Scene discovery
# ---------------------------------------------------------------------------

def _discover_scenes(
    orig_root: str, dedup_root: str, scene_filter: Optional[str]
) -> List[Tuple[str, str, str]]:
    """Return list of (orig_layout, dedup_layout, scene_id) tuples."""
    results = []
    for sub in ("commercial", "home"):
        orig_base = os.path.join(orig_root, "GRScenes100", sub)
        dedup_base = os.path.join(dedup_root, "GRScenes100", sub)
        if not os.path.isdir(orig_base):
            continue
        for scene_id in sorted(os.listdir(orig_base)):
            if scene_filter and scene_id != scene_filter:
                continue
            orig_layout = os.path.join(orig_base, scene_id, "layout.usd")
            dedup_layout = os.path.join(dedup_base, scene_id, "layout.usd")
            if os.path.isfile(orig_layout) and os.path.isfile(dedup_layout):
                results.append((orig_layout, dedup_layout, scene_id))
    return results


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    parser = argparse.ArgumentParser(
        description="Verify dedup bounding boxes by comparing original vs dedup'd datasets"
    )
    parser.add_argument("--orig-root", required=True, help="Path to original dataset root")
    parser.add_argument("--dedup-root", required=True, help="Path to dedup'd dataset root")
    parser.add_argument("--category", default=None, help="Filter prims by category path segment")
    parser.add_argument("--scene", default=None, help="Single scene ID to process")
    parser.add_argument("--threshold", type=float, default=1.0, help="Displacement threshold for flagging")
    parser.add_argument("--workers", type=int, default=1, help="Number of parallel workers")
    parser.add_argument("--out", required=True, help="Output JSON report path")
    args = parser.parse_args()

    orig_root = os.path.abspath(args.orig_root)
    dedup_root = os.path.abspath(args.dedup_root)

    scenes = _discover_scenes(orig_root, dedup_root, args.scene)
    if not scenes:
        print(f"No scenes found. orig_root={orig_root}, dedup_root={dedup_root}, scene={args.scene}")
        sys.exit(1)

    print(f"Processing {len(scenes)} scene(s), category={args.category}, threshold={args.threshold}")

    work_items = [
        (orig_layout, dedup_layout, scene_id, args.category, args.threshold)
        for orig_layout, dedup_layout, scene_id in scenes
    ]

    t0 = time.time()
    if args.workers > 1 and len(work_items) > 1:
        with Pool(args.workers) as pool:
            scene_results = pool.map(_process_scene, work_items)
    else:
        scene_results = [_process_scene(w) for w in work_items]
    elapsed = time.time() - t0

    # Aggregate
    agg = {
        "total_scenes": len(scene_results),
        "total_prims": 0,
        "matched_prims": 0,
        "missing_orig": 0,
        "missing_dedup": 0,
        "centroid_displaced_gt_001": 0,
        "centroid_displaced_gt_01": 0,
        "centroid_displaced_gt_1": 0,
        "max_centroid_displacement": 0.0,
        "bbox_iou_lt_095": 0,
        "bbox_iou_lt_090": 0,
    }

    for sr in scene_results:
        agg["total_prims"] += sr["total_prims"]
        agg["matched_prims"] += sr["matched"]
        agg["missing_orig"] += sr["missing_orig"]
        agg["missing_dedup"] += sr["missing_dedup"]
        if sr["max_displacement"] > agg["max_centroid_displacement"]:
            agg["max_centroid_displacement"] = sr["max_displacement"]
        for p in sr["prims"]:
            disp = p.get("displacement")
            iou = p.get("bbox_iou")
            if disp is not None:
                if disp > 0.01:
                    agg["centroid_displaced_gt_001"] += 1
                if disp > 0.1:
                    agg["centroid_displaced_gt_01"] += 1
                if disp > 1.0:
                    agg["centroid_displaced_gt_1"] += 1
            if iou is not None:
                if iou < 0.95:
                    agg["bbox_iou_lt_095"] += 1
                if iou < 0.90:
                    agg["bbox_iou_lt_090"] += 1

    report = {
        "config": {
            "orig_root": orig_root,
            "dedup_root": dedup_root,
            "category": args.category,
            "threshold": args.threshold,
        },
        "scenes": scene_results,
        "aggregate": agg,
    }

    # Write output
    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(report, f, indent=2)

    # Console summary
    print(f"\n=== BBOX VERIFICATION SUMMARY ===")
    print(f"Scenes: {agg['total_scenes']}")
    print(f"Total prims: {agg['total_prims']}")
    print(f"Matched (both have verts): {agg['matched_prims']}")
    print(f"Missing orig: {agg['missing_orig']}  Missing dedup: {agg['missing_dedup']}")
    print(f"Centroid displaced > 0.01: {agg['centroid_displaced_gt_001']}")
    print(f"Centroid displaced > 0.1: {agg['centroid_displaced_gt_01']}")
    print(f"Centroid displaced > 1.0: {agg['centroid_displaced_gt_1']}")
    print(f"Max displacement: {agg['max_centroid_displacement']:.4f}")
    print(f"Bbox IoU < 0.95: {agg['bbox_iou_lt_095']}")
    print(f"Bbox IoU < 0.90: {agg['bbox_iou_lt_090']}")
    print(f"Elapsed: {elapsed:.1f}s")
    print(f"Report written to: {out_path}")


if __name__ == "__main__":
    main()
