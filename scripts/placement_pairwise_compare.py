#!/usr/bin/env python3
"""Compare scene placement between two dataset snapshots.

This is a generalized, read-only version of verify_all_scenes_vs_test0.py.
It can compare:
  - current layout.usd across two dataset roots
  - current vs earliest pre_c1 backup within one dataset
  - current vs latest pre_patch backup within one dataset

For each matched scene pair, it compares referenced object prims by path,
transforms mesh vertices to world space, and measures centroid displacement.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys
import time
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np

try:
    from pxr import Gf, Usd, UsdGeom
except ImportError as exc:
    print(f"Error: pxr (USD) not available: {exc}", file=sys.stderr)
    print("Run via ./scripts/isaac_python.sh", file=sys.stderr)
    sys.exit(1)


VALID_LAYOUT_MODES = {
    "current",
    "earliest_pre_c1",
    "latest_pre_patch",
}

DISP_THRESHOLDS = (0.01, 0.1, 1.0, 10.0)
_ASSET_MESH_CACHE: Dict[str, List[Tuple[str, List[Tuple[float, float, float]]]]] = {}


def _find_backup_by_pattern(scene_dir: str, prefix: str) -> Optional[str]:
    candidates = [
        f for f in os.listdir(scene_dir)
        if f.startswith(prefix) and f.endswith(".usd")
    ]
    if not candidates:
        return None

    def extract_timestamp(name: str) -> str:
        m = re.search(r"\.(\d{8}_\d{6})\.usd$", name)
        return m.group(1) if m else ""

    candidates.sort(key=extract_timestamp)
    return os.path.join(scene_dir, candidates[0])


def _find_latest_backup_by_pattern(scene_dir: str, prefix: str) -> Optional[str]:
    candidates = [
        f for f in os.listdir(scene_dir)
        if f.startswith(prefix) and f.endswith(".usd")
    ]
    if not candidates:
        return None

    def extract_timestamp(name: str) -> str:
        m = re.search(r"\.(\d{8}_\d{6})\.usd$", name)
        return m.group(1) if m else ""

    candidates.sort(key=extract_timestamp)
    return os.path.join(scene_dir, candidates[-1])


def select_layout(scene_dir: str, mode: str) -> Optional[str]:
    if mode == "current":
        candidate = os.path.join(scene_dir, "layout.usd")
        return candidate if os.path.isfile(candidate) else None
    if mode == "earliest_pre_c1":
        return _find_backup_by_pattern(scene_dir, "layout.pre_c1_")
    if mode == "latest_pre_patch":
        return _find_latest_backup_by_pattern(scene_dir, "layout.pre_patch.")
    raise ValueError(f"Unsupported layout mode: {mode}")


def _norm_abs(path: str) -> str:
    return os.path.normpath(os.path.abspath(path.replace("\\", "/")))


def build_old_asset_locator(bak_root: Optional[str], dataset_root: str) -> Dict[str, str]:
    locator: Dict[str, str] = {}
    if not bak_root:
        return locator

    dedup_dir = os.path.join(_norm_abs(bak_root), "_dedup_assets")
    dataset_root = _norm_abs(dataset_root)
    if not os.path.isdir(dedup_dir):
        return locator

    for batch_name in sorted(os.listdir(dedup_dir)):
        batch_dir = os.path.join(dedup_dir, batch_name)
        assets_dir = os.path.join(batch_dir, "GRScenes_assets")
        if not os.path.isdir(assets_dir):
            continue
        for category in os.listdir(assets_dir):
            cat_dir = os.path.join(assets_dir, category)
            if not os.path.isdir(cat_dir):
                continue
            for uid in os.listdir(cat_dir):
                usd_path = os.path.join(cat_dir, uid, "usd", f"{uid}.usd")
                if not os.path.isfile(usd_path):
                    continue
                original_abs = _norm_abs(
                    os.path.join(dataset_root, "GRScenes_assets", category, uid, "usd", f"{uid}.usd")
                )
                locator[original_abs] = _norm_abs(usd_path)
    return locator


def discover_scene_pairs(
    left_root: str,
    right_root: str,
    left_mode: str,
    right_mode: str,
    scene_filter: Optional[str] = None,
) -> Tuple[List[Tuple[str, str, str]], List[Dict[str, str]], List[Dict[str, str]]]:
    """Discover matched scene pairs across two dataset roots / layout modes."""
    pairs: List[Tuple[str, str, str]] = []
    left_only: List[Dict[str, str]] = []
    right_only: List[Dict[str, str]] = []

    for subcategory in ("home", "commercial"):
        left_base = os.path.join(left_root, "GRScenes100", subcategory)
        right_base = os.path.join(right_root, "GRScenes100", subcategory)

        left_ids = set(os.listdir(left_base)) if os.path.isdir(left_base) else set()
        right_ids = set(os.listdir(right_base)) if os.path.isdir(right_base) else set()

        for scene_id in sorted(left_ids & right_ids):
            full_id = f"{subcategory}/{scene_id}"
            if scene_filter and scene_filter not in full_id:
                continue
            left_scene_dir = os.path.join(left_base, scene_id)
            right_scene_dir = os.path.join(right_base, scene_id)
            left_layout = select_layout(left_scene_dir, left_mode)
            right_layout = select_layout(right_scene_dir, right_mode)
            if left_layout and right_layout:
                pairs.append((left_layout, right_layout, full_id))
            elif left_layout and not right_layout:
                right_only.append({
                    "scene_id": full_id,
                    "missing_side": "right",
                    "left_layout": left_layout,
                    "right_mode": right_mode,
                })
            elif right_layout and not left_layout:
                left_only.append({
                    "scene_id": full_id,
                    "missing_side": "left",
                    "right_layout": right_layout,
                    "left_mode": left_mode,
                })

        for scene_id in sorted(left_ids - right_ids):
            full_id = f"{subcategory}/{scene_id}"
            if scene_filter and scene_filter not in full_id:
                continue
            left_layout = select_layout(os.path.join(left_base, scene_id), left_mode)
            if left_layout:
                right_only.append({
                    "scene_id": full_id,
                    "missing_side": "right_root",
                    "left_layout": left_layout,
                })

        for scene_id in sorted(right_ids - left_ids):
            full_id = f"{subcategory}/{scene_id}"
            if scene_filter and scene_filter not in full_id:
                continue
            right_layout = select_layout(os.path.join(right_base, scene_id), right_mode)
            if right_layout:
                left_only.append({
                    "scene_id": full_id,
                    "missing_side": "left_root",
                    "right_layout": right_layout,
                })

    return pairs, left_only, right_only


def _get_reference_asset_paths(prim):
    refs = []
    for spec in prim.GetPrimStack():
        ref_items = spec.referenceList
        for ref in ref_items.GetAddedOrExplicitItems():
            if ref.assetPath:
                resolved = spec.layer.ComputeAbsolutePath(ref.assetPath)
                refs.append(resolved if resolved else ref.assetPath)
    return refs


def _find_object_prims(stage):
    result = []
    for prim in stage.Traverse():
        refs = _get_reference_asset_paths(prim)
        if not refs:
            continue
        is_asset_ref = any("GRScenes_assets" in r or "models/" in r for r in refs)
        if is_asset_ref:
            result.append((prim, refs))
    return result


def _resolve_asset_path(
    asset_abs: str,
    old_asset_locator: Dict[str, str],
) -> Optional[str]:
    asset_abs = _norm_abs(asset_abs)
    if os.path.isfile(asset_abs):
        return asset_abs
    return old_asset_locator.get(asset_abs)


def _find_all_meshes_with_points_composed(prim, xform_cache):
    root_path = str(prim.GetPath())
    results = []
    for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if not child.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(child)
        pts = mesh.GetPointsAttr().Get()
        if not pts or len(pts) == 0:
            continue
        world_xform = xform_cache.GetLocalToWorldTransform(child)
        world_pts = []
        for p in pts:
            gf_p = Gf.Vec3d(float(p[0]), float(p[1]), float(p[2]))
            wp = world_xform.Transform(gf_p)
            world_pts.append([wp[0], wp[1], wp[2]])
        rel_path = str(child.GetPath())[len(root_path):]
        results.append((rel_path, np.array(world_pts, dtype=np.float64)))
    return results


def _default_asset_root(stage) -> Optional[Usd.Prim]:
    default_prim = stage.GetDefaultPrim()
    if default_prim and default_prim.IsValid():
        return default_prim
    children = [c for c in stage.GetPseudoRoot().GetChildren() if c and c.IsValid()]
    return children[0] if children else None


def _load_asset_meshes_in_asset_space(asset_actual_path: str):
    asset_actual_path = _norm_abs(asset_actual_path)
    if asset_actual_path in _ASSET_MESH_CACHE:
        return _ASSET_MESH_CACHE[asset_actual_path]

    stage = Usd.Stage.Open(asset_actual_path)
    if stage is None:
        raise RuntimeError(f"failed_to_open_asset: {asset_actual_path}")

    root_prim = _default_asset_root(stage)
    if root_prim is None or not root_prim.IsValid():
        raise RuntimeError(f"asset_has_no_default_prim: {asset_actual_path}")

    xcache = UsdGeom.XformCache()
    root_path = str(root_prim.GetPath())
    meshes = []
    for child in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies()):
        if not child.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(child)
        pts = mesh.GetPointsAttr().Get()
        if not pts or len(pts) == 0:
            continue
        mesh_xform = xcache.GetLocalToWorldTransform(child)
        mesh_points = []
        for p in pts:
            gf_p = Gf.Vec3d(float(p[0]), float(p[1]), float(p[2]))
            wp = mesh_xform.Transform(gf_p)
            mesh_points.append((wp[0], wp[1], wp[2]))
        rel_path = str(child.GetPath())[len(root_path):]
        meshes.append((rel_path, mesh_points))

    _ASSET_MESH_CACHE[asset_actual_path] = meshes
    return meshes


def _find_all_meshes_with_points(
    prim,
    ref_abs: str,
    layout_xform_cache,
    old_asset_locator: Dict[str, str],
):
    actual_asset_path = _resolve_asset_path(ref_abs, old_asset_locator)
    if not actual_asset_path:
        return []

    asset_meshes = _load_asset_meshes_in_asset_space(actual_asset_path)
    prim_world = layout_xform_cache.GetLocalToWorldTransform(prim)
    results = []
    for rel_path, asset_points in asset_meshes:
        world_pts = []
        for x, y, z in asset_points:
            wp = prim_world.Transform(Gf.Vec3d(float(x), float(y), float(z)))
            world_pts.append([wp[0], wp[1], wp[2]])
        results.append((rel_path, np.array(world_pts, dtype=np.float64)))
    return results


def _aggregate_centroid(mesh_list):
    total_pts = 0
    weighted_sum = np.zeros(3)
    for _, pts in mesh_list:
        n = len(pts)
        weighted_sum += pts.sum(axis=0)
        total_pts += n
    if total_pts == 0:
        return None, 0, len(mesh_list)
    return weighted_sum / total_pts, total_pts, len(mesh_list)


def _displacement_stats(values: Sequence[float]) -> Dict[str, float]:
    if not values:
        return {}
    arr = np.array(values)
    return {
        "mean": round(float(arr.mean()), 6),
        "median": round(float(np.median(arr)), 6),
        "max": round(float(arr.max()), 6),
        "p95": round(float(np.percentile(arr, 95)), 6),
        "p99": round(float(np.percentile(arr, 99)), 6),
    }


def _breakdown(values: Sequence[float], thresholds: Sequence[float]) -> Dict[str, int]:
    return {
        f"gt_{thr}": int(sum(1 for v in values if v > thr))
        for thr in thresholds
    }


def compare_scene(
    left_layout: str,
    right_layout: str,
    scene_id: str,
    left_locator: Dict[str, str],
    right_locator: Dict[str, str],
) -> Dict[str, object]:
    result: Dict[str, object] = {
        "scene_id": scene_id,
        "left_layout": left_layout,
        "right_layout": right_layout,
        "status": "ok",
        "error": None,
    }

    use_composed_geometry = not left_locator and not right_locator

    try:
        if use_composed_geometry:
            stage_left = Usd.Stage.Open(left_layout)
            stage_right = Usd.Stage.Open(right_layout)
        else:
            stage_left = Usd.Stage.Open(left_layout, load=Usd.Stage.LoadNone)
            stage_right = Usd.Stage.Open(right_layout, load=Usd.Stage.LoadNone)
    except Exception as exc:
        result["status"] = "error"
        result["error"] = f"stage_open_failed: {exc}"
        return result

    if not stage_left:
        result["status"] = "error"
        result["error"] = "failed_to_open_left_layout"
        return result
    if not stage_right:
        result["status"] = "error"
        result["error"] = "failed_to_open_right_layout"
        return result

    xc_left = UsdGeom.XformCache()
    xc_right = UsdGeom.XformCache()

    left_prims = _find_object_prims(stage_left)
    right_prims = _find_object_prims(stage_right)

    left_by_path = {str(p.GetPath()): (p, r) for p, r in left_prims}
    right_by_path = {str(p.GetPath()): (p, r) for p, r in right_prims}

    common_paths = sorted(set(left_by_path.keys()) & set(right_by_path.keys()))
    only_left = sorted(set(left_by_path.keys()) - set(right_by_path.keys()))
    only_right = sorted(set(right_by_path.keys()) - set(left_by_path.keys()))

    compared = []
    category_disps = defaultdict(list)
    no_mesh_count = 0

    for path in common_paths:
        left_prim, left_refs = left_by_path[path]
        right_prim, right_refs = right_by_path[path]

        if use_composed_geometry:
            meshes_left = _find_all_meshes_with_points_composed(left_prim, xc_left)
            meshes_right = _find_all_meshes_with_points_composed(right_prim, xc_right)
        else:
            meshes_left = _find_all_meshes_with_points(left_prim, left_refs[0], xc_left, left_locator)
            meshes_right = _find_all_meshes_with_points(right_prim, right_refs[0], xc_right, right_locator)
        if not meshes_left or not meshes_right:
            no_mesh_count += 1
            continue

        centroid_left, verts_left, nmesh_left = _aggregate_centroid(meshes_left)
        centroid_right, verts_right, nmesh_right = _aggregate_centroid(meshes_right)
        if centroid_left is None or centroid_right is None:
            no_mesh_count += 1
            continue

        displacement = float(np.linalg.norm(centroid_left - centroid_right))

        left_mesh_map = {rp: pts for rp, pts in meshes_left}
        right_mesh_map = {rp: pts for rp, pts in meshes_right}
        common_meshes = set(left_mesh_map.keys()) & set(right_mesh_map.keys())

        per_mesh_disps = []
        for mesh_path in common_meshes:
            c_left = left_mesh_map[mesh_path].mean(axis=0)
            c_right = right_mesh_map[mesh_path].mean(axis=0)
            per_mesh_disps.append(float(np.linalg.norm(c_left - c_right)))

        max_mesh_disp = max(per_mesh_disps) if per_mesh_disps else 0.0
        ref_changed = left_refs != right_refs

        parts = path.split("/")
        category = parts[4] if len(parts) >= 5 else "unknown"
        category_disps[category].append(displacement)

        compared.append({
            "prim_path": path,
            "displacement": round(displacement, 6),
            "max_per_mesh_displacement": round(max_mesh_disp, 6),
            "ref_changed": ref_changed,
            "left_ref_sample": left_refs[:2],
            "right_ref_sample": right_refs[:2],
            "verts_left": verts_left,
            "verts_right": verts_right,
            "meshes_left": nmesh_left,
            "meshes_right": nmesh_right,
        })

    compared.sort(key=lambda x: x["displacement"], reverse=True)
    all_disps = [c["displacement"] for c in compared]
    all_max_mesh = [c["max_per_mesh_displacement"] for c in compared]
    ref_changed_disps = [c["displacement"] for c in compared if c["ref_changed"]]
    ref_same_disps = [c["displacement"] for c in compared if not c["ref_changed"]]

    category_summary = {}
    for category, disps in sorted(category_disps.items()):
        category_summary[category] = {
            "count": len(disps),
            "displacement_stats": _displacement_stats(disps),
            "displaced_breakdown": _breakdown(disps, DISP_THRESHOLDS),
        }

    result.update({
        "total_prims_left": len(left_by_path),
        "total_prims_right": len(right_by_path),
        "total_common": len(common_paths),
        "total_compared": len(compared),
        "total_no_mesh": no_mesh_count,
        "only_in_left": len(only_left),
        "only_in_right": len(only_right),
        "only_in_left_paths": only_left[:20],
        "only_in_right_paths": only_right[:20],
        "displacement_stats": _displacement_stats(all_disps),
        "displaced_breakdown": _breakdown(all_disps, DISP_THRESHOLDS),
        "max_per_mesh_breakdown": _breakdown(all_max_mesh, DISP_THRESHOLDS),
        "ref_changed_count": len(ref_changed_disps),
        "ref_changed_breakdown": _breakdown(ref_changed_disps, DISP_THRESHOLDS),
        "ref_changed_displacement_stats": _displacement_stats(ref_changed_disps),
        "ref_same_count": len(ref_same_disps),
        "ref_same_breakdown": _breakdown(ref_same_disps, DISP_THRESHOLDS),
        "ref_same_displacement_stats": _displacement_stats(ref_same_disps),
        "category_breakdown": category_summary,
        "top_20_worst": compared[:20],
    })
    return result


def compute_aggregate(scene_results: Iterable[Dict[str, object]]) -> Dict[str, object]:
    ok_results = [r for r in scene_results if r.get("status") == "ok"]
    error_results = [r for r in scene_results if r.get("status") != "ok"]

    total_compared = sum(int(r.get("total_compared", 0)) for r in ok_results)
    total_common = sum(int(r.get("total_common", 0)) for r in ok_results)
    total_no_mesh = sum(int(r.get("total_no_mesh", 0)) for r in ok_results)
    total_only_left = sum(int(r.get("only_in_left", 0)) for r in ok_results)
    total_only_right = sum(int(r.get("only_in_right", 0)) for r in ok_results)
    total_ref_changed = sum(int(r.get("ref_changed_count", 0)) for r in ok_results)
    total_ref_same = sum(int(r.get("ref_same_count", 0)) for r in ok_results)

    total_disp = {f"gt_{thr}": 0 for thr in DISP_THRESHOLDS}
    total_mesh_disp = {f"gt_{thr}": 0 for thr in DISP_THRESHOLDS}
    total_ref_changed_disp = {f"gt_{thr}": 0 for thr in DISP_THRESHOLDS}
    total_ref_same_disp = {f"gt_{thr}": 0 for thr in DISP_THRESHOLDS}

    category_acc = defaultdict(list)
    global_worst = []

    for r in ok_results:
        for key, value in r.get("displaced_breakdown", {}).items():
            total_disp[key] += int(value)
        for key, value in r.get("max_per_mesh_breakdown", {}).items():
            total_mesh_disp[key] += int(value)
        for key, value in r.get("ref_changed_breakdown", {}).items():
            total_ref_changed_disp[key] += int(value)
        for key, value in r.get("ref_same_breakdown", {}).items():
            total_ref_same_disp[key] += int(value)

        for category, info in r.get("category_breakdown", {}).items():
            stats = info.get("displacement_stats", {})
            max_disp = stats.get("max")
            if max_disp is not None:
                category_acc[category].append(float(max_disp))

        for item in r.get("top_20_worst", []):
            global_worst.append({
                "scene_id": r["scene_id"],
                **item,
            })

    global_worst.sort(key=lambda x: x["displacement"], reverse=True)

    return {
        "scenes_ok": len(ok_results),
        "scenes_error": len(error_results),
        "total_common_prims": total_common,
        "total_compared": total_compared,
        "total_no_mesh": total_no_mesh,
        "total_only_in_left": total_only_left,
        "total_only_in_right": total_only_right,
        "total_ref_changed": total_ref_changed,
        "total_ref_same": total_ref_same,
        "displaced_breakdown": total_disp,
        "max_per_mesh_breakdown": total_mesh_disp,
        "ref_changed_breakdown": total_ref_changed_disp,
        "ref_same_breakdown": total_ref_same_disp,
        "category_maxima": {
            category: round(max(values), 6)
            for category, values in sorted(category_acc.items())
            if values
        },
        "global_top_50_worst": global_worst[:50],
        "scene_errors": [
            {"scene_id": r.get("scene_id"), "error": r.get("error")}
            for r in error_results
        ],
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Compare scene placement between two dataset snapshots.",
    )
    parser.add_argument("--left-root", required=True)
    parser.add_argument("--right-root", required=True)
    parser.add_argument("--left-mode", choices=sorted(VALID_LAYOUT_MODES), default="current")
    parser.add_argument("--right-mode", choices=sorted(VALID_LAYOUT_MODES), default="current")
    parser.add_argument("--label", required=True, help="Label for this comparison run")
    parser.add_argument("--out", required=True, help="Output JSON path")
    parser.add_argument("--scene-filter", default=None)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--left-bak-root", default=None)
    parser.add_argument("--right-bak-root", default=None)
    args = parser.parse_args(argv)

    left_root = os.path.abspath(args.left_root)
    right_root = os.path.abspath(args.right_root)
    left_locator = build_old_asset_locator(args.left_bak_root, left_root)
    right_locator = build_old_asset_locator(args.right_bak_root, right_root)

    print(f"Left root:  {left_root}")
    print(f"Right root: {right_root}")
    print(f"Modes:      {args.left_mode} vs {args.right_mode}")
    print(f"Label:      {args.label}")
    print(f"Backup locators: left={len(left_locator)} right={len(right_locator)}")

    pairs, left_only, right_only = discover_scene_pairs(
        left_root=left_root,
        right_root=right_root,
        left_mode=args.left_mode,
        right_mode=args.right_mode,
        scene_filter=args.scene_filter,
    )
    print(f"Matched scene pairs: {len(pairs)}")
    print(f"Unmatched left-side scenes:  {len(left_only)}")
    print(f"Unmatched right-side scenes: {len(right_only)}")

    if not pairs:
        print("ERROR: no matched scene pairs found", file=sys.stderr)
        return 1

    scene_results = []
    t0 = time.time()

    if args.workers <= 1:
        for idx, (left_layout, right_layout, scene_id) in enumerate(pairs, start=1):
            print(f"[{idx}/{len(pairs)}] {scene_id}")
            scene_results.append(
                compare_scene(left_layout, right_layout, scene_id, left_locator, right_locator)
            )
    else:
        with ProcessPoolExecutor(max_workers=args.workers) as executor:
            futures = {
                executor.submit(
                    compare_scene,
                    left_layout,
                    right_layout,
                    scene_id,
                    left_locator,
                    right_locator,
                ): scene_id
                for left_layout, right_layout, scene_id in pairs
            }
            done = 0
            for future in as_completed(futures):
                done += 1
                scene_id = futures[future]
                try:
                    result = future.result()
                except Exception as exc:
                    result = {
                        "scene_id": scene_id,
                        "status": "error",
                        "error": f"worker_exception: {exc}",
                    }
                scene_results.append(result)
                if result.get("status") == "ok":
                    displaced = result.get("displaced_breakdown", {}).get("gt_0.01", 0)
                    compared = result.get("total_compared", 0)
                    print(
                        f"  [{done}/{len(pairs)}] {scene_id}: ok "
                        f"compared={compared} displaced>0.01={displaced}"
                    )
                else:
                    print(f"  [{done}/{len(pairs)}] {scene_id}: ERROR {result.get('error')}")

    scene_results.sort(key=lambda r: r["scene_id"])
    elapsed = round(time.time() - t0, 1)
    aggregate = compute_aggregate(scene_results)

    output = {
        "label": args.label,
        "left_root": left_root,
        "right_root": right_root,
        "left_mode": args.left_mode,
        "right_mode": args.right_mode,
        "scene_filter": args.scene_filter,
        "elapsed_seconds": elapsed,
        "matched_scene_count": len(pairs),
        "left_only_scenes": left_only,
        "right_only_scenes": right_only,
        "aggregate": aggregate,
        "per_scene": scene_results,
    }

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2, ensure_ascii=False)

    print()
    print("=" * 72)
    print("PAIRWISE COMPARISON SUMMARY")
    print("=" * 72)
    print(f"Scenes ok/error:         {aggregate['scenes_ok']}/{aggregate['scenes_error']}")
    print(f"Total compared prims:    {aggregate['total_compared']}")
    print(f"Displaced > 0.01:        {aggregate['displaced_breakdown']['gt_0.01']}")
    print(f"Displaced > 0.1:         {aggregate['displaced_breakdown']['gt_0.1']}")
    print(f"Displaced > 1.0:         {aggregate['displaced_breakdown']['gt_1.0']}")
    print(f"Displaced > 10.0:        {aggregate['displaced_breakdown']['gt_10.0']}")
    print(f"Ref changed prims:       {aggregate['total_ref_changed']}")
    print(f"Ref changed > 0.01:      {aggregate['ref_changed_breakdown']['gt_0.01']}")
    print(f"Ref same prims:          {aggregate['total_ref_same']}")
    print(f"Ref same > 0.01:         {aggregate['ref_same_breakdown']['gt_0.01']}")
    print(f"Only in left/right:      {aggregate['total_only_in_left']}/{aggregate['total_only_in_right']}")
    print(f"Report saved to:         {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
