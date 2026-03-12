#!/usr/bin/env python3
"""
Verify placement correctness of all normalized scenes against test0 ground truth.

For each scene, opens both the test0 and normalized layout.usd, matches prims
by path, transforms mesh vertices to world space, and compares centroids.

Produces a JSON report with per-scene breakdown, aggregate statistics, and a
pass/fail verdict based on configurable thresholds.

Usage (requires Isaac Sim Python for pxr):
    ./scripts/isaac_python.sh scripts/verify_all_scenes_vs_test0.py \
        --test0-root ./GRScenes-test0 \
        --normalized-root ./GRScenes-test1-normalized \
        --out check_reports/placement_verification.json \
        --workers 8
"""

import argparse
import json
import os
import sys
import time
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed

import numpy as np


# ---------------------------------------------------------------------------
# USD helpers (imported lazily inside worker processes)
# ---------------------------------------------------------------------------

def _get_reference_asset_path(prim):
    """Get resolved reference asset paths for a prim (direct only)."""
    from pxr import Sdf
    refs = []
    for spec in prim.GetPrimStack():
        ref_items = spec.referenceList
        for ref in ref_items.GetAddedOrExplicitItems():
            if ref.assetPath:
                resolved = spec.layer.ComputeAbsolutePath(ref.assetPath)
                refs.append(resolved if resolved else ref.assetPath)
    return refs


def _find_object_prims(stage):
    """Find prims that reference external GRScenes assets."""
    from pxr import Usd
    result = []
    for prim in stage.Traverse():
        refs = _get_reference_asset_path(prim)
        if not refs:
            continue
        is_asset_ref = any("GRScenes_assets" in r or "models/" in r for r in refs)
        if is_asset_ref:
            result.append((prim, refs))
    return result


def _find_all_meshes_with_points(prim, xform_cache):
    """Find all Mesh prims under *prim*, transform vertices to world space.

    Returns list of (relative_path, world_points_ndarray).
    """
    from pxr import Usd, UsdGeom, Gf
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


def _aggregate_centroid(mesh_list):
    """Compute vertex-count-weighted centroid from a list of (path, pts)."""
    total_pts = 0
    weighted_sum = np.zeros(3)
    for _, pts in mesh_list:
        n = len(pts)
        weighted_sum += pts.sum(axis=0)
        total_pts += n
    if total_pts == 0:
        return None, 0, len(mesh_list)
    return weighted_sum / total_pts, total_pts, len(mesh_list)


# ---------------------------------------------------------------------------
# Per-scene comparison (runs in a worker process)
# ---------------------------------------------------------------------------

def compare_scene(test0_layout, norm_layout, scene_id):
    """Compare a single scene. Returns a result dict."""
    from pxr import Usd, UsdGeom

    result = {
        "scene_id": scene_id,
        "test0_layout": test0_layout,
        "norm_layout": norm_layout,
        "status": "ok",
        "error": None,
    }

    try:
        stage_t0 = Usd.Stage.Open(test0_layout)
        if not stage_t0:
            result["status"] = "error"
            result["error"] = "Failed to open test0 layout"
            return result

        stage_norm = Usd.Stage.Open(norm_layout)
        if not stage_norm:
            result["status"] = "error"
            result["error"] = "Failed to open normalized layout"
            return result
    except Exception as e:
        result["status"] = "error"
        result["error"] = f"Stage open failed: {e}"
        return result

    xc_t0 = UsdGeom.XformCache()
    xc_norm = UsdGeom.XformCache()

    t0_prims = _find_object_prims(stage_t0)
    norm_prims = _find_object_prims(stage_norm)

    t0_by_path = {str(p.GetPath()): (p, r) for p, r in t0_prims}
    norm_by_path = {str(p.GetPath()): (p, r) for p, r in norm_prims}

    common_paths = sorted(set(t0_by_path.keys()) & set(norm_by_path.keys()))
    only_t0 = sorted(set(t0_by_path.keys()) - set(norm_by_path.keys()))
    only_norm = sorted(set(norm_by_path.keys()) - set(t0_by_path.keys()))

    compared = []
    no_mesh_count = 0
    displaced_counts = {0.01: 0, 0.1: 0, 1.0: 0, 10.0: 0}
    category_disps = defaultdict(list)

    for path in common_paths:
        prim_t0, refs_t0 = t0_by_path[path]
        prim_norm, refs_norm = norm_by_path[path]
        ref_changed = (refs_t0 != refs_norm)

        meshes_t0 = _find_all_meshes_with_points(prim_t0, xc_t0)
        meshes_norm = _find_all_meshes_with_points(prim_norm, xc_norm)

        if not meshes_t0 or not meshes_norm:
            no_mesh_count += 1
            continue

        centroid_t0, verts_t0, nmesh_t0 = _aggregate_centroid(meshes_t0)
        centroid_norm, verts_norm, nmesh_norm = _aggregate_centroid(meshes_norm)

        if centroid_t0 is None or centroid_norm is None:
            no_mesh_count += 1
            continue

        displacement = float(np.linalg.norm(centroid_t0 - centroid_norm))

        # Per-mesh max displacement
        t0_mesh_map = {rp: pts for rp, pts in meshes_t0}
        norm_mesh_map = {rp: pts for rp, pts in meshes_norm}
        common_meshes = set(t0_mesh_map.keys()) & set(norm_mesh_map.keys())

        per_mesh_disps = []
        for mp in common_meshes:
            c_t0 = t0_mesh_map[mp].mean(axis=0)
            c_norm = norm_mesh_map[mp].mean(axis=0)
            per_mesh_disps.append(float(np.linalg.norm(c_t0 - c_norm)))

        max_mesh_disp = max(per_mesh_disps) if per_mesh_disps else 0.0

        for thresh in displaced_counts:
            if displacement > thresh:
                displaced_counts[thresh] += 1

        # Extract category from path: /Root/Meshes/<group>/<category>/...
        parts = path.split("/")
        cat = parts[4] if len(parts) >= 5 else "unknown"
        category_disps[cat].append(displacement)

        compared.append({
            "prim_path": path,
            "displacement": round(displacement, 6),
            "max_per_mesh_displacement": round(max_mesh_disp, 6),
            "ref_changed": ref_changed,
            "verts_test0": verts_t0,
            "verts_normalized": verts_norm,
            "meshes_test0": nmesh_t0,
            "meshes_normalized": nmesh_norm,
        })

    # Sort by displacement descending
    compared.sort(key=lambda x: x["displacement"], reverse=True)

    # Per-category summary
    cat_summary = {}
    for cat, disps in sorted(category_disps.items()):
        arr = np.array(disps)
        cat_summary[cat] = {
            "count": len(disps),
            "mean_disp": round(float(arr.mean()), 6),
            "max_disp": round(float(arr.max()), 6),
            "displaced_gt_0.01": int(np.sum(arr > 0.01)),
        }

    # Aggregate displacement stats
    all_disps = [c["displacement"] for c in compared]
    all_max_mesh = [c["max_per_mesh_displacement"] for c in compared]

    if all_disps:
        disp_arr = np.array(all_disps)
        disp_stats = {
            "mean": round(float(disp_arr.mean()), 6),
            "median": round(float(np.median(disp_arr)), 6),
            "max": round(float(disp_arr.max()), 6),
            "p95": round(float(np.percentile(disp_arr, 95)), 6),
            "p99": round(float(np.percentile(disp_arr, 99)), 6),
        }
    else:
        disp_stats = {}

    # Ref-changed vs ref-same breakdown
    ref_changed_disps = [c["displacement"] for c in compared if c["ref_changed"]]
    ref_same_disps = [c["displacement"] for c in compared if not c["ref_changed"]]

    result.update({
        "total_prims_test0": len(t0_by_path),
        "total_prims_normalized": len(norm_by_path),
        "total_common": len(common_paths),
        "total_compared": len(compared),
        "total_no_mesh": no_mesh_count,
        "only_in_test0": len(only_t0),
        "only_in_normalized": len(only_norm),
        "displaced_gt_0.01": displaced_counts[0.01],
        "displaced_gt_0.1": displaced_counts[0.1],
        "displaced_gt_1.0": displaced_counts[1.0],
        "displaced_gt_10.0": displaced_counts[10.0],
        "max_per_mesh_gt_0.01": sum(1 for d in all_max_mesh if d > 0.01),
        "displacement_stats": disp_stats,
        "ref_changed_count": len(ref_changed_disps),
        "ref_changed_displaced_gt_0.01": sum(1 for d in ref_changed_disps if d > 0.01),
        "ref_same_count": len(ref_same_disps),
        "ref_same_displaced_gt_0.01": sum(1 for d in ref_same_disps if d > 0.01),
        "category_breakdown": cat_summary,
        "top_10_worst": compared[:10],
        "only_in_test0_paths": only_t0[:10],
        "only_in_normalized_paths": only_norm[:10],
    })
    return result


# ---------------------------------------------------------------------------
# Scene discovery
# ---------------------------------------------------------------------------

def discover_scenes(test0_root, normalized_root, scene_filter=None):
    """Find all scene pairs and report coverage gaps.

    Scans GRScenes100/{home,commercial}/<scene_id>/layout.usd in both roots.

    Returns:
        matched: list of (test0_layout, norm_layout, scene_id)
        no_test0: list of scene_ids in normalized but missing from test0
        no_norm: list of scene_ids in test0 but missing from normalized
    """
    matched = []
    no_test0 = []
    no_norm = []
    grscenes_dir = "GRScenes100"

    for subcategory in ("home", "commercial"):
        t0_base = os.path.join(test0_root, grscenes_dir, subcategory)
        norm_base = os.path.join(normalized_root, grscenes_dir, subcategory)

        t0_ids = set()
        norm_ids = set()

        if os.path.isdir(t0_base):
            t0_ids = {d for d in os.listdir(t0_base)
                      if os.path.isfile(os.path.join(t0_base, d, "layout.usd"))}
        if os.path.isdir(norm_base):
            norm_ids = {d for d in os.listdir(norm_base)
                        if os.path.isfile(os.path.join(norm_base, d, "layout.usd"))}

        for scene_id in sorted(norm_ids & t0_ids):
            full_id = f"{subcategory}/{scene_id}"
            if scene_filter and scene_filter not in full_id:
                continue
            matched.append((
                os.path.join(t0_base, scene_id, "layout.usd"),
                os.path.join(norm_base, scene_id, "layout.usd"),
                full_id,
            ))

        for scene_id in sorted(norm_ids - t0_ids):
            no_test0.append(f"{subcategory}/{scene_id}")

        for scene_id in sorted(t0_ids - norm_ids):
            no_norm.append(f"{subcategory}/{scene_id}")

    return matched, no_test0, no_norm


# ---------------------------------------------------------------------------
# Pass/fail verdict
# ---------------------------------------------------------------------------

HIGHLIGHT_CATEGORIES = {"curtain", "towel", "book", "desk", "other"}


def compute_verdict(scene_results, scenes_without_test0=None):
    """Compute aggregate pass/fail verdict from per-scene results."""
    checks = []
    total_displaced_001 = 0
    total_max_mesh_001 = 0
    total_no_mesh = 0
    total_only_t0 = 0
    total_only_norm = 0
    total_compared = 0
    total_errors = 0

    for sr in scene_results:
        if sr["status"] == "error":
            total_errors += 1
            continue
        total_displaced_001 += sr.get("displaced_gt_0.01", 0)
        total_max_mesh_001 += sr.get("max_per_mesh_gt_0.01", 0)
        total_no_mesh += sr.get("total_no_mesh", 0)
        total_only_t0 += sr.get("only_in_test0", 0)
        total_only_norm += sr.get("only_in_normalized", 0)
        total_compared += sr.get("total_compared", 0)

    checks.append({
        "check": "displaced_gt_0.01 == 0",
        "value": total_displaced_001,
        "pass": total_displaced_001 == 0,
    })
    checks.append({
        "check": "max_per_mesh_displacement_gt_0.01 == 0",
        "value": total_max_mesh_001,
        "pass": total_max_mesh_001 == 0,
    })
    checks.append({
        "check": "only_in_test0 == 0",
        "value": total_only_t0,
        "pass": total_only_t0 == 0,
    })
    checks.append({
        "check": "only_in_normalized == 0",
        "value": total_only_norm,
        "pass": total_only_norm == 0,
    })
    checks.append({
        "check": "no_scene_errors",
        "value": total_errors,
        "pass": total_errors == 0,
    })

    all_pass = all(c["pass"] for c in checks)

    # Highlighted category aggregate
    highlight = {}
    for cat in HIGHLIGHT_CATEGORIES:
        cat_total = 0
        cat_displaced = 0
        cat_max_disp = 0.0
        for sr in scene_results:
            if sr["status"] == "error":
                continue
            cb = sr.get("category_breakdown", {})
            if cat in cb:
                cat_total += cb[cat]["count"]
                cat_displaced += cb[cat]["displaced_gt_0.01"]
                cat_max_disp = max(cat_max_disp, cb[cat]["max_disp"])
        highlight[cat] = {
            "total_prims": cat_total,
            "displaced_gt_0.01": cat_displaced,
            "max_displacement": round(cat_max_disp, 6),
        }

    return {
        "pass": all_pass,
        "checks": checks,
        "totals": {
            "scenes_ok": sum(1 for s in scene_results if s["status"] == "ok"),
            "scenes_error": total_errors,
            "total_compared": total_compared,
            "total_displaced_gt_0.01": total_displaced_001,
            "total_max_mesh_gt_0.01": total_max_mesh_001,
            "total_no_mesh": total_no_mesh,
            "total_only_in_test0": total_only_t0,
            "total_only_in_normalized": total_only_norm,
        },
        "scenes_without_test0": scenes_without_test0 or [],
        "highlight_categories": highlight,
    }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Verify normalized scene placement against test0 ground truth."
    )
    parser.add_argument(
        "--test0-root", default="./GRScenes-test0",
        help="Root directory of test0 dataset (default: ./GRScenes-test0)",
    )
    parser.add_argument(
        "--normalized-root", default="./GRScenes-test1-normalized",
        help="Root directory of normalized dataset (default: ./GRScenes-test1-normalized)",
    )
    parser.add_argument(
        "--out", default="check_reports/placement_verification.json",
        help="Output JSON report path (default: check_reports/placement_verification.json)",
    )
    parser.add_argument(
        "--workers", type=int, default=8,
        help="Number of parallel worker processes (default: 8)",
    )
    parser.add_argument(
        "--scene-filter", default=None,
        help="Only process scenes whose ID contains this substring",
    )
    args = parser.parse_args()

    test0_root = os.path.abspath(args.test0_root)
    norm_root = os.path.abspath(args.normalized_root)

    print(f"Test0 root:       {test0_root}")
    print(f"Normalized root:  {norm_root}")

    scenes, no_test0, no_norm = discover_scenes(test0_root, norm_root, args.scene_filter)
    print(f"Discovered {len(scenes)} matched scene pairs")
    if no_test0:
        print(f"Scenes in normalized but NOT in test0 (skipped): {len(no_test0)}")
        for s in no_test0[:10]:
            print(f"  - {s}")
        if len(no_test0) > 10:
            print(f"  ... and {len(no_test0) - 10} more")
    if no_norm:
        print(f"Scenes in test0 but NOT in normalized: {len(no_norm)}")
        for s in no_norm[:10]:
            print(f"  - {s}")

    if not scenes:
        print("ERROR: No scene pairs found. Check paths.")
        sys.exit(1)

    t_start = time.time()
    scene_results = []

    if args.workers <= 1:
        for i, (t0, norm, sid) in enumerate(scenes):
            print(f"[{i+1}/{len(scenes)}] {sid} ...")
            r = compare_scene(t0, norm, sid)
            scene_results.append(r)
            _print_scene_summary(r)
    else:
        print(f"Running with {args.workers} workers...")
        with ProcessPoolExecutor(max_workers=args.workers) as executor:
            futures = {
                executor.submit(compare_scene, t0, norm, sid): sid
                for t0, norm, sid in scenes
            }
            done_count = 0
            for future in as_completed(futures):
                done_count += 1
                sid = futures[future]
                try:
                    r = future.result()
                except Exception as e:
                    r = {
                        "scene_id": sid,
                        "status": "error",
                        "error": f"Worker exception: {e}",
                    }
                scene_results.append(r)
                status = "OK" if r["status"] == "ok" else f"ERROR: {r.get('error','?')}"
                disp_info = ""
                if r["status"] == "ok":
                    d01 = r.get("displaced_gt_0.01", 0)
                    compared = r.get("total_compared", 0)
                    disp_info = f" compared={compared} displaced>0.01={d01}"
                print(f"  [{done_count}/{len(scenes)}] {sid}: {status}{disp_info}")

    elapsed = time.time() - t_start

    # Sort results by scene_id for deterministic output
    scene_results.sort(key=lambda x: x["scene_id"])

    verdict = compute_verdict(scene_results, scenes_without_test0=no_test0)

    output = {
        "test0_root": test0_root,
        "normalized_root": norm_root,
        "elapsed_seconds": round(elapsed, 1),
        "total_scenes_matched": len(scenes),
        "scenes_without_test0": no_test0,
        "scenes_without_normalized": no_norm,
        "verdict": verdict,
        "per_scene": scene_results,
    }

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w") as f:
        json.dump(output, f, indent=2)

    # Print summary
    print()
    print("=" * 70)
    print("VERIFICATION SUMMARY")
    print("=" * 70)
    v = verdict
    print(f"Scenes matched & processed:    {v['totals']['scenes_ok']} ok, {v['totals']['scenes_error']} errors")
    print(f"Scenes without test0 (skipped): {len(no_test0)}")
    print(f"Total prims compared:          {v['totals']['total_compared']}")
    print(f"Displaced > 0.01:              {v['totals']['total_displaced_gt_0.01']}")
    print(f"Max-mesh displaced > 0.01:     {v['totals']['total_max_mesh_gt_0.01']}")
    print(f"Only in test0:                 {v['totals']['total_only_in_test0']}")
    print(f"Only in normalized:            {v['totals']['total_only_in_normalized']}")
    print(f"No mesh data:                  {v['totals']['total_no_mesh']}")
    print()

    print("Pass/fail checks:")
    for c in v["checks"]:
        mark = "PASS" if c["pass"] else "FAIL"
        print(f"  [{mark}] {c['check']} (value={c['value']})")
    print()

    print("Highlighted categories:")
    for cat in sorted(HIGHLIGHT_CATEGORIES):
        info = v["highlight_categories"].get(cat, {})
        total = info.get("total_prims", 0)
        disp = info.get("displaced_gt_0.01", 0)
        maxd = info.get("max_displacement", 0.0)
        print(f"  {cat:15s}  prims={total:4d}  displaced>0.01={disp:3d}  max_disp={maxd:.6f}")
    print()

    verdict_str = "PASS" if v["pass"] else "FAIL"
    print(f"Overall verdict: {verdict_str}")
    print(f"Elapsed: {elapsed:.1f}s")
    print(f"Report saved to: {args.out}")

    sys.exit(0 if v["pass"] else 1)


def _print_scene_summary(r):
    """Print a one-line summary for sequential mode."""
    if r["status"] != "ok":
        print(f"  ERROR: {r.get('error', '?')}")
        return
    d01 = r.get("displaced_gt_0.01", 0)
    compared = r.get("total_compared", 0)
    max_d = 0.0
    if r.get("top_10_worst"):
        max_d = r["top_10_worst"][0]["displacement"]
    print(f"  compared={compared} displaced>0.01={d01} max_disp={max_d:.6f}")


if __name__ == "__main__":
    main()
