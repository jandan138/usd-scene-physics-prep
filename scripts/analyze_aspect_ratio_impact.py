#!/usr/bin/env python3
"""Analyze how many shape_invariant dedup pairs would be rejected by an
aspect-ratio guard.

For each SI group, loads canonical + first non-canonical asset, computes
bbox extents, and checks if the dimension ratio exceeds a threshold.

Usage:
    python scripts/analyze_aspect_ratio_impact.py \
        [--threshold 2.0] [--workers 8] [--max-groups-per-cat 0]
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import numpy as np

# ---------------------------------------------------------------------------
# Vertex extraction (inline to avoid import issues in subprocesses)
# ---------------------------------------------------------------------------

def _extract_bbox_extents(usd_path: str) -> Optional[np.ndarray]:
    """Return sorted-descending bbox extents (3,) or None."""
    try:
        from pxr import Usd, UsdGeom, Gf
    except ImportError:
        return None
    try:
        stage = Usd.Stage.Open(usd_path, load=Usd.Stage.LoadNone)
    except Exception:
        return None
    if stage is None:
        return None
    default_prim = stage.GetDefaultPrim()
    if not default_prim or not default_prim.IsValid():
        return None

    all_pts = []
    for prim in Usd.PrimRange(default_prim):
        mesh = UsdGeom.Mesh(prim)
        if not mesh:
            continue
        pts_attr = mesh.GetPointsAttr()
        if not pts_attr:
            continue
        pts = pts_attr.Get()
        if not pts or len(pts) == 0:
            continue
        # Get local-to-instance-space transform
        xf_api = UsdGeom.Xformable(prim)
        world_mat = xf_api.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        # We want instance space = default prim space, so compute relative
        dp_xf = UsdGeom.Xformable(default_prim)
        dp_world = dp_xf.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        dp_world_inv = dp_world.GetInverse()
        chain = world_mat * dp_world_inv
        mat_np = np.array(
            [[chain[i][j] for j in range(4)] for i in range(4)], dtype=np.float64
        )
        pts_np = np.array(pts, dtype=np.float64)
        ones = np.ones((pts_np.shape[0], 1), dtype=np.float64)
        pts_h = np.hstack([pts_np, ones])
        pts_t = (pts_h @ mat_np)[:, :3]
        all_pts.append(pts_t)

    if not all_pts:
        return None
    pts = np.vstack(all_pts)
    extents = pts.max(axis=0) - pts.min(axis=0)
    extents = np.sort(extents)[::-1]  # descending
    return extents


def _compute_aspect_ratio(extents: np.ndarray, degen_thr: float = 1e-6) -> float:
    """Max ratio of non-degenerate dimensions."""
    non_degen = extents[extents > degen_thr]
    if len(non_degen) < 2:
        return 1.0
    return float(non_degen[0] / non_degen[-1])


# ---------------------------------------------------------------------------
# Worker function for a single group
# ---------------------------------------------------------------------------

@dataclass
class GroupResult:
    category: str
    sig: str
    canon_path: str
    other_path: str
    canon_extents: Optional[List[float]] = None
    other_extents: Optional[List[float]] = None
    canon_ratio: float = 1.0
    other_ratio: float = 1.0
    cross_ratio: float = 1.0
    rejected: bool = False
    error: str = ""


def _check_group(args: Tuple) -> dict:
    """Check a single group. Returns dict for serialization."""
    category, sig, canon_path, other_path, threshold = args
    result = {
        "category": category,
        "sig": sig,
        "canon_path": canon_path,
        "other_path": other_path,
        "canon_ratio": 1.0,
        "other_ratio": 1.0,
        "cross_ratio": 1.0,
        "rejected": False,
        "error": "",
    }

    canon_ext = _extract_bbox_extents(canon_path)
    if canon_ext is None:
        result["error"] = "canon_load_fail"
        return result

    other_ext = _extract_bbox_extents(other_path)
    if other_ext is None:
        result["error"] = "other_load_fail"
        return result

    result["canon_extents"] = canon_ext.tolist()
    result["other_extents"] = other_ext.tolist()

    canon_ratio = _compute_aspect_ratio(canon_ext)
    other_ratio = _compute_aspect_ratio(other_ext)
    result["canon_ratio"] = round(canon_ratio, 3)
    result["other_ratio"] = round(other_ratio, 3)

    # Cross-check: compare corresponding dimensions
    # Sort both descending, compute max ratio between corresponding dims
    degen_thr = 1e-6
    c_nd = canon_ext[canon_ext > degen_thr]
    o_nd = other_ext[other_ext > degen_thr]
    if len(c_nd) > 0 and len(o_nd) > 0:
        # Pad to same length
        n = max(len(c_nd), len(o_nd))
        c_pad = np.pad(c_nd, (0, n - len(c_nd)), constant_values=degen_thr)
        o_pad = np.pad(o_nd, (0, n - len(o_nd)), constant_values=degen_thr)
        ratios = np.maximum(c_pad / o_pad, o_pad / c_pad)
        cross_ratio = float(ratios.max())
    else:
        cross_ratio = 1.0
    result["cross_ratio"] = round(cross_ratio, 3)

    # Reject if cross-dimension ratio exceeds threshold
    result["rejected"] = cross_ratio > threshold

    return result


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Aspect ratio impact analysis for SI dedup")
    parser.add_argument("--threshold", type=float, default=2.0,
                        help="Max dimension ratio before rejection (default: 2.0)")
    parser.add_argument("--workers", type=int, default=16,
                        help="Parallel workers (default: 16)")
    parser.add_argument("--max-groups-per-cat", type=int, default=0,
                        help="Max groups to check per category (0=all)")
    parser.add_argument("--report-dir", type=str,
                        default="check_reports/test0_rebuilt_dedup/shape_invariant",
                        help="Path to shape_invariant reports")
    parser.add_argument("--out-json", type=str, default="",
                        help="Save detailed results to JSON")
    args = parser.parse_args()

    report_dir = args.report_dir
    if not os.path.isabs(report_dir):
        report_dir = os.path.join(
            "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep",
            report_dir,
        )

    # Collect all (category, sig, canon, other) tuples
    work_items = []
    cat_group_counts = {}
    categories = sorted(os.listdir(report_dir))
    for cat in categories:
        fp = os.path.join(report_dir, cat, f"{cat}_asset_mesh_dedup_shape_invariant.json")
        if not os.path.exists(fp):
            continue
        with open(fp) as f:
            data = json.load(f)
        groups = data.get("duplicates", [])
        if not groups:
            continue
        cat_group_counts[cat] = len(groups)
        limit = args.max_groups_per_cat if args.max_groups_per_cat > 0 else len(groups)
        for grp in groups[:limit]:
            paths = grp.get("usd_paths", [])
            if len(paths) < 2:
                continue
            work_items.append((cat, grp["sig"], paths[0], paths[1], args.threshold))

    print(f"Scanning {len(work_items)} groups across {len(cat_group_counts)} categories "
          f"(threshold={args.threshold})")
    print(f"Using {args.workers} workers...")

    t0 = time.time()
    results = []
    done = 0
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        futures = {pool.submit(_check_group, item): item for item in work_items}
        for fut in as_completed(futures):
            done += 1
            if done % 500 == 0:
                elapsed = time.time() - t0
                print(f"  ... {done}/{len(work_items)} ({elapsed:.0f}s)")
            try:
                results.append(fut.result())
            except Exception as e:
                item = futures[fut]
                results.append({
                    "category": item[0], "sig": item[1],
                    "canon_path": item[2], "other_path": item[3],
                    "error": str(e), "rejected": False,
                    "canon_ratio": 1.0, "other_ratio": 1.0, "cross_ratio": 1.0,
                })

    elapsed = time.time() - t0
    print(f"Done in {elapsed:.1f}s\n")

    # Aggregate per category
    cat_stats = {}
    for r in results:
        cat = r["category"]
        if cat not in cat_stats:
            cat_stats[cat] = {
                "si_groups": cat_group_counts.get(cat, 0),
                "checked": 0, "rejected": 0, "errors": 0,
                "max_cross_ratio": 0.0, "sample_pair": "",
            }
        s = cat_stats[cat]
        s["checked"] += 1
        if r.get("error"):
            s["errors"] += 1
        if r["rejected"]:
            s["rejected"] += 1
        cr = r.get("cross_ratio", 1.0)
        if cr > s["max_cross_ratio"]:
            s["max_cross_ratio"] = cr
            c_hash = os.path.basename(os.path.dirname(os.path.dirname(r["canon_path"])))
            o_hash = os.path.basename(os.path.dirname(os.path.dirname(r["other_path"])))
            s["sample_pair"] = f"{c_hash[:8]} vs {o_hash[:8]}"

    # Print summary
    print(f"{'='*80}")
    print(f"  ASPECT RATIO IMPACT ANALYSIS (threshold={args.threshold})")
    print(f"{'='*80}")
    print(f"{'Category':<28} {'SI_grps':>8} {'Checked':>8} {'Rejected':>8} {'Pct':>7} {'MaxRatio':>9}  Sample")
    print(f"{'-'*28} {'-'*8} {'-'*8} {'-'*8} {'-'*7} {'-'*9}  {'-'*20}")

    total_groups = 0
    total_checked = 0
    total_rejected = 0
    total_errors = 0

    for cat in sorted(cat_stats.keys()):
        s = cat_stats[cat]
        total_groups += s["si_groups"]
        total_checked += s["checked"]
        total_rejected += s["rejected"]
        total_errors += s["errors"]
        pct = (s["rejected"] / s["checked"] * 100) if s["checked"] > 0 else 0
        print(f"{cat:<28} {s['si_groups']:>8} {s['checked']:>8} {s['rejected']:>8} "
              f"{pct:>6.1f}% {s['max_cross_ratio']:>9.2f}  {s['sample_pair']}")

    print(f"{'-'*28} {'-'*8} {'-'*8} {'-'*8} {'-'*7} {'-'*9}")
    pct = (total_rejected / total_checked * 100) if total_checked > 0 else 0
    print(f"{'TOTAL':<28} {total_groups:>8} {total_checked:>8} {total_rejected:>8} "
          f"{pct:>6.1f}%")
    if total_errors:
        print(f"\nErrors (load failures): {total_errors}")

    # Save detailed JSON if requested
    if args.out_json:
        out_path = args.out_json
        if not os.path.isabs(out_path):
            out_path = os.path.join(
                "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep", out_path
            )
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        # Sort rejected first
        results_sorted = sorted(results, key=lambda r: (-r.get("cross_ratio", 0),))
        with open(out_path, "w") as f:
            json.dump({
                "threshold": args.threshold,
                "total_groups": total_groups,
                "total_checked": total_checked,
                "total_rejected": total_rejected,
                "total_errors": total_errors,
                "rejection_pct": round(pct, 2),
                "per_category": cat_stats,
                "details": results_sorted,
            }, f, indent=2)
        print(f"\nDetailed results saved to: {out_path}")


if __name__ == "__main__":
    main()
