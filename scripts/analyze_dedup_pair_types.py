#!/usr/bin/env python3
"""Classify non-geom_only dedup pairs by vertex-space relationship.

Performs stratified sampling and Procrustes analysis on union-merged dedup pairs
to determine which compensation strategies are needed for each pair type.

Classification labels:
  - hierarchy_only:       vertices already identical (max diff < threshold)
  - rigid_rotation_only:  det(R)=+1, Procrustes residual < threshold
  - mirror_only:          det(R)=-1, residual < threshold after reflection fix
  - rigid_scale:          rotation + uniform scale, residual < threshold
  - mirror_scale:         reflection + uniform scale, residual < threshold
  - not_same_geometry:    residual > threshold after best alignment
  - vertex_count_mismatch: different number of vertices (centroid-only analysis)
  - load_error:           failed to open/parse one or both assets

Output: JSON report with per-stratum statistics and sampled pair details.

Usage:
  python scripts/analyze_dedup_pair_types.py \
    --union-report check_reports/test0_rebuilt_dedup/union_3way/all_categories_union_merged.json \
    --dedup-dir check_reports/test0_rebuilt_dedup \
    --dataset-root GRScenes-test0-rebuilt-normalized \
    --bak-root GRScenes-test0-rebuilt-normalized_bak \
    --out-dir check_reports/test0_rebuilt_dedup/pair_type_investigation \
    [--samples-per-stratum 100] [--workers 8] [--seed 42]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import random
import sys
import time
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
from typing import Any, Dict, List, Optional, Set, Tuple

import numpy as np

try:
    from pxr import Gf, Sdf, Usd, UsdGeom
except ImportError as exc:
    print(f"Error: pxr (USD) not available: {exc}", file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Threshold for "identical" vertices (floating point noise)
IDENTITY_THRESHOLD = 0.01
# Threshold for Procrustes residual to consider a valid alignment
ALIGNMENT_THRESHOLD = 0.05
# Max vertices to use for alignment (subsample if more)
MAX_VERTICES_FOR_ALIGNMENT = 5000


# ---------------------------------------------------------------------------
# USD helpers
# ---------------------------------------------------------------------------

def _get_local_matrix(prim) -> Gf.Matrix4d:
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return Gf.Matrix4d(1.0)
    res = xf.GetLocalTransformation(Usd.TimeCode.Default())
    if isinstance(res, tuple):
        return res[0]
    return res


def _get_chain_transform(ancestor, descendant) -> Gf.Matrix4d:
    chain = []
    cur = descendant
    while cur and cur.GetPath() != ancestor.GetPath():
        chain.append(cur)
        cur = cur.GetParent()
    chain.reverse()
    result = Gf.Matrix4d(1.0)
    for p in chain:
        result = _get_local_matrix(p) * result
    return result


def _extract_world_vertices(asset_usd: str) -> Optional[np.ndarray]:
    """Extract all mesh vertices transformed to defaultPrim space."""
    try:
        stage = Usd.Stage.Open(asset_usd, load=Usd.Stage.LoadNone)
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

        # Transform points to defaultPrim space
        chain_mat = _get_chain_transform(default_prim, prim)
        mat_np = np.array([[chain_mat[i][j] for j in range(4)] for i in range(4)])

        pts_np = np.array(pts, dtype=np.float64)
        ones = np.ones((pts_np.shape[0], 1), dtype=np.float64)
        pts_h = np.hstack([pts_np, ones])  # Nx4
        # Row-vector convention: p_world = p_local * M
        pts_world = pts_h @ mat_np
        all_pts.append(pts_world[:, :3])

    if not all_pts:
        return None
    return np.vstack(all_pts)


# ---------------------------------------------------------------------------
# Procrustes analysis
# ---------------------------------------------------------------------------

def _procrustes_classify(
    pts_a: np.ndarray,
    pts_b: np.ndarray,
) -> Dict[str, Any]:
    """Classify the geometric relationship between two point sets.

    Returns dict with:
      label: classification string
      det_R: determinant of best rotation/reflection
      residual: per-vertex RMSE after best alignment
      scale_ratio: uniform scale ratio (b/a norms)
      max_displacement_before: max vertex displacement before alignment
      centroid_displacement: displacement of centroids
    """
    result: Dict[str, Any] = {}

    n_a, n_b = len(pts_a), len(pts_b)
    result["vertex_count_a"] = n_a
    result["vertex_count_b"] = n_b

    if n_a != n_b:
        # Centroid-only analysis
        centroid_a = pts_a.mean(axis=0)
        centroid_b = pts_b.mean(axis=0)
        result["label"] = "vertex_count_mismatch"
        result["centroid_displacement"] = float(np.linalg.norm(centroid_a - centroid_b))
        result["det_R"] = None
        result["residual"] = None
        result["scale_ratio"] = None
        result["max_displacement_before"] = None
        return result

    # Check if already identical
    diffs = np.linalg.norm(pts_a - pts_b, axis=1)
    max_diff = float(diffs.max())
    result["max_displacement_before"] = max_diff

    if max_diff < IDENTITY_THRESHOLD:
        result["label"] = "hierarchy_only"
        result["det_R"] = 1.0
        result["residual"] = float(diffs.mean())
        result["scale_ratio"] = 1.0
        result["centroid_displacement"] = float(
            np.linalg.norm(pts_a.mean(axis=0) - pts_b.mean(axis=0))
        )
        return result

    # Center both point clouds
    centroid_a = pts_a.mean(axis=0)
    centroid_b = pts_b.mean(axis=0)
    result["centroid_displacement"] = float(np.linalg.norm(centroid_a - centroid_b))

    a_centered = pts_a - centroid_a
    b_centered = pts_b - centroid_b

    # Subsample if too many vertices
    if n_a > MAX_VERTICES_FOR_ALIGNMENT:
        rng = np.random.RandomState(42)
        idx = rng.choice(n_a, MAX_VERTICES_FOR_ALIGNMENT, replace=False)
        a_sub = a_centered[idx]
        b_sub = b_centered[idx]
    else:
        a_sub = a_centered
        b_sub = b_centered

    # Compute norms for scale detection
    norm_a = np.linalg.norm(a_sub)
    norm_b = np.linalg.norm(b_sub)
    if norm_a < 1e-10 or norm_b < 1e-10:
        result["label"] = "not_same_geometry"
        result["det_R"] = None
        result["residual"] = float("inf")
        result["scale_ratio"] = None
        return result

    scale_ratio = norm_b / norm_a
    result["scale_ratio"] = float(scale_ratio)

    # --- Try 1: rigid alignment (no scale) ---
    label, det_r, residual = _try_procrustes(a_sub, b_sub, a_centered, b_centered)
    if residual < ALIGNMENT_THRESHOLD:
        if abs(det_r - 1.0) < 0.01:
            result["label"] = "rigid_rotation_only"
        else:
            result["label"] = "mirror_only"
        result["det_R"] = float(det_r)
        result["residual"] = float(residual)
        return result

    # --- Try 2: with uniform scale ---
    a_scaled = a_sub / norm_a
    b_scaled = b_sub / norm_b
    a_centered_scaled = a_centered / norm_a
    b_centered_scaled = b_centered / norm_b

    label2, det_r2, residual2_norm = _try_procrustes(
        a_scaled, b_scaled, a_centered_scaled, b_centered_scaled
    )
    # Scale residual back
    residual2 = residual2_norm * norm_b

    if residual2 < ALIGNMENT_THRESHOLD:
        if abs(det_r2 - 1.0) < 0.01:
            result["label"] = "rigid_scale"
        else:
            result["label"] = "mirror_scale"
        result["det_R"] = float(det_r2)
        result["residual"] = float(residual2)
        return result

    # --- No good alignment found ---
    result["label"] = "not_same_geometry"
    result["det_R"] = float(det_r)
    result["residual"] = float(min(residual, residual2))
    return result


def _try_procrustes(
    a_sub: np.ndarray,
    b_sub: np.ndarray,
    a_full: np.ndarray,
    b_full: np.ndarray,
) -> Tuple[str, float, float]:
    """Run Procrustes on subsampled points, evaluate on full set.

    Returns (label, det_R, rmse_residual).
    """
    # Cross-covariance matrix
    H = a_sub.T @ b_sub
    U, S, Vt = np.linalg.svd(H)
    det = np.linalg.det(Vt.T @ U.T)

    # Optimal rotation (may include reflection)
    R = Vt.T @ U.T

    # Apply rotation to full set
    a_rotated = a_full @ R.T
    diffs = np.linalg.norm(a_rotated - b_full, axis=1)
    rmse = float(np.sqrt(np.mean(diffs ** 2)))

    return ("", float(det), rmse)


# ---------------------------------------------------------------------------
# Pair loading & mode classification
# ---------------------------------------------------------------------------

def _uid_from_path(p: str) -> str:
    """Extract uid from path like .../category/uid/usd/uid.usd"""
    return os.path.basename(os.path.dirname(os.path.dirname(p)))


def build_mode_pair_sets(dedup_dir: str) -> Dict[str, Set[Tuple[str, str]]]:
    """Build set of (canonical_uid, old_uid) pairs for each dedup mode."""
    mode_sets: Dict[str, Set[Tuple[str, str]]] = {}

    for mode in ["geom_only", "shape_invariant", "topo_filesize"]:
        pairs: Set[Tuple[str, str]] = set()
        mode_dir = os.path.join(dedup_dir, mode)
        if not os.path.isdir(mode_dir):
            continue
        for cat in os.listdir(mode_dir):
            report = os.path.join(
                mode_dir, cat, f"{cat}_asset_mesh_dedup_{mode}.json"
            )
            if not os.path.isfile(report):
                continue
            with open(report) as f:
                data = json.load(f)
            for d in data.get("duplicates", []):
                paths = d.get("usd_paths", [])
                if len(paths) < 2:
                    continue
                uids = sorted([_uid_from_path(p) for p in paths])
                canonical = uids[0]
                for uid in uids[1:]:
                    pairs.add((canonical, uid))
        mode_sets[mode] = pairs

    return mode_sets


def classify_union_pairs(
    union_report: str,
    mode_sets: Dict[str, Set[Tuple[str, str]]],
) -> Dict[str, List[Dict[str, str]]]:
    """Classify each union pair into a stratum based on which modes matched it."""
    with open(union_report) as f:
        union = json.load(f)

    geom = mode_sets.get("geom_only", set())
    shape = mode_sets.get("shape_invariant", set())
    topo = mode_sets.get("topo_filesize", set())

    strata: Dict[str, List[Dict[str, str]]] = defaultdict(list)

    for d in union.get("duplicates", []):
        paths = d.get("usd_paths", [])
        if len(paths) < 2:
            continue
        uids = [_uid_from_path(p) for p in paths]
        # Extract category from path
        cat = os.path.basename(os.path.dirname(os.path.dirname(os.path.dirname(paths[0]))))
        canonical_uid = uids[0]
        canonical_path = paths[0]

        for uid, path in zip(uids[1:], paths[1:]):
            pair = (min(canonical_uid, uid), max(canonical_uid, uid))
            rpair = (pair[1], pair[0])
            in_g = pair in geom or rpair in geom
            in_s = pair in shape or rpair in shape
            in_t = pair in topo or rpair in topo

            if in_g:
                stratum = "has_geom"  # Skip — safe
            elif in_s and in_t:
                stratum = "shape+topo"
            elif in_s:
                stratum = "shape_only"
            elif in_t:
                stratum = "topo_only"
            else:
                stratum = "none"

            strata[stratum].append({
                "category": cat,
                "canonical_uid": canonical_uid,
                "canonical_path": canonical_path,
                "old_uid": uid,
                "old_path": path,
            })

    return dict(strata)


def _resolve_asset_usd(
    dataset_root: str,
    bak_root: str,
    rel_path: str,
) -> Optional[str]:
    """Resolve a relative asset USD path, checking dataset first, then backup."""
    # Direct path under dataset root
    abs_path = os.path.join(dataset_root, rel_path)
    if os.path.isfile(abs_path):
        return abs_path

    # Also try without dataset prefix in the relative path
    # e.g., rel_path = "GRScenes-test0-rebuilt-normalized/GRScenes_assets/cat/uid/usd/uid.usd"
    # Try stripping dataset dir name
    parts = rel_path.replace("\\", "/").split("/")
    if len(parts) > 1 and parts[0].startswith("GRScenes"):
        sub_path = "/".join(parts[1:])
        abs_path2 = os.path.join(dataset_root, sub_path)
        if os.path.isfile(abs_path2):
            return abs_path2

    # Search in backup dedup assets
    if bak_root:
        dedup_dir = os.path.join(bak_root, "_dedup_assets")
        if os.path.isdir(dedup_dir):
            # Extract category/uid/usd/uid.usd from rel_path
            # Find GRScenes_assets/ prefix
            idx = rel_path.find("GRScenes_assets/")
            if idx >= 0:
                suffix = rel_path[idx:]  # GRScenes_assets/cat/uid/usd/uid.usd
                for batch_name in os.listdir(dedup_dir):
                    candidate = os.path.join(dedup_dir, batch_name, suffix)
                    if os.path.isfile(candidate):
                        return candidate

    return None


def _analyze_single_pair(args: Tuple) -> Dict[str, Any]:
    """Worker function for parallel pair analysis."""
    pair_info, dataset_root, bak_root = args
    canonical_path = pair_info["canonical_path"]
    old_path = pair_info["old_path"]

    result = {
        "category": pair_info["category"],
        "canonical_uid": pair_info["canonical_uid"],
        "old_uid": pair_info["old_uid"],
    }

    # Resolve asset paths
    canonical_usd = _resolve_asset_usd(dataset_root, bak_root, canonical_path)
    old_usd = _resolve_asset_usd(dataset_root, bak_root, old_path)

    if not canonical_usd or not old_usd:
        result["label"] = "load_error"
        result["error"] = f"canonical={canonical_usd is not None}, old={old_usd is not None}"
        return result

    # Extract vertices
    try:
        pts_canonical = _extract_world_vertices(canonical_usd)
        pts_old = _extract_world_vertices(old_usd)
    except Exception as e:
        result["label"] = "load_error"
        result["error"] = str(e)
        return result

    if pts_canonical is None or pts_old is None:
        result["label"] = "load_error"
        result["error"] = "no vertices extracted"
        return result

    # Run Procrustes classification
    classification = _procrustes_classify(pts_canonical, pts_old)
    result.update(classification)
    return result


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv: Optional[list] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--union-report", required=True)
    parser.add_argument("--dedup-dir", required=True)
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument("--bak-root", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--samples-per-stratum", type=int, default=100)
    parser.add_argument("--workers", type=int, default=8)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)
    t0 = time.time()

    print("=== Step 1: Build mode pair sets ===")
    mode_sets = build_mode_pair_sets(args.dedup_dir)
    for mode, pairs in mode_sets.items():
        print(f"  {mode}: {len(pairs)} pairs")

    print("\n=== Step 2: Classify union pairs into strata ===")
    strata = classify_union_pairs(args.union_report, mode_sets)
    for stratum, pairs in sorted(strata.items(), key=lambda x: -len(x[1])):
        print(f"  {stratum}: {len(pairs)} pairs")

    print("\n=== Step 3: Stratified sampling ===")
    rng = random.Random(args.seed)
    target_strata = ["topo_only", "shape_only", "shape+topo", "none"]
    sampled: Dict[str, List[Dict]] = {}
    for stratum in target_strata:
        pool = strata.get(stratum, [])
        n = min(args.samples_per_stratum, len(pool))
        sampled[stratum] = rng.sample(pool, n) if n > 0 else []
        print(f"  {stratum}: sampled {n}/{len(pool)}")

    print("\n=== Step 4: Procrustes analysis ===")
    all_results: Dict[str, List[Dict]] = {}

    for stratum in target_strata:
        pairs = sampled.get(stratum, [])
        if not pairs:
            all_results[stratum] = []
            continue

        print(f"\n  Analyzing {stratum} ({len(pairs)} pairs)...")
        work_items = [(p, args.dataset_root, args.bak_root) for p in pairs]

        results = []
        with ProcessPoolExecutor(max_workers=args.workers) as executor:
            futures = {executor.submit(_analyze_single_pair, w): i for i, w in enumerate(work_items)}
            done = 0
            for future in as_completed(futures):
                done += 1
                if done % 20 == 0 or done == len(pairs):
                    print(f"    {done}/{len(pairs)}")
                try:
                    r = future.result()
                    results.append(r)
                except Exception as e:
                    results.append({
                        "label": "load_error",
                        "error": str(e),
                        "category": pairs[futures[future]].get("category", "?"),
                        "canonical_uid": pairs[futures[future]].get("canonical_uid", "?"),
                        "old_uid": pairs[futures[future]].get("old_uid", "?"),
                    })

        all_results[stratum] = results

    # --- Compile statistics ---
    print("\n=== Step 5: Compile statistics ===")
    stats: Dict[str, Dict[str, Any]] = {}
    for stratum in target_strata:
        results = all_results.get(stratum, [])
        label_counts: Dict[str, int] = defaultdict(int)
        for r in results:
            label_counts[r.get("label", "unknown")] += 1

        total = len(results)
        stats[stratum] = {
            "total_in_union": len(strata.get(stratum, [])),
            "sampled": total,
            "label_counts": dict(label_counts),
            "label_pcts": {
                k: round(100 * v / total, 1) if total > 0 else 0
                for k, v in label_counts.items()
            },
        }

        print(f"\n  {stratum} ({total} sampled / {stats[stratum]['total_in_union']} total):")
        for label, count in sorted(label_counts.items(), key=lambda x: -x[1]):
            pct = 100 * count / total if total > 0 else 0
            print(f"    {label}: {count} ({pct:.1f}%)")

    # --- Compile full report ---
    elapsed = time.time() - t0

    # Extrapolate to full union
    extrapolation: Dict[str, Dict[str, int]] = {}
    for stratum in target_strata:
        s = stats.get(stratum, {})
        total_union = s.get("total_in_union", 0)
        sampled_n = s.get("sampled", 0)
        pcts = s.get("label_pcts", {})
        extrapolation[stratum] = {
            label: int(round(pct / 100 * total_union))
            for label, pct in pcts.items()
        }

    # Risk classification
    safe_labels = {"hierarchy_only", "rigid_rotation_only"}
    mirror_labels = {"mirror_only", "mirror_scale"}
    scale_labels = {"rigid_scale"}
    unsafe_labels = {"not_same_geometry", "vertex_count_mismatch", "load_error", "unknown"}

    risk_summary: Dict[str, int] = defaultdict(int)
    for stratum in target_strata:
        ext = extrapolation.get(stratum, {})
        for label, count in ext.items():
            if label in safe_labels:
                risk_summary["safe_procrustes"] += count
            elif label in mirror_labels:
                risk_summary["needs_mirror_handling"] += count
            elif label in scale_labels:
                risk_summary["needs_scale_handling"] += count
            elif label in unsafe_labels:
                risk_summary["should_exclude"] += count
            else:
                risk_summary["unknown"] += count

    # Add geom_only safe count
    has_geom_count = len(strata.get("has_geom", []))
    risk_summary["safe_geom_only"] = has_geom_count

    report = {
        "meta": {
            "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "elapsed_sec": round(elapsed, 1),
            "seed": args.seed,
            "samples_per_stratum": args.samples_per_stratum,
            "union_report": args.union_report,
        },
        "strata_stats": stats,
        "extrapolation": extrapolation,
        "risk_summary": risk_summary,
        "risk_summary_notes": {
            "safe_geom_only": "geom_only pairs — identical vertices, transform compensation only",
            "safe_procrustes": "rigid rotation only — can be compensated with Procrustes R matrix",
            "needs_mirror_handling": "reflection/mirror — needs det(R)=-1 aware compensation",
            "needs_scale_handling": "rotation + uniform scale — needs scale-aware compensation",
            "should_exclude": "not geometrically similar — must exclude from dedup",
            "unknown": "unclassified",
        },
        "sampled_details": {
            stratum: all_results.get(stratum, [])
            for stratum in target_strata
        },
    }

    # Save report
    report_path = os.path.join(args.out_dir, "pair_type_investigation.json")
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    print(f"\nReport saved to: {report_path}")

    # Save summary markdown
    md_path = os.path.join(args.out_dir, "pair_type_investigation_summary.md")
    with open(md_path, "w", encoding="utf-8") as f:
        f.write("# Dedup Pair Type Investigation\n\n")
        f.write(f"Date: {report['meta']['timestamp']}\n")
        f.write(f"Elapsed: {elapsed:.0f}s\n\n")

        f.write("## Per-Stratum Classification\n\n")
        f.write("| Stratum | Total | Sampled | hierarchy_only | rigid_rot | mirror | rigid_scale | mirror_scale | not_same | vtx_mismatch | load_err |\n")
        f.write("|---------|-------|---------|---------------|-----------|--------|-------------|-------------|----------|-------------|----------|\n")
        for stratum in target_strata:
            s = stats.get(stratum, {})
            lc = s.get("label_counts", {})
            f.write(
                f"| {stratum} | {s.get('total_in_union', 0)} | {s.get('sampled', 0)} "
                f"| {lc.get('hierarchy_only', 0)} | {lc.get('rigid_rotation_only', 0)} "
                f"| {lc.get('mirror_only', 0)} | {lc.get('rigid_scale', 0)} "
                f"| {lc.get('mirror_scale', 0)} | {lc.get('not_same_geometry', 0)} "
                f"| {lc.get('vertex_count_mismatch', 0)} | {lc.get('load_error', 0)} |\n"
            )

        f.write("\n## Risk Summary (Extrapolated to Full Union)\n\n")
        f.write("| Risk Category | Estimated Count | Strategy |\n")
        f.write("|--------------|----------------|----------|\n")
        for cat, count in sorted(risk_summary.items(), key=lambda x: -x[1]):
            note = report["risk_summary_notes"].get(cat, "")
            f.write(f"| {cat} | {count} | {note} |\n")

        total_all = sum(risk_summary.values())
        safe_total = risk_summary.get("safe_geom_only", 0) + risk_summary.get("safe_procrustes", 0)
        f.write(f"\n**Total pairs**: {total_all}\n")
        f.write(f"**Safe with current + Procrustes**: {safe_total} ({100*safe_total/total_all:.1f}%)\n")
        f.write(f"**Needs mirror handling**: {risk_summary.get('needs_mirror_handling', 0)}\n")
        f.write(f"**Should exclude**: {risk_summary.get('should_exclude', 0)}\n")

    print(f"Summary saved to: {md_path}")

    # Print risk summary
    print("\n=== RISK SUMMARY (Extrapolated) ===")
    total_all = sum(risk_summary.values())
    for cat, count in sorted(risk_summary.items(), key=lambda x: -x[1]):
        print(f"  {cat}: {count} ({100*count/total_all:.1f}%)")
    print(f"  TOTAL: {total_all}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
