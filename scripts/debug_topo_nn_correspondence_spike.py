#!/usr/bin/env python3
"""Phase 2 spike: evaluate NN correspondence recovery for topo_filesize pairs.

Reads cert JSONL, samples pairs into buckets (poor/gap/good/geom_only),
runs baseline (procrustes_full) and candidate A (bbox normalize + KDTree NN
reorder + Procrustes + denormalize) on each pair, outputs CSV.

Usage:
    python3 scripts/debug_topo_nn_correspondence_spike.py \
        --cert-jsonl check_reports/.../pair_certificates.jsonl \
        --dataset-root GRScenes-test0-rebuilt-normalize-prededup \
        --out-csv check_reports/test0_rebuilt_dedup/topo_nn_spike_results.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# Try importing from compute_vertex_transform; fall back to local copies
# ---------------------------------------------------------------------------

_USE_UPSTREAM = False

try:
    # Add project root to path so we can import scripts.compute_vertex_transform
    _project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _project_root not in sys.path:
        sys.path.insert(0, _project_root)

    from scripts.compute_vertex_transform import (
        _normalize_to_unit_bbox,
        _try_single_procrustes,
        extract_instance_space_vertices,
        procrustes_full,
    )
    _USE_UPSTREAM = True
    logger.info("Using upstream compute_vertex_transform functions.")
except ImportError as exc:
    logger.warning(
        "Could not import from compute_vertex_transform (%s); "
        "using local copies.", exc,
    )

# ---------------------------------------------------------------------------
# Local copies of core functions (used when pxr import fails)
# ---------------------------------------------------------------------------

if not _USE_UPSTREAM:
    # --- Constants ---
    MAX_VERTICES_FOR_ALIGNMENT = 5000

    def _normalize_to_unit_bbox(
        pts: np.ndarray,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """Normalize points to unit bounding box (uniform scale, preserving aspect)."""
        bbox_min = pts.min(axis=0)
        bbox_max = pts.max(axis=0)
        extent = bbox_max - bbox_min
        max_extent = max(float(extent.max()), 1e-10)
        normalized = (pts - bbox_min) / max_extent
        return normalized, bbox_min, max_extent

    def _try_single_procrustes(
        a_centered: np.ndarray,
        b_centered: np.ndarray,
        allow_reflection: bool,
        allow_scale: bool,
    ) -> Tuple[np.ndarray, float, float]:
        """Run one Procrustes variant. Returns (R_3x3, scale, rmse)."""
        norm_a = np.linalg.norm(a_centered)
        norm_b = np.linalg.norm(b_centered)
        if norm_a < 1e-10 or norm_b < 1e-10:
            return np.eye(3), 1.0, float("inf")

        if allow_scale:
            a_work = a_centered / norm_a
            b_work = b_centered / norm_b
        else:
            a_work = a_centered
            b_work = b_centered

        n = len(a_work)
        if n > MAX_VERTICES_FOR_ALIGNMENT:
            rng = np.random.RandomState(42)
            idx = rng.choice(n, MAX_VERTICES_FOR_ALIGNMENT, replace=False)
            a_sub = a_work[idx]
            b_sub = b_work[idx]
        else:
            a_sub = a_work
            b_sub = b_work

        H = a_sub.T @ b_sub
        U, S, Vt = np.linalg.svd(H)
        R_row = U @ Vt
        det = np.linalg.det(R_row)

        if not allow_reflection and det < 0:
            U_copy = U.copy()
            U_copy[:, -1] *= -1
            R_row = U_copy @ Vt

        R = R_row
        if allow_scale:
            scale = norm_b / norm_a
        else:
            scale = 1.0

        a_aligned = a_centered @ (scale * R)
        diffs = np.linalg.norm(a_aligned - b_centered, axis=1)
        rmse = float(np.sqrt(np.mean(diffs ** 2)))
        return R, scale, rmse

    def procrustes_full(
        pts_canon: np.ndarray, pts_old: np.ndarray
    ) -> np.ndarray:
        """Enhanced Procrustes: try 4 combos, pick lowest RMSE. Returns 4x4 V."""
        if len(pts_canon) != len(pts_old):
            raise ValueError(
                f"Vertex count mismatch: canon={len(pts_canon)}, old={len(pts_old)}"
            )
        c_canon = pts_canon.mean(axis=0)
        c_old = pts_old.mean(axis=0)
        a = pts_canon - c_canon
        b = pts_old - c_old

        combos = [
            (False, False), (True, False), (False, True), (True, True),
        ]
        best_R, best_scale, best_rmse = np.eye(3), 1.0, float("inf")
        for allow_refl, allow_scl in combos:
            R, s, rmse = _try_single_procrustes(a, b, allow_refl, allow_scl)
            if rmse < best_rmse:
                best_R, best_scale, best_rmse = R, s, rmse

        sR = best_scale * best_R
        t = c_old - c_canon @ sR
        V = np.eye(4, dtype=np.float64)
        V[:3, :3] = sR
        V[3, :3] = t
        return V

    def extract_instance_space_vertices(
        asset_usd: str,
    ) -> Optional[np.ndarray]:
        """Extract all mesh vertices transformed to defaultPrim space (requires pxr)."""
        try:
            from pxr import Gf, Usd, UsdGeom
        except ImportError:
            raise RuntimeError(
                "pxr is required for extract_instance_space_vertices. "
                "Run via isaac_python.sh or install usd-core."
            )

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

            # chain transform to defaultPrim space
            chain = []
            cur = prim
            while cur and cur.GetPath() != default_prim.GetPath():
                chain.append(cur)
                cur = cur.GetParent()
            chain.reverse()
            result = Gf.Matrix4d(1.0)
            for p in chain:
                xf = UsdGeom.Xformable(p)
                if xf:
                    res = xf.GetLocalTransformation(Usd.TimeCode.Default())
                    local = res[0] if isinstance(res, tuple) else res
                else:
                    local = Gf.Matrix4d(1.0)
                result = local * result

            mat_np = np.array(
                [[result[i][j] for j in range(4)] for i in range(4)],
                dtype=np.float64,
            )
            pts_np = np.array(pts, dtype=np.float64)
            ones = np.ones((pts_np.shape[0], 1), dtype=np.float64)
            pts_h = np.hstack([pts_np, ones])
            pts_instance = pts_h @ mat_np
            all_pts.append(pts_instance[:, :3])

        if not all_pts:
            return None
        return np.vstack(all_pts)


# ---------------------------------------------------------------------------
# Candidate A: bbox normalize + KDTree NN reorder + Procrustes + denormalize
# ---------------------------------------------------------------------------

def candidate_a_v_matrix(
    pts_canon: np.ndarray,
    pts_old: np.ndarray,
) -> Tuple[np.ndarray, float, float, float]:
    """Compute V via bbox normalize + NN reorder + Procrustes + denormalize.

    Returns:
        (V_4x4, nn_mean_dist, nn_close_pct, nn_unique_ratio)
    """
    from scipy.spatial import cKDTree

    # 1. bbox normalize
    c_norm, c_min, c_ext = _normalize_to_unit_bbox(pts_canon)
    o_norm, o_min, o_ext = _normalize_to_unit_bbox(pts_old)

    # 2. NN correspondence recovery
    tree = cKDTree(o_norm)
    dists, indices = tree.query(c_norm)
    o_norm_reordered = o_norm[indices]

    # NN quality metrics
    nn_mean_dist = float(np.mean(dists))
    nn_close_pct = float(np.sum(dists < 0.01) / len(dists) * 100.0)
    nn_unique_ratio = float(len(np.unique(indices)) / len(indices))

    # 3. Procrustes in normalized space (now index-aligned)
    c_c = c_norm.mean(axis=0)
    c_o = o_norm_reordered.mean(axis=0)
    a = c_norm - c_c
    b = o_norm_reordered - c_o

    best_R, best_s, best_rmse = np.eye(3), 1.0, float("inf")
    for allow_refl, allow_scl in [
        (False, False), (True, False), (False, True), (True, True),
    ]:
        R, s, rmse = _try_single_procrustes(a, b, allow_refl, allow_scl)
        if rmse < best_rmse:
            best_R, best_s, best_rmse = R, s, rmse

    R_norm = best_s * best_R
    t_norm = c_o - c_c @ R_norm

    # 4. Denormalize: build V (same formula as compute_V_shape_invariant)
    #    p_old_instance = ((p_canon_instance - c_min) / c_ext) @ R_norm + t_norm) * o_ext + o_min
    sR = R_norm * (o_ext / c_ext)  # 3x3
    t = (-c_min / c_ext) @ R_norm * o_ext + t_norm * o_ext + o_min  # 1x3

    V = np.eye(4, dtype=np.float64)
    V[:3, :3] = sR
    V[3, :3] = t

    return V, nn_mean_dist, nn_close_pct, nn_unique_ratio


# ---------------------------------------------------------------------------
# Evaluation helpers
# ---------------------------------------------------------------------------

def eval_v_matrix(
    V: np.ndarray,
    pts_canon: np.ndarray,
    pts_old: np.ndarray,
) -> Tuple[float, float]:
    """Evaluate V: returns (rmse, bbox_max_abs).

    RMSE is computed only when vertex counts match; otherwise inf.
    """
    # Transform canonical pts using V (row-vector: p_old ~ p_canon @ V)
    canon_h = np.hstack([pts_canon, np.ones((len(pts_canon), 1))])
    canon_transformed = (canon_h @ V)[:, :3]

    # bbox delta
    bbox_old_min = pts_old.min(axis=0)
    bbox_old_max = pts_old.max(axis=0)
    bbox_new_min = canon_transformed.min(axis=0)
    bbox_new_max = canon_transformed.max(axis=0)
    delta_min = np.abs(bbox_old_min - bbox_new_min)
    delta_max = np.abs(bbox_old_max - bbox_new_max)
    bbox_max_abs = float(max(delta_min.max(), delta_max.max()))

    # RMSE (only for same vertex count)
    if len(pts_old) == len(canon_transformed):
        diffs = np.linalg.norm(pts_old - canon_transformed, axis=1)
        rmse = float(np.sqrt(np.mean(diffs ** 2)))
    else:
        rmse = float("inf")

    return rmse, bbox_max_abs


# ---------------------------------------------------------------------------
# Sampling
# ---------------------------------------------------------------------------

def load_and_sample_pairs(
    cert_jsonl: str,
    seed: int = 42,
) -> List[Dict]:
    """Load cert JSONL and sample into 4 buckets.

    Returns list of dicts with keys: old_usd, canon_usd, mode, bucket.
    """
    topo_poor: List[Dict] = []
    topo_gap: List[Dict] = []
    topo_good: List[Dict] = []
    geom_only_pool: List[Dict] = []

    with open(cert_jsonl) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            rec = json.loads(line)
            mode = rec.get("mode", "")
            old_usd = rec.get("old_asset", "")
            canon_usd = rec.get("canonical_asset", "")

            # Need bbox_delta.max_abs for topo_filesize bucketing
            bbox_delta = rec.get("bbox_delta")
            if isinstance(bbox_delta, dict):
                max_abs = bbox_delta.get("max_abs", 0.0)
            else:
                max_abs = 0.0

            entry = {
                "old_usd": old_usd,
                "canon_usd": canon_usd,
                "mode": mode,
            }

            if mode == "topo_filesize":
                if max_abs > 0.5:
                    entry["bucket"] = "poor"
                    topo_poor.append(entry)
                elif max_abs > 0.15:
                    entry["bucket"] = "gap"
                    topo_gap.append(entry)
                elif max_abs <= 0.01:
                    entry["bucket"] = "good"
                    topo_good.append(entry)
                # Skip 0.01 < max_abs <= 0.15 (not in any bucket)
            elif mode == "geom_only":
                entry["bucket"] = "geom_only"
                geom_only_pool.append(entry)

    rng = np.random.RandomState(seed)

    def _sample(pool, n):
        if len(pool) <= n:
            return pool
        idx = rng.choice(len(pool), n, replace=False)
        return [pool[i] for i in idx]

    sampled = []
    sampled.extend(_sample(topo_poor, 100))
    sampled.extend(_sample(topo_gap, 100))
    sampled.extend(_sample(topo_good, 100))
    sampled.extend(_sample(geom_only_pool, 50))

    logger.info(
        "Sampled: poor=%d (of %d), gap=%d (of %d), good=%d (of %d), "
        "geom_only=%d (of %d)",
        min(len(topo_poor), 100), len(topo_poor),
        min(len(topo_gap), 100), len(topo_gap),
        min(len(topo_good), 100), len(topo_good),
        min(len(geom_only_pool), 50), len(geom_only_pool),
    )

    return sampled


# ---------------------------------------------------------------------------
# Main processing
# ---------------------------------------------------------------------------

CSV_COLUMNS = [
    "old_usd", "canon_usd", "mode", "bucket", "n_verts",
    "baseline_rmse", "baseline_bbox_max_abs",
    "candidate_rmse", "candidate_bbox_max_abs",
    "nn_mean_dist", "nn_close_pct", "nn_unique_ratio",
    "is_shuffle_heuristic", "wall_time_s",
]


def process_pair(
    entry: Dict,
    dataset_root: str,
) -> Optional[Dict]:
    """Process a single pair: run baseline + candidate A, return CSV row dict."""
    old_rel = entry["old_usd"]
    canon_rel = entry["canon_usd"]
    mode = entry["mode"]
    bucket = entry["bucket"]

    old_abs = os.path.join(dataset_root, old_rel)
    canon_abs = os.path.join(dataset_root, canon_rel)

    if not os.path.isfile(old_abs):
        logger.warning("Missing old asset: %s", old_abs)
        return None
    if not os.path.isfile(canon_abs):
        logger.warning("Missing canon asset: %s", canon_abs)
        return None

    t0 = time.perf_counter()

    # Load vertices
    pts_canon = extract_instance_space_vertices(canon_abs)
    pts_old = extract_instance_space_vertices(old_abs)

    if pts_canon is None or pts_old is None:
        logger.warning(
            "Could not extract vertices: old=%s canon=%s",
            old_rel, canon_rel,
        )
        return None

    n_verts = len(pts_canon)

    # --- Baseline: procrustes_full (for topo_filesize) or identity (for geom_only) ---
    if mode == "geom_only":
        V_baseline = np.eye(4, dtype=np.float64)
    else:
        if len(pts_canon) != len(pts_old):
            logger.warning(
                "Vertex count mismatch (%d vs %d): old=%s canon=%s",
                len(pts_canon), len(pts_old), old_rel, canon_rel,
            )
            return None
        try:
            V_baseline = procrustes_full(pts_canon, pts_old)
        except Exception as exc:
            logger.warning("Baseline procrustes failed: %s (%s)", old_rel, exc)
            return None

    baseline_rmse, baseline_bbox_max_abs = eval_v_matrix(
        V_baseline, pts_canon, pts_old,
    )

    # --- Candidate A: bbox normalize + NN reorder + Procrustes + denorm ---
    try:
        V_candidate, nn_mean_dist, nn_close_pct, nn_unique_ratio = (
            candidate_a_v_matrix(pts_canon, pts_old)
        )
    except Exception as exc:
        logger.warning("Candidate A failed: %s (%s)", old_rel, exc)
        return None

    candidate_rmse, candidate_bbox_max_abs = eval_v_matrix(
        V_candidate, pts_canon, pts_old,
    )

    wall_time = time.perf_counter() - t0

    # Shuffle heuristic
    is_shuffle = (
        baseline_bbox_max_abs > 0.15 and nn_close_pct > 95.0
    )

    return {
        "old_usd": old_rel,
        "canon_usd": canon_rel,
        "mode": mode,
        "bucket": bucket,
        "n_verts": n_verts,
        "baseline_rmse": f"{baseline_rmse:.8g}",
        "baseline_bbox_max_abs": f"{baseline_bbox_max_abs:.8g}",
        "candidate_rmse": f"{candidate_rmse:.8g}",
        "candidate_bbox_max_abs": f"{candidate_bbox_max_abs:.8g}",
        "nn_mean_dist": f"{nn_mean_dist:.8g}",
        "nn_close_pct": f"{nn_close_pct:.4f}",
        "nn_unique_ratio": f"{nn_unique_ratio:.6f}",
        "is_shuffle_heuristic": str(is_shuffle),
        "wall_time_s": f"{wall_time:.4f}",
    }


def main():
    parser = argparse.ArgumentParser(
        description="Phase 2 spike: NN correspondence recovery for topo_filesize pairs",
    )
    parser.add_argument(
        "--cert-jsonl",
        required=True,
        help="Path to pair_certificates.jsonl from cert pipeline",
    )
    parser.add_argument(
        "--dataset-root",
        required=True,
        help="Root directory containing asset USD files "
             "(e.g. GRScenes-test0-rebuilt-normalize-prededup)",
    )
    parser.add_argument(
        "--out-csv",
        default="check_reports/test0_rebuilt_dedup/topo_nn_spike_results.csv",
        help="Output CSV path (default: %(default)s)",
    )
    parser.add_argument(
        "--seed", type=int, default=42,
        help="Random seed for sampling (default: 42)",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    # Validate inputs
    if not os.path.isfile(args.cert_jsonl):
        logger.error("Cert JSONL not found: %s", args.cert_jsonl)
        sys.exit(1)
    if not os.path.isdir(args.dataset_root):
        logger.error("Dataset root not found: %s", args.dataset_root)
        sys.exit(1)

    # Ensure output directory exists
    out_dir = os.path.dirname(args.out_csv)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    # Sample pairs
    pairs = load_and_sample_pairs(args.cert_jsonl, seed=args.seed)
    logger.info("Total sampled pairs: %d", len(pairs))

    if not pairs:
        logger.error("No pairs sampled — check cert JSONL contents")
        sys.exit(1)

    # Process pairs
    results = []
    n_skip = 0
    for i, entry in enumerate(pairs):
        row = process_pair(entry, args.dataset_root)
        if row is not None:
            results.append(row)
        else:
            n_skip += 1

        if (i + 1) % 50 == 0 or i == len(pairs) - 1:
            logger.info(
                "Progress: %d/%d processed, %d skipped",
                i + 1, len(pairs), n_skip,
            )

    # Write CSV
    with open(args.out_csv, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS)
        writer.writeheader()
        writer.writerows(results)

    logger.info("Wrote %d rows to %s (skipped %d)", len(results), args.out_csv, n_skip)

    # Print summary
    bucket_counts = {}
    for r in results:
        b = r["bucket"]
        bucket_counts[b] = bucket_counts.get(b, 0) + 1
    logger.info("Bucket counts: %s", bucket_counts)


if __name__ == "__main__":
    main()
