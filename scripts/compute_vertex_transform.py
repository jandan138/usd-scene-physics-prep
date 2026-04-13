#!/usr/bin/env python3
"""Compute the V matrix that maps canonical asset vertices to old asset vertices.

Used in dedup compensation to correct for vertex-space differences between
canonical and replaced assets. The full compensation formula is:

    M_new_local = M_canon_internal^-1 * V * M_old_internal * M_old_local

where V captures the vertex-space mapping: p_mesh_old = p_mesh_canon * V
(row-vector convention, consistent with USD).

Three dedup modes produce different V matrices:
  - geom_only:        vertices identical => V = Identity
  - topo_filesize:    same vertex order, different coords => Procrustes SVD
                      (with shuffle-gated NN fallback when vertex order differs)
  - shape_invariant:  bbox-normalized matching => denorm + Procrustes/ICP
  - transitive:       BFS through intermediaries, accumulate V along path

Public API:
  extract_instance_space_vertices(asset_usd) -> Optional[np.ndarray]
  procrustes_full(pts_canon, pts_old) -> np.ndarray(4x4)
  compute_V_shape_invariant(pts_canon, pts_old) -> np.ndarray(4x4)
  icp_in_normalized_space(c_norm, o_norm, max_iter) -> (R_3x3, t_3, rmse)
  build_pair_certificate(old_usd, canonical_usd, mode, ...) -> dict
  build_mode_index(dedup_dir) -> dict
  determine_compensation_mode(old_path, canonical_path, mode_index) -> str
  find_transitive_V(old_usd, canonical_usd, group_members, mode_index) -> np.ndarray
  compute_V_for_pair(old_usd, canonical_usd, mode, ...) -> Gf.Matrix4d
  numpy_to_gf_matrix4d(m) -> Gf.Matrix4d
"""

from __future__ import annotations

import json
import logging
import os
from collections import defaultdict, deque
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# Lazy pxr import — module can be imported without pxr for testing
_pxr_loaded = False
Gf = None
Usd = None
UsdGeom = None


def _ensure_pxr():
    """Lazily import pxr modules."""
    global _pxr_loaded, Gf, Usd, UsdGeom
    if _pxr_loaded:
        return
    from pxr import Gf as _Gf, Usd as _Usd, UsdGeom as _UsdGeom

    Gf = _Gf
    Usd = _Usd
    UsdGeom = _UsdGeom
    _pxr_loaded = True


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Max vertices for Procrustes subsampling
MAX_VERTICES_FOR_ALIGNMENT = 5000
# Threshold for considering alignment successful
ALIGNMENT_THRESHOLD = 0.05
BBOX_GATED_ALLOWED_MODES = ("geom_only", "topo_filesize", "shape_invariant")
_TRANSITIVE_WITNESS_MODE_RISK = {
    "geom_only": 0,
    "topo_filesize": 1,
    "shape_invariant": 2,
}

# NN fallback constants — see docs/operations/topo_vertex_shuffle_correspondence_fix_plan.md
# Baseline bbox_delta above this triggers NN attempt (spike data: shuffle pairs all > 0.15)
_NN_FALLBACK_BASELINE_THRESHOLD = 0.15
# NN close_pct must exceed this to confirm genuine shuffle (spike: all shuffle pairs > 95%)
_NN_FALLBACK_CLOSE_PCT_THRESHOLD = 95.0
# Candidate bbox_delta must be below this to accept NN result (spike: all shuffle < 0.004)
_NN_FALLBACK_ACCEPT_THRESHOLD = 0.01

# NN Tier2 constants — relaxed gate for non-shuffle pairs with good NN quality
_NN_TIER2_BBOX_THRESHOLD = 0.05
_NN_TIER2_UNIQUE_THRESHOLD = 0.5
_NN_TIER2_MEAN_DIST_THRESHOLD = 0.02
_NN_TIER2_CLOSE_PCT_FLOOR = 10.0

# v_source tracing — populated by _topo_same_vtx_with_nn_fallback, read by build_pair_certificate
_last_v_source: Dict[str, Any] = {}


def _zero_bbox_delta() -> Dict[str, object]:
    return {
        "min": [0.0, 0.0, 0.0],
        "max": [0.0, 0.0, 0.0],
        "max_abs": 0.0,
    }


def _base_pair_certificate(
    old_usd: str,
    canonical_usd: str,
    mode: str,
    policy: str,
) -> Dict[str, Any]:
    return {
        "old_asset": old_usd,
        "canonical_asset": canonical_usd,
        "mode": mode,
        "policy": policy,
        "eligible": False,
        "reject_reason": None,
        "bbox_delta": _zero_bbox_delta(),
        "footprint_extent_delta": 0.0,
        "footprint_axis_delta": 0.0,
        "centroid_delta": 0.0,
        "vertex_rmse": None,
        "rmse_available": False,
        "rmse_unavailable_reason": None,
        "alternate_proof_kind": None,
        "alternate_proof_passed": False,
        "proof_source": None,
    }


# ---------------------------------------------------------------------------
# USD vertex extraction
# ---------------------------------------------------------------------------


def _get_local_matrix(prim):
    """Return local transform matrix of a prim as Gf.Matrix4d."""
    _ensure_pxr()
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return Gf.Matrix4d(1.0)
    res = xf.GetLocalTransformation(Usd.TimeCode.Default())
    if isinstance(res, tuple):
        return res[0]
    return res


def _get_chain_transform(ancestor, descendant):
    """Accumulated local transform from ancestor's child down to descendant."""
    _ensure_pxr()
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


def extract_instance_space_vertices(asset_usd: str) -> Optional[np.ndarray]:
    """Extract all mesh vertices transformed to defaultPrim (instance) space.

    Opens the USD file, finds all meshes under the default prim, and
    transforms each mesh's local vertices into the default prim's coordinate
    space using the chain of prim-local transforms.

    Args:
        asset_usd: Path to the asset USD file.

    Returns:
        Nx3 numpy array of vertices in instance space, or None if no meshes found.
    """
    _ensure_pxr()
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
        mat_np = np.array(
            [[chain_mat[i][j] for j in range(4)] for i in range(4)],
            dtype=np.float64,
        )

        pts_np = np.array(pts, dtype=np.float64)
        ones = np.ones((pts_np.shape[0], 1), dtype=np.float64)
        pts_h = np.hstack([pts_np, ones])  # Nx4
        # Row-vector convention: p_instance = p_local * M_chain
        pts_instance = pts_h @ mat_np
        all_pts.append(pts_instance[:, :3])

    if not all_pts:
        return None
    return np.vstack(all_pts)


# ---------------------------------------------------------------------------
# Procrustes alignment
# ---------------------------------------------------------------------------


def _try_single_procrustes(
    a_centered: np.ndarray,
    b_centered: np.ndarray,
    allow_reflection: bool,
    allow_scale: bool,
) -> Tuple[np.ndarray, float, float]:
    """Run one Procrustes variant.

    Args:
        a_centered: Centered canonical points (Nx3).
        b_centered: Centered old points (Nx3).
        allow_reflection: If True, allow det(R) < 0.
        allow_scale: If True, compute and apply uniform scale.

    Returns:
        (R_3x3, scale_factor, rmse) where R is 3x3 rotation/reflection,
        scale_factor is the uniform scale, and rmse is the residual.
    """
    norm_a = np.linalg.norm(a_centered)
    norm_b = np.linalg.norm(b_centered)
    if norm_a < 1e-10 or norm_b < 1e-10:
        return np.eye(3), 1.0, float("inf")

    # Optionally normalize for scale
    if allow_scale:
        a_work = a_centered / norm_a
        b_work = b_centered / norm_b
    else:
        a_work = a_centered
        b_work = b_centered

    # Subsample for SVD if too many vertices
    n = len(a_work)
    if n > MAX_VERTICES_FOR_ALIGNMENT:
        rng = np.random.RandomState(42)
        idx = rng.choice(n, MAX_VERTICES_FOR_ALIGNMENT, replace=False)
        a_sub = a_work[idx]
        b_sub = b_work[idx]
    else:
        a_sub = a_work
        b_sub = b_work

    # SVD on cross-covariance: H = A^T @ B
    # Standard Procrustes gives R_col = V @ U^T such that A @ R_col^T ≈ B.
    # For row-vector convention (A @ R_row ≈ B), R_row = R_col^T = U @ V^T.
    H = a_sub.T @ b_sub
    U, S, Vt = np.linalg.svd(H)

    # R_row = U @ Vt; check det for reflection
    R_row = U @ Vt
    det = np.linalg.det(R_row)

    if not allow_reflection and det < 0:
        # Force proper rotation by flipping sign of last column of U
        U_copy = U.copy()
        U_copy[:, -1] *= -1
        R_row = U_copy @ Vt
    elif allow_reflection and det > 0:
        # If we want reflection but SVD gave rotation, try flipping
        # to see if reflection gives lower RMSE (handled by caller
        # trying both combos)
        pass

    R = R_row

    # Compute scale
    if allow_scale:
        scale = norm_b / norm_a
    else:
        scale = 1.0

    # Evaluate on full set (row-vector): residual = || a_centered @ (s*R) - b_centered ||
    a_aligned = a_centered @ (scale * R)
    diffs = np.linalg.norm(a_aligned - b_centered, axis=1)
    rmse = float(np.sqrt(np.mean(diffs**2)))

    return R, scale, rmse


def procrustes_full(pts_canon: np.ndarray, pts_old: np.ndarray) -> np.ndarray:
    """Enhanced Procrustes: try 4 combinations, pick lowest RMSE.

    Tries rotation-only, rotation+reflection, rotation+scale, and
    rotation+reflection+scale. Returns the 4x4 V matrix (row-vector
    convention) that best maps canonical vertices to old vertices:

        p_old = p_canon @ V  (approximately)

    The V matrix encodes: translate to origin, apply scale*rotation, translate
    to old centroid.

    Args:
        pts_canon: Nx3 canonical asset vertices.
        pts_old: Nx3 old asset vertices (same vertex count required).

    Returns:
        4x4 numpy array V such that p_old ~ p_canon @ V (row-vector).

    Raises:
        ValueError: If point sets have different lengths.
    """
    if len(pts_canon) != len(pts_old):
        raise ValueError(
            f"Vertex count mismatch: canon={len(pts_canon)}, old={len(pts_old)}"
        )

    # Center both
    c_canon = pts_canon.mean(axis=0)
    c_old = pts_old.mean(axis=0)
    a = pts_canon - c_canon
    b = pts_old - c_old

    # Try 4 combinations: (allow_reflection, allow_scale)
    combos = [
        (False, False),  # rotation only
        (True, False),  # rotation + reflection
        (False, True),  # rotation + scale
        (True, True),  # rotation + reflection + scale
    ]

    best_R = np.eye(3)
    best_scale = 1.0
    best_rmse = float("inf")

    for allow_refl, allow_scl in combos:
        R, s, rmse = _try_single_procrustes(a, b, allow_refl, allow_scl)
        if rmse < best_rmse:
            best_R = R
            best_scale = s
            best_rmse = rmse

    # Build 4x4 V matrix (row-vector convention)
    # p_old = (p_canon - c_canon) @ (s*R) + c_old
    #       = p_canon @ (s*R) + (c_old - c_canon @ (s*R))
    sR = best_scale * best_R  # 3x3
    t = c_old - c_canon @ sR  # 1x3 translation

    V = np.eye(4, dtype=np.float64)
    V[:3, :3] = sR
    V[3, :3] = t

    return V


# ---------------------------------------------------------------------------
# Shuffle-gated NN fallback (shared helper for topo_filesize same-vtx-count)
# ---------------------------------------------------------------------------


def _bbox_delta_max_abs(
    V_np: np.ndarray, pts_canon: np.ndarray, pts_old: np.ndarray
) -> float:
    """Compute max absolute bbox delta after applying V to canonical points."""
    canon_h = np.hstack([pts_canon, np.ones((len(pts_canon), 1))])
    canon_transformed = (canon_h @ V_np)[:, :3]
    delta_min = np.abs(pts_old.min(axis=0) - canon_transformed.min(axis=0))
    delta_max = np.abs(pts_old.max(axis=0) - canon_transformed.max(axis=0))
    return float(max(delta_min.max(), delta_max.max()))


def _nn_procrustes_core(
    pts_canon: np.ndarray,
    pts_old: np.ndarray,
) -> Tuple[np.ndarray, float, float, float]:
    """NN reorder + Procrustes in bbox-normalized space, denormalized to instance space.

    Returns:
        (V_4x4, nn_close_pct, nn_unique_ratio, nn_mean_dist_norm)
    """
    from scipy.spatial import cKDTree

    c_norm, c_min, c_ext = _normalize_to_unit_bbox(pts_canon)
    o_norm, o_min, o_ext = _normalize_to_unit_bbox(pts_old)

    tree = cKDTree(o_norm)
    dists, indices = tree.query(c_norm)
    o_norm_reordered = o_norm[indices]

    nn_close_pct = float(np.sum(dists < 0.01) / len(dists) * 100.0)
    nn_unique_ratio = float(len(np.unique(indices)) / len(indices))
    nn_mean_dist_norm = float(np.mean(dists))

    # Procrustes in normalized space on reordered points
    c_c = c_norm.mean(axis=0)
    c_o = o_norm_reordered.mean(axis=0)
    a = c_norm - c_c
    b = o_norm_reordered - c_o

    best_R, best_s, best_rmse = np.eye(3), 1.0, float("inf")
    for allow_refl, allow_scl in [
        (False, False),
        (True, False),
        (False, True),
        (True, True),
    ]:
        R, s, rmse = _try_single_procrustes(a, b, allow_refl, allow_scl)
        if rmse < best_rmse:
            best_R, best_s, best_rmse = R, s, rmse

    R_norm = best_s * best_R
    t_norm = c_o - c_c @ R_norm

    # Denormalize (same formula as compute_V_shape_invariant)
    sR = R_norm * (o_ext / c_ext)
    t = (-c_min / c_ext) @ R_norm * o_ext + t_norm * o_ext + o_min

    V = np.eye(4, dtype=np.float64)
    V[:3, :3] = sR
    V[3, :3] = t

    return V, nn_close_pct, nn_unique_ratio, nn_mean_dist_norm


def _nn_procrustes_in_normalized_space(
    pts_canon: np.ndarray,
    pts_old: np.ndarray,
) -> Tuple[np.ndarray, float, float]:
    """NN reorder + Procrustes (backward-compatible wrapper).

    Returns:
        (V_4x4, nn_close_pct, nn_unique_ratio)
    """
    V, nn_close_pct, nn_unique_ratio, _nn_mean = _nn_procrustes_core(
        pts_canon,
        pts_old,
    )
    return V, nn_close_pct, nn_unique_ratio


def _nn_procrustes_with_stats(
    pts_canon: np.ndarray,
    pts_old: np.ndarray,
) -> Tuple[np.ndarray, float, float, float]:
    """NN reorder + Procrustes returning all stats including nn_mean_dist_norm.

    Returns:
        (V_4x4, nn_close_pct, nn_unique_ratio, nn_mean_dist_norm)
    """
    return _nn_procrustes_core(pts_canon, pts_old)


def _topo_same_vtx_with_nn_fallback(
    pts_canon: np.ndarray,
    pts_old: np.ndarray,
) -> np.ndarray:
    """Procrustes for topo_filesize same-vertex-count, with shuffle-gated NN fallback.

    1. Run baseline procrustes_full (index-aligned).
    2. If baseline bbox_delta > threshold AND env kill-switch not set:
       a. Run NN reorder + Procrustes in normalized space.
       b. If nn_close_pct > 95% (confirms genuine shuffle) AND
          candidate bbox_delta <= 0.01: accept candidate.
       c. Otherwise: keep baseline.
    3. Return V_4x4.
    """
    V_baseline = procrustes_full(pts_canon, pts_old)

    # Kill-switch: DEDUP_DISABLE_NN_FALLBACK=1 → always use baseline
    if os.environ.get("DEDUP_DISABLE_NN_FALLBACK") == "1":
        _last_v_source.clear()
        _last_v_source["v_source"] = "baseline"
        return V_baseline

    baseline_bbox = _bbox_delta_max_abs(V_baseline, pts_canon, pts_old)
    if baseline_bbox <= _NN_FALLBACK_BASELINE_THRESHOLD:
        _last_v_source.clear()
        _last_v_source["v_source"] = "baseline"
        return V_baseline

    # Baseline is bad — attempt NN fallback
    try:
        V_nn, nn_close_pct, nn_unique_ratio, nn_mean_dist_norm = (
            _nn_procrustes_with_stats(pts_canon, pts_old)
        )
    except Exception as exc:
        logger.warning("NN fallback failed (%s), keeping baseline V", exc)
        _last_v_source.clear()
        _last_v_source["v_source"] = "baseline"
        return V_baseline

    candidate_bbox = _bbox_delta_max_abs(V_nn, pts_canon, pts_old)

    # Gate 1 (Tier1): NN must confirm genuine shuffle (nn_close_pct > 95%)
    if nn_close_pct > _NN_FALLBACK_CLOSE_PCT_THRESHOLD:
        # Gate 2: candidate must actually be better
        if candidate_bbox > _NN_FALLBACK_ACCEPT_THRESHOLD:
            logger.debug(
                "NN fallback: candidate_bbox=%.6f > %.4f, keeping baseline",
                candidate_bbox,
                _NN_FALLBACK_ACCEPT_THRESHOLD,
            )
            _last_v_source.clear()
            _last_v_source["v_source"] = "baseline"
            return V_baseline

        logger.info(
            "NN fallback accepted: baseline_bbox=%.4f -> candidate_bbox=%.6f "
            "(nn_close_pct=%.1f%%, nn_unique_ratio=%.3f)",
            baseline_bbox,
            candidate_bbox,
            nn_close_pct,
            nn_unique_ratio,
        )
        _last_v_source.clear()
        _last_v_source["v_source"] = "tier1_shuffle"
        return V_nn

    # Gate 1B: Tier2 — relaxed NN gate for non-shuffle pairs
    if os.environ.get("DEDUP_DISABLE_NN_TIER2") == "1":
        _last_v_source.clear()
        _last_v_source["v_source"] = "baseline"
        return V_baseline

    # Allow env override for bbox threshold
    tier2_bbox = _NN_TIER2_BBOX_THRESHOLD
    _bbox_override = os.environ.get("DEDUP_NN_TIER2_BBOX")
    if _bbox_override:
        try:
            tier2_bbox = float(_bbox_override)
        except ValueError:
            pass

    if (
        nn_close_pct >= _NN_TIER2_CLOSE_PCT_FLOOR
        and candidate_bbox <= tier2_bbox
        and nn_mean_dist_norm <= _NN_TIER2_MEAN_DIST_THRESHOLD
        and nn_unique_ratio >= _NN_TIER2_UNIQUE_THRESHOLD
    ):
        logger.info(
            "NN Tier2 accepted: baseline_bbox=%.4f -> candidate_bbox=%.6f "
            "(nn_close_pct=%.1f%%, nn_unique_ratio=%.3f, nn_mean_dist_norm=%.6f)",
            baseline_bbox,
            candidate_bbox,
            nn_close_pct,
            nn_unique_ratio,
            nn_mean_dist_norm,
        )
        _last_v_source.clear()
        _last_v_source["v_source"] = "tier2_nn"
        _last_v_source["tier2_nn_bbox"] = candidate_bbox
        _last_v_source["tier2_nn_unique_ratio"] = nn_unique_ratio
        _last_v_source["tier2_nn_mean_dist_norm"] = nn_mean_dist_norm
        return V_nn

    logger.debug(
        "NN Tier2 rejected: candidate_bbox=%.6f, nn_close_pct=%.1f%%, "
        "nn_unique_ratio=%.3f, nn_mean_dist_norm=%.6f",
        candidate_bbox,
        nn_close_pct,
        nn_unique_ratio,
        nn_mean_dist_norm,
    )
    _last_v_source.clear()
    _last_v_source["v_source"] = "baseline"
    return V_baseline


# ---------------------------------------------------------------------------
# Shape-invariant V computation
# ---------------------------------------------------------------------------


def _normalize_to_unit_bbox(pts: np.ndarray) -> Tuple[np.ndarray, np.ndarray, float]:
    """Normalize points to unit bounding box (uniform scale, preserving aspect).

    Returns:
        (normalized_pts, bbox_min, max_extent)
    """
    bbox_min = pts.min(axis=0)
    bbox_max = pts.max(axis=0)
    extent = bbox_max - bbox_min
    max_extent = max(float(extent.max()), 1e-10)
    normalized = (pts - bbox_min) / max_extent
    return normalized, bbox_min, max_extent


def icp_in_normalized_space(
    c_norm: np.ndarray,
    o_norm: np.ndarray,
    max_iter: int = 30,
) -> Tuple[np.ndarray, np.ndarray, float]:
    """ICP alignment in normalized [0,1] space using scipy KDTree.

    Finds the rotation R and translation t that minimizes:
        || c_norm @ R + t - o_norm_matched ||

    where o_norm_matched are the closest points in o_norm for each point in
    the transformed c_norm.

    Args:
        c_norm: Nx3 normalized canonical vertices.
        o_norm: Mx3 normalized old vertices.
        max_iter: Maximum ICP iterations.

    Returns:
        (R_3x3, t_3, rmse) — rotation, translation, and final RMSE.
    """
    from scipy.spatial import KDTree

    # Subsample if very large
    if len(c_norm) > MAX_VERTICES_FOR_ALIGNMENT:
        rng = np.random.RandomState(42)
        idx = rng.choice(len(c_norm), MAX_VERTICES_FOR_ALIGNMENT, replace=False)
        src = c_norm[idx].copy()
    else:
        src = c_norm.copy()

    tree = KDTree(o_norm)
    R_acc = np.eye(3, dtype=np.float64)
    t_acc = np.zeros(3, dtype=np.float64)
    prev_rmse = float("inf")

    for _ in range(max_iter):
        # Transform source
        transformed = src @ R_acc + t_acc

        # Find closest points
        dists, indices = tree.query(transformed)
        matched = o_norm[indices]

        # Compute Procrustes between transformed src and matched
        c_src = transformed.mean(axis=0)
        c_matched = matched.mean(axis=0)
        a = transformed - c_src
        b = matched - c_matched

        H = a.T @ b
        U, S, Vt = np.linalg.svd(H)
        # Row-vector convention: R_row = U @ Vt
        R_step = U @ Vt
        # Ensure proper rotation
        if np.linalg.det(R_step) < 0:
            U_copy = U.copy()
            U_copy[:, -1] *= -1
            R_step = U_copy @ Vt

        t_step = c_matched - c_src @ R_step

        # Accumulate: new_transform = old_transform then R_step + t_step
        # p' = (p @ R_acc + t_acc) @ R_step + t_step
        #     = p @ (R_acc @ R_step) + (t_acc @ R_step + t_step)
        R_acc = R_acc @ R_step
        t_acc = t_acc @ R_step + t_step

        # Check convergence
        rmse = float(np.sqrt(np.mean(dists**2)))
        if abs(prev_rmse - rmse) < 1e-8:
            break
        prev_rmse = rmse

    # Final RMSE
    final_transformed = src @ R_acc + t_acc
    dists_final, _ = tree.query(final_transformed)
    rmse_final = float(np.sqrt(np.mean(dists_final**2)))

    return R_acc, t_acc, rmse_final


def _flat_normal_axis(
    pts_norm: np.ndarray, var_threshold: float = 0.01
) -> Optional[int]:
    """Return the normal axis index if pts_norm is a flat (planar) asset.

    Computes per-axis variance of the normalized vertices.  If the smallest
    variance is below *var_threshold*, the asset is considered flat and the
    axis with that minimum variance is its normal axis (0=X, 1=Y, 2=Z).
    Returns None for non-flat (volumetric) assets.

    Args:
        pts_norm: Nx3 vertices already normalized to unit bounding box.
        var_threshold: Variance cutoff for the "flat" test.

    Returns:
        Axis index {0, 1, 2} or None.
    """
    var = np.var(pts_norm, axis=0)
    min_ax = int(np.argmin(var))
    return min_ax if float(var[min_ax]) < var_threshold else None


def compute_V_shape_invariant(
    pts_canon: np.ndarray,
    pts_old: np.ndarray,
    aspect_ratio_threshold: float = 1.2,
    size_diff_threshold: float = 0.03,
) -> np.ndarray:
    """Compute V matrix for shape_invariant dedup pairs.

    Normalizes both point sets to unit bounding box, aligns in normalized
    space (Procrustes if same vertex count, ICP if different), then
    constructs V that chains: denorm_canon_inv -> R_norm -> denorm_old.

    The result maps canonical instance-space vertices to old instance-space
    vertices: p_old = p_canon @ V.

    Args:
        pts_canon: Nx3 canonical vertices in instance space.
        pts_old: Mx3 old vertices in instance space.
        aspect_ratio_threshold: Maximum allowed ratio between corresponding
            bbox extents.  Pairs exceeding this are rejected with RuntimeError.

    Returns:
        4x4 numpy array V (row-vector convention).

    Raises:
        RuntimeError: If bbox aspect ratios or absolute sizes differ beyond thresholds.
    """
    # Aspect ratio guard: reject pairs with mismatched bbox proportions
    ext_canon = np.sort(pts_canon.max(axis=0) - pts_canon.min(axis=0))[::-1]
    ext_old = np.sort(pts_old.max(axis=0) - pts_old.min(axis=0))[::-1]
    for i, (ec, eo) in enumerate(zip(ext_canon, ext_old)):
        if ec < 1e-6 and eo < 1e-6:
            continue
        if ec < 1e-6 or eo < 1e-6:
            raise RuntimeError(
                f"aspect ratio mismatch: dim[{i}] canon={ec:.4f} vs old={eo:.4f} (one degenerate)"
            )
        ratio = max(ec, eo) / min(ec, eo)
        if ratio >= aspect_ratio_threshold:
            raise RuntimeError(
                f"aspect ratio mismatch: dim[{i}] ratio={ratio:.2f} >= {aspect_ratio_threshold} "
                f"(canon={ec:.4f}, old={eo:.4f})"
            )

    # Absolute size difference guard: reject pairs where any dimension
    # differs by more than size_diff_threshold (relative to the larger).
    # Catches cases like ground strips of similar proportions but different
    # lengths (e.g. 6560 vs 7290) that would stick out of walls.
    for i, (ec, eo) in enumerate(zip(ext_canon, ext_old)):
        mx = max(ec, eo)
        if mx < 1e-6:
            continue
        rel_diff = abs(ec - eo) / mx
        if rel_diff >= size_diff_threshold:
            raise RuntimeError(
                f"size diff mismatch: dim[{i}] rel_diff={rel_diff:.3f} >= {size_diff_threshold} "
                f"(canon={ec:.4f}, old={eo:.4f}, diff={abs(ec - eo):.4f})"
            )

    # Normalize both to unit bbox
    c_norm, c_min, c_ext = _normalize_to_unit_bbox(pts_canon)
    o_norm, o_min, o_ext = _normalize_to_unit_bbox(pts_old)

    # Plane-normal mismatch guard:
    # If both assets are flat (planar) but their normal axes differ, Procrustes
    # would encode a spurious ~90° rotation into V.  For these cases we skip the
    # rotational alignment and keep only the scale term (R_norm = Identity).
    c_ax = _flat_normal_axis(c_norm)
    o_ax = _flat_normal_axis(o_norm)

    if c_ax is not None and o_ax is not None and c_ax != o_ax:
        logger.warning(
            "compute_V_shape_invariant: plane normal mismatch "
            "(canon_axis=%d, old_axis=%d) — setting R_norm=Identity, t_norm=0 "
            "(scale-only V).",
            c_ax,
            o_ax,
        )
        R_norm = np.eye(3, dtype=np.float64)
        t_norm = np.zeros(3, dtype=np.float64)
    # Align in normalized space
    elif len(pts_canon) == len(pts_old):
        # Same vertex count: use Procrustes in normalized space
        c_c = c_norm.mean(axis=0)
        c_o = o_norm.mean(axis=0)
        a = c_norm - c_c
        b = o_norm - c_o

        # Try all 4 Procrustes variants in normalized space
        best_R = np.eye(3)
        best_s = 1.0
        best_rmse = float("inf")
        for allow_refl, allow_scl in [
            (False, False),
            (True, False),
            (False, True),
            (True, True),
        ]:
            R, s, rmse = _try_single_procrustes(a, b, allow_refl, allow_scl)
            if rmse < best_rmse:
                best_R, best_s, best_rmse = R, s, rmse

        R_norm = best_s * best_R
        t_norm = c_o - c_c @ R_norm
    else:
        # Different vertex count: ICP fallback
        R_norm_3x3, t_norm_3, _ = icp_in_normalized_space(c_norm, o_norm)
        R_norm = R_norm_3x3
        t_norm = t_norm_3

    # Build V that chains: canon_instance -> canon_norm -> aligned_norm -> old_instance
    #
    # Step 1: canon_instance -> canon_norm: p_cn = (p_ci - c_min) / c_ext
    # Step 2: canon_norm -> old_norm:       p_on = p_cn @ R_norm + t_norm
    # Step 3: old_norm -> old_instance:     p_oi = p_on * o_ext + o_min
    #
    # Combined (row-vector):
    #   p_oi = ((p_ci - c_min) / c_ext) @ R_norm + t_norm) * o_ext + o_min
    #        = p_ci @ (R_norm * o_ext / c_ext) + (-c_min/c_ext @ R_norm * o_ext + t_norm * o_ext + o_min)
    #
    # In 4x4: V = T(-c_min) * S(1/c_ext) * [R_norm, t_norm] * S(o_ext) * T(o_min)

    # Build each step as 4x4 and multiply
    # T_shift_canon: translate by -c_min, scale by 1/c_ext
    # Using direct formula for efficiency:
    sR = R_norm * (o_ext / c_ext)  # 3x3
    t = (-c_min / c_ext) @ R_norm * o_ext + t_norm * o_ext + o_min  # 1x3

    V = np.eye(4, dtype=np.float64)
    V[:3, :3] = sR
    V[3, :3] = t

    return V


# ---------------------------------------------------------------------------
# Mode index and dispatch
# ---------------------------------------------------------------------------


def _uid_from_path(p: str) -> str:
    """Extract uid from path like .../category/uid/usd/uid.usd"""
    return os.path.basename(os.path.dirname(os.path.dirname(p)))


def build_mode_index(dedup_dir: str) -> Dict[Tuple[str, str], str]:
    """Build asset-pair-to-mode lookup from dedup report directories.

    Scans geom_only/, topo_filesize/, shape_invariant/ subdirectories for
    per-category report JSONs. For each duplicate group, creates entries
    mapping (canonical_uid, member_uid) to the dedup mode.

    Priority: geom_only > topo_filesize > shape_invariant (higher priority
    modes overwrite lower ones).

    Args:
        dedup_dir: Root directory containing mode subdirectories.

    Returns:
        Dict mapping (canonical_uid, old_uid) -> mode_name.
    """
    index: Dict[Tuple[str, str], str] = {}

    # Process in reverse priority order so higher priority overwrites
    for mode in ["shape_invariant", "topo_filesize", "geom_only"]:
        mode_dir = os.path.join(dedup_dir, mode)
        if not os.path.isdir(mode_dir):
            continue
        for cat in os.listdir(mode_dir):
            report = os.path.join(mode_dir, cat, f"{cat}_asset_mesh_dedup_{mode}.json")
            if not os.path.isfile(report):
                continue
            with open(report) as f:
                data = json.load(f)
            for dup in data.get("duplicates", []):
                paths = dup.get("usd_paths", [])
                if len(paths) < 2:
                    continue
                uids = [_uid_from_path(p) for p in paths]
                for i in range(len(uids)):
                    for j in range(i + 1, len(uids)):
                        index[(uids[i], uids[j])] = mode
                        index[(uids[j], uids[i])] = mode

    return index


def determine_compensation_mode(
    old_path: str,
    canonical_path: str,
    mode_index: Dict[Tuple[str, str], str],
) -> str:
    """Determine the best compensation mode for a pair.

    Extracts UIDs from paths and looks up in mode_index.

    Args:
        old_path: Path to old asset USD.
        canonical_path: Path to canonical asset USD.
        mode_index: Dict from build_mode_index().

    Returns:
        Mode string: "geom_only", "topo_filesize", "shape_invariant",
        or "transitive" if no direct match found.
    """
    old_uid = _uid_from_path(old_path)
    canonical_uid = _uid_from_path(canonical_path)

    mode = mode_index.get((canonical_uid, old_uid))
    if mode is not None:
        return mode
    return "transitive"


def _build_group_graph(
    group_members: List[str],
    mode_index: Dict[Tuple[str, str], str],
) -> Dict[str, List[Tuple[str, str, str]]]:
    """Build adjacency graph of UIDs connected by direct dedup modes.

    Returns dict: uid -> [(neighbor_uid, mode, neighbor_usd_path), ...]
    """
    uid_to_path = {}
    for p in group_members:
        uid = _uid_from_path(p)
        uid_to_path[uid] = p

    uids = list(uid_to_path.keys())
    graph: Dict[str, List[Tuple[str, str, str]]] = defaultdict(list)

    for i, u1 in enumerate(uids):
        for u2 in uids[i + 1 :]:
            mode = mode_index.get((u1, u2))
            if mode is not None:
                graph[u1].append((u2, mode, uid_to_path[u2]))
                graph[u2].append((u1, mode, uid_to_path[u1]))

    return dict(graph)


def _sorted_graph_neighbors(
    graph: Dict[str, List[Tuple[str, str, str]]],
    uid: str,
) -> List[Tuple[str, str, str]]:
    return sorted(
        graph.get(uid, []),
        key=lambda item: (
            _TRANSITIVE_WITNESS_MODE_RISK.get(item[1], 99),
            item[0],
            item[1],
            item[2],
        ),
    )


def _path_witness_key(path: List[Tuple[str, str, str, str, str]]) -> Tuple[Any, ...]:
    mode_risks = tuple(_TRANSITIVE_WITNESS_MODE_RISK.get(step[2], 99) for step in path)
    mode_names = tuple(step[2] for step in path)
    uid_chain = tuple([path[0][0]] + [step[1] for step in path]) if path else tuple()
    return (sum(mode_risks), mode_risks, mode_names, uid_chain)


def _witness_metadata_from_path(
    path: List[Tuple[str, str, str, str, str]],
) -> Dict[str, Any]:
    uids = [path[0][0]] if path else []
    modes = []
    usd_paths = [path[0][3]] if path else []
    for _, to_uid, mode, _, to_usd in path:
        uids.append(to_uid)
        modes.append(mode)
        usd_paths.append(to_usd)
    return {
        "uids": uids,
        "modes": modes,
        "usd_paths": usd_paths,
        "length": len(path),
    }


def _compute_edge_V(
    from_uid: str, to_uid: str, mode: str, from_usd: str, to_usd: str
) -> np.ndarray:
    if mode == "geom_only":
        return np.eye(4, dtype=np.float64)

    pts_from = extract_instance_space_vertices(from_usd)
    pts_to = extract_instance_space_vertices(to_usd)
    if pts_from is None or pts_to is None:
        raise RuntimeError(f"Could not extract vertices for edge {from_uid}->{to_uid}")

    if mode == "topo_filesize":
        if len(pts_from) != len(pts_to):
            return compute_V_shape_invariant(pts_from, pts_to)
        return _topo_same_vtx_with_nn_fallback(pts_from, pts_to)
    if mode == "shape_invariant":
        return compute_V_shape_invariant(pts_from, pts_to)
    if len(pts_from) == len(pts_to):
        return procrustes_full(pts_from, pts_to)
    return compute_V_shape_invariant(pts_from, pts_to)


def _find_transitive_witness_path(
    old_usd: str,
    canonical_usd: str,
    group_members: List[str],
    mode_index: Dict[Tuple[str, str], str],
) -> List[Tuple[str, str, str, str, str]]:
    canonical_uid = _uid_from_path(canonical_usd)
    old_uid = _uid_from_path(old_usd)

    if canonical_uid == old_uid:
        return []

    graph = _build_group_graph(group_members, mode_index)
    uid_to_path = {_uid_from_path(p): p for p in group_members}
    shortest_candidates: List[List[Tuple[str, str, str, str, str]]] = []
    visited_depth: Dict[str, int] = {canonical_uid: 0}
    queue: deque = deque()
    queue.append((canonical_uid, []))
    best_depth: Optional[int] = None

    while queue:
        cur_uid, path = queue.popleft()
        cur_depth = len(path)
        if best_depth is not None and cur_depth >= best_depth:
            continue
        for neighbor_uid, mode, neighbor_usd in _sorted_graph_neighbors(graph, cur_uid):
            next_depth = cur_depth + 1
            prev_depth = visited_depth.get(neighbor_uid)
            if prev_depth is not None and prev_depth < next_depth:
                continue
            new_path = path + [
                (
                    cur_uid,
                    neighbor_uid,
                    mode,
                    uid_to_path.get(cur_uid, ""),
                    neighbor_usd,
                )
            ]
            if neighbor_uid == old_uid:
                best_depth = (
                    next_depth if best_depth is None else min(best_depth, next_depth)
                )
                if next_depth == best_depth:
                    shortest_candidates.append(new_path)
                continue
            visited_depth[neighbor_uid] = next_depth
            queue.append((neighbor_uid, new_path))

    if not shortest_candidates:
        raise RuntimeError(
            f"No transitive path from canonical={canonical_uid} to old={old_uid} "
            f"in group of {len(group_members)} members"
        )

    return min(shortest_candidates, key=_path_witness_key)


def find_transitive_V(
    old_usd: str,
    canonical_usd: str,
    group_members: List[str],
    mode_index: Dict[Tuple[str, str], str],
    *,
    return_witness: bool = False,
) -> Any:
    """BFS through group members to find V via intermediate steps.

    Finds a path from canonical to old where each edge has a direct dedup
    mode match, then accumulates V along the path.

    Args:
        old_usd: Path to old asset USD.
        canonical_usd: Path to canonical asset USD.
        group_members: List of all USD paths in the dedup group.
        mode_index: Dict from build_mode_index().

    Returns:
        4x4 numpy array V (accumulated along BFS path).

    Raises:
        RuntimeError: If no path found between canonical and old.
    """
    path = _find_transitive_witness_path(
        old_usd=old_usd,
        canonical_usd=canonical_usd,
        group_members=group_members,
        mode_index=mode_index,
    )
    V_total = _accumulate_V_along_path(path)
    if return_witness:
        return V_total, _witness_metadata_from_path(path)
    return V_total


def _accumulate_V_along_path(
    path: List[Tuple[str, str, str, str, str]],
) -> np.ndarray:
    """Accumulate V matrices along a BFS path.

    Each step: (from_uid, to_uid, mode, from_usd, to_usd).
    V_total = V_step1 @ V_step2 @ ... (row-vector: p_final = p_start @ V_total)
    """
    V_total = np.eye(4, dtype=np.float64)

    for from_uid, to_uid, mode, from_usd, to_usd in path:
        V_step = _compute_edge_V(from_uid, to_uid, mode, from_usd, to_usd)
        V_total = V_total @ V_step

    return V_total


def _evaluate_certificate_with_V(
    cert: Dict[str, Any],
    *,
    canonical_pts: np.ndarray,
    old_pts: np.ndarray,
    V: np.ndarray,
    mode: str,
) -> Dict[str, Any]:
    canon_h = np.hstack([canonical_pts, np.ones((len(canonical_pts), 1))])
    canon_transformed = (canon_h @ V)[:, :3]

    bbox_old = {
        "min": old_pts.min(axis=0).tolist(),
        "max": old_pts.max(axis=0).tolist(),
    }
    bbox_new = {
        "min": canon_transformed.min(axis=0).tolist(),
        "max": canon_transformed.max(axis=0).tolist(),
    }
    delta_min = [abs(a - b) for a, b in zip(bbox_old["min"], bbox_new["min"])]
    delta_max = [abs(a - b) for a, b in zip(bbox_old["max"], bbox_new["max"])]
    max_abs = max(max(delta_min), max(delta_max))

    cert["bbox_delta"] = {"min": delta_min, "max": delta_max, "max_abs": max_abs}
    cert["bbox_delta_available"] = True
    centroid_old = np.mean(old_pts, axis=0)
    centroid_new = np.mean(canon_transformed, axis=0)
    cert["centroid_delta"] = float(np.linalg.norm(centroid_old - centroid_new))
    cert["footprint_extent_delta"] = 0.0
    cert["footprint_axis_delta"] = 0.0

    if len(old_pts) == len(canon_transformed):
        cert["vertex_rmse"] = float(
            np.sqrt(np.mean(np.sum((old_pts - canon_transformed) ** 2, axis=1)))
        )
        cert["rmse_available"] = True
        cert["rmse_unavailable_reason"] = None
    else:
        cert["vertex_rmse"] = None
        cert["rmse_available"] = False
        cert["rmse_unavailable_reason"] = "vertex_count_mismatch"

    CERT_BBOX_PRECHECK_THRESHOLD = 0.5
    if max_abs > CERT_BBOX_PRECHECK_THRESHOLD:
        cert["eligible"] = False
        cert["reject_reason"] = f"bbox_precheck_failed_{mode}"
        if mode == "transitive":
            cert["transitive_endpoint_verification_passed"] = False
        return cert

    cert["eligible"] = True
    cert["reject_reason"] = None
    if mode == "transitive":
        cert["transitive_endpoint_verification_passed"] = True
    return cert


# ---------------------------------------------------------------------------
# Pair certification
# ---------------------------------------------------------------------------


def build_pair_certificate(
    old_usd: str,
    canonical_usd: str,
    mode: str,
    *,
    policy: str = "bbox_primary_rmse_observe",
    allowed_modes: Sequence[str] = BBOX_GATED_ALLOWED_MODES,
) -> Dict[str, Any]:
    """Build a structured bbox-gated certificate for a candidate pair.

    All bbox-gated allowed modes (geom_only, topo_filesize, shape_invariant)
    are eligible for certification:

    - `geom_only`: report membership proves identical mesh-local geometry
    - `topo_filesize`: same vertex order, Procrustes SVD alignment
    - `shape_invariant`: bbox-normalized matching with denorm + Procrustes/ICP
    - RMSE remains secondary evidence for Policy A and hardens only when the
      chosen policy says so for a computable metric

    Rewrite-time compensation must still succeed later for the concrete scene
    prim, but the pair itself is admitted as proof-capable for its mode.
    """
    cert = _base_pair_certificate(
        old_usd=old_usd,
        canonical_usd=canonical_usd,
        mode=mode,
        policy=policy,
    )

    if not old_usd or not canonical_usd:
        cert["reject_reason"] = "missing_asset_path"
        cert["rmse_unavailable_reason"] = "missing_asset_path"
        return cert

    if not os.path.isfile(old_usd):
        cert["reject_reason"] = "old_asset_missing"
        cert["rmse_unavailable_reason"] = "old_asset_missing"
        return cert

    if not os.path.isfile(canonical_usd):
        cert["reject_reason"] = "canonical_asset_missing"
        cert["rmse_unavailable_reason"] = "canonical_asset_missing"
        return cert

    if mode not in set(allowed_modes):
        cert["reject_reason"] = f"mode_not_enabled_{mode}"
        cert["rmse_unavailable_reason"] = f"mode_not_enabled_{mode}"
        return cert

    old_pts = extract_instance_space_vertices(old_usd)
    canonical_pts = extract_instance_space_vertices(canonical_usd)
    if old_pts is None or canonical_pts is None:
        cert["reject_reason"] = "mesh_probe_failed"
        cert["rmse_unavailable_reason"] = "mesh_probe_failed"
        return cert

    if mode == "geom_only":
        # V=Identity, mesh identical -> bbox delta is 0 by definition
        cert["eligible"] = True
        cert["reject_reason"] = None
        cert["bbox_delta"] = _zero_bbox_delta()
        cert["bbox_delta_available"] = True
        cert["footprint_extent_delta"] = 0.0
        cert["footprint_axis_delta"] = 0.0
        cert["centroid_delta"] = 0.0
        cert["vertex_rmse"] = 0.0
        cert["rmse_available"] = True
        cert["rmse_unavailable_reason"] = None
        cert["v_source"] = "identity"
        _proof = "geom_only_exact_world_proof"
    else:
        try:
            V_np = _compute_numpy_V_for_pair(old_usd, canonical_usd, mode)
            cert["v_source"] = _last_v_source.get("v_source", "baseline")
            if cert["v_source"] == "tier2_nn":
                cert["tier2_nn_bbox"] = _last_v_source.get("tier2_nn_bbox")
                cert["tier2_nn_unique_ratio"] = _last_v_source.get(
                    "tier2_nn_unique_ratio"
                )
                cert["tier2_nn_mean_dist_norm"] = _last_v_source.get(
                    "tier2_nn_mean_dist_norm"
                )
            cert = _evaluate_certificate_with_V(
                cert,
                canonical_pts=canonical_pts,
                old_pts=old_pts,
                V=V_np,
                mode=mode,
            )
            if not cert["eligible"]:
                return cert

        except Exception as exc:
            # FAIL CLOSED — mark ALL metrics unavailable/None
            cert["eligible"] = False
            cert["reject_reason"] = f"v_computation_failed_{mode}"
            cert["bbox_delta"] = None
            cert["bbox_delta_available"] = False
            cert["centroid_delta"] = None
            cert["vertex_rmse"] = None
            cert["rmse_available"] = False
            cert["rmse_unavailable_reason"] = str(exc)[:200]
            cert["footprint_extent_delta"] = 0.0
            cert["footprint_axis_delta"] = 0.0
            return cert

        cert["eligible"] = True
        cert["reject_reason"] = None
        _proof = f"{mode}_bbox_gated_proof"

    cert["alternate_proof_kind"] = _proof
    cert["alternate_proof_passed"] = True
    cert["proof_source"] = _proof

    return cert


def build_transitive_pair_certificate(
    old_usd: str,
    canonical_usd: str,
    *,
    group_members: List[str],
    mode_index: Dict[Tuple[str, str], str],
    policy: str = "bbox_primary_rmse_observe",
) -> Dict[str, Any]:
    cert = _base_pair_certificate(
        old_usd=old_usd,
        canonical_usd=canonical_usd,
        mode="transitive",
        policy=policy,
    )

    if not old_usd or not canonical_usd:
        cert["reject_reason"] = "missing_asset_path"
        cert["rmse_unavailable_reason"] = "missing_asset_path"
        return cert

    if not os.path.isfile(old_usd):
        cert["reject_reason"] = "old_asset_missing"
        cert["rmse_unavailable_reason"] = "old_asset_missing"
        return cert

    if not os.path.isfile(canonical_usd):
        cert["reject_reason"] = "canonical_asset_missing"
        cert["rmse_unavailable_reason"] = "canonical_asset_missing"
        return cert

    old_pts = extract_instance_space_vertices(old_usd)
    canonical_pts = extract_instance_space_vertices(canonical_usd)
    if old_pts is None or canonical_pts is None:
        cert["reject_reason"] = "mesh_probe_failed"
        cert["rmse_unavailable_reason"] = "mesh_probe_failed"
        return cert

    try:
        V_np, witness = find_transitive_V(
            old_usd,
            canonical_usd,
            group_members,
            mode_index,
            return_witness=True,
        )
        cert["v_source"] = "transitive_witness"
        cert["transitive_witness_uids"] = witness.get("uids", [])
        cert["transitive_witness_modes"] = witness.get("modes", [])
        cert["transitive_witness_length"] = witness.get("length", 0)
        cert["transitive_endpoint_pair_directly_verified"] = True
        cert["transitive_endpoint_verification_kind"] = "composed_witness_V"
        cert["transitive_endpoint_verification_passed"] = False
        cert = _evaluate_certificate_with_V(
            cert,
            canonical_pts=canonical_pts,
            old_pts=old_pts,
            V=V_np,
            mode="transitive",
        )
        if not cert["eligible"]:
            return cert
    except Exception as exc:
        cert["eligible"] = False
        cert["reject_reason"] = "v_computation_failed_transitive"
        cert["bbox_delta"] = None
        cert["bbox_delta_available"] = False
        cert["centroid_delta"] = None
        cert["vertex_rmse"] = None
        cert["rmse_available"] = False
        cert["rmse_unavailable_reason"] = str(exc)[:200]
        cert["footprint_extent_delta"] = 0.0
        cert["footprint_axis_delta"] = 0.0
        cert.setdefault("transitive_endpoint_pair_directly_verified", False)
        cert.setdefault("transitive_endpoint_verification_kind", None)
        cert.setdefault("transitive_endpoint_verification_passed", False)
        return cert

    cert["alternate_proof_kind"] = "transitive_bbox_gated_proof"
    cert["alternate_proof_passed"] = True
    cert["proof_source"] = "transitive_bbox_gated_proof"
    return cert


# ---------------------------------------------------------------------------
# Top-level dispatch
# ---------------------------------------------------------------------------


def numpy_to_gf_matrix4d(m: np.ndarray):
    """Convert a 4x4 numpy array to pxr.Gf.Matrix4d.

    Args:
        m: 4x4 numpy array (row-vector convention).

    Returns:
        Gf.Matrix4d instance.
    """
    _ensure_pxr()
    gf_m = Gf.Matrix4d()
    for i in range(4):
        gf_m.SetRow(i, Gf.Vec4d(*m[i].tolist()))
    return gf_m


def _compute_numpy_V_for_pair(
    old_usd: str,
    canonical_usd: str,
    mode: str,
    mode_index: Optional[Dict[Tuple[str, str], str]] = None,
    group_members: Optional[List[str]] = None,
):
    """Compute V matrix for a dedup pair as a 4x4 numpy array.

    This helper keeps certificate evaluation free of pxr/Gf requirements.
    """

    if mode == "geom_only":
        return np.eye(4, dtype=np.float64)

    if mode == "transitive":
        if mode_index is None or group_members is None:
            raise ValueError(
                "mode_index and group_members required for transitive mode"
            )
        return find_transitive_V(old_usd, canonical_usd, group_members, mode_index)

    pts_canon = extract_instance_space_vertices(canonical_usd)
    pts_old = extract_instance_space_vertices(old_usd)
    if pts_canon is None or pts_old is None:
        raise RuntimeError(
            f"Could not extract vertices: canon={pts_canon is not None}, "
            f"old={pts_old is not None}"
        )

    if mode == "topo_filesize":
        if len(pts_canon) == len(pts_old):
            return _topo_same_vtx_with_nn_fallback(pts_canon, pts_old)
        return compute_V_shape_invariant(pts_canon, pts_old)

    if mode == "shape_invariant":
        return compute_V_shape_invariant(pts_canon, pts_old)

    raise ValueError(f"Unknown mode: {mode!r}")


def compute_V_for_pair(
    old_usd: str,
    canonical_usd: str,
    mode: str,
    mode_index: Optional[Dict[Tuple[str, str], str]] = None,
    group_members: Optional[List[str]] = None,
):
    """Compute V matrix for a dedup pair and return as Gf.Matrix4d.

    Top-level dispatch that selects the computation method based on mode:
      - "geom_only":        V = Identity
      - "topo_filesize":    V = Procrustes (or shape_invariant if vtx mismatch)
      - "shape_invariant":  V = bbox denorm + Procrustes/ICP
      - "transitive":       V = BFS accumulated path

    Args:
        old_usd: Path to old (replaced) asset USD.
        canonical_usd: Path to canonical (kept) asset USD.
        mode: Dedup mode string.
        mode_index: Required for "transitive" mode.
        group_members: Required for "transitive" mode — all USD paths in group.

    Returns:
        Gf.Matrix4d V matrix (row-vector: p_old = p_canon * V).
    """
    _ensure_pxr()

    V_np = _compute_numpy_V_for_pair(
        old_usd,
        canonical_usd,
        mode,
        mode_index=mode_index,
        group_members=group_members,
    )
    return numpy_to_gf_matrix4d(V_np)
