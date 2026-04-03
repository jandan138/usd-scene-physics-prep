"""Unit tests for scripts/compute_vertex_transform.py (numpy-only functions)."""

import sys
import os

import numpy as np
import pytest

# Add scripts/ to path so we can import the module without pxr
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from compute_vertex_transform import (
    build_pair_certificate,
    procrustes_full,
    compute_V_shape_invariant,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _apply_V(pts: np.ndarray, V: np.ndarray) -> np.ndarray:
    """Apply 4x4 row-vector transform: p_out = p_in @ V (homogeneous)."""
    N = pts.shape[0]
    pts_h = np.hstack([pts, np.ones((N, 1), dtype=np.float64)])
    return (pts_h @ V)[:, :3]


def _rot_y(deg: float) -> np.ndarray:
    """3x3 rotation matrix around Y axis (row-vector convention)."""
    rad = np.radians(deg)
    c, s = np.cos(rad), np.sin(rad)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c],
    ], dtype=np.float64)


# A small set of non-degenerate points used by multiple tests
BASE_PTS = np.array([
    [0.0, 0.0, 0.0],
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
    [1.0, 1.0, 1.0],
    [2.0, 0.5, 0.3],
    [0.3, 2.0, 0.7],
    [0.7, 0.3, 2.0],
], dtype=np.float64)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

class TestProcroustesFull:
    """Tests for procrustes_full()."""

    def test_identity_vertices(self):
        """Same points in both args -> V should be identity."""
        pts = BASE_PTS.copy()
        V = procrustes_full(pts, pts)
        assert np.allclose(V, np.eye(4), atol=1e-6)

    def test_90deg_y_rotation(self):
        """Rotate pts by 90 deg around Y -> V should recover the rotation."""
        canon = BASE_PTS.copy()
        R = _rot_y(90.0)
        old = canon @ R  # row-vector rotation

        V = procrustes_full(canon, old)

        # Round-trip check
        aligned = _apply_V(canon, V)
        assert np.allclose(aligned, old, atol=1e-6), (
            f"max residual: {np.abs(aligned - old).max():.2e}"
        )

        # Upper-left 3x3 should approximate R_y(90)
        assert np.allclose(V[:3, :3], R, atol=1e-5)

    def test_x_axis_mirror(self):
        """Mirror across X axis: negate X coordinates."""
        canon = BASE_PTS.copy()
        old = canon.copy()
        old[:, 0] *= -1  # negate X

        V = procrustes_full(canon, old)

        # Round-trip
        aligned = _apply_V(canon, V)
        assert np.allclose(aligned, old, atol=1e-6)

        # V[0,0] should be approximately -1 (X mirror)
        assert V[0, 0] < -0.9, f"V[0,0] = {V[0, 0]}"

    def test_uniform_scale_2x(self):
        """Scale all points by 2 -> V diagonal should reflect scale ~2."""
        canon = BASE_PTS.copy()
        old = canon * 2.0

        V = procrustes_full(canon, old)

        # Round-trip
        aligned = _apply_V(canon, V)
        assert np.allclose(aligned, old, atol=1e-6)

        # Upper-left 3x3 diagonal should be ~2
        diag = np.diag(V[:3, :3])
        assert np.allclose(np.abs(diag), 2.0, atol=0.1), f"diag = {diag}"

    def test_rotation_scale_translation(self):
        """Combined rotation + scale + translation -> V recovers full affine."""
        canon = BASE_PTS.copy()
        R = _rot_y(45.0)
        s = 1.5
        t = np.array([10.0, -5.0, 3.0])
        old = canon @ (s * R) + t

        V = procrustes_full(canon, old)

        aligned = _apply_V(canon, V)
        assert np.allclose(aligned, old, atol=1e-5), (
            f"max residual: {np.abs(aligned - old).max():.2e}"
        )

    def test_round_trip_complex(self):
        """100 random points with random rotation + scale + translation."""
        rng = np.random.RandomState(123)
        canon = rng.randn(100, 3).astype(np.float64)

        # Random rotation via QR decomposition
        M = rng.randn(3, 3)
        Q, _ = np.linalg.qr(M)
        if np.linalg.det(Q) < 0:
            Q[:, 0] *= -1  # ensure proper rotation
        s = 2.3
        t = np.array([7.0, -3.0, 11.0])
        old = canon @ (s * Q) + t

        V = procrustes_full(canon, old)

        aligned = _apply_V(canon, V)
        residual = np.sqrt(np.mean(np.sum((aligned - old) ** 2, axis=1)))
        assert residual < 1e-6, f"RMSE = {residual:.2e}"

    def test_vertex_count_mismatch_raises(self):
        """Different vertex counts should raise ValueError."""
        a = np.random.randn(10, 3)
        b = np.random.randn(12, 3)
        with pytest.raises(ValueError, match="Vertex count mismatch"):
            procrustes_full(a, b)

    def test_procrustes_reflection_determinant(self):
        """Reflection (det(R) = -1) should still produce valid round-trip."""
        canon = BASE_PTS.copy()
        # Full reflection: negate all axes
        old = -canon

        V = procrustes_full(canon, old)

        aligned = _apply_V(canon, V)
        assert np.allclose(aligned, old, atol=1e-6)

        # det of the 3x3 part should be negative (reflection)
        det = np.linalg.det(V[:3, :3])
        assert det < 0, f"det = {det} (expected negative for reflection)"


class TestShapeInvariant:
    """Tests for compute_V_shape_invariant()."""

    def test_different_bbox_same_shape(self):
        """Same shape shifted to a different bbox location."""
        canon = BASE_PTS.copy()  # bbox roughly [0,2]x[0,2]x[0,2]
        offset = np.array([5.0, 5.0, 5.0])
        old = canon + offset

        V = compute_V_shape_invariant(canon, old)

        aligned = _apply_V(canon, V)
        assert np.allclose(aligned, old, atol=1e-3), (
            f"max residual: {np.abs(aligned - old).max():.2e}"
        )

    def test_different_scale(self):
        """Large absolute size deltas are now rejected by the stricter guard."""
        canon = BASE_PTS.copy()
        old = canon * 5.0

        with pytest.raises(RuntimeError, match="aspect ratio mismatch"):
            compute_V_shape_invariant(canon, old)

    def test_different_vertex_count_icp(self):
        """Different vertex counts triggers ICP path."""
        rng = np.random.RandomState(42)
        canon = rng.rand(50, 3).astype(np.float64) * 10
        # old is a near-superset with the same overall scale, which should
        # stay inside the stricter bbox guards while still exercising ICP.
        extra = canon[:10] + rng.normal(scale=1e-3, size=(10, 3))
        old = np.vstack([canon, extra])

        # Should not raise — exercises the ICP branch
        V = compute_V_shape_invariant(canon, old)
        assert V.shape == (4, 4)


class TestTransitive:
    """Test transitive V accumulation (V_AB @ V_BC maps A -> C)."""

    def test_transitive_2step(self):
        """A -> B (scale 2x) -> C (rotate 90 deg Y). V_AB @ V_BC maps A to C."""
        A = BASE_PTS.copy()

        # Step 1: B = A * 2
        B = A * 2.0
        V_AB = procrustes_full(A, B)

        # Step 2: C = B rotated 90 deg around Y
        R = _rot_y(90.0)
        C = B @ R
        V_BC = procrustes_full(B, C)

        # Transitive: V_AC = V_AB @ V_BC
        V_AC = V_AB @ V_BC

        aligned = _apply_V(A, V_AC)
        assert np.allclose(aligned, C, atol=1e-5), (
            f"max residual: {np.abs(aligned - C).max():.2e}"
        )


class TestPairCertificate:
    """Tests for build_pair_certificate()."""

    def test_geom_only_exact_proof(self, monkeypatch):
        monkeypatch.setattr(
            "compute_vertex_transform.extract_instance_space_vertices",
            lambda path: BASE_PTS.copy(),
        )
        monkeypatch.setattr("compute_vertex_transform.os.path.isfile", lambda path: True)

        cert = build_pair_certificate(
            old_usd="/tmp/old.usd",
            canonical_usd="/tmp/canon.usd",
            mode="geom_only",
        )

        assert cert["eligible"] is True
        assert cert["reject_reason"] is None
        assert cert["proof_source"] == "geom_only_exact_world_proof"
        assert cert["vertex_rmse"] == 0.0
        assert cert["rmse_available"] is True
        assert cert["bbox_delta"]["max_abs"] == 0.0

    def test_unsupported_mode_rejected(self, monkeypatch):
        """Modes not in allowed_modes (e.g. transitive) are rejected."""
        monkeypatch.setattr("compute_vertex_transform.os.path.isfile", lambda path: True)
        cert = build_pair_certificate(
            old_usd="/tmp/old.usd",
            canonical_usd="/tmp/canon.usd",
            mode="transitive",
        )
        assert cert["eligible"] is False
        assert cert["reject_reason"] == "mode_not_enabled_transitive"
        assert cert["rmse_available"] is False

    def test_topo_filesize_with_v_computation(self, monkeypatch):
        """topo_filesize mode computes real V and bbox delta."""
        canon_pts = BASE_PTS.copy()
        # Simulate old = canon shifted by small translation
        old_pts = canon_pts + np.array([0.001, 0.001, 0.001])

        monkeypatch.setattr("compute_vertex_transform.os.path.isfile", lambda path: True)
        monkeypatch.setattr(
            "compute_vertex_transform.extract_instance_space_vertices",
            lambda path: old_pts.copy() if "old" in path else canon_pts.copy(),
        )

        cert = build_pair_certificate(
            old_usd="/tmp/old.usd",
            canonical_usd="/tmp/canon.usd",
            mode="topo_filesize",
        )

        assert cert["eligible"] is True
        assert cert["proof_source"] == "topo_filesize_bbox_gated_proof"
        assert cert.get("bbox_delta_available") is True
        assert cert["bbox_delta"]["max_abs"] < 0.5  # below precheck threshold
        assert cert["rmse_available"] is True

    def test_shape_invariant_v_failure_closes(self, monkeypatch):
        """V computation failure -> fail closed with None metrics."""
        monkeypatch.setattr("compute_vertex_transform.os.path.isfile", lambda path: True)
        monkeypatch.setattr(
            "compute_vertex_transform.extract_instance_space_vertices",
            lambda path: BASE_PTS.copy(),
        )
        monkeypatch.setattr(
            "compute_vertex_transform.compute_V_for_pair",
            lambda *args, **kwargs: (_ for _ in ()).throw(RuntimeError("test failure")),
        )

        cert = build_pair_certificate(
            old_usd="/tmp/old.usd",
            canonical_usd="/tmp/canon.usd",
            mode="shape_invariant",
        )

        assert cert["eligible"] is False
        assert "v_computation_failed" in cert["reject_reason"]
        assert cert["bbox_delta"] is None
        assert cert["bbox_delta_available"] is False
        assert cert["rmse_available"] is False

    def test_missing_asset_rejected(self, monkeypatch):
        def _isfile(path):
            return path != "/tmp/missing.usd"

        monkeypatch.setattr("compute_vertex_transform.os.path.isfile", _isfile)
        cert = build_pair_certificate(
            old_usd="/tmp/missing.usd",
            canonical_usd="/tmp/canon.usd",
            mode="geom_only",
        )
        assert cert["eligible"] is False
        assert cert["reject_reason"] == "old_asset_missing"
