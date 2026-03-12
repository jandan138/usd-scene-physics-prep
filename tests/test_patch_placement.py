#!/usr/bin/env python3
"""Unit tests for dedup placement patch logic.

Tests the compensation math, edge cases, and helper functions used by
scripts/patch_dedup_placement.py and the fixed compensation formula in
scripts/rewrite_layout_asset_refs_with_compensation.py.

Core formula (row-vector convention, p' = p * M):
    M_new_local = M_canon_inst^{-1} * M_old_inst * M_old_local

Run with: python -m pytest tests/test_patch_placement.py -v
"""

import math
import os
import sys

import pytest

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

try:
    from pxr import Gf
    HAS_PXR = True
except ImportError:
    HAS_PXR = False

pxr_required = pytest.mark.skipif(not HAS_PXR, reason="pxr (USD) not available")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _mat4_close(a, b, atol=1e-9):
    """Return True if two Gf.Matrix4d are element-wise close."""
    for r in range(4):
        for c in range(4):
            if abs(a[r][c] - b[r][c]) > atol:
                return False
    return True


def _make_translate(tx, ty, tz):
    """Build a translation-only Matrix4d."""
    m = Gf.Matrix4d(1.0)
    m.SetTranslate(Gf.Vec3d(tx, ty, tz))
    return m


def _make_scale(sx, sy, sz):
    """Build a scale-only Matrix4d."""
    m = Gf.Matrix4d()
    m.SetDiagonal(Gf.Vec4d(sx, sy, sz, 1.0))
    return m


def _make_rotate_z(degrees):
    """Build a rotation around Z axis."""
    m = Gf.Matrix4d(1.0)
    m.SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), degrees))
    return m


def _compensate(M_canon_inst, M_old_inst, M_old_local):
    """Apply the correct compensation formula."""
    return M_canon_inst.GetInverse() * M_old_inst * M_old_local


# ===========================================================================
# Compensation formula math tests
# ===========================================================================

@pxr_required
class TestCompensationFormula:
    """Tests for M_new_local = M_canon_inst^{-1} * M_old_inst * M_old_local."""

    def test_identity_delta_is_noop(self):
        """When M_old_inst == M_canon_inst, new_local == old_local."""
        M_inst = _make_scale(2.0, 3.0, 4.0) * _make_translate(1, 2, 3)
        M_old_local = _make_translate(10, 20, 30)
        M_new_local = _compensate(M_inst, M_inst, M_old_local)
        assert _mat4_close(M_new_local, M_old_local)

    def test_both_identity_inst_is_noop(self):
        """When both M_old_inst and M_canon_inst are identity, new_local == old_local."""
        I = Gf.Matrix4d(1.0)
        M_old_local = _make_translate(5, 10, 15) * _make_rotate_z(45)
        M_new_local = _compensate(I, I, M_old_local)
        assert _mat4_close(M_new_local, M_old_local)

    def test_scale_only_compensation(self):
        """Scale-only: old_inst scales by 2, canon by 3 -> compensated correctly."""
        M_old_inst = _make_scale(2.0, 2.0, 2.0)
        M_canon_inst = _make_scale(3.0, 3.0, 3.0)
        M_old_local = _make_translate(6.0, 12.0, 18.0)

        M_new_local = _compensate(M_canon_inst, M_old_inst, M_old_local)

        # Verify world position invariant: M_old_inst * M_old_local == M_canon_inst * M_new_local
        world_old = M_old_inst * M_old_local
        world_new = M_canon_inst * M_new_local
        assert _mat4_close(world_old, world_new)

    def test_nonuniform_scale_compensation(self):
        """Non-uniform scale compensation preserves world position."""
        M_old_inst = _make_scale(1.0, 2.0, 3.0)
        M_canon_inst = _make_scale(4.0, 5.0, 6.0)
        M_old_local = _make_translate(10.0, 20.0, 30.0)

        M_new_local = _compensate(M_canon_inst, M_old_inst, M_old_local)

        world_old = M_old_inst * M_old_local
        world_new = M_canon_inst * M_new_local
        assert _mat4_close(world_old, world_new)

    def test_rotation_compensation(self):
        """Rotation delta preserves world position."""
        M_old_inst = _make_rotate_z(30)
        M_canon_inst = _make_rotate_z(75)
        M_old_local = _make_translate(5, 10, 0)

        M_new_local = _compensate(M_canon_inst, M_old_inst, M_old_local)

        world_old = M_old_inst * M_old_local
        world_new = M_canon_inst * M_new_local
        assert _mat4_close(world_old, world_new)

    def test_translation_compensation(self):
        """Translation delta preserves world position."""
        M_old_inst = _make_translate(1, 2, 3)
        M_canon_inst = _make_translate(4, 5, 6)
        M_old_local = _make_scale(2, 2, 2) * _make_translate(10, 20, 30)

        M_new_local = _compensate(M_canon_inst, M_old_inst, M_old_local)

        world_old = M_old_inst * M_old_local
        world_new = M_canon_inst * M_new_local
        assert _mat4_close(world_old, world_new)

    def test_combined_rot_scale_translate(self):
        """Combined rotation + scale + translation preserves world position."""
        M_old_inst = _make_rotate_z(45) * _make_scale(2, 3, 4) * _make_translate(1, 2, 3)
        M_canon_inst = _make_rotate_z(-30) * _make_scale(5, 1, 2) * _make_translate(10, 0, 5)
        M_old_local = _make_rotate_z(60) * _make_translate(7, 8, 9)

        M_new_local = _compensate(M_canon_inst, M_old_inst, M_old_local)

        world_old = M_old_inst * M_old_local
        world_new = M_canon_inst * M_new_local
        assert _mat4_close(world_old, world_new)

    def test_old_inst_identity_canon_nontrivial(self):
        """When old_inst is identity, new_local = canon^{-1} * old_local."""
        I = Gf.Matrix4d(1.0)
        M_canon_inst = _make_scale(3, 3, 3) * _make_translate(5, 5, 5)
        M_old_local = _make_translate(10, 20, 30)

        M_new_local = _compensate(M_canon_inst, I, M_old_local)

        world_old = I * M_old_local
        world_new = M_canon_inst * M_new_local
        assert _mat4_close(world_old, world_new)

    def test_canon_inst_identity_old_nontrivial(self):
        """When canon_inst is identity, new_local = old_inst * old_local."""
        I = Gf.Matrix4d(1.0)
        M_old_inst = _make_scale(2, 2, 2) * _make_rotate_z(90)
        M_old_local = _make_translate(1, 0, 0)

        M_new_local = _compensate(I, M_old_inst, M_old_local)

        # canon^{-1} = I^{-1} = I, so new_local = old_inst * old_local
        expected = M_old_inst * M_old_local
        assert _mat4_close(M_new_local, expected)


# ===========================================================================
# Wrong formula validation (proves the old code was wrong)
# ===========================================================================

@pxr_required
class TestWrongFormulaFails:
    """Demonstrate that the old right-multiply formula produces wrong results."""

    def _wrong_compensate(self, M_old_world, M_old_inst, M_canon_inst, M_parent_world):
        """The OLD wrong formula from L539."""
        new_world = M_old_world * M_old_inst * M_canon_inst.GetInverse()
        new_local = M_parent_world.GetInverse() * new_world
        return new_local

    def test_wrong_formula_breaks_world_position(self):
        """Old formula does NOT preserve world position when inst differs.

        Use rotation + non-uniform scale to ensure non-commutativity.
        Uniform-scale-only deltas commute with everything, masking the bug.
        """
        M_old_inst = _make_rotate_z(30) * _make_scale(2.0, 1.0, 3.0)
        M_canon_inst = _make_rotate_z(-15) * _make_scale(1.0, 4.0, 2.0)
        M_old_local = _make_rotate_z(45) * _make_translate(6.0, 12.0, 18.0)
        M_parent_world = _make_translate(100, 200, 300) * _make_rotate_z(10)

        M_old_world = M_old_inst * M_old_local * M_parent_world
        wrong_local = self._wrong_compensate(M_old_world, M_old_inst, M_canon_inst, M_parent_world)

        # Check: does M_canon_inst * wrong_local == M_old_inst * M_old_local?
        world_old = M_old_inst * M_old_local
        world_wrong = M_canon_inst * wrong_local
        # They should NOT match (proving the old formula is wrong)
        assert not _mat4_close(world_old, world_wrong, atol=0.01), \
            "Old formula unexpectedly preserved world position -- test logic error"

    def test_wrong_formula_is_noop_when_all_identity(self):
        """Old formula collapses to exact no-op when inst=I and parent=I.

        This is the Bug 1 scenario: _get_asset_internal_matrix read /Root (identity)
        for both old and canonical.  With identity inst, the formula becomes:
            new_world = old_world * I * I = old_world
            new_local = parent^{-1} * old_world

        When parent is also identity:
            new_local = I^{-1} * (I * old_local * I) = old_local  (exact no-op)

        In production, Bug 1 made inst=I for all assets. For root-level prims
        (parent=I), compensation was a true no-op, masking Bug 2 entirely.
        """
        I = Gf.Matrix4d(1.0)
        M_old_local = _make_rotate_z(45) * _make_translate(6, 12, 18)

        M_old_world = I * M_old_local * I
        wrong_local = self._wrong_compensate(M_old_world, I, I, I)

        assert _mat4_close(wrong_local, M_old_local)


# ===========================================================================
# Edge case tests for _get_asset_internal_matrix logic
# ===========================================================================

@pxr_required
class TestGetAssetInternalMatrixEdgeCases:
    """Test edge cases for /Root/Instance fallback behavior."""

    def test_identity_fallback_produces_noop(self):
        """When both old and canonical have identity internal, compensation is no-op."""
        I = Gf.Matrix4d(1.0)
        M_old_local = _make_translate(5, 10, 15) * _make_scale(2, 2, 2)
        M_new_local = _compensate(I, I, M_old_local)
        assert _mat4_close(M_new_local, M_old_local)

    def test_one_has_instance_other_identity(self):
        """When old asset has Instance but canonical doesn't (identity), still correct."""
        M_old_inst = _make_scale(2, 2, 2)
        M_canon_inst = Gf.Matrix4d(1.0)  # fallback identity
        M_old_local = _make_translate(10, 20, 30)

        M_new_local = _compensate(M_canon_inst, M_old_inst, M_old_local)

        world_old = M_old_inst * M_old_local
        world_new = M_canon_inst * M_new_local
        assert _mat4_close(world_old, world_new)


# ===========================================================================
# Consistency with normalize_asset_transforms.py pattern
# ===========================================================================

@pxr_required
class TestConsistencyWithNormalizeAssetTransforms:
    """Verify our formula follows the same left-multiply pattern as
    normalize_asset_transforms.py L458: M_scene_new = T_center * R_y2z_inv * M_scene_old."""

    def test_left_multiply_pattern(self):
        """Both formulas use the pattern: M_new = delta * M_old (left-multiply)."""
        # Our formula: delta = M_canon_inst^{-1} * M_old_inst
        M_old_inst = _make_scale(2, 2, 2)
        M_canon_inst = _make_scale(3, 3, 3)
        M_old_local = _make_translate(10, 20, 30)

        delta = M_canon_inst.GetInverse() * M_old_inst
        M_new_local = delta * M_old_local  # left-multiply

        # Verify it's the same as our _compensate function
        M_check = _compensate(M_canon_inst, M_old_inst, M_old_local)
        assert _mat4_close(M_new_local, M_check)

    def test_right_multiply_is_different(self):
        """Right-multiply gives a DIFFERENT result (proving order matters).

        Use rotation-based delta (not uniform scale) so left/right differ.
        """
        M_old_inst = _make_rotate_z(30) * _make_scale(2, 1, 3)
        M_canon_inst = _make_rotate_z(-15) * _make_scale(1, 4, 2)
        M_old_local = _make_rotate_z(45) * _make_translate(10, 20, 30)

        delta = M_canon_inst.GetInverse() * M_old_inst

        left_result = delta * M_old_local
        right_result = M_old_local * delta

        # With non-commutative transforms, left != right
        assert not _mat4_close(left_result, right_result, atol=0.01), \
            "Left and right multiply should differ with non-commutative transforms"

        # Only left-multiply preserves world position
        world_old = M_old_inst * M_old_local
        world_left = M_canon_inst * left_result
        world_right = M_canon_inst * right_result
        assert _mat4_close(world_old, world_left)
        assert not _mat4_close(world_old, world_right, atol=0.01)


# ===========================================================================
# Helper function tests (patch script utilities)
# ===========================================================================

class TestEarliestPreC1Selection:
    """Test selection of earliest .pre_c1 backup by timestamp in filename."""

    def test_sort_by_timestamp(self):
        """Should pick the file with the earliest (smallest) timestamp."""
        filenames = [
            "layout.pre_c1_20260312_120000.usd",
            "layout.pre_c1_20260311_080000.usd",
            "layout.pre_c1_20260312_150000.usd",
        ]
        # The selection logic: sort lexicographically and take first
        earliest = sorted(filenames)[0]
        assert earliest == "layout.pre_c1_20260311_080000.usd"

    def test_single_backup(self):
        """Single backup is always selected."""
        filenames = ["layout.pre_c1_20260312_120000.usd"]
        earliest = sorted(filenames)[0]
        assert earliest == filenames[0]

    def test_different_labels_sort_correctly(self):
        """Different labels (.pre_c1 vs .pre_patch) sort correctly."""
        filenames = [
            "layout.pre_patch.20260312_150000.usd",
            "layout.pre_c1_20260311_080000.usd",
        ]
        # .pre_c1 sorts before .pre_patch
        earliest = sorted(filenames)[0]
        assert "pre_c1" in earliest


class TestMappingConsolidation:
    """Test consolidation of multiple per-category mapping JSONs into a single dict."""

    def test_basic_merge(self):
        """Multiple category mappings merge into one dict."""
        cat1 = {"old_A": "canon_A", "old_B": "canon_A"}
        cat2 = {"old_C": "canon_C", "old_D": "canon_C"}

        consolidated = {}
        for mapping in [cat1, cat2]:
            consolidated.update(mapping)

        assert len(consolidated) == 4
        assert consolidated["old_A"] == "canon_A"
        assert consolidated["old_C"] == "canon_C"

    def test_no_cross_category_conflicts(self):
        """Different categories should not have overlapping old keys."""
        cat_bottle = {"bottle/old1": "bottle/canon1"}
        cat_cup = {"cup/old1": "cup/canon1"}

        consolidated = {}
        for mapping in [cat_bottle, cat_cup]:
            for k, v in mapping.items():
                assert k not in consolidated, f"Cross-category conflict: {k}"
                consolidated[k] = v

        assert len(consolidated) == 2

    def test_empty_mappings_ignored(self):
        """Empty mapping dicts should not cause issues."""
        mappings = [{"old_A": "canon_A"}, {}, {"old_B": "canon_B"}, {}]
        consolidated = {}
        for m in mappings:
            consolidated.update(m)
        assert len(consolidated) == 2


class TestManifestGeneration:
    """Test that a patch manifest can be built from mapping + scene data."""

    def test_prims_with_no_ref_change_skipped(self):
        """Prims whose orig_ref == current_ref should be skipped."""
        mapping = {"old_A": "canon_A", "old_B": "canon_B"}
        # Simulate prim refs: prim1 was changed, prim2 was not
        prim_refs = {
            "prim1": {"orig": "old_A", "current": "canon_A"},
            "prim2": {"orig": "canon_B", "current": "canon_B"},  # same = not changed
        }

        to_patch = []
        for prim, refs in prim_refs.items():
            if refs["orig"] == refs["current"]:
                continue  # skip unchanged
            if refs["orig"] in mapping:
                to_patch.append(prim)

        assert to_patch == ["prim1"]
        assert "prim2" not in to_patch

    def test_prims_not_in_mapping_skipped(self):
        """Prims with orig_ref not in dedup mapping should be skipped."""
        mapping = {"old_A": "canon_A"}
        prim_refs = {
            "prim1": {"orig": "old_A", "current": "canon_A"},
            "prim2": {"orig": "unrelated_asset", "current": "unrelated_asset"},
        }

        to_patch = [p for p, r in prim_refs.items()
                     if r["orig"] != r["current"] and r["orig"] in mapping]
        assert to_patch == ["prim1"]


# ===========================================================================
# Integration test placeholders (to be filled after T7 patch script is done)
# ===========================================================================

@pxr_required
class TestPatchIntegration:
    """Integration tests for patch_dedup_placement.py.

    These require the patch script (T7) to be written first.
    Currently placeholders that verify fundamental invariants.
    """

    @pytest.mark.skip(reason="Requires patch_dedup_placement.py (T7)")
    def test_dry_run_no_file_changes(self):
        """Dry-run mode should produce a report but not modify any files."""
        pass

    @pytest.mark.skip(reason="Requires patch_dedup_placement.py (T7)")
    def test_idempotent_double_patch(self):
        """Running patch twice should produce the same result (idempotent)."""
        pass

    @pytest.mark.skip(reason="Requires patch_dedup_placement.py (T7)")
    def test_pre_patch_backup_created(self):
        """Patch should create a .pre_patch backup before modifying layout."""
        pass

    @pytest.mark.skip(reason="Requires patch_dedup_placement.py (T7)")
    def test_rollback_from_pre_patch_backup(self):
        """Rollback from .pre_patch backup should restore original layout."""
        pass
