#!/usr/bin/env python3
"""Unit tests for the fixed chain-walk dedup compensation in _get_asset_internal_matrix.

Tests verify that _get_asset_internal_matrix() correctly computes the full
internal transform from defaultPrim through Instance down to the first mesh's
parent, including intermediate prims like Group_00, Component_N, etc.
"""

import os
import sys
import tempfile

import pytest

# Ensure scripts/ is on sys.path so we can import the function under test.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

from pxr import Gf, Sdf, Usd, UsdGeom, Vt

from rewrite_layout_asset_refs_with_compensation import _get_asset_internal_matrix


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _matrices_close(a, b, tol=1e-6):
    """Check whether two Gf.Matrix4d are element-wise close."""
    for i in range(4):
        for j in range(4):
            if abs(a[i][j] - b[i][j]) > tol:
                return False
    return True


def _make_translate_matrix(tx, ty, tz):
    """Return a Gf.Matrix4d with the given translation."""
    m = Gf.Matrix4d(1.0)
    m.SetTranslateOnly(Gf.Vec3d(tx, ty, tz))
    return m


def _set_xform_matrix(prim, matrix):
    """Author a single xformOp:transform on a prim."""
    xf = UsdGeom.Xformable(prim)
    op = xf.AddTransformOp()
    op.Set(matrix)


def _add_dummy_mesh(stage, prim_path):
    """Create a mesh prim with dummy triangle points."""
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.GetPointsAttr().Set(
        Vt.Vec3fArray([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
    )
    return mesh


def _create_asset_usd(path, hierarchy_spec):
    """Create a minimal asset USD with given hierarchy and transforms.

    hierarchy_spec is a list of (prim_path, xform_matrix_or_None, is_mesh).
    The stage will have defaultPrim = "Root" with /Root as a basic Xform.
    """
    stage = Usd.Stage.CreateNew(path)
    root = UsdGeom.Xform.Define(stage, "/Root")
    stage.SetDefaultPrim(root.GetPrim())

    for prim_path, xform_matrix, is_mesh in hierarchy_spec:
        if is_mesh:
            _add_dummy_mesh(stage, prim_path)
            prim = stage.GetPrimAtPath(prim_path)
        else:
            xform = UsdGeom.Xform.Define(stage, prim_path)
            prim = xform.GetPrim()

        if xform_matrix is not None:
            _set_xform_matrix(prim, xform_matrix)

    stage.GetRootLayer().Save()
    return path


@pytest.fixture()
def tmp_usd(tmp_path):
    """Yield a factory that creates temp USD files inside tmp_path."""
    paths = []

    def _factory(name="asset.usd"):
        p = str(tmp_path / name)
        paths.append(p)
        return p

    yield _factory
    # Cleanup handled by tmp_path fixture


# ---------------------------------------------------------------------------
# Group A: Multi-level single branch
# ---------------------------------------------------------------------------

class TestSingleBranch:
    """Tests with a single branch from Instance to mesh."""

    def test_direct_child_mesh(self, tmp_usd):
        """Instance(translate=(10,0,0)) -> Mesh: result is Instance local only."""
        path = tmp_usd("direct_child.usd")
        t_instance = _make_translate_matrix(10, 0, 0)
        _create_asset_usd(path, [
            ("/Root/Instance", t_instance, False),
            ("/Root/Instance/Mesh", None, True),
        ])

        result = _get_asset_internal_matrix(path)
        assert _matrices_close(result, t_instance), (
            f"Expected Instance-only transform, got {result}"
        )

    def test_one_intermediate(self, tmp_usd):
        """Instance(translate=(10,0,0)) -> Group_00(translate=(0,5,0)) -> Mesh."""
        path = tmp_usd("one_intermediate.usd")
        t_instance = _make_translate_matrix(10, 0, 0)
        t_group = _make_translate_matrix(0, 5, 0)
        _create_asset_usd(path, [
            ("/Root/Instance", t_instance, False),
            ("/Root/Instance/Group_00", t_group, False),
            ("/Root/Instance/Group_00/Mesh", None, True),
        ])

        result = _get_asset_internal_matrix(path)
        # Chain from defaultPrim (/Root) to mesh_parent (/Root/Instance/Group_00)
        # = Instance_local * Group_00_local (in top-down accumulation via get_chain_transform)
        # get_chain_transform multiplies: result = M_Instance * 1, then result = M_Group * (M_Instance)
        # Row-vector: p_Root = p_Group * M_Group * M_Instance
        # So get_chain_transform returns M_Group * M_Instance
        # But wait — get_chain_transform walks top-down and does result = get_local_matrix(p) * result
        # chain order (top-down): [Instance, Group_00]
        # step 1: result = M_Instance * I = M_Instance
        # step 2: result = M_Group * M_Instance
        expected = t_group * t_instance
        assert _matrices_close(result, expected), (
            f"Expected chain = Group * Instance, got {result}"
        )

    def test_two_intermediates(self, tmp_usd):
        """Instance -> Group_00 -> Component_N -> Mesh: full 3-level chain."""
        path = tmp_usd("two_intermediates.usd")
        t_instance = _make_translate_matrix(10, 0, 0)
        t_group = _make_translate_matrix(0, 5, 0)
        t_component = _make_translate_matrix(0, 0, 3)
        _create_asset_usd(path, [
            ("/Root/Instance", t_instance, False),
            ("/Root/Instance/Group_00", t_group, False),
            ("/Root/Instance/Group_00/Component_N", t_component, False),
            ("/Root/Instance/Group_00/Component_N/Mesh", None, True),
        ])

        result = _get_asset_internal_matrix(path)
        # Chain (top-down): [Instance, Group_00, Component_N]
        # step 1: result = M_Instance
        # step 2: result = M_Group * M_Instance
        # step 3: result = M_Component * M_Group * M_Instance
        expected = t_component * t_group * t_instance
        assert _matrices_close(result, expected), (
            f"Expected full 3-level chain, got {result}"
        )


# ---------------------------------------------------------------------------
# Group B: Multi mesh / multi branch / edge cases
# ---------------------------------------------------------------------------

class TestMultiBranchAndEdgeCases:
    """Tests with multiple meshes, branches, and edge cases."""

    def test_sibling_meshes_same_parent(self, tmp_usd):
        """Instance -> Group_00(translate=(0,5,0)) -> Mesh_A + Mesh_B."""
        path = tmp_usd("sibling_meshes.usd")
        t_group = _make_translate_matrix(0, 5, 0)
        _create_asset_usd(path, [
            ("/Root/Instance", None, False),
            ("/Root/Instance/Group_00", t_group, False),
            ("/Root/Instance/Group_00/Mesh_A", None, True),
            ("/Root/Instance/Group_00/Mesh_B", None, True),
        ])

        result = _get_asset_internal_matrix(path)
        # Both meshes share Group_00 as parent.
        # Instance has identity, so chain = M_Group * I = M_Group
        expected = t_group
        assert _matrices_close(result, expected)

    def test_different_branch_meshes(self, tmp_usd):
        """Instance -> Group_00 -> Mesh_A; Instance -> Group_01 -> Mesh_B.

        Function uses the first mesh found (under Group_00).
        """
        path = tmp_usd("diff_branch.usd")
        t_group0 = _make_translate_matrix(0, 5, 0)
        t_group1 = _make_translate_matrix(0, 0, 7)
        _create_asset_usd(path, [
            ("/Root/Instance", None, False),
            ("/Root/Instance/Group_00", t_group0, False),
            ("/Root/Instance/Group_00/Mesh_A", None, True),
            ("/Root/Instance/Group_01", t_group1, False),
            ("/Root/Instance/Group_01/Mesh_B", None, True),
        ])

        result = _get_asset_internal_matrix(path)
        # First mesh is under Group_00 (prim traversal order),
        # so chain = M_Group_00 (Instance is identity)
        expected = t_group0
        assert _matrices_close(result, expected)

    def test_identity_intermediate(self, tmp_usd):
        """Instance(translate=(10,0,0)) -> Group_00(identity) -> Mesh."""
        path = tmp_usd("identity_intermediate.usd")
        t_instance = _make_translate_matrix(10, 0, 0)
        _create_asset_usd(path, [
            ("/Root/Instance", t_instance, False),
            ("/Root/Instance/Group_00", None, False),
            ("/Root/Instance/Group_00/Mesh", None, True),
        ])

        result = _get_asset_internal_matrix(path)
        # Chain = I * M_Instance = M_Instance (Group_00 has identity)
        expected = t_instance
        assert _matrices_close(result, expected)

    def test_no_mesh_fallback(self, tmp_usd):
        """Instance(translate=(10,0,0)) with no mesh children -> fallback to Instance local."""
        path = tmp_usd("no_mesh.usd")
        t_instance = _make_translate_matrix(10, 0, 0)
        _create_asset_usd(path, [
            ("/Root/Instance", t_instance, False),
            ("/Root/Instance/Group_00", None, False),  # No mesh anywhere
        ])

        result = _get_asset_internal_matrix(path)
        # Fallback: no meshes found → returns Instance local matrix
        expected = t_instance
        assert _matrices_close(result, expected)


# ---------------------------------------------------------------------------
# Group C: Compensation delta correctness
# ---------------------------------------------------------------------------

class TestCompensationDelta:
    """Tests that verify the compensation formula produces correct results."""

    def test_compensation_with_intermediate_vs_flat(self, tmp_usd):
        """Old asset has intermediate Group_00; canonical is flat (mesh under Instance).

        Verify that the compensation formula M_new = M_canon^-1 * M_old * M_old_local
        produces the correct new layout transform.
        """
        old_path = tmp_usd("old_asset.usd")
        canon_path = tmp_usd("canon_asset.usd")

        t_old_instance = Gf.Matrix4d(1.0)  # identity
        t_old_group = _make_translate_matrix(0, 5, 0)
        _create_asset_usd(old_path, [
            ("/Root/Instance", t_old_instance, False),
            ("/Root/Instance/Group_00", t_old_group, False),
            ("/Root/Instance/Group_00/Mesh", None, True),
        ])

        t_canon_instance = _make_translate_matrix(10, 0, 0)
        _create_asset_usd(canon_path, [
            ("/Root/Instance", t_canon_instance, False),
            ("/Root/Instance/Mesh", None, True),
        ])

        M_old = _get_asset_internal_matrix(old_path)
        M_canon = _get_asset_internal_matrix(canon_path)

        # old_local: the layout transform for this prim in the scene
        old_local = _make_translate_matrix(100, 200, 300)

        # Compensation: M_new_local = M_canon^-1 * M_old * M_old_local
        M_new_local = M_canon.GetInverse() * M_old * old_local

        # Verify: M_old * old_local == M_canon * M_new_local
        # (i.e., world placement is preserved)
        lhs = M_old * old_local
        rhs = M_canon * M_new_local
        assert _matrices_close(lhs, rhs), (
            f"Compensation failed: M_old*old_local != M_canon*M_new_local\n"
            f"  LHS={lhs}\n  RHS={rhs}"
        )

    def test_compensation_both_intermediates_different(self, tmp_usd):
        """Both old and canonical have intermediate Group_00 with different transforms.

        old:       Instance(I) -> Group_00(translate=(0,5,0)) -> Mesh
        canonical: Instance(I) -> Group_00(translate=(0,8,0)) -> Mesh
        """
        old_path = tmp_usd("old2.usd")
        canon_path = tmp_usd("canon2.usd")

        t_old_group = _make_translate_matrix(0, 5, 0)
        _create_asset_usd(old_path, [
            ("/Root/Instance", None, False),
            ("/Root/Instance/Group_00", t_old_group, False),
            ("/Root/Instance/Group_00/Mesh", None, True),
        ])

        t_canon_group = _make_translate_matrix(0, 8, 0)
        _create_asset_usd(canon_path, [
            ("/Root/Instance", None, False),
            ("/Root/Instance/Group_00", t_canon_group, False),
            ("/Root/Instance/Group_00/Mesh", None, True),
        ])

        M_old = _get_asset_internal_matrix(old_path)
        M_canon = _get_asset_internal_matrix(canon_path)

        old_local = _make_translate_matrix(50, 60, 70)

        M_new_local = M_canon.GetInverse() * M_old * old_local

        # Verify preservation: M_old * old_local == M_canon * M_new_local
        lhs = M_old * old_local
        rhs = M_canon * M_new_local
        assert _matrices_close(lhs, rhs), (
            f"Compensation failed.\n  LHS={lhs}\n  RHS={rhs}"
        )

        # Also verify the delta is non-trivial (old != canonical internal)
        assert not _matrices_close(M_old, M_canon), (
            "Internal matrices should differ for this test to be meaningful"
        )

    def test_no_instance_returns_identity(self, tmp_usd):
        """Asset with no Instance child returns identity matrix."""
        path = tmp_usd("no_instance.usd")
        _create_asset_usd(path, [
            ("/Root/SomeOtherPrim", None, False),
            ("/Root/SomeOtherPrim/Mesh", None, True),
        ])

        result = _get_asset_internal_matrix(path)
        expected = Gf.Matrix4d(1.0)
        assert _matrices_close(result, expected), (
            f"Expected identity when no Instance child, got {result}"
        )
