#!/usr/bin/env python3
"""File-system safety tests for normalize_asset_transforms.py."""

import importlib.util
import os
import sys
from pathlib import Path

import pytest

pytest.importorskip("pxr")

REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "normalize_asset_transforms.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "normalize_asset_transforms", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


mod = _load_module()


def test_ensure_safe_output_usd_target_rejects_symlink_destination(tmp_path):
    src = tmp_path / "src.usd"
    src.write_text("#usda 1.0\n")
    real_dst = tmp_path / "real_dst.usd"
    real_dst.write_text("#usda 1.0\n")
    dst = tmp_path / "dst.usd"
    dst.symlink_to(real_dst)

    with pytest.raises(RuntimeError, match="symlink target"):
        mod._ensure_safe_output_usd_target(str(src), str(dst))


def test_ensure_safe_output_usd_target_rejects_samefile_destination(tmp_path):
    src = tmp_path / "same.usd"
    src.write_text("#usda 1.0\n")

    with pytest.raises(RuntimeError, match="same file"):
        mod._ensure_safe_output_usd_target(str(src), str(src))


def test_is_flat_main_usd_entry_matches_only_primary_flat_usd(tmp_path):
    flat_main = tmp_path / "abc123.usd"
    flat_main.write_text("#usda 1.0\n")
    aux_json = tmp_path / "abc123_annotation.json"
    aux_json.write_text("{}\n")

    assert mod._is_flat_main_usd_entry("abc123.usd", "abc123", str(flat_main))
    assert not mod._is_flat_main_usd_entry(
        "abc123_annotation.json", "abc123", str(aux_json)
    )


def test_flat_main_usd_path_remains_available_for_regular_output(tmp_path):
    src_asset_dir = tmp_path / "src" / "wall" / "abc123"
    src_asset_dir.mkdir(parents=True)
    main_usd = src_asset_dir / "abc123.usd"
    main_usd.write_text("#usda 1.0\n")
    annotation = src_asset_dir / "abc123_annotation.json"
    annotation.write_text("{}\n")

    dst_asset_dir = tmp_path / "out" / "wall" / "abc123"

    # Mirror the Phase 1 asset-support copy loop semantics.
    for item in os.listdir(src_asset_dir):
        src_item = src_asset_dir / item
        dst_item = dst_asset_dir / item
        if mod._is_flat_main_usd_entry(item, "abc123", str(src_item)):
            continue
        dst_asset_dir.mkdir(parents=True, exist_ok=True)
        os.symlink(os.path.abspath(src_item), dst_item)

    dst_main = dst_asset_dir / "abc123.usd"
    dst_annotation = dst_asset_dir / "abc123_annotation.json"

    assert not dst_main.exists()
    assert not os.path.islink(dst_main)
    assert dst_annotation.is_symlink()

    # The normalize step must be able to materialize the primary USD as a real file.
    dst_main.write_text("#usda 1.0\n")
    assert dst_main.exists()
    assert not dst_main.is_symlink()


# ---------------------------------------------------------------------------
# Regression test for _get_chain_transform multiply order (bug fix)
# ---------------------------------------------------------------------------

class _FakePrim:
    """Minimal mock of a USD prim with a parent chain and local matrix."""

    def __init__(self, path: str, parent=None):
        self._path = path
        self._parent = parent

    def GetPath(self):
        return self._path

    def GetParent(self):
        return self._parent


def test_get_chain_transform_multiply_order():
    """Regression: _get_chain_transform must use descendant-first multiply order.

    For a chain [A_child, grandchild] (top-down), the correct result is:
        M_chain = M_grandchild_local * M_A_child_local

    The old buggy code computed M_A_child_local * M_grandchild_local, which
    gives wrong results when the matrices don't commute (e.g. rotation +
    translation).

    This test uses two non-commutative 4x4 matrices and verifies the function
    produces the correct (descendant-first) product, not the transposed one.
    """
    from pxr import Gf, UsdGeom

    # Build a fake 3-prim hierarchy: ancestor -> child -> grandchild
    ancestor = _FakePrim("/Root")
    child = _FakePrim("/Root/Child", parent=ancestor)
    grandchild = _FakePrim("/Root/Child/Grandchild", parent=child)

    # Define two non-commutative matrices:
    # M_child = 90-degree rotation around Z axis
    # M_grandchild = pure translation (5, 0, 0)
    m_child = Gf.Matrix4d(
        0, 1, 0, 0,
        -1, 0, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1,
    )
    m_grandchild = Gf.Matrix4d(
        1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        5, 0, 0, 1,
    )

    # Verify they don't commute
    assert m_child * m_grandchild != m_grandchild * m_child, \
        "Test matrices must be non-commutative"

    # Expected: M_grandchild * M_child (descendant-first, correct)
    expected = m_grandchild * m_child

    # Wrong (old bug): M_child * M_grandchild
    wrong = m_child * m_grandchild

    # Monkey-patch _get_local_matrix to return our known matrices
    local_matrices = {
        "/Root/Child": m_child,
        "/Root/Child/Grandchild": m_grandchild,
    }
    original_fn = mod._get_local_matrix

    def mock_get_local_matrix(prim):
        path = prim.GetPath() if hasattr(prim.GetPath(), '__str__') else prim.GetPath()
        return local_matrices[str(path)]

    mod._get_local_matrix = mock_get_local_matrix
    try:
        result = mod._get_chain_transform(ancestor, grandchild)
    finally:
        mod._get_local_matrix = original_fn

    assert result == expected, \
        f"_get_chain_transform should use descendant-first multiply order, got wrong result"
    assert result != wrong, \
        f"_get_chain_transform result matches the old buggy (parent-first) order"
