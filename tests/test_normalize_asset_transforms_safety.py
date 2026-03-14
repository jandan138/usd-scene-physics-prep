#!/usr/bin/env python3
"""Safety tests for normalize_asset_transforms.py file output guards."""

import importlib.util
import os
import sys
import types
from pathlib import Path


def _install_fake_pxr():
    class _DummyMatrix4d:
        def __init__(self, *args, **kwargs):
            pass

        def SetRotate(self, *args, **kwargs):
            return None

    class _DummyRotation:
        def __init__(self, *args, **kwargs):
            pass

    class _DummyVec3d:
        def __init__(self, *args, **kwargs):
            pass

    fake_gf = types.SimpleNamespace(
        Matrix4d=_DummyMatrix4d,
        Rotation=_DummyRotation,
        Vec3d=_DummyVec3d,
    )
    fake_pxr = types.SimpleNamespace(
        Gf=fake_gf,
        Sdf=types.SimpleNamespace(),
        Usd=types.SimpleNamespace(),
        UsdGeom=types.SimpleNamespace(),
    )
    sys.modules["pxr"] = fake_pxr


def _load_module():
    _install_fake_pxr()
    repo_root = Path(__file__).resolve().parents[1]
    script_path = repo_root / "scripts" / "normalize_asset_transforms.py"
    spec = importlib.util.spec_from_file_location(
        "normalize_asset_transforms_safety", script_path
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_flat_layout_main_usd_is_not_treated_as_auxiliary_file(tmp_path):
    mod = _load_module()
    src_item = tmp_path / "abc123.usd"
    src_item.write_text("#usd")

    assert mod._is_flat_layout_main_usd("abc123.usd", "abc123", str(src_item)) is True
    assert mod._is_flat_layout_main_usd("note.json", "abc123", str(src_item)) is False


def test_guard_rejects_symlink_export_target(tmp_path):
    mod = _load_module()
    src = tmp_path / "src.usd"
    dst_real = tmp_path / "dst_real.usd"
    dst_link = tmp_path / "dst.usd"
    src.write_text("src")
    dst_real.write_text("dst")
    dst_link.symlink_to(dst_real)

    try:
        mod._guard_normalize_output_path(str(src), str(dst_link))
    except RuntimeError as exc:
        assert "symlink target" in str(exc)
    else:
        raise AssertionError("expected symlink export guard to fail")


def test_guard_rejects_samefile_export_target(tmp_path):
    mod = _load_module()
    src = tmp_path / "asset.usd"
    src.write_text("src")

    try:
        mod._guard_normalize_output_path(str(src), str(src))
    except RuntimeError as exc:
        assert "source file" in str(exc)
    else:
        raise AssertionError("expected samefile export guard to fail")


def test_guard_allows_regular_output_path(tmp_path):
    mod = _load_module()
    src = tmp_path / "src.usd"
    dst = tmp_path / "dst.usd"
    src.write_text("src")
    dst.write_text("dst")

    mod._guard_normalize_output_path(str(src), str(dst))
    assert os.path.isfile(dst)
