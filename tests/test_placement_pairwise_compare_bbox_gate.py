#!/usr/bin/env python3
"""Focused tests for bbox-gated placement pairwise compare."""

import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

try:
    from pxr import Gf, Usd, UsdGeom, Vt
    HAS_PXR = True
except ImportError:
    HAS_PXR = False

pxr_required = pytest.mark.skipif(not HAS_PXR, reason="pxr (USD) not available")

from placement_pairwise_compare import compare_scene


def _make_asset(path: str, *, translate=(0.0, 0.0, 0.0), point_offset=(0.0, 0.0, 0.0)) -> str:
    stage = Usd.Stage.CreateNew(path)
    root = UsdGeom.Xform.Define(stage, "/Root")
    stage.SetDefaultPrim(root.GetPrim())
    inst = UsdGeom.Xform.Define(stage, "/Root/Instance")
    xf = UsdGeom.Xformable(inst.GetPrim())
    op = xf.AddTranslateOp()
    op.Set(Gf.Vec3d(*translate))
    ox, oy, oz = point_offset
    mesh = UsdGeom.Mesh.Define(stage, "/Root/Instance/Mesh")
    mesh.GetPointsAttr().Set(
        Vt.Vec3fArray(
            [
                (0 + ox, 0 + oy, 0 + oz),
                (1 + ox, 0 + oy, 0 + oz),
                (0 + ox, 1 + oy, 0 + oz),
            ]
        )
    )
    stage.GetRootLayer().Save()
    return path


def _make_layout(path: str, asset_path: str) -> str:
    stage = Usd.Stage.CreateNew(path)
    root = UsdGeom.Xform.Define(stage, "/Root")
    stage.SetDefaultPrim(root.GetPrim())
    obj = UsdGeom.Xform.Define(stage, "/Root/Object")
    obj.GetPrim().GetReferences().AddReference(asset_path)
    stage.GetRootLayer().Save()
    return path


@pxr_required
def test_ref_changed_identical_geometry_has_no_hard_fail(tmp_path):
    left_asset = _make_asset(str(tmp_path / "GRScenes_assets" / "chair" / "left" / "usd" / "left.usd"))
    right_asset = _make_asset(str(tmp_path / "GRScenes_assets" / "chair" / "right" / "usd" / "right.usd"))
    left_layout = _make_layout(str(tmp_path / "left_layout.usd"), left_asset)
    right_layout = _make_layout(str(tmp_path / "right_layout.usd"), right_asset)

    result = compare_scene(
        left_layout,
        right_layout,
        "home/scene1",
        {},
        {},
        bbox_policy="bbox_primary_rmse_observe",
        eps_bbox=0.01,
        eps_pos=0.01,
        eps_angle=1.0,
        eps_geom=0.01,
    )

    assert result["status"] == "ok"
    assert result["ref_changed_hard_fail_count"] == 0


@pxr_required
def test_ref_changed_internal_drift_triggers_hard_fail(tmp_path):
    left_asset = _make_asset(
        str(tmp_path / "GRScenes_assets" / "chair" / "left" / "usd" / "left.usd"),
        translate=(0.0, 0.0, 0.0),
    )
    right_asset = _make_asset(
        str(tmp_path / "GRScenes_assets" / "chair" / "right" / "usd" / "right.usd"),
        point_offset=(5.0, 0.0, 0.0),
    )
    left_layout = _make_layout(str(tmp_path / "left_layout.usd"), left_asset)
    right_layout = _make_layout(str(tmp_path / "right_layout.usd"), right_asset)

    result = compare_scene(
        left_layout,
        right_layout,
        "home/scene1",
        {},
        {},
        bbox_policy="bbox_primary_rmse_observe",
        eps_bbox=0.01,
        eps_pos=0.01,
        eps_angle=1.0,
        eps_geom=0.01,
    )

    assert result["status"] == "ok"
    assert result["ref_changed_hard_fail_count"] >= 1
    assert result["blocking_reason_counts"]


@pxr_required
def test_observe_tier2_bbox_drift_is_soft_fail(tmp_path):
    """Under observe policy + topo_filesize mode, bbox drift is a soft warning, not hard fail."""
    left_uid = "aaa111"
    right_uid = "bbb222"
    left_asset = _make_asset(
        str(tmp_path / "GRScenes_assets" / "chair" / left_uid / "usd" / f"{left_uid}.usd"),
    )
    right_asset = _make_asset(
        str(tmp_path / "GRScenes_assets" / "chair" / right_uid / "usd" / f"{right_uid}.usd"),
        point_offset=(0.2, 0.0, 0.0),  # bbox drift > 0.15 threshold
    )
    left_layout = _make_layout(str(tmp_path / "left_layout.usd"), left_asset)
    right_layout = _make_layout(str(tmp_path / "right_layout.usd"), right_asset)

    # mode_index keyed by (canon_uid, old_uid) — maps to topo_filesize
    mode_index = {(right_uid, left_uid): "topo_filesize"}

    result = compare_scene(
        left_layout,
        right_layout,
        "home/scene1",
        {},
        {},
        bbox_policy="bbox_primary_rmse_observe",
        eps_bbox=0.01,
        eps_pos=0.01,
        eps_angle=1.0,
        eps_geom=0.01,
        mode_index=mode_index,
    )

    assert result["status"] == "ok"
    assert result["ref_changed_hard_fail_count"] == 0
    assert result["ref_changed_soft_fail_count"] >= 1
    assert result["soft_reason_counts"]


@pxr_required
def test_harder_tier2_bbox_drift_is_still_hard_fail(tmp_path):
    """Under harder policy + topo_filesize mode, bbox drift remains a hard fail."""
    left_uid = "aaa111"
    right_uid = "bbb222"
    left_asset = _make_asset(
        str(tmp_path / "GRScenes_assets" / "chair" / left_uid / "usd" / f"{left_uid}.usd"),
    )
    right_asset = _make_asset(
        str(tmp_path / "GRScenes_assets" / "chair" / right_uid / "usd" / f"{right_uid}.usd"),
        point_offset=(0.2, 0.0, 0.0),  # bbox drift > 0.15 threshold
    )
    left_layout = _make_layout(str(tmp_path / "left_layout.usd"), left_asset)
    right_layout = _make_layout(str(tmp_path / "right_layout.usd"), right_asset)

    mode_index = {(right_uid, left_uid): "topo_filesize"}

    result = compare_scene(
        left_layout,
        right_layout,
        "home/scene1",
        {},
        {},
        bbox_policy="bbox_primary_rmse_harder",
        eps_bbox=0.01,
        eps_pos=0.01,
        eps_angle=1.0,
        eps_geom=0.01,
        mode_index=mode_index,
    )

    assert result["status"] == "ok"
    assert result["ref_changed_hard_fail_count"] >= 1
    assert result["blocking_reason_counts"]
