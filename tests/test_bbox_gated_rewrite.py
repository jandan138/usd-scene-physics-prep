#!/usr/bin/env python3
"""Focused bbox-gated rewrite tests."""

import json
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

try:
    from pxr import Sdf, Usd, UsdGeom, Vt

    HAS_PXR = True
except ImportError:
    HAS_PXR = False

pxr_required = pytest.mark.skipif(not HAS_PXR, reason="pxr (USD) not available")

from rewrite_layout_asset_refs_with_compensation import MappingPair, rewrite_layout


def _make_asset_usd(path: str) -> str:
    stage = Usd.Stage.CreateNew(path)
    root = UsdGeom.Xform.Define(stage, "/Root")
    stage.SetDefaultPrim(root.GetPrim())
    inst = UsdGeom.Xform.Define(stage, "/Root/Instance")
    mesh = UsdGeom.Mesh.Define(stage, "/Root/Instance/Mesh")
    mesh.GetPointsAttr().Set(Vt.Vec3fArray([(0, 0, 0), (1, 0, 0), (0, 1, 0)]))
    stage.GetRootLayer().Save()
    assert inst
    return path


def _make_layout_with_payload(path: str, asset_path: str) -> str:
    stage = Usd.Stage.CreateNew(path)
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())
    obj = UsdGeom.Xform.Define(stage, "/World/Object")
    obj.GetPrim().GetPayloads().AddPayload(asset_path)
    stage.GetRootLayer().Save()
    return path


def _make_layout_with_multi_refs(path: str, asset_a: str, asset_b: str) -> str:
    stage = Usd.Stage.CreateNew(path)
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())
    obj = UsdGeom.Xform.Define(stage, "/World/Object")
    refs = Sdf.ReferenceListOp.CreateExplicit(
        [
            Sdf.Reference(assetPath=asset_a),
            Sdf.Reference(assetPath=asset_b),
        ]
    )
    obj.GetPrim().SetMetadata("references", refs)
    stage.GetRootLayer().Save()
    return path


def _make_layout_with_single_ref(path: str, asset_path: str) -> str:
    stage = Usd.Stage.CreateNew(path)
    root = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(root.GetPrim())
    obj = UsdGeom.Xform.Define(stage, "/World/Object")
    refs = Sdf.ReferenceListOp.CreateExplicit([Sdf.Reference(assetPath=asset_path)])
    obj.GetPrim().SetMetadata("references", refs)
    stage.GetRootLayer().Save()
    return path


def _transitive_cert_row(*, reject_reason=None, eligible=True):
    return {
        "old_asset": "GRScenes_assets/bed/old/usd/old.usd",
        "canonical_asset": "GRScenes_assets/bed/canon/usd/canon.usd",
        "mode": "transitive",
        "eligible": eligible,
        "reject_reason": reject_reason,
        "transitive_witness_uids": ["canon", "mid", "old"],
        "transitive_witness_modes": ["geom_only", "geom_only"],
        "transitive_witness_length": 2,
        "transitive_endpoint_pair_directly_verified": True,
        "transitive_endpoint_verification_kind": "composed_witness_V",
        "transitive_endpoint_verification_passed": reject_reason is None,
    }


@pxr_required
def test_bbox_gated_payload_rewrite_rejected_without_mutation(tmp_path):
    old_asset = _make_asset_usd(str(tmp_path / "old.usd"))
    canon_asset = _make_asset_usd(str(tmp_path / "canon.usd"))
    layout = _make_layout_with_payload(str(tmp_path / "layout.usd"), old_asset)
    out_layout = str(tmp_path / "layout.out.usd")

    summary = rewrite_layout(
        layout_usd=layout,
        out_usd=out_layout,
        subset_root=str(tmp_path),
        mapping_pairs=[MappingPair(old_abs=old_asset, canonical_abs=canon_asset)],
        apply_compensation=True,
        set_instanceable=False,
        dry_run=False,
        report_out=str(tmp_path / "report.json"),
        max_preview=20,
        bbox_gated=True,
        bbox_policy="bbox_primary_rmse_observe",
    )

    assert summary["counts"]["payloads_changed"] == 0
    assert summary["counts"]["reject_records"] == 1

    stage = Usd.Stage.Open(out_layout)
    prim = stage.GetPrimAtPath("/World/Object")
    payloads = prim.GetPrimStack()[0].payloadList
    items = list(payloads.GetAddedOrExplicitItems())
    assert len(items) == 1
    assert items[0].assetPath == old_asset


@pxr_required
def test_bbox_gated_multi_ref_rejected_without_mutation(tmp_path):
    old_a = _make_asset_usd(str(tmp_path / "old_a.usd"))
    old_b = _make_asset_usd(str(tmp_path / "old_b.usd"))
    canon = _make_asset_usd(str(tmp_path / "canon.usd"))
    layout = _make_layout_with_multi_refs(str(tmp_path / "layout.usd"), old_a, old_b)
    out_layout = str(tmp_path / "layout.out.usd")

    summary = rewrite_layout(
        layout_usd=layout,
        out_usd=out_layout,
        subset_root=str(tmp_path),
        mapping_pairs=[
            MappingPair(old_abs=old_a, canonical_abs=canon),
            MappingPair(old_abs=old_b, canonical_abs=canon),
        ],
        apply_compensation=True,
        set_instanceable=False,
        dry_run=False,
        report_out=str(tmp_path / "report.json"),
        max_preview=20,
        bbox_gated=True,
        bbox_policy="bbox_primary_rmse_observe",
    )

    assert summary["counts"]["refs_changed"] == 0
    assert summary["counts"]["reject_records"] == 1

    stage = Usd.Stage.Open(out_layout)
    prim = stage.GetPrimAtPath("/World/Object")
    refs = prim.GetMetadata("references")
    items = list(refs.GetAddedOrExplicitItems())
    assert [item.assetPath for item in items] == [old_a, old_b]


@pxr_required
def test_bbox_gated_transitive_ref_rewrite_uses_certificate_witness(tmp_path):
    dataset_root = tmp_path / "dataset"
    mode_reports_dir = tmp_path / "mode_reports"
    mode_reports_dir.mkdir()

    category_root = dataset_root / "GRScenes_assets" / "bed"
    canon_uid = "canon"
    mid_uid = "mid"
    old_uid = "old"
    canon_asset = _make_asset_usd(
        str(category_root / canon_uid / "usd" / f"{canon_uid}.usd")
    )
    _make_asset_usd(str(category_root / mid_uid / "usd" / f"{mid_uid}.usd"))
    old_asset = _make_asset_usd(str(category_root / old_uid / "usd" / f"{old_uid}.usd"))

    scene_dir = dataset_root / "GRScenes100" / "home" / "SCENE1"
    layout = _make_layout_with_single_ref(str(scene_dir / "layout.usd"), old_asset)
    out_layout = str(scene_dir / "layout.out.usd")
    cert_path = tmp_path / "pair_certificates.jsonl"
    cert_row = _transitive_cert_row()
    cert_path.write_text(json.dumps(cert_row) + "\n", encoding="utf-8")

    summary = rewrite_layout(
        layout_usd=layout,
        out_usd=out_layout,
        subset_root=str(dataset_root),
        mapping_pairs=[MappingPair(old_abs=old_asset, canonical_abs=canon_asset)],
        apply_compensation=True,
        set_instanceable=False,
        dry_run=False,
        report_out=str(tmp_path / "report.json"),
        max_preview=20,
        v_matrix_mode="auto",
        mode_reports_dir=str(mode_reports_dir),
        bbox_gated=True,
        bbox_policy="bbox_primary_rmse_observe",
        certificate_jsonl=str(cert_path),
    )

    assert summary["counts"]["refs_changed"] == 1
    assert summary["counts"]["reject_records"] == 0

    stage = Usd.Stage.Open(out_layout)
    prim = stage.GetPrimAtPath("/World/Object")
    refs = prim.GetMetadata("references")
    items = list(refs.GetAddedOrExplicitItems())
    assert len(items) == 1
    assert items[0].assetPath == canon_asset


@pxr_required
def test_bbox_gated_transitive_ref_rewrite_prefers_certified_row(tmp_path):
    dataset_root = tmp_path / "dataset"
    mode_reports_dir = tmp_path / "mode_reports"
    mode_reports_dir.mkdir()

    category_root = dataset_root / "GRScenes_assets" / "bed"
    canon_asset = _make_asset_usd(str(category_root / "canon" / "usd" / "canon.usd"))
    _make_asset_usd(str(category_root / "mid" / "usd" / "mid.usd"))
    old_asset = _make_asset_usd(str(category_root / "old" / "usd" / "old.usd"))

    scene_dir = dataset_root / "GRScenes100" / "home" / "SCENE1"
    layout = _make_layout_with_single_ref(str(scene_dir / "layout.usd"), old_asset)
    out_layout = str(scene_dir / "layout.out.usd")
    cert_path = tmp_path / "pair_certificates.jsonl"
    cert_path.write_text(
        json.dumps(_transitive_cert_row())
        + "\n"
        + json.dumps(
            _transitive_cert_row(
                reject_reason="bbox_precheck_failed_transitive", eligible=False
            )
        )
        + "\n",
        encoding="utf-8",
    )

    summary = rewrite_layout(
        layout_usd=layout,
        out_usd=out_layout,
        subset_root=str(dataset_root),
        mapping_pairs=[MappingPair(old_abs=old_asset, canonical_abs=canon_asset)],
        apply_compensation=True,
        set_instanceable=False,
        dry_run=False,
        report_out=str(tmp_path / "report.json"),
        max_preview=20,
        v_matrix_mode="auto",
        mode_reports_dir=str(mode_reports_dir),
        bbox_gated=True,
        bbox_policy="bbox_primary_rmse_observe",
        certificate_jsonl=str(cert_path),
    )

    assert summary["counts"]["refs_changed"] == 1
    assert summary["counts"]["reject_records"] == 0


@pxr_required
def test_bbox_gated_transitive_ref_rewrite_rejects_noncertified_row(tmp_path):
    dataset_root = tmp_path / "dataset"
    mode_reports_dir = tmp_path / "mode_reports"
    mode_reports_dir.mkdir()

    category_root = dataset_root / "GRScenes_assets" / "bed"
    canon_asset = _make_asset_usd(str(category_root / "canon" / "usd" / "canon.usd"))
    _make_asset_usd(str(category_root / "mid" / "usd" / "mid.usd"))
    old_asset = _make_asset_usd(str(category_root / "old" / "usd" / "old.usd"))

    scene_dir = dataset_root / "GRScenes100" / "home" / "SCENE1"
    layout = _make_layout_with_single_ref(str(scene_dir / "layout.usd"), old_asset)
    out_layout = str(scene_dir / "layout.out.usd")
    cert_path = tmp_path / "pair_certificates.jsonl"
    cert_path.write_text(
        json.dumps(
            _transitive_cert_row(
                reject_reason="bbox_precheck_failed_transitive", eligible=False
            )
        )
        + "\n",
        encoding="utf-8",
    )

    summary = rewrite_layout(
        layout_usd=layout,
        out_usd=out_layout,
        subset_root=str(dataset_root),
        mapping_pairs=[MappingPair(old_abs=old_asset, canonical_abs=canon_asset)],
        apply_compensation=True,
        set_instanceable=False,
        dry_run=False,
        report_out=str(tmp_path / "report.json"),
        max_preview=20,
        v_matrix_mode="auto",
        mode_reports_dir=str(mode_reports_dir),
        bbox_gated=True,
        bbox_policy="bbox_primary_rmse_observe",
        certificate_jsonl=str(cert_path),
    )

    assert summary["counts"]["refs_changed"] == 0
    assert summary["counts"]["reject_records"] == 1

    stage = Usd.Stage.Open(out_layout)
    prim = stage.GetPrimAtPath("/World/Object")
    refs = prim.GetMetadata("references")
    items = list(refs.GetAddedOrExplicitItems())
    assert len(items) == 1
    assert items[0].assetPath == old_asset
