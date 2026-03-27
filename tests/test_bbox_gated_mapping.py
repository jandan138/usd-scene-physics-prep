"""Tests for bbox-gated mapping build."""

import json
import os
import sys
from argparse import Namespace
from collections import Counter


sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import c1_build_bulk_mapping_from_dedup_report as mapping_mod


def _write_report(path, duplicates):
    path.write_text(
        json.dumps({"duplicates": duplicates}, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def test_bbox_gated_filters_and_rebuilds(tmp_path, monkeypatch):
    report = tmp_path / "report.json"
    _write_report(
        report,
        [
            {
                "sig": "g1",
                "usd_paths": [
                    "/root/GRScenes_assets/chair/a/usd/a.usd",
                    "/root/GRScenes_assets/chair/b/usd/b.usd",
                    "/root/GRScenes_assets/chair/c/usd/c.usd",
                ],
            }
        ],
    )

    dataset_root = tmp_path / "dataset"
    (dataset_root / "GRScenes100").mkdir(parents=True)

    usage = Counter(
        {
            "GRScenes_assets/chair/a/usd/a.usd": 10,
            "GRScenes_assets/chair/b/usd/b.usd": 20,
            "GRScenes_assets/chair/c/usd/c.usd": 1,
        }
    )
    monkeypatch.setattr(mapping_mod, "_count_layout_asset_usage", lambda *args, **kwargs: usage)

    def _fake_cert(old_usd, canonical_usd, mode, policy):
        old_rel = old_usd.split("/GRScenes_assets/", 1)[1]
        if old_rel.startswith("chair/c/"):
            return {
                "eligible": False,
                "reject_reason": "mode_not_enabled_shape_invariant",
                "bbox_delta": {"min": [0, 0, 0], "max": [0, 0, 0], "max_abs": 0.0},
                "footprint_extent_delta": 0.0,
                "footprint_axis_delta": 0.0,
                "centroid_delta": 0.0,
                "vertex_rmse": None,
                "rmse_available": False,
                "rmse_unavailable_reason": "mode_not_enabled_shape_invariant",
                "alternate_proof_kind": None,
                "alternate_proof_passed": False,
                "proof_source": None,
            }
        return {
            "eligible": True,
            "reject_reason": None,
            "bbox_delta": {"min": [0, 0, 0], "max": [0, 0, 0], "max_abs": 0.0},
            "footprint_extent_delta": 0.0,
            "footprint_axis_delta": 0.0,
            "centroid_delta": 0.0,
            "vertex_rmse": 0.0,
            "rmse_available": True,
            "rmse_unavailable_reason": None,
            "alternate_proof_kind": "geom_only_exact_world_proof",
            "alternate_proof_passed": True,
            "proof_source": "geom_only_exact_world_proof",
        }

    monkeypatch.setattr(mapping_mod._cvt, "build_pair_certificate", _fake_cert)

    out_mapping = tmp_path / "filtered_mapping.json"
    out_stats = tmp_path / "filtered_mapping.stats.json"
    out_cert_jsonl = tmp_path / "pair_certificates.jsonl"
    out_cert_summary = tmp_path / "pair_certificate_summary.json"
    out_graph = tmp_path / "certified_graph.json"

    args = Namespace(
        report=str(report),
        dataset_root=str(dataset_root),
        category=None,
        out_mapping_json=str(out_mapping),
        out_stats_json=str(out_stats),
        progress_every=0,
        bbox_policy="bbox_primary_rmse_observe",
        dedup_mode="geom_only",
        out_certificate_jsonl=str(out_cert_jsonl),
        out_certificate_summary_json=str(out_cert_summary),
        out_certified_graph_json=str(out_graph),
        revoked_edges_jsonl=None,
    )
    rc = mapping_mod._run_bbox_gated(args)
    assert rc == 0

    mapping = json.loads(out_mapping.read_text(encoding="utf-8"))
    assert mapping == {
        "GRScenes_assets/chair/a/usd/a.usd": "GRScenes_assets/chair/b/usd/b.usd"
    }

    graph = json.loads(out_graph.read_text(encoding="utf-8"))
    eligible_components = [c for c in graph["components"] if c["eligible"]]
    assert len(eligible_components) == 1
    assert eligible_components[0]["canonical"] == "GRScenes_assets/chair/b/usd/b.usd"


def test_bbox_gated_revoked_edge_removed(tmp_path, monkeypatch):
    report = tmp_path / "report.json"
    _write_report(
        report,
        [
            {
                "sig": "g1",
                "usd_paths": [
                    "/root/GRScenes_assets/chair/a/usd/a.usd",
                    "/root/GRScenes_assets/chair/b/usd/b.usd",
                ],
            }
        ],
    )
    revoked = tmp_path / "revoked.jsonl"
    revoked.write_text(
        json.dumps(
            {
                "old_asset": "GRScenes_assets/chair/b/usd/b.usd",
                "canonical_asset": "GRScenes_assets/chair/a/usd/a.usd",
            },
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )

    dataset_root = tmp_path / "dataset"
    (dataset_root / "GRScenes100").mkdir(parents=True)
    monkeypatch.setattr(mapping_mod, "_count_layout_asset_usage", lambda *args, **kwargs: Counter())
    monkeypatch.setattr(
        mapping_mod._cvt,
        "build_pair_certificate",
        lambda *args, **kwargs: {
            "eligible": True,
            "reject_reason": None,
            "bbox_delta": {"min": [0, 0, 0], "max": [0, 0, 0], "max_abs": 0.0},
            "footprint_extent_delta": 0.0,
            "footprint_axis_delta": 0.0,
            "centroid_delta": 0.0,
            "vertex_rmse": 0.0,
            "rmse_available": True,
            "rmse_unavailable_reason": None,
            "alternate_proof_kind": "geom_only_exact_world_proof",
            "alternate_proof_passed": True,
            "proof_source": "geom_only_exact_world_proof",
        },
    )

    out_mapping = tmp_path / "filtered_mapping.json"
    out_stats = tmp_path / "filtered_mapping.stats.json"
    out_cert_jsonl = tmp_path / "pair_certificates.jsonl"
    out_cert_summary = tmp_path / "pair_certificate_summary.json"
    out_graph = tmp_path / "certified_graph.json"
    args = Namespace(
        report=str(report),
        dataset_root=str(dataset_root),
        category=None,
        out_mapping_json=str(out_mapping),
        out_stats_json=str(out_stats),
        progress_every=0,
        bbox_policy="bbox_primary_rmse_observe",
        dedup_mode="geom_only",
        out_certificate_jsonl=str(out_cert_jsonl),
        out_certificate_summary_json=str(out_cert_summary),
        out_certified_graph_json=str(out_graph),
        revoked_edges_jsonl=str(revoked),
    )

    mapping_mod._run_bbox_gated(args)
    mapping = json.loads(out_mapping.read_text(encoding="utf-8"))
    assert mapping == {}

    cert_rows = [json.loads(line) for line in out_cert_jsonl.read_text(encoding="utf-8").splitlines() if line.strip()]
    assert cert_rows[0]["reject_reason"] == "revoked_edge"
