import json
import os
import sys
from pathlib import Path
import shutil

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import prepare_promoted_clone_workspace as mod


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _fixture_roots(tmp_path: Path):
    src_dataset = tmp_path / "src_dataset"
    src_c1_bulk = tmp_path / "src_c1_bulk"
    workspace = tmp_path / "workspace"
    reports_root = tmp_path / "v8_prededup"

    _write(src_dataset / "GRScenes100" / "home" / "SCENE1" / "layout.usd", "LAYOUT")
    _write(
        src_dataset / "GRScenes_assets" / "book" / "keep" / "usd" / "keep.usd",
        "#usda 1.0\n",
    )
    _write(src_dataset / "Material" / "mdl" / "example.mdl", "mdl")

    cat_root = src_c1_bulk / "book_bbox_primary_rmse_observe_v1"
    _write(
        cat_root / "01_cert" / "filtered_mapping.json",
        json.dumps(
            {
                "GRScenes_assets/book/old/usd/old.usd": "GRScenes_assets/book/keep/usd/keep.usd"
            },
            indent=2,
        ),
    )
    _write(
        cat_root / "01_cert" / "filtered_mapping.stats.json",
        json.dumps({"mapping_pairs": 1}, indent=2),
    )
    _write(cat_root / "01_cert" / "pair_certificates.jsonl", "{}\n")
    _write(
        cat_root / "01_cert" / "certified_graph.json",
        json.dumps({"components": 1}, indent=2),
    )
    _write(cat_root / "02_apply" / "stale.json", "stale")

    empty_root = src_c1_bulk / "rug_bbox_primary_rmse_observe_v1"
    empty_root.mkdir(parents=True, exist_ok=True)

    _write(
        reports_root / "union_3way" / "all_categories_union_merged.json",
        json.dumps({"categories": ["book"]}, indent=2),
    )
    _write(reports_root / "mode_summary.json", json.dumps({"ok": True}, indent=2))

    return src_dataset, src_c1_bulk, workspace


def test_prepare_workspace_copies_dataset_and_seeds_only_01_cert(tmp_path):
    src_dataset, src_c1_bulk, workspace = _fixture_roots(tmp_path)

    result = mod.prepare_promoted_clone_workspace(
        source_dataset_root=src_dataset,
        source_c1_bulk_root=src_c1_bulk,
        workspace_root=workspace,
        group_label="test0_transitive_apply_seeded",
    )

    assert (
        workspace / "dataset" / "GRScenes100" / "home" / "SCENE1" / "layout.usd"
    ).exists()
    assert (workspace / "dataset" / "Material" / "mdl" / "example.mdl").exists()
    assert (workspace / "bak").is_dir()
    assert (workspace / "notes").is_dir()
    assert (
        workspace
        / "c1_bulk"
        / "book_bbox_primary_rmse_observe_v1"
        / "01_cert"
        / "filtered_mapping.json"
    ).exists()
    assert not (
        workspace / "c1_bulk" / "book_bbox_primary_rmse_observe_v1" / "02_apply"
    ).exists()
    assert result["seeded_categories"] == ["book"]


def test_prepare_workspace_writes_manifest_and_run_script(tmp_path):
    src_dataset, src_c1_bulk, workspace = _fixture_roots(tmp_path)

    result = mod.prepare_promoted_clone_workspace(
        source_dataset_root=src_dataset,
        source_c1_bulk_root=src_c1_bulk,
        workspace_root=workspace,
        group_label="test0_transitive_apply_seeded",
    )

    manifest = json.loads(
        (workspace / "workspace_manifest.json").read_text(encoding="utf-8")
    )
    run_script = (workspace / "run_promoted_clone.sh").read_text(encoding="utf-8")

    assert manifest["group_label"] == "test0_transitive_apply_seeded"
    assert manifest["seeded_category_count"] == 1
    assert "--step6-mode apply" in run_script
    assert "--bbox-gated" in run_script
    assert str(workspace / "dataset") in run_script
    assert str(workspace / "bak") in run_script
    assert str(workspace / "c1_bulk") in run_script
    assert result["run_script_path"] == str(workspace / "run_promoted_clone.sh")


def test_prepare_workspace_refuses_existing_root(tmp_path):
    src_dataset, src_c1_bulk, workspace = _fixture_roots(tmp_path)
    workspace.mkdir()

    try:
        mod.prepare_promoted_clone_workspace(
            source_dataset_root=src_dataset,
            source_c1_bulk_root=src_c1_bulk,
            workspace_root=workspace,
            group_label="test0_transitive_apply_seeded",
        )
    except FileExistsError as exc:
        assert str(workspace) in str(exc)
    else:
        raise AssertionError("expected FileExistsError")


def test_run_script_uses_apply_mode_and_layout_usd_only(tmp_path):
    src_dataset, src_c1_bulk, workspace = _fixture_roots(tmp_path)

    mod.prepare_promoted_clone_workspace(
        source_dataset_root=src_dataset,
        source_c1_bulk_root=src_c1_bulk,
        workspace_root=workspace,
        group_label="test0_transitive_apply_seeded",
    )

    run_script = (workspace / "run_promoted_clone.sh").read_text(encoding="utf-8")
    assert "--step6-mode apply" in run_script
    assert "--scene-files layout.usd" in run_script
    assert "--dedup-mode geom_only" in run_script


def test_run_script_uses_requested_policy_tag_and_out_version(tmp_path):
    src_dataset = tmp_path / "src_dataset"
    src_c1_bulk = tmp_path / "src_c1_bulk"
    workspace = tmp_path / "workspace"
    reports_root = tmp_path / "v8_prededup"

    _write(src_dataset / "GRScenes100" / "home" / "SCENE1" / "layout.usd", "LAYOUT")
    _write(src_dataset / "Material" / "mdl" / "example.mdl", "mdl")
    cert_root = src_c1_bulk / "book_custom_policy_v2" / "01_cert"
    _write(cert_root / "filtered_mapping.json", json.dumps({"a": "b"}, indent=2))
    _write(
        cert_root / "filtered_mapping.stats.json",
        json.dumps({"mapping_pairs": 1}, indent=2),
    )
    _write(cert_root / "pair_certificates.jsonl", "{}\n")
    _write(cert_root / "certified_graph.json", json.dumps({"components": 1}, indent=2))
    _write(
        reports_root / "union_3way" / "all_categories_union_merged.json",
        json.dumps({"categories": ["book"]}, indent=2),
    )
    _write(reports_root / "mode_summary.json", json.dumps({"ok": True}, indent=2))

    mod.prepare_promoted_clone_workspace(
        source_dataset_root=src_dataset,
        source_c1_bulk_root=src_c1_bulk,
        workspace_root=workspace,
        group_label="test0_transitive_apply_seeded",
        policy_tag="custom_policy",
        out_version="v2",
    )

    run_script = (workspace / "run_promoted_clone.sh").read_text(encoding="utf-8")
    assert "--bbox-policy custom_policy" in run_script
    assert "--out-version v2" in run_script


def test_prepare_workspace_rejects_incomplete_cert_bundle_before_copy(tmp_path):
    src_dataset = tmp_path / "src_dataset"
    src_c1_bulk = tmp_path / "src_c1_bulk"
    workspace = tmp_path / "workspace"
    reports_root = tmp_path / "v8_prededup"

    _write(src_dataset / "GRScenes100" / "home" / "SCENE1" / "layout.usd", "LAYOUT")
    cert_root = src_c1_bulk / "book_bbox_primary_rmse_observe_v1" / "01_cert"
    _write(cert_root / "filtered_mapping.json", json.dumps({"a": "b"}, indent=2))
    _write(
        cert_root / "filtered_mapping.stats.json",
        json.dumps({"mapping_pairs": 1}, indent=2),
    )
    _write(
        reports_root / "union_3way" / "all_categories_union_merged.json",
        json.dumps({"categories": ["book"]}, indent=2),
    )
    _write(reports_root / "mode_summary.json", json.dumps({"ok": True}, indent=2))

    try:
        mod.prepare_promoted_clone_workspace(
            source_dataset_root=src_dataset,
            source_c1_bulk_root=src_c1_bulk,
            workspace_root=workspace,
            group_label="test0_transitive_apply_seeded",
        )
    except ValueError as exc:
        message = str(exc)
        assert "book_bbox_primary_rmse_observe_v1" in message
        assert "pair_certificates.jsonl" in message
        assert not workspace.exists()
    else:
        raise AssertionError("expected ValueError")


def test_prepare_workspace_fails_before_copy_when_inferred_reports_are_missing(
    tmp_path,
):
    src_dataset = tmp_path / "src_dataset"
    src_c1_bulk = tmp_path / "src_c1_bulk"
    workspace = tmp_path / "workspace"

    _write(src_dataset / "GRScenes100" / "home" / "SCENE1" / "layout.usd", "LAYOUT")
    cert_root = src_c1_bulk / "book_bbox_primary_rmse_observe_v1" / "01_cert"
    _write(cert_root / "filtered_mapping.json", json.dumps({"a": "b"}, indent=2))
    _write(
        cert_root / "filtered_mapping.stats.json",
        json.dumps({"mapping_pairs": 1}, indent=2),
    )
    _write(cert_root / "pair_certificates.jsonl", "{}\n")
    _write(cert_root / "certified_graph.json", json.dumps({"components": 1}, indent=2))

    try:
        mod.prepare_promoted_clone_workspace(
            source_dataset_root=src_dataset,
            source_c1_bulk_root=src_c1_bulk,
            workspace_root=workspace,
            group_label="test0_transitive_apply_seeded",
        )
    except FileNotFoundError as exc:
        message = str(exc)
        assert "all_categories_union_merged.json" in message or "v8_prededup" in message
        assert not workspace.exists()
    else:
        raise AssertionError("expected FileNotFoundError")


def test_prepare_workspace_uses_rsync_and_writes_copy_markers(tmp_path, monkeypatch):
    src_dataset, src_c1_bulk, workspace = _fixture_roots(tmp_path)
    calls = []

    def fake_run(cmd, check):
        calls.append((cmd, check))
        assert check is True
        assert cmd[:3] == ["rsync", "-a", "--partial"]
        assert cmd[3] == str(src_dataset) + "/"
        assert cmd[4] == str(workspace / "dataset") + "/"
        shutil.copytree(
            src_dataset, workspace / "dataset", symlinks=True, dirs_exist_ok=True
        )

    monkeypatch.setattr(mod.subprocess, "run", fake_run)

    mod.prepare_promoted_clone_workspace(
        source_dataset_root=src_dataset,
        source_c1_bulk_root=src_c1_bulk,
        workspace_root=workspace,
        group_label="test0_transitive_apply_seeded",
    )

    assert len(calls) == 1
    assert (workspace / mod.COPY_STARTED_MARKER).exists()
    assert (workspace / mod.COPY_COMPLETE_MARKER).exists()


def test_prepare_workspace_resumes_recognized_partial_clone_state(
    tmp_path, monkeypatch
):
    src_dataset, src_c1_bulk, workspace = _fixture_roots(tmp_path)
    workspace.mkdir()
    (workspace / mod.COPY_STARTED_MARKER).write_text("started\n", encoding="utf-8")
    (workspace / "dataset").mkdir()
    _write(
        workspace / "dataset" / "GRScenes100" / "home" / "SCENE1" / "partial.txt",
        "PARTIAL",
    )

    calls = []

    def fake_run(cmd, check):
        calls.append(cmd)
        shutil.copytree(
            src_dataset, workspace / "dataset", symlinks=True, dirs_exist_ok=True
        )

    monkeypatch.setattr(mod.subprocess, "run", fake_run)

    result = mod.prepare_promoted_clone_workspace(
        source_dataset_root=src_dataset,
        source_c1_bulk_root=src_c1_bulk,
        workspace_root=workspace,
        group_label="test0_transitive_apply_seeded",
    )

    assert len(calls) == 1
    assert (workspace / mod.COPY_COMPLETE_MARKER).exists()
    assert (workspace / "bak").is_dir()
    assert result["seeded_categories"] == ["book"]


def test_prepare_workspace_rejects_unrecognized_partial_workspace(tmp_path):
    src_dataset, src_c1_bulk, workspace = _fixture_roots(tmp_path)
    workspace.mkdir()
    (workspace / "dataset").mkdir()

    try:
        mod.prepare_promoted_clone_workspace(
            source_dataset_root=src_dataset,
            source_c1_bulk_root=src_c1_bulk,
            workspace_root=workspace,
            group_label="test0_transitive_apply_seeded",
        )
    except FileExistsError as exc:
        assert str(workspace) in str(exc)
    else:
        raise AssertionError("expected FileExistsError")


def test_prepare_workspace_rejects_partial_clone_with_extra_root_artifact(tmp_path):
    src_dataset, src_c1_bulk, workspace = _fixture_roots(tmp_path)
    workspace.mkdir()
    (workspace / mod.COPY_STARTED_MARKER).write_text("started\n", encoding="utf-8")
    (workspace / "dataset").mkdir()
    (workspace / "unexpected.txt").write_text("noise\n", encoding="utf-8")

    try:
        mod.prepare_promoted_clone_workspace(
            source_dataset_root=src_dataset,
            source_c1_bulk_root=src_c1_bulk,
            workspace_root=workspace,
            group_label="test0_transitive_apply_seeded",
        )
    except FileExistsError as exc:
        assert str(workspace) in str(exc)
    else:
        raise AssertionError("expected FileExistsError")
