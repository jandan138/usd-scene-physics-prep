import json
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))

import export_transitive_full_baseline_dataset as mod


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def _fixture_roots(tmp_path: Path):
    dataset_root = tmp_path / "dataset"
    rerun_root = tmp_path / "full_rerun"
    output_root = tmp_path / "delivery"

    _write(dataset_root / "Material" / "mdl" / "example.mdl", "mdl")
    _write(
        dataset_root / "GRScenes_assets" / "bottle" / "keep_a" / "usd" / "keep_a.usd",
        "#usda 1.0\n",
    )
    _write(
        dataset_root / "GRScenes_assets" / "bottle" / "keep_b" / "usd" / "keep_b.usd",
        "#usda 1.0\n",
    )
    _write(
        dataset_root / "GRScenes_assets" / "bottle" / "drop_c" / "usd" / "drop_c.usd",
        "#usda 1.0\n",
    )
    _write(dataset_root / "GRScenes100" / "home" / "SCENE1" / "layout.usd", "ORIGINAL")
    _write(dataset_root / "GRScenes100" / "home" / "SCENE2" / "layout.usd", "ORIGINAL")

    suffix = "layout.c1_v8_tier2_rollout_transitive_full_dlc_bbox_primary_rmse_observe_v1.usd"
    _write(dataset_root / "GRScenes100" / "home" / "SCENE1" / suffix, "FINAL_SCENE1")
    _write(dataset_root / "GRScenes100" / "home" / "SCENE2" / suffix, "FINAL_SCENE2")
    _write(
        rerun_root
        / "bottle_bbox_primary_rmse_observe_v1"
        / "01_cert"
        / "filtered_mapping.stats.json",
        json.dumps({"candidate_count": 10, "retained_count": 3}),
    )
    return dataset_root, rerun_root, output_root, suffix


def test_export_uses_final_rerun_layouts_as_layout_usd(tmp_path, monkeypatch):
    dataset_root, rerun_root, output_root, suffix = _fixture_roots(tmp_path)
    monkeypatch.setattr(
        mod,
        "collect_layout_asset_refs",
        lambda path: {
            "GRScenes_assets/bottle/keep_a/usd/keep_a.usd",
            "GRScenes_assets/bottle/keep_b/usd/keep_b.usd",
        },
    )

    mod.export_transitive_full_baseline_dataset(
        dataset_root=dataset_root,
        full_rerun_root=rerun_root,
        output_root=output_root,
        rerun_layout_name=suffix,
        full_rerun_job_id="job-123",
    )

    assert (
        output_root / "GRScenes100" / "home" / "SCENE1" / "layout.usd"
    ).read_text() == "FINAL_SCENE1"
    assert (
        output_root / "GRScenes100" / "home" / "SCENE2" / "layout.usd"
    ).read_text() == "FINAL_SCENE2"


def test_export_keeps_only_referenced_assets(tmp_path, monkeypatch):
    dataset_root, rerun_root, output_root, suffix = _fixture_roots(tmp_path)
    refs = {
        dataset_root / "GRScenes100" / "home" / "SCENE1" / suffix: {
            "GRScenes_assets/bottle/keep_a/usd/keep_a.usd",
        },
        dataset_root / "GRScenes100" / "home" / "SCENE2" / suffix: {
            "GRScenes_assets/bottle/keep_b/usd/keep_b.usd",
        },
    }
    monkeypatch.setattr(mod, "collect_layout_asset_refs", lambda path: refs[Path(path)])

    mod.export_transitive_full_baseline_dataset(
        dataset_root=dataset_root,
        full_rerun_root=rerun_root,
        output_root=output_root,
        rerun_layout_name=suffix,
        full_rerun_job_id="job-123",
    )

    assert (
        output_root / "GRScenes_assets" / "bottle" / "keep_a" / "usd" / "keep_a.usd"
    ).exists()
    assert (
        output_root / "GRScenes_assets" / "bottle" / "keep_b" / "usd" / "keep_b.usd"
    ).exists()
    assert not (output_root / "GRScenes_assets" / "bottle" / "drop_c").exists()


def test_export_writes_manifest_summary_readme_and_dangling_refs(tmp_path, monkeypatch):
    dataset_root, rerun_root, output_root, suffix = _fixture_roots(tmp_path)
    monkeypatch.setattr(
        mod,
        "collect_layout_asset_refs",
        lambda path: {
            "GRScenes_assets/bottle/keep_a/usd/keep_a.usd",
            "GRScenes_assets/bottle/missing_x/usd/missing_x.usd",
        },
    )

    mod.export_transitive_full_baseline_dataset(
        dataset_root=dataset_root,
        full_rerun_root=rerun_root,
        output_root=output_root,
        rerun_layout_name=suffix,
        full_rerun_job_id="job-123",
    )

    manifest = json.loads((output_root / "MANIFEST.json").read_text())
    dangling = json.loads((output_root / "dangling_references.json").read_text())
    summary = json.loads((output_root / "asset_pruning_summary.json").read_text())
    readme = (output_root / "README.md").read_text()

    assert manifest["full_rerun_job_id"] == "job-123"
    assert manifest["dangling_reference_count"] == 1
    assert manifest["full_rerun_metrics"]["stats_file_count"] == 1
    assert dangling["dangling_reference_count"] == 1
    assert summary["retained_asset_count"] == 1
    assert summary["omitted_asset_count"] == 2
    assert "transitive-capable full rerun" in readme


def test_export_requires_fresh_output_root(tmp_path, monkeypatch):
    dataset_root, rerun_root, output_root, suffix = _fixture_roots(tmp_path)
    output_root.mkdir()
    monkeypatch.setattr(mod, "collect_layout_asset_refs", lambda path: set())

    with pytest.raises(RuntimeError, match="Output root already exists"):
        mod.export_transitive_full_baseline_dataset(
            dataset_root=dataset_root,
            full_rerun_root=rerun_root,
            output_root=output_root,
            rerun_layout_name=suffix,
            full_rerun_job_id="job-123",
        )


def test_collect_layout_asset_refs_falls_back_without_pxr(tmp_path):
    layout_path = tmp_path / "layout.usda"
    _write(
        layout_path,
        """#usda 1.0
def Xform \"Root\" (
    prepend references = [
        @GRScenes_assets/bottle/keep_a/usd/keep_a.usd@,
        @./local.usd@
    ]
)
{
    def Xform \"Child\" (
        prepend references = @GRScenes_assets/chair/keep_b/usd/keep_b.usd@
    )
    {
    }
}
""",
    )

    assert mod.collect_layout_asset_refs(layout_path) == {
        "GRScenes_assets/bottle/keep_a/usd/keep_a.usd",
        "GRScenes_assets/chair/keep_b/usd/keep_b.usd",
    }


def test_cli_reexecs_with_isaac_wrapper_when_pxr_missing(monkeypatch, tmp_path):
    calls = []
    script_path = tmp_path / "scripts" / "export_transitive_full_baseline_dataset.py"
    wrapper_path = tmp_path / "scripts" / "isaac_python.sh"
    _write(script_path, "#!/usr/bin/env python3\n")
    _write(wrapper_path, "#!/usr/bin/env bash\n")

    monkeypatch.setattr(mod, "_can_import_pxr", lambda: False)
    monkeypatch.setattr(
        mod.subprocess,
        "run",
        lambda cmd, check, env: (
            calls.append((cmd, check, env.copy())) or SimpleNamespace(returncode=0)
        ),
    )

    exit_code = mod._maybe_reexec_with_isaac_python(
        ["--dataset-root", "dataset"],
        script_path=script_path,
        wrapper_path=wrapper_path,
    )

    assert exit_code == 0
    assert calls[0][0] == [
        str(wrapper_path),
        str(script_path),
        "--dataset-root",
        "dataset",
    ]
    assert calls[0][2]["EXPORT_TRANSITIVE_FULL_BASELINE_UNDER_ISAAC"] == "1"


def test_cli_reexecs_with_isaac_wrapper_when_pxr_importerror(monkeypatch, tmp_path):
    calls = []
    script_path = tmp_path / "scripts" / "export_transitive_full_baseline_dataset.py"
    wrapper_path = tmp_path / "scripts" / "isaac_python.sh"
    _write(script_path, "#!/usr/bin/env python3\n")
    _write(wrapper_path, "#!/usr/bin/env bash\n")

    monkeypatch.setattr(
        mod,
        "_can_import_pxr",
        lambda: (_ for _ in ()).throw(ImportError("libusd.so missing")),
    )
    monkeypatch.setattr(
        mod.subprocess,
        "run",
        lambda cmd, check, env: (
            calls.append((cmd, check, env.copy())) or SimpleNamespace(returncode=0)
        ),
    )

    exit_code = mod._maybe_reexec_with_isaac_python(
        ["--dataset-root", "dataset"],
        script_path=script_path,
        wrapper_path=wrapper_path,
    )

    assert exit_code == 0
    assert calls[0][0][0] == str(wrapper_path)


def test_normalize_asset_ref_rewrites_absolute_dataset_path(tmp_path):
    dataset_root = tmp_path / "dataset"
    layout_path = dataset_root / "GRScenes100" / "home" / "SCENE1" / "layout.usd"
    asset_path = (
        dataset_root / "GRScenes_assets" / "bottle" / "keep_a" / "usd" / "keep_a.usd"
    )

    assert (
        mod._normalize_asset_ref(layout_path, str(asset_path))
        == "GRScenes_assets/bottle/keep_a/usd/keep_a.usd"
    )


def test_normalize_asset_ref_rewrites_relative_grscenes_assets_path(tmp_path):
    layout_path = tmp_path / "GRScenes100" / "home" / "SCENE1" / "layout.usd"

    assert (
        mod._normalize_asset_ref(
            layout_path,
            "../../../GRScenes_assets/bottle/keep_a/usd/keep_a.usd",
        )
        == "GRScenes_assets/bottle/keep_a/usd/keep_a.usd"
    )


def test_export_records_dangling_when_asset_file_is_missing(tmp_path, monkeypatch):
    dataset_root, rerun_root, output_root, suffix = _fixture_roots(tmp_path)
    monkeypatch.setattr(
        mod,
        "collect_layout_asset_refs",
        lambda path: {"GRScenes_assets/bottle/keep_a/usd/missing_leaf.usd"},
    )

    mod.export_transitive_full_baseline_dataset(
        dataset_root=dataset_root,
        full_rerun_root=rerun_root,
        output_root=output_root,
        rerun_layout_name=suffix,
        full_rerun_job_id="job-123",
    )

    manifest = json.loads((output_root / "MANIFEST.json").read_text())
    dangling = json.loads((output_root / "dangling_references.json").read_text())
    summary = json.loads((output_root / "asset_pruning_summary.json").read_text())

    assert manifest["retained_asset_count"] == 0
    assert manifest["dangling_reference_count"] == 1
    assert dangling["dangling_references"] == [
        "GRScenes_assets/bottle/keep_a/usd/missing_leaf.usd"
    ]
    assert summary["retained_asset_count"] == 0
    assert summary["per_category"]["bottle"] == {"retained": 0, "omitted": 3}
