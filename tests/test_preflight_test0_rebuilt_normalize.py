#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "preflight_test0_rebuilt_normalize.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "preflight_test0_rebuilt_normalize", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


mod = _load_module()


def _make_source_root(tmp_path: Path) -> Path:
    root = tmp_path / "GRScenes-test0-rebuilt"
    (root / "GRScenes_assets" / "wall" / "abc" / "usd").mkdir(parents=True)
    (root / "GRScenes_assets" / "ground" / "def" / "usd").mkdir(parents=True)
    (root / "GRScenes_assets" / "other" / "ghi" / "usd").mkdir(parents=True)
    (root / "GRScenes_assets" / "cabinet" / "jkl" / "usd").mkdir(parents=True)
    for category, uid in (("wall", "abc"), ("ground", "def"), ("other", "ghi"), ("cabinet", "jkl")):
        (root / "GRScenes_assets" / category / uid / "usd" / f"{uid}.usd").write_text("#usda 1.0\n")
    (root / "GRScenes100" / "home" / "scene_a").mkdir(parents=True)
    (root / "GRScenes100" / "commercial" / "scene_b").mkdir(parents=True)
    (root / "Material" / "mdl").mkdir(parents=True)
    return root


def test_find_main_usd_prefers_normalized_layout(tmp_path):
    root = _make_source_root(tmp_path)
    path = mod._find_main_usd(root, "wall", "abc")
    assert path.name == "abc.usd"
    assert "usd" in path.parts


def test_run_preflight_fails_when_output_exists(tmp_path, monkeypatch):
    root = _make_source_root(tmp_path)
    out_root = tmp_path / "out"
    out_root.mkdir()
    report_root = tmp_path / "report"

    monkeypatch.setattr(
        mod,
        "DEFAULT_SAMPLE_ASSETS",
        (
            {"category": "wall", "uid": "abc"},
            {"category": "ground", "uid": "def"},
            {"category": "other", "uid": "ghi"},
            {"category": "cabinet", "uid": "jkl"},
        ),
    )
    monkeypatch.setattr(
        mod,
        "EXPECTED_INVENTORY",
        {
            "category_count": 4,
            "scene_count": 2,
            "home_scene_count": 1,
            "commercial_scene_count": 1,
        },
    )

    report = mod.run_preflight(
        source_root=root,
        normalized_root=out_root,
        report_root=report_root,
        compare_root=None,
        sample_assets=mod.DEFAULT_SAMPLE_ASSETS,
        compute_centers=False,
    )
    assert report["status"] == "fail"
    assert any("normalized root must not exist" in item for item in report["failures"])


def test_sample_asset_report_records_compare_root(tmp_path):
    source_root = _make_source_root(tmp_path / "source")
    compare_root = _make_source_root(tmp_path / "compare")
    sample_assets = [{"category": "wall", "uid": "abc"}]

    reports = mod._sample_asset_report(
        source_root=source_root,
        compare_root=compare_root,
        sample_assets=sample_assets,
        compute_centers=False,
        compute_asset_center=None,
    )
    assert len(reports) == 1
    assert reports[0]["compare_root"]["is_file"] is True
    assert "sha256" in reports[0]["compare_root"]
