#!/usr/bin/env python3
"""Unit tests for scripts/test0_rebuilt_normalize.py."""

import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "test0_rebuilt_normalize.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "test0_rebuilt_normalize",
        SCRIPT_PATH,
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


mod = _load_module()


def test_parse_dlc_table_extracts_named_rows():
    output = """
+----------------------+------------------+-----------+
|         Name         |      JobId       | JobStatus |
+----------------------+------------------+-----------+
| norm_p1_run_wall_0_1 | dlcjob-wall      | Running   |
| norm_p1_run_floor_0_1| dlcjob-floor     | Succeeded |
+----------------------+------------------+-----------+
"""
    rows = mod._parse_dlc_table(output)

    assert len(rows) == 2
    assert rows[0]["Name"] == "norm_p1_run_wall_0_1"
    assert rows[0]["JobId"] == "dlcjob-wall"
    assert rows[0]["JobStatus"] == "Running"
    assert rows[1]["JobStatus"] == "Succeeded"


def test_phase1_display_name_matches_launch_job_shape():
    assert mod._phase1_display_name("run123", "wall") == "norm_p1_run123_wall_0_1"


def test_collect_inventory_counts_categories_and_scene_split(tmp_path):
    root = tmp_path / "GRScenes-test0-rebuilt"
    (root / "GRScenes_assets" / "wall").mkdir(parents=True)
    (root / "GRScenes_assets" / "chair").mkdir(parents=True)
    (root / "GRScenes100" / "home" / "A_usd").mkdir(parents=True)
    (root / "GRScenes100" / "home" / "B_usd").mkdir(parents=True)
    (root / "GRScenes100" / "commercial" / "C_usd").mkdir(parents=True)

    inventory = mod._collect_inventory(root)

    assert inventory == {
        "category_count": 2,
        "scene_count": 3,
        "home_scene_count": 2,
        "commercial_scene_count": 1,
    }


def test_resolve_main_usd_prefers_normalized_layout_and_falls_back_to_flat(tmp_path):
    root = tmp_path / "dataset"
    normalized = root / "GRScenes_assets" / "wall" / "uid1" / "usd" / "uid1.usd"
    normalized.parent.mkdir(parents=True)
    normalized.write_text("#usda 1.0\n")

    flat = root / "GRScenes_assets" / "chair" / "uid2" / "uid2.usd"
    flat.parent.mkdir(parents=True)
    flat.write_text("#usda 1.0\n")

    assert mod._resolve_main_usd(root, "wall/uid1") == normalized
    assert mod._resolve_main_usd(root, "chair/uid2") == flat


def test_compare_hashes_detects_changed_and_missing_files(tmp_path):
    keep = tmp_path / "keep.usd"
    change = tmp_path / "change.usd"
    missing = tmp_path / "missing.usd"
    keep.write_text("keep-v1\n")
    change.write_text("change-v1\n")
    missing.write_text("gone-v1\n")

    baseline = mod._capture_hashes([keep, change, missing])

    change.write_text("change-v2\n")
    missing.unlink()

    result = mod._compare_hashes(baseline)

    assert result["checked_count"] == 3
    assert result["changed_count"] == 1
    assert result["missing_count"] == 1
    assert result["changed"][0]["path"] == str(change)
    assert result["missing"] == [str(missing)]


def test_count_output_main_usd_symlinks_only_flags_primary_usds(tmp_path):
    output_root = tmp_path / "normalized"
    real_main = output_root / "real_main.usd"
    real_main.parent.mkdir(parents=True)
    real_main.write_text("#usda 1.0\n")

    main_symlink = output_root / "GRScenes_assets" / "wall" / "uid1" / "usd" / "uid1.usd"
    main_symlink.parent.mkdir(parents=True)
    main_symlink.symlink_to(real_main)

    aux = output_root / "GRScenes_assets" / "wall" / "uid1" / "usd" / "preview.usda"
    aux.write_text("#usda 1.0\n")

    regular_main = output_root / "GRScenes_assets" / "chair" / "uid2" / "usd" / "uid2.usd"
    regular_main.parent.mkdir(parents=True)
    regular_main.write_text("#usda 1.0\n")

    report = mod._count_output_main_usd_symlinks(output_root)

    assert report["checked_main_usd_count"] == 2
    assert report["symlink_main_usd_count"] == 1
    assert report["symlink_main_usd_preview"] == [str(main_symlink)]
