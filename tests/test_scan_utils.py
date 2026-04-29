"""Tests for scan_utils module (pure-Python functions, no pxr needed)."""
import json
import tempfile
import os
from pathlib import Path

import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from scan_utils import (
    _parse_uid_from_report_style_asset_usd,
    _abs_from_usd_ref,
    _abs_to_subset_rel,
    _normalize_mapping_key,
    _iter_usd_files,
    build_old_asset_path_set,
    build_combined_old_asset_path_set,
)


class TestParseUid:
    def test_standard_path(self):
        uid = _parse_uid_from_report_style_asset_usd(
            "GRScenes_assets/chair/abc123/usd/abc123.usd"
        )
        assert uid == "abc123"

    def test_absolute_path(self):
        uid = _parse_uid_from_report_style_asset_usd(
            "/root/dataset/GRScenes_assets/chair/xyz789/usd/xyz789.usd"
        )
        assert uid == "xyz789"

    def test_non_asset_path(self):
        uid = _parse_uid_from_report_style_asset_usd("/some/other/path.usd")
        assert uid is None


class TestAbsFromUsdRef:
    def test_relative_ref(self):
        result = _abs_from_usd_ref("/base/scene", "./asset.usd")
        assert result.endswith("/base/scene/asset.usd")

    def test_absolute_ref(self):
        result = _abs_from_usd_ref("/base", "/absolute/path.usd")
        assert result == os.path.abspath("/absolute/path.usd")

    def test_sdf_wrapped_ref(self):
        result = _abs_from_usd_ref("/base", "@/absolute/path.usd@")
        assert result == os.path.abspath("/absolute/path.usd")

    def test_empty(self):
        assert _abs_from_usd_ref("/base", "") == ""


class TestAbsToSubsetRel:
    def test_with_dataset_name(self):
        rel = _abs_to_subset_rel(
            "/root/dataset/GRScenes_assets/chair/uid/usd/uid.usd",
            "dataset",
        )
        assert rel == "GRScenes_assets/chair/uid/usd/uid.usd"

    def test_without_dataset_name(self):
        rel = _abs_to_subset_rel(
            "/root/GRScenes_assets/chair/uid/usd/uid.usd",
            "other",
        )
        assert rel == "GRScenes_assets/chair/uid/usd/uid.usd"

    def test_no_match(self):
        assert _abs_to_subset_rel("/some/random/path", "dataset") is None


class TestIterUsdFiles:
    def test_excludes_pre_files(self, tmp_path):
        (tmp_path / "scene").mkdir()
        (tmp_path / "scene" / "layout.usd").write_text("x")
        (tmp_path / "scene" / "layout.pre_backup.usd").write_text("x")
        files = _iter_usd_files(tmp_path, exclude_dir_contains=[])
        names = {f.name for f in files}
        assert "layout.usd" in names
        assert "layout.pre_backup.usd" not in names

    def test_excludes_c1_files(self, tmp_path):
        (tmp_path / "scene").mkdir()
        (tmp_path / "scene" / "layout.usd").write_text("x")
        (tmp_path / "scene" / "layout.c1_test.usd").write_text("x")
        files = _iter_usd_files(tmp_path, exclude_dir_contains=[])
        names = {f.name for f in files}
        assert "layout.usd" in names
        assert "layout.c1_test.usd" not in names

    def test_excludes_parallel_files(self, tmp_path):
        (tmp_path / "scene").mkdir()
        (tmp_path / "scene" / "layout.usd").write_text("x")
        (tmp_path / "scene" / "layout.parallel_test.usd").write_text("x")
        files = _iter_usd_files(tmp_path, exclude_dir_contains=[])
        names = {f.name for f in files}
        assert "layout.usd" in names
        assert "layout.parallel_test.usd" not in names

    def test_excludes_dirs(self, tmp_path):
        (tmp_path / "scene").mkdir()
        (tmp_path / "bak").mkdir()
        (tmp_path / "bak" / "_dedup_assets" / "stuff").mkdir(parents=True)
        (tmp_path / "scene" / "layout.usd").write_text("x")
        (tmp_path / "bak" / "_dedup_assets" / "stuff" / "stale.usd").write_text("x")
        files = _iter_usd_files(tmp_path, exclude_dir_contains=["/_dedup_assets/"])
        names = {f.name for f in files}
        assert "layout.usd" in names
        assert "stale.usd" not in names


class TestBuildOldAssetPathSet:
    def test_build_from_mapping(self, tmp_path):
        mapping = tmp_path / "mapping.json"
        mapping.write_text(json.dumps({
            "GRScenes_assets/chair/old_uid/usd/old_uid.usd": "GRScenes_assets/chair/new_uid/usd/new_uid.usd",
            "GRScenes_assets/chair/old2/usd/old2.usd": "GRScenes_assets/chair/new_uid/usd/new_uid.usd",
        }))
        result = build_old_asset_path_set(str(mapping), "dataset")
        assert "GRScenes_assets/chair/old_uid/usd/old_uid.usd" in result
        assert "GRScenes_assets/chair/old2/usd/old2.usd" in result

    def test_combined_mappings(self, tmp_path):
        m1 = tmp_path / "m1.json"
        m1.write_text(json.dumps({
            "GRScenes_assets/chair/a/usd/a.usd": "x",
        }))
        m2 = tmp_path / "m2.json"
        m2.write_text(json.dumps({
            "GRScenes_assets/table/b/usd/b.usd": "y",
        }))
        result = build_combined_old_asset_path_set(
            [str(m1), str(m2)], "dataset"
        )
        assert "GRScenes_assets/chair/a/usd/a.usd" in result
        assert "GRScenes_assets/table/b/usd/b.usd" in result
        assert len(result) == 2
