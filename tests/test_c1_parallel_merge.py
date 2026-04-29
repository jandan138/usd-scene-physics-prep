"""Tests for c1_parallel_merge module."""
import json
import tempfile
from pathlib import Path
import sys, os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "scripts"))
from c1_parallel_merge import (
    load_filtered_mapping,
    merge_category_mappings,
    discover_category_mappings,
    gate_check_phase1,
)


class TestLoadFilteredMapping:
    def test_dict_format(self, tmp_path):
        p = tmp_path / "m.json"
        p.write_text(json.dumps({"old": "canonical", "old2": "canonical2"}))
        result = load_filtered_mapping(p)
        assert result == {"old": "canonical", "old2": "canonical2"}

    def test_list_format(self, tmp_path):
        p = tmp_path / "m.json"
        p.write_text(json.dumps([
            {"old": "a", "canonical": "x"},
            {"old": "b", "canonical": "y"},
        ]))
        result = load_filtered_mapping(p)
        assert result == {"a": "x", "b": "y"}


class TestMergeCategoryMappings:
    def test_disjoint_mappings(self):
        mappings = [
            ("book", {"book_old": "book_canonical"}),
            ("chair", {"chair_old": "chair_canonical"}),
        ]
        combined, conflicts = merge_category_mappings(mappings)
        assert len(combined) == 2
        assert combined["book_old"] == "book_canonical"
        assert combined["chair_old"] == "chair_canonical"
        assert len(conflicts) == 0

    def test_same_target_overlap_benign(self):
        mappings = [
            ("cat1", {"old": "canonical"}),
            ("cat2", {"old": "canonical"}),
        ]
        combined, conflicts = merge_category_mappings(mappings)
        assert len(combined) == 1
        assert combined["old"] == "canonical"
        assert len(conflicts) == 0

    def test_different_target_conflict(self):
        mappings = [
            ("cat1", {"old": "canonical_A"}),
            ("cat2", {"old": "canonical_B"}),
        ]
        combined, conflicts = merge_category_mappings(mappings)
        assert len(conflicts) == 1
        assert conflicts[0]["old_key"] == "old"
        assert conflicts[0]["existing"]["target"] == "canonical_A"
        assert conflicts[0]["conflicting"]["target"] == "canonical_B"

    def test_empty_mappings(self):
        combined, conflicts = merge_category_mappings([])
        assert len(combined) == 0
        assert len(conflicts) == 0

    def test_idempotent(self):
        mappings = [("cat", {"a": "b", "c": "d"})]
        c1, _ = merge_category_mappings(mappings)
        c2, _ = merge_category_mappings(mappings)
        assert c1 == c2

    def test_many_categories(self):
        mappings = [(f"cat{i}", {f"old_{i}": f"new_{i}"}) for i in range(100)]
        combined, conflicts = merge_category_mappings(mappings)
        assert len(combined) == 100
        assert len(conflicts) == 0


class TestGateCheckPhase1:
    def test_all_pass(self, tmp_path):
        c1_bulk = tmp_path / "c1_bulk"
        for cat in ["book", "chair"]:
            d = c1_bulk / f"{cat}_bbox_primary_rmse_observe_v1"
            d.mkdir(parents=True)
            (d / "phase1_done.json").write_text(
                json.dumps({"status": "ok", "audit_passed": True})
            )
        ok, failed = gate_check_phase1(
            c1_bulk, "bbox_primary_rmse_observe", "v1", ["book", "chair"]
        )
        assert ok
        assert len(failed) == 0

    def test_missing_file(self, tmp_path):
        c1_bulk = tmp_path / "c1_bulk"
        ok, failed = gate_check_phase1(
            c1_bulk, "bbox_primary_rmse_observe", "v1", ["book"]
        )
        assert not ok
        assert "missing phase1_done.json" in failed[0]

    def test_failed_status(self, tmp_path):
        c1_bulk = tmp_path / "c1_bulk"
        d = c1_bulk / "book_bbox_primary_rmse_observe_v1"
        d.mkdir(parents=True)
        (d / "phase1_done.json").write_text(
            json.dumps({"status": "failed", "audit_passed": False})
        )
        ok, failed = gate_check_phase1(
            c1_bulk, "bbox_primary_rmse_observe", "v1", ["book"]
        )
        assert not ok
        assert "status=failed" in failed[0]

    def test_audit_not_passed(self, tmp_path):
        c1_bulk = tmp_path / "c1_bulk"
        d = c1_bulk / "book_bbox_primary_rmse_observe_v1"
        d.mkdir(parents=True)
        (d / "phase1_done.json").write_text(
            json.dumps({"status": "ok", "audit_passed": False})
        )
        ok, failed = gate_check_phase1(
            c1_bulk, "bbox_primary_rmse_observe", "v1", ["book"]
        )
        assert not ok
        assert "audit not passed" in failed[0]
