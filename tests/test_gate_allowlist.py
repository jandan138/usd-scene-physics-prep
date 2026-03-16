#!/usr/bin/env python3
"""Tests for allowlist-aware gate logic in normalize pipeline.

Tests cover:
- compute_gate_verdict (check_normalize_gate_from_reports.py): max_missing_centers param
- build_verdict (build_normalize_gate_verdict.py): allowlist-aware center gap logic
- Allowlist parsing helpers: extracting known-bad counts from allowlist_verdict.json
"""

from __future__ import annotations

import os
import sys

import pytest

# Ensure project root is on sys.path so we can import from scripts/
_PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from scripts.check_normalize_gate_from_reports import compute_gate_verdict
from scripts.build_normalize_gate_verdict import build_verdict


# ---------------------------------------------------------------------------
# Helpers to build minimal synthetic data
# ---------------------------------------------------------------------------

def _make_pairwise(displaced_gt: int = 0, ref_same_gt: int = 0):
    return {
        "aggregate": {
            "displaced_breakdown": {"gt_0.01": displaced_gt},
            "ref_same_breakdown": {"gt_0.01": ref_same_gt},
            "global_top_50_worst": [],
        }
    }


def _make_audit(
    common_ref: int = 1000,
    center_found: int = 1000,
    matrix_mismatch: int = 0,
    inert_descendant: int = 0,
    became_inert: int = 0,
    missing_pre_c1: list | None = None,
):
    return {
        "aggregate": {
            "totals": {
                "common_ref_prim_count": common_ref,
                "center_found": center_found,
                "matrix_mismatch": matrix_mismatch,
                "pre_c1_inert_descendant_xform_override_count": inert_descendant,
                "became_inert_descendant_xform_override_count": became_inert,
            },
            "top_50_inert_descendant_xform_overrides": [],
        },
        "missing_pre_c1_scenes": missing_pre_c1 or [],
    }


# Realistic allowlist_verdict.json structure for testing extraction logic.
_SAMPLE_ALLOWLIST = {
    "default_allowlist": {
        "allowed_nonzero_exit_codes": {
            "book": [1],
            "cabinet": [1],
            "other": [1],
            "person": [1],
            "door_UUID1": [1],
            "door_UUID2": [1],
        },
        "allowed_asset_errors": {
            "book": {
                "book/aaaa1111/usd/aaaa1111.usd": ["No center computed"],
                "book/aaaa2222/usd/aaaa2222.usd": ["No center computed"],
                "book/aaaa3333/usd/aaaa3333.usd": ["No center computed"],
            },
            "cabinet": {
                "cabinet/b98d6f3a/usd/b98d6f3a.usd": ["No meshes found"],
            },
            "other": {
                "other/d41d8cd9/usd/d41d8cd9.usd": ["No meshes found"],
            },
            "person": {
                "person/35131a3c/usd/35131a3c.usd": ["No meshes found"],
            },
            "door_UUID1": {
                "door_UUID1/d41d8cd9/usd/d41d8cd9.usd": ["No meshes found"],
            },
            "door_UUID2": {
                "door_UUID2/d41d8cd9/usd/d41d8cd9.usd": ["No meshes found"],
            },
        },
    },
}


def _count_allowed_asset_errors(allowlist: dict) -> int:
    """Count total unique asset paths across all categories in allowed_asset_errors."""
    errors = allowlist.get("default_allowlist", {}).get("allowed_asset_errors", {})
    return sum(len(assets) for assets in errors.values())


def _collect_allowed_asset_keys(allowlist: dict) -> set:
    """Collect all asset path keys across all categories in allowed_asset_errors."""
    errors = allowlist.get("default_allowlist", {}).get("allowed_asset_errors", {})
    keys = set()
    for assets in errors.values():
        keys.update(assets.keys())
    return keys


# ===================================================================
# Tests for compute_gate_verdict (check_normalize_gate_from_reports.py)
# ===================================================================


class TestComputeGateVerdictCenterGap:
    """Tests for the max_missing_centers parameter in compute_gate_verdict."""

    def test_no_max_missing_strict(self):
        """Default max_missing_centers=0: any center gap => fail."""
        pairwise = _make_pairwise()
        audit = _make_audit(common_ref=1000, center_found=954)
        verdict = compute_gate_verdict(pairwise, audit, max_missing_centers=0)

        assert verdict["passed"] is False
        assert verdict["checks"]["center_gap_within_allowance"] is False
        assert "center_gap=46" in verdict["reason"]

    def test_max_missing_covers_gap(self):
        """max_missing_centers=50 covers gap=46 => center check passes."""
        pairwise = _make_pairwise()
        audit = _make_audit(common_ref=1000, center_found=954)
        verdict = compute_gate_verdict(pairwise, audit, max_missing_centers=50)

        assert verdict["passed"] is True
        assert verdict["checks"]["center_gap_within_allowance"] is True

    def test_max_missing_exact_boundary(self):
        """max_missing_centers exactly equals gap => passes."""
        pairwise = _make_pairwise()
        audit = _make_audit(common_ref=1000, center_found=954)
        verdict = compute_gate_verdict(pairwise, audit, max_missing_centers=46)

        assert verdict["passed"] is True
        assert verdict["checks"]["center_gap_within_allowance"] is True

    def test_max_missing_partial(self):
        """max_missing_centers=30 doesn't cover gap=46 => fail."""
        pairwise = _make_pairwise()
        audit = _make_audit(common_ref=1000, center_found=954)
        verdict = compute_gate_verdict(pairwise, audit, max_missing_centers=30)

        assert verdict["passed"] is False
        assert verdict["checks"]["center_gap_within_allowance"] is False
        assert "center_gap=46 > max_missing_centers=30" in verdict["reason"]

    def test_zero_gap_always_passes(self):
        """center_found == common_ref => passes regardless of max_missing."""
        pairwise = _make_pairwise()
        audit = _make_audit(common_ref=1000, center_found=1000)

        # With max_missing_centers=0 (strict)
        verdict0 = compute_gate_verdict(pairwise, audit, max_missing_centers=0)
        assert verdict0["passed"] is True
        assert verdict0["checks"]["center_gap_within_allowance"] is True

        # With max_missing_centers=50
        verdict50 = compute_gate_verdict(pairwise, audit, max_missing_centers=50)
        assert verdict50["passed"] is True
        assert verdict50["checks"]["center_gap_within_allowance"] is True

    def test_all_checks_pass_clean_data(self):
        """All zero/matching metrics => pass."""
        pairwise = _make_pairwise()
        audit = _make_audit()
        verdict = compute_gate_verdict(pairwise, audit)

        assert verdict["passed"] is True
        assert verdict["status"] == "pass_normalize_gate"
        assert all(verdict["checks"].values())
        assert verdict["reason"] == "ok"

    def test_displaced_failure_independent_of_center(self):
        """Displacement failure even when center gap is within allowance."""
        pairwise = _make_pairwise(displaced_gt=100)
        audit = _make_audit(common_ref=1000, center_found=954)
        verdict = compute_gate_verdict(pairwise, audit, max_missing_centers=50)

        assert verdict["passed"] is False
        assert verdict["checks"]["center_gap_within_allowance"] is True
        assert verdict["checks"]["pairwise_displaced_gt_0_01_ok"] is False

    def test_metrics_include_max_missing_and_skipped(self):
        """Verify metrics dict contains max_missing_centers and skipped_missing_center."""
        pairwise = _make_pairwise(displaced_gt=5)
        audit = _make_audit(common_ref=2000, center_found=1950, matrix_mismatch=3)
        verdict = compute_gate_verdict(pairwise, audit, max_missing_centers=100)

        m = verdict["metrics"]
        assert m["pairwise_displaced_gt_0.01"] == 5
        assert m["common_ref_prim_count"] == 2000
        assert m["center_found"] == 1950
        assert m["matrix_mismatch"] == 3
        assert m["max_missing_centers"] == 100
        assert m["skipped_missing_center"] == 50  # 2000 - 1950

    def test_default_max_missing_is_zero(self):
        """When max_missing_centers is not passed, defaults to 0 (strict)."""
        pairwise = _make_pairwise()
        audit = _make_audit(common_ref=100, center_found=99)
        verdict = compute_gate_verdict(pairwise, audit)

        assert verdict["passed"] is False
        assert verdict["metrics"]["max_missing_centers"] == 0


# ===================================================================
# Tests for build_verdict (build_normalize_gate_verdict.py)
# ===================================================================


class TestBuildVerdict:
    """Tests for build_verdict in build_normalize_gate_verdict.py."""

    def test_build_verdict_no_allowlist_strict(self):
        """Without max_missing_centers (default=0), gate requires strict equality."""
        pairwise = _make_pairwise()
        audit = _make_audit(common_ref=1000, center_found=954)
        v = build_verdict(
            run_id="test_strict",
            pairwise=pairwise,
            audit=audit,
            pairwise_path="/tmp/pw.json",
            audit_path="/tmp/audit.json",
        )

        assert v["status"] == "fail_normalize_s1"

    def test_build_verdict_with_allowlist_covers_gap(self):
        """max_missing_centers covers gap => pass."""
        pairwise = _make_pairwise()
        audit = _make_audit(common_ref=1000, center_found=954)
        v = build_verdict(
            run_id="test_allowlist",
            pairwise=pairwise,
            audit=audit,
            pairwise_path="/tmp/pw.json",
            audit_path="/tmp/audit.json",
            max_missing_centers=50,
        )

        assert v["status"] == "pass_normalize_s1"

    def test_build_verdict_with_allowlist_partial_coverage(self):
        """max_missing_centers=30 doesn't cover gap=46 => fail."""
        pairwise = _make_pairwise()
        audit = _make_audit(common_ref=1000, center_found=954)
        v = build_verdict(
            run_id="test_partial",
            pairwise=pairwise,
            audit=audit,
            pairwise_path="/tmp/pw.json",
            audit_path="/tmp/audit.json",
            max_missing_centers=30,
        )

        assert v["status"] == "fail_normalize_s1"

    def test_build_verdict_all_pass(self):
        """All metrics clean => pass."""
        pairwise = _make_pairwise()
        audit = _make_audit()
        v = build_verdict(
            run_id="test_run",
            pairwise=pairwise,
            audit=audit,
            pairwise_path="/tmp/pw.json",
            audit_path="/tmp/audit.json",
        )

        assert v["status"] == "pass_normalize_s1"
        assert v["run_id"] == "test_run"

    def test_build_verdict_displaced_fails(self):
        """Displacement > 0 => fail."""
        pairwise = _make_pairwise(displaced_gt=10)
        audit = _make_audit()
        v = build_verdict(
            run_id="test_run",
            pairwise=pairwise,
            audit=audit,
            pairwise_path="/tmp/pw.json",
            audit_path="/tmp/audit.json",
        )

        assert v["status"] == "fail_normalize_s1"

    def test_build_verdict_reports_metrics(self):
        """Verify gates dict contains max_missing_centers and skipped_missing_center."""
        pairwise = _make_pairwise()
        audit = _make_audit(common_ref=500, center_found=490)
        v = build_verdict(
            run_id="r1",
            pairwise=pairwise,
            audit=audit,
            pairwise_path="/tmp/pw.json",
            audit_path="/tmp/audit.json",
            max_missing_centers=20,
        )

        gates = v["gates"]
        assert "normalize_pairwise_gt_0.01" in gates
        assert "normalize_ref_same_gt_0.01" in gates
        assert "audit_center_found" in gates
        assert "audit_common_ref_prim_count" in gates
        assert "audit_matrix_mismatch" in gates
        assert "max_missing_centers" in gates
        assert "skipped_missing_center" in gates
        assert gates["audit_center_found"] == 490
        assert gates["audit_common_ref_prim_count"] == 500
        assert gates["max_missing_centers"] == 20
        assert gates["skipped_missing_center"] == 10  # 500 - 490

    def test_build_verdict_worst_prim_empty(self):
        """When no worst prim data, worst_prim still present but empty."""
        pairwise = _make_pairwise()
        audit = _make_audit()
        v = build_verdict(
            run_id="r1",
            pairwise=pairwise,
            audit=audit,
            pairwise_path="/tmp/pw.json",
            audit_path="/tmp/audit.json",
        )

        assert "worst_prim" in v

    def test_build_verdict_reports_paths(self):
        """Reports dict contains absolute paths."""
        pairwise = _make_pairwise()
        audit = _make_audit()
        v = build_verdict(
            run_id="r1",
            pairwise=pairwise,
            audit=audit,
            pairwise_path="/tmp/pw.json",
            audit_path="/tmp/audit.json",
        )

        assert v["reports"]["pairwise_normalize_only"] == "/tmp/pw.json"
        assert v["reports"]["audit_normalize_phase2"] == "/tmp/audit.json"

    def test_build_verdict_descendant_override_fails(self):
        """Inert descendant overrides > 0 => fail."""
        pairwise = _make_pairwise()
        audit = _make_audit(inert_descendant=5)
        v = build_verdict(
            run_id="r1",
            pairwise=pairwise,
            audit=audit,
            pairwise_path="/tmp/pw.json",
            audit_path="/tmp/audit.json",
        )

        assert v["status"] == "fail_normalize_s1"
        assert "descendant xform overrides" in v["next_action"]

    def test_build_verdict_missing_pre_c1_fails(self):
        """Missing pre_c1 scenes => fail."""
        pairwise = _make_pairwise()
        audit = _make_audit(missing_pre_c1=["scene_a", "scene_b"])
        v = build_verdict(
            run_id="r1",
            pairwise=pairwise,
            audit=audit,
            pairwise_path="/tmp/pw.json",
            audit_path="/tmp/audit.json",
        )

        assert v["status"] == "fail_normalize_s1"

    def test_build_verdict_zero_gap_passes_with_default(self):
        """Zero center gap passes even with default max_missing_centers=0."""
        pairwise = _make_pairwise()
        audit = _make_audit(common_ref=1000, center_found=1000)
        v = build_verdict(
            run_id="r1",
            pairwise=pairwise,
            audit=audit,
            pairwise_path="/tmp/pw.json",
            audit_path="/tmp/audit.json",
        )

        assert v["status"] == "pass_normalize_s1"


# ===================================================================
# Tests for allowlist parsing and extraction
# ===================================================================


class TestAllowlistParsing:
    """Test parsing allowlist_verdict.json to derive max_missing_centers."""

    def test_count_known_bad_from_allowlist(self):
        """Total allowed_asset_errors across all categories matches expected count."""
        total = _count_allowed_asset_errors(_SAMPLE_ALLOWLIST)
        # book: 3, cabinet: 1, other: 1, person: 1, door_UUID1: 1, door_UUID2: 1
        assert total == 8

    def test_collect_all_keys_from_allowed_asset_errors(self):
        """All keys from allowed_asset_errors across all categories are collected."""
        keys = _collect_allowed_asset_keys(_SAMPLE_ALLOWLIST)
        # 8 unique paths: book(3) + cabinet(1) + other(1) + person(1) + door_UUID1(1) + door_UUID2(1)
        assert len(keys) == 8
        assert "book/aaaa1111/usd/aaaa1111.usd" in keys
        assert "cabinet/b98d6f3a/usd/b98d6f3a.usd" in keys
        assert "other/d41d8cd9/usd/d41d8cd9.usd" in keys
        assert "person/35131a3c/usd/35131a3c.usd" in keys
        assert "door_UUID1/d41d8cd9/usd/d41d8cd9.usd" in keys
        assert "door_UUID2/d41d8cd9/usd/d41d8cd9.usd" in keys

    def test_door_star_categories_included(self):
        """door_* categories with exit code 1 have their assets in allowed_asset_errors."""
        errors = _SAMPLE_ALLOWLIST["default_allowlist"]["allowed_asset_errors"]
        exit_codes = _SAMPLE_ALLOWLIST["default_allowlist"]["allowed_nonzero_exit_codes"]

        door_categories = [k for k in exit_codes if k.startswith("door_")]
        assert len(door_categories) == 2

        # Each door_* category's d41d8cd9 asset is in allowed_asset_errors
        for cat in door_categories:
            assert cat in errors
            assets = errors[cat]
            assert any("d41d8cd9" in path for path in assets)

    def test_empty_allowlist(self):
        """Empty allowlist => zero allowed errors, behaves like no allowlist."""
        empty = {"default_allowlist": {"allowed_nonzero_exit_codes": {}, "allowed_asset_errors": {}}}
        assert _count_allowed_asset_errors(empty) == 0
        assert _collect_allowed_asset_keys(empty) == set()

    def test_missing_allowlist_keys(self):
        """Malformed allowlist with missing keys => zero count, no crash."""
        assert _count_allowed_asset_errors({}) == 0
        assert _collect_allowed_asset_keys({}) == set()
        assert _count_allowed_asset_errors({"default_allowlist": {}}) == 0

    def test_allowlist_count_feeds_max_missing_centers(self):
        """End-to-end: allowlist count used as max_missing_centers makes gate pass."""
        total_allowed = _count_allowed_asset_errors(_SAMPLE_ALLOWLIST)  # 8

        pairwise = _make_pairwise()
        # Gap = 8, exactly matching the allowlist count
        audit = _make_audit(common_ref=1000, center_found=992)

        # Strict mode: fails
        v_strict = compute_gate_verdict(pairwise, audit, max_missing_centers=0)
        assert v_strict["passed"] is False

        # With allowlist-derived max_missing_centers: passes
        v_allow = compute_gate_verdict(pairwise, audit, max_missing_centers=total_allowed)
        assert v_allow["passed"] is True

        # Also test build_verdict
        bv = build_verdict(
            run_id="e2e",
            pairwise=pairwise,
            audit=audit,
            pairwise_path="/tmp/pw.json",
            audit_path="/tmp/audit.json",
            max_missing_centers=total_allowed,
        )
        assert bv["status"] == "pass_normalize_s1"
        assert bv["gates"]["max_missing_centers"] == total_allowed
        assert bv["gates"]["skipped_missing_center"] == 8
