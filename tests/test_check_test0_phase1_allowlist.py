#!/usr/bin/env python3

import importlib.util
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "check_test0_phase1_allowlist.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "check_test0_phase1_allowlist", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


mod = _load_module()


def test_evaluate_category_uses_normalize_report_when_logs_missing(tmp_path):
    phase1_root = tmp_path / "phase1"
    category_dir = phase1_root / "plate"
    category_dir.mkdir(parents=True)
    (category_dir / "centers_plate.json").write_text("{}\n")
    report = {
        "meta": {"errors_count": 0},
        "errors": [],
    }
    (category_dir / "normalize_report.json").write_text(json.dumps(report))

    result = mod._evaluate_category(
        phase1_root=phase1_root,
        category="plate",
        summary_entry=None,
    )
    assert result["returncode"] == 0
    assert result["failures"] == []


def test_parse_asset_errors_from_report_reads_machine_report():
    report = {
        "errors": [
            {"asset": "plate/abc/usd/abc.usd", "error": "bad mesh"},
            {"asset": "plate/abc/usd/abc.usd", "error": "bad mesh"},
        ]
    }
    errors = mod._parse_asset_errors_from_report(report)
    assert errors == [{"asset": "plate/abc/usd/abc.usd", "message": "bad mesh"}]


def test_submit_logs_ignored_in_category_discovery(tmp_path):
    """_submit_logs directory should not be treated as a category."""
    phase1_root = tmp_path / "phase1"
    (phase1_root / "bottle").mkdir(parents=True)
    (phase1_root / "_submit_logs").mkdir(parents=True)
    (phase1_root / "bottle" / "centers_bottle.json").write_text("{}\n")

    categories = mod._discover_categories(phase1_root, {})
    assert "bottle" in categories
    assert "_submit_logs" not in categories


def test_door_uuid_allowed_exit_codes():
    """door_<UUID> categories should allow exit code 1."""
    codes = mod._allowed_exit_codes_for("door_03709DDE_5114_47C3_B930_CC023B19FEEE")
    assert codes == {0, 1}


def test_door_uuid_allowed_asset_errors():
    """door_<UUID> categories should allowlist d41d8cd98f00b204e9800998ecf8427e asset."""
    errors = mod._allowed_asset_errors_for("door_03709DDE_5114_47C3_B930_CC023B19FEEE")
    expected_key = (
        "door_03709DDE_5114_47C3_B930_CC023B19FEEE/"
        "d41d8cd98f00b204e9800998ecf8427e/usd/"
        "d41d8cd98f00b204e9800998ecf8427e.usd"
    )
    assert expected_key in errors
    assert "No meshes found under /Root/Instance" in errors[expected_key]


def test_book_allowed_exit_codes():
    """book category should allow exit code 1."""
    codes = mod._allowed_exit_codes_for("book")
    assert codes == {0, 1}


def test_verdict_passes_with_door_category_failure(tmp_path):
    """A door_<UUID> category with only d41d... asset error should pass."""
    phase1_root = tmp_path / "phase1"
    cat_name = "door_03709DDE_5114_47C3_B930_CC023B19FEEE"
    category_dir = phase1_root / cat_name
    category_dir.mkdir(parents=True)
    (category_dir / f"centers_{cat_name}.json").write_text("{}\n")
    report = {
        "meta": {"errors_count": 1},
        "errors": [{
            "asset": (
                f"{cat_name}/d41d8cd98f00b204e9800998ecf8427e/usd/"
                "d41d8cd98f00b204e9800998ecf8427e.usd"
            ),
            "error": "No meshes found under /Root/Instance",
        }],
    }
    (category_dir / "normalize_report.json").write_text(json.dumps(report))

    result = mod._evaluate_category(
        phase1_root=phase1_root,
        category=cat_name,
        summary_entry=None,
    )
    assert result["failures"] == [], f"Unexpected failures: {result['failures']}"
    assert len(result["allowlisted_asset_errors"]) == 1


def test_unknown_category_with_error_fails(tmp_path):
    """A non-allowlisted category with errors should fail."""
    phase1_root = tmp_path / "phase1"
    category_dir = phase1_root / "widget"
    category_dir.mkdir(parents=True)
    (category_dir / "centers_widget.json").write_text("{}\n")
    report = {
        "meta": {"errors_count": 1},
        "errors": [{
            "asset": "widget/abc123/usd/abc123.usd",
            "error": "No meshes found under /Root/Instance",
        }],
    }
    (category_dir / "normalize_report.json").write_text(json.dumps(report))

    result = mod._evaluate_category(
        phase1_root=phase1_root,
        category="widget",
        summary_entry=None,
    )
    assert len(result["failures"]) > 0
    assert len(result["unexpected_asset_errors"]) == 1
