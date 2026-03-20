#!/usr/bin/env python3

import importlib.util
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_PATH = REPO_ROOT / "scripts" / "orchestrate_test0_rebuilt_normalize.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "orchestrate_test0_rebuilt_normalize", SCRIPT_PATH
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


mod = _load_module()


def test_parse_dlc_job_table_reads_rows():
    table = """
+------+------+-----------+
| Name | JobId| JobStatus |
+------+------+-----------+
| foo  | id1  | Running   |
| bar  | id2  | Succeeded |
+------+------+-----------+
"""
    rows = mod._parse_dlc_job_table(table)
    assert len(rows) == 2
    assert rows[0]["Name"] == "foo"
    assert rows[1]["JobStatus"] == "Succeeded"


def test_derive_phase1_job_names_includes_run_id():
    names = mod._derive_phase1_job_names(["wall", "other"], "abc123")
    assert names == [
        "norm_p1_abc123_wall_0_1",
        "norm_p1_abc123_other_0_1",
    ]
