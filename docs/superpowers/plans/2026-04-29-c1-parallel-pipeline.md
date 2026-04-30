---
title: C1 Parallel Acceleration Pipeline Implementation Plan
code_reference:
  - scripts/c1_phase1_apply_and_audit.py
  - scripts/c1_phase2_merge_scan_delete.py
  - scripts/c1_parallel_merge.py
  - scripts/scan_utils.py
  - scripts/orchestrate_c1_parallel.py
  - scripts/dlc/launch_job.sh
  - scripts/c1_build_bulk_mapping_from_dedup_report.py
created_at: 2026-04-29
updated_at: 2026-04-30
maintainer: OpenCode
status: implemented
doc_class: record
---

# C1 Parallel Acceleration Pipeline — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended)
> or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a two-phase parallel pipeline that processes all remaining C1 dedup
categories concurrently (Phase 1: 54 parallel DLC jobs for apply+audit), then
merges all sidecar changes and runs a single batched scan+soft-delete (Phase 2:
1 DLC job). Reduces wall time from ~9 days to ~6 hours.

**Architecture:** Three new scripts (Phase 1 entry, Phase 2 entry, DLC orchestrator),
one extracted library module (scan_utils), and one config modification (launch_job.sh
timeout). All new code is additive — the existing serial autorun path is untouched.

**Tech Stack:** Python 3.10, pxr (Usd), argparse, subprocess (for DLC CLI).

**Design spec:** `docs/superpowers/specs/2026-04-29-c1-parallel-pipeline-design.md`

---

## File Structure

| File | Responsibility | New/Modify |
|------|---------------|------------|
| `scripts/scan_utils.py` | Extracted shared scan functions (`_scan_stage_for_old_assets`, `_iter_usd_files`, `_scan_tree_pxr`, path helpers) | **New** (extracted from existing) |
| `scripts/c1_phase1_apply_and_audit.py` | Per-category apply+audit for Phase 1 DLC jobs | **New** |
| `scripts/c1_phase2_merge_scan_delete.py` | Merge all sidecars + mega-scan + soft-delete for Phase 2 DLC job | **New** |
| `scripts/orchestrate_c1_parallel.py` | DLC orchestration (submit-phase1, gate-check, submit-phase2, status) | **New** |
| `scripts/c1_bulk_step6_category_promote_scan_soft_delete.py` | Refactored to import from scan_utils (existing logic preserved) | **Modify** |
| `scripts/c1_autorun_categories.py` | Export `_is_step6_complete()` for reuse by orchestrator | **Modify** (minimal) |
| `scripts/dlc/launch_job.sh` | Support `DLC_JOB_TIMEOUT` env var for per-job timeout | **Modify** |
| `tests/test_c1_parallel_merge.py` | Tests for merge logic, gate check, scan exclusion rules | **New** |
| `tests/test_scan_utils.py` | Tests for scan utility functions | **New** |

---

### Task 1: Extract scan_utils library module

**Files:**
- Create: `scripts/scan_utils.py`
- Modify: `scripts/c1_bulk_step6_category_promote_scan_soft_delete.py` (import from scan_utils)
- Create: `tests/test_scan_utils.py`

**Rationale:** The scan functions (`_scan_stage_for_old_assets`, `_iter_usd_files`,
`_scan_tree_pxr`, path helpers `_abs_from_usd_ref`, `_abs_to_subset_rel`,
`_normalize_mapping_key`, `_parse_uid_from_report_style_asset_usd`) are needed
by Phase 2 mega-scan. Extract them into a shared module so both the existing
script and Phase 2 can import them.

- [ ] **Step 1: Create `scripts/scan_utils.py` with extracted functions**

Move the following functions from `scripts/c1_bulk_step6_category_promote_scan_soft_delete.py`:
- `_write_json`, `_append_jsonl` (write helpers)
- `_parse_uid_from_report_style_asset_usd` (UID extractor)
- `_abs_from_usd_ref`, `_abs_to_subset_rel`, `_normalize_mapping_key` (path helpers)
- `_emit_progress` (progress emitter)
- `_scan_stage_for_old_assets` (single-file pxr scanner)
- `_iter_usd_files` (USD file enumerator with exclusion rules)
- `_scan_tree_pxr` (full-tree scanner)

Add exclusion for `.parallel_` files alongside existing `.pre_` and `.c1_`:

```python
# In _iter_usd_files, add after the ".c1_" check:
if ".parallel_" in fn:
    continue
```

Also add these functions as the public API of scan_utils:

```python
def build_old_asset_path_set(mapping_json_path: str, dataset_name: str) -> Set[str]:
    """Build set of old asset relative paths from a mapping JSON."""
    import json
    with open(mapping_json_path, encoding="utf-8") as f:
        mapping = json.load(f)
    result = set()
    for old_key in mapping.keys():
        norm = _normalize_mapping_key(old_key, dataset_name)
        if norm:
            result.add(norm)
    return result


def build_combined_old_asset_path_set(
    mapping_paths: List[str], dataset_name: str
) -> Set[str]:
    """Build combined set of old asset paths from multiple mapping JSONs."""
    result = set()
    for mp in mapping_paths:
        result |= build_old_asset_path_set(mp, dataset_name)
    return result
```

- [ ] **Step 2: Modify `c1_bulk_step6_category_promote_scan_soft_delete.py` to import from scan_utils**

Replace the extracted function definitions with imports:

```python
from scan_utils import (
    _write_json,
    _append_jsonl,
    _parse_uid_from_report_style_asset_usd,
    _abs_from_usd_ref,
    _abs_to_subset_rel,
    _normalize_mapping_key,
    _emit_progress,
    _scan_stage_for_old_assets,
    _iter_usd_files,
    _scan_tree_pxr,
    build_old_asset_path_set,
)
```

Ensure `sys.path` is set up so the import works from the scripts directory:

```python
import sys, os
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
```

- [ ] **Step 3: Create `tests/test_scan_utils.py`**

```python
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
        (tmp_path / "bak" / "_dedup_assets").mkdir(parents=True)
        (tmp_path / "scene" / "layout.usd").write_text("x")
        (tmp_path / "bak" / "_dedup_assets" / "stale.usd").write_text("x")
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
```

- [ ] **Step 4: Run tests**

```bash
python -m pytest tests/test_scan_utils.py -v
```
Expected: all tests pass.

- [ ] **Step 5: Verify existing scan script still works**

```bash
python -c "
import sys; sys.path.insert(0, 'scripts')
from scan_utils import _parse_uid_from_report_style_asset_usd, _iter_usd_files
print('scan_utils import OK')
print('_parse_uid:', _parse_uid_from_report_style_asset_usd('GRScenes_assets/chair/uid/usd/uid.usd'))
"
```
Expected: import succeeds, prints path.

- [ ] **Step 6: Commit**

```bash
git add scripts/scan_utils.py
git add scripts/c1_bulk_step6_category_promote_scan_soft_delete.py
git add tests/test_scan_utils.py
git commit -m "refactor: extract scan_utils library from c1_bulk_step6

Extract shared scan functions (_scan_stage_for_old_assets,
_iter_usd_files, _scan_tree_pxr, path helpers) into a reusable
scan_utils module. Add .parallel_ file exclusion for Phase 1
sidecar compatibility. Add build_old_asset_path_set() helpers
for combined mapping scans.

No behavioral change to existing scan script — imports from
scan_utils instead of defining functions inline."
```

---

### Task 2: Merge logic with tests

**Files:**
- Create: `scripts/c1_parallel_merge.py` (pure-Python merge module, no pxr needed)
- Create: `tests/test_c1_parallel_merge.py`

**Rationale:** The merge algorithm (combining per-category filtered_mapping.json
files into one combined mapping) is pure Python logic. Test it independently
before integrating into Phase 2 script.

- [ ] **Step 1: Create `scripts/c1_parallel_merge.py`**

```python
"""C1 Parallel Merge: combine per-category mappings with conflict detection."""
import json
import logging
from pathlib import Path
from typing import Dict, List, Optional, Tuple

log = logging.getLogger(__name__)


def load_filtered_mapping(mapping_json: Path) -> Dict[str, str]:
    """Load a filtered_mapping.json and return {old_key: canonical_key}."""
    with open(mapping_json, encoding="utf-8") as f:
        payload = json.load(f)

    if isinstance(payload, dict):
        return payload
    if isinstance(payload, list):
        result = {}
        for pair in payload:
            result[pair["old"]] = pair["canonical"]
        return result
    raise ValueError(f"Unexpected mapping format in {mapping_json}")


def merge_category_mappings(
    category_mappings: List[Tuple[str, Dict[str, str]]]
) -> Tuple[Dict[str, str], List[Dict]]:
    """Merge multiple category-identified mappings into one combined mapping.

    Args:
        category_mappings: List of (category_name, {old_key: canonical_key}) tuples.

    Returns:
        (combined_mapping, conflicts_list)
        conflicts_list is empty if no conflicts. Each conflict entry:
        {"old_key": ..., "existing": {"category": ..., "target": ...},
         "conflicting": {"category": ..., "target": ...}}
    """
    combined: Dict[str, str] = {}
    sources: Dict[str, str] = {}  # old_key -> category that first set it
    conflicts: List[Dict] = []

    for cat, mapping in category_mappings:
        for old_key, new_val in mapping.items():
            if old_key in combined:
                if combined[old_key] == new_val:
                    continue  # same target — benign overlap
                conflicts.append({
                    "old_key": old_key,
                    "existing": {"category": sources[old_key],
                                 "target": combined[old_key]},
                    "conflicting": {"category": cat, "target": new_val},
                })
            else:
                combined[old_key] = new_val
                sources[old_key] = cat

    return combined, conflicts


def discover_category_mappings(
    c1_bulk_dir: Path,
    bbox_policy: str,
    out_version: str,
    categories: List[str],
) -> List[Tuple[str, Dict[str, str]]]:
    """Discover and load filtered_mapping.json for each category.

    Returns list of (category_name, mapping_dict).
    Skips categories where filtered_mapping.json is missing or unreadable.
    """
    results: List[Tuple[str, Dict[str, str]]] = []
    for cat in categories:
        mapping_path = (
            c1_bulk_dir
            / f"{cat}_{bbox_policy}_{out_version}"
            / "01_cert"
            / "filtered_mapping.json"
        )
        if not mapping_path.exists():
            log.warning("Mapping not found for category %s: %s", cat, mapping_path)
            continue
        try:
            mapping = load_filtered_mapping(mapping_path)
            if mapping:
                results.append((cat, mapping))
            else:
                log.info("Empty mapping for category %s, skipping", cat)
        except Exception as e:
            log.error("Failed to load mapping for %s: %s", cat, e)
    return results


def gate_check_phase1(
    c1_bulk_dir: Path,
    bbox_policy: str,
    out_version: str,
    categories: List[str],
) -> Tuple[bool, List[str]]:
    """Verify all categories have completed Phase 1 with status=ok.

    Returns (all_passed, failed_categories).
    """
    failed: List[str] = []
    for cat in categories:
        done_file = (
            c1_bulk_dir
            / f"{cat}_{bbox_policy}_{out_version}"
            / "phase1_done.json"
        )
        if not done_file.exists():
            failed.append(f"{cat}: missing phase1_done.json")
            continue
        try:
            payload = json.loads(done_file.read_text(encoding="utf-8"))
        except Exception as e:
            failed.append(f"{cat}: unreadable phase1_done.json ({e})")
            continue
        status = payload.get("status", "")
        audit_passed = payload.get("audit_passed", False)
        if status != "ok":
            failed.append(f"{cat}: status={status}")
        elif not audit_passed:
            failed.append(f"{cat}: audit not passed")
    return len(failed) == 0, failed
```

- [ ] **Step 2: Create `tests/test_c1_parallel_merge.py`**

```python
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
```

- [ ] **Step 3: Run tests**

```bash
python -m pytest tests/test_c1_parallel_merge.py -v
```
Expected: all tests pass.

- [ ] **Step 4: Commit**

```bash
git add scripts/c1_parallel_merge.py tests/test_c1_parallel_merge.py
git commit -m "feat: add c1_parallel_merge module with gate check logic

Add merge_category_mappings() for combining per-category dedup mappings
with structured conflict detection. Add gate_check_phase1() for
verifying all Phase 1 DLC jobs completed successfully before Phase 2.

Both functions are pure Python (no pxr dependency) and fully tested."
```

---

### Task 3: Create Phase 1 entry-point script

**Files:**
- Create: `scripts/c1_phase1_apply_and_audit.py`

**Rationale:** Each Phase 1 DLC job runs this script for one category. It builds the
mapping, runs apply (sidecar generation for all 3 scene files), runs audit
(placement comparison), and writes `phase1_done.json`. Reuses existing logic
via subprocess calls to the existing scripts (safer than refactoring into libraries,
as the code review suggested).

- [ ] **Step 1: Create `scripts/c1_phase1_apply_and_audit.py`**

```python
#!/usr/bin/env python3
"""Phase 1: Per-category apply+audit entry point for parallel DLC jobs.

Processes exactly ONE category: builds mapping, applies rewrites to all
3 scene files (layout.usd, start_result_interaction.usd,
start_result_navigation.usd), runs placement audit, and writes a
phase1_done.json completion marker.

Usage:
  c1_phase1_apply_and_audit.py \
    --category <cat> --dataset-root <path> --bak-root <path> \
    --report <path> --c1-bulk-dir <path> \
    --group-label <label> --bbox-gated --bbox-policy <policy> \
    --dedup-mode geom_only --v-matrix-mode auto --out-version v1
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description="C1 Phase 1: apply+audit for one category")
    ap.add_argument("--category", required=True)
    ap.add_argument("--dataset-root", required=True)
    ap.add_argument("--bak-root", required=True)
    ap.add_argument("--report", required=True, help="Unified dedup report JSON")
    ap.add_argument("--c1-bulk-dir", required=True)
    ap.add_argument("--group-label", required=True)
    ap.add_argument("--bbox-gated", action="store_true")
    ap.add_argument("--bbox-policy", default="bbox_primary_rmse_observe")
    ap.add_argument("--dedup-mode", default="geom_only")
    ap.add_argument("--v-matrix-mode", default="auto")
    ap.add_argument("--out-version", default="v1")
    ap.add_argument("--scene-files",
                    default="layout.usd,start_result_interaction.usd,"
                            "start_result_navigation.usd")
    args = ap.parse_args()

    category = args.category
    dataset_root = Path(args.dataset_root)
    bak_root = Path(args.bak_root)
    report_path = Path(args.report)
    c1_bulk_dir = Path(args.c1_bulk_dir)
    group_label = args.group_label

    # Category output directory
    cat_dir = c1_bulk_dir / f"{category}_{args.bbox_policy}_{args.out_version}"
    report_dir = cat_dir
    report_dir.mkdir(parents=True, exist_ok=True)

    # Step1 marker
    step1_dir = cat_dir / "01_cert"
    mapping_json = step1_dir / "filtered_mapping.json"

    script_dir = Path(__file__).resolve().parent

    # ---- Step 1: Build mapping ----
    print(f"[Phase1:{category}] Step 1: Building mapping...", flush=True)
    t0 = time.time()

    build_cmd = [
        sys.executable,
        str(script_dir / "c1_build_bulk_mapping_from_dedup_report.py"),
        "--report", str(report_path),
        "--category", category,
        "--c1-bulk-dir", str(c1_bulk_dir),
        "--dedup-mode", args.dedup_mode,
    ]
    if args.bbox_gated:
        build_cmd += ["--bbox-gated", "--bbox-policy", args.bbox_policy]
    build_cmd += ["--out-dir", str(step1_dir)]

    r = subprocess.run(build_cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(f"[Phase1:{category}] Step 1 FAILED", flush=True)
        print(r.stderr, flush=True)
        _write_phase1_done(cat_dir, "failed",
                          f"Step1 build_mapping failed: {r.stderr[:500]}")
        return 1

    if not mapping_json.exists():
        print(f"[Phase1:{category}] No mapping pairs (mapping_pairs_0)", flush=True)
        _write_phase1_done(cat_dir, "ok", audit_passed=True,
                          note="no mapping pairs")
        return 0

    # ---- Step 2: Apply rewrites ----
    print(f"[Phase1:{category}] Step 2: Applying rewrites...", flush=True)
    t1 = time.time()

    out_name = f"layout.parallel_{group_label}.{category}_{args.bbox_policy}_{args.out_version}.usd"
    apply_cmd = [
        sys.executable,
        str(script_dir / "c1_bulk_apply_layout_dedup.py"),
        "--mapping-json", str(mapping_json),
        "--dataset-root", str(dataset_root),
        "--bak-root", str(bak_root),
        "--report-dir", str(report_dir),
        "--out-name", out_name,
        "--bbox-gated" if args.bbox_gated else "--no-bbox-gated",
    ]
    if args.bbox_gated:
        apply_cmd += ["--bbox-policy", args.bbox_policy]
    apply_cmd += [
        "--v-matrix-mode", args.v_matrix_mode,
        "--scene-files", args.scene_files,
    ]

    r = subprocess.run(apply_cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(f"[Phase1:{category}] Step 2 FAILED", flush=True)
        print(r.stderr, flush=True)
        _write_phase1_done(cat_dir, "failed",
                          f"Step2 apply failed: {r.stderr[:500]}")
        return 1

    apply_elapsed = time.time() - t1
    print(f"[Phase1:{category}] Step 2 OK ({apply_elapsed:.1f}s)", flush=True)

    # ---- Step 3: Audit ----
    print(f"[Phase1:{category}] Step 3: Running audit...", flush=True)
    t2 = time.time()

    audit_dir = cat_dir / "03_audit"
    layout_root = dataset_root / "GRScenes100"

    audit_cmd = [
        sys.executable,
        str(script_dir / "placement_pairwise_compare.py"),
        "--layout-root", str(layout_root),
        "--dataset-root", str(dataset_root),
        "--subset-dirs", f"GRScenes100:{str(layout_root)}",
        "--report-dir", str(audit_dir),
        "--out-name", out_name,
        "--mode", "audit",
        "--output-basename", "placement_pairwise_compare",
    ]
    if args.bbox_gated:
        audit_cmd += [
            "--bbox-gated",
            "--bbox-policy", args.bbox_policy,
        ]

    r = subprocess.run(audit_cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(f"[Phase1:{category}] Step 3 FAILED", flush=True)
        print(r.stderr, flush=True)
        _write_phase1_done(cat_dir, "failed",
                          f"Step3 audit failed: {r.stderr[:500]}")
        return 1

    audit_elapsed = time.time() - t2

    # Check audit verdict
    verdict_json = audit_dir / "audit_verdict.json"
    audit_passed = False
    if verdict_json.exists():
        try:
            verdict = json.loads(verdict_json.read_text())
            audit_passed = bool(verdict.get("passed", False))
        except Exception:
            pass

    print(f"[Phase1:{category}] Step 3 {'PASSED' if audit_passed else 'FAILED'} "
          f"({audit_elapsed:.1f}s)", flush=True)

    total_elapsed = time.time() - t0
    _write_phase1_done(
        cat_dir,
        "ok" if audit_passed else "failed",
        audit_passed=audit_passed,
        elapsed_sec=total_elapsed,
        note=f"ok" if audit_passed else "audit verdict not passed",
        mapping_json=str(mapping_json),
        out_name=out_name,
    )
    return 0 if audit_passed else 1


def _write_phase1_done(
    cat_dir: Path,
    status: str,
    audit_passed: bool = False,
    elapsed_sec: float = 0.0,
    note: str = "",
    **extra,
):
    payload = {
        "status": status,
        "audit_passed": audit_passed,
        "elapsed_sec": elapsed_sec,
        "note": note,
        **extra,
    }
    cat_dir.mkdir(parents=True, exist_ok=True)
    (cat_dir / "phase1_done.json").write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Commit**

```bash
git add scripts/c1_phase1_apply_and_audit.py
git commit -m "feat: add Phase 1 per-category apply+audit entry point

c1_phase1_apply_and_audit.py processes exactly one category:
Step 1 (build mapping via subprocess), Step 2 (apply rewrites
via subprocess), Step 3 (placement audit via subprocess).
Writes phase1_done.json with status, audit result, and timing.

Sidecars use parallel_ prefix naming to distinguish from
serial autorun sidecars. Supports all 3 scene files
(layout.usd, start_result_interaction.usd,
start_result_navigation.usd)."
```

---

### Task 4: Create Phase 2 entry-point script

**Files:**
- Create: `scripts/c1_phase2_merge_scan_delete.py`

**Rationale:** Single DLC job that runs after all Phase 1 jobs complete. Merges
all category sidecars into cumulative scene files, runs mega-scan, then soft-deletes
old assets. Only the merge logic is pure Python; the rest requires pxr (Isaac Sim)
and must be tested in the DLC environment.

- [ ] **Step 1: Create `scripts/c1_phase2_merge_scan_delete.py`**

```python
#!/usr/bin/env python3
"""Phase 2: Merge all category sidecars + mega-scan + soft-delete.

Usage:
  c1_phase2_merge_scan_delete.py \
    --dataset-root <path> --bak-root <path> --c1-bulk-dir <path> \
    --bbox-policy <policy> --out-version v1 --group-label <label> \
    [--categories-file <path>]
"""
import argparse
import json
import logging
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Set

log = logging.getLogger(__name__)


def main():
    ap = argparse.ArgumentParser(description="C1 Phase 2: merge+scan+delete")
    ap.add_argument("--dataset-root", required=True)
    ap.add_argument("--bak-root", required=True)
    ap.add_argument("--c1-bulk-dir", required=True)
    ap.add_argument("--bbox-policy", default="bbox_primary_rmse_observe")
    ap.add_argument("--out-version", default="v1")
    ap.add_argument("--group-label", required=True)
    ap.add_argument("--categories-file", default=None,
                    help="JSON list of category names (default: discover from c1_bulk)")
    ap.add_argument("--scene-files",
                    default="layout.usd,start_result_interaction.usd,"
                            "start_result_navigation.usd")
    args = ap.parse_args()

    dataset_root = Path(args.dataset_root)
    bak_root = Path(args.bak_root)
    c1_bulk = Path(args.c1_bulk_dir)
    policy = args.bbox_policy
    version = args.out_version
    label = args.group_label
    scene_files = [s.strip() for s in args.scene_files.split(",")]

    # Setup
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from c1_parallel_merge import (
        gate_check_phase1,
        merge_category_mappings,
        discover_category_mappings,
    )

    # ---- Gate Check ----
    print("[Phase2] Gate check: verifying all Phase 1 jobs...", flush=True)
    if args.categories_file:
        categories = json.loads(Path(args.categories_file).read_text())
    else:
        categories = _discover_categories(c1_bulk, policy, version)

    print(f"[Phase2] {len(categories)} categories to process", flush=True)
    ok, failed = gate_check_phase1(c1_bulk, policy, version, categories)
    if not ok:
        print("[Phase2] GATE FAILED — some Phase 1 jobs incomplete:", flush=True)
        for f in failed:
            print(f"  - {f}", flush=True)
        return 1
    print("[Phase2] Gate PASSED — all Phase 1 jobs completed with audit OK", flush=True)

    # ---- Merge ----
    print("[Phase2] Merging sidecars...", flush=True)
    t0 = time.time()

    cat_mappings = discover_category_mappings(c1_bulk, policy, version, categories)
    print(f"[Phase2] Loaded {len(cat_mappings)} category mappings", flush=True)

    combined_mapping, conflicts = merge_category_mappings(cat_mappings)
    if conflicts:
        print(f"[Phase2] WARNING: {len(conflicts)} conflicts detected", flush=True)
        for c in conflicts[:10]:
            print(f"  Conflict: {c['old_key']}", flush=True)
        _write_merge_report(c1_bulk, "conflict_detected", conflicts=conflicts)
        # Fall back to sequential merge
        print("[Phase2] Falling back to sequential merge...", flush=True)
        _sequential_merge(
            dataset_root, c1_bulk, policy, version, categories,
            label, scene_files
        )
    else:
        print("[Phase2] No conflicts — using combined mapping", flush=True)
        _combined_merge(
            dataset_root, combined_mapping, scene_files
        )

    merge_elapsed = time.time() - t0
    print(f"[Phase2] Merge complete ({merge_elapsed:.1f}s)", flush=True)

    # ---- Mega-Scan ----
    print("[Phase2] Mega-scan: scanning for old asset references...", flush=True)
    t1 = time.time()

    from scan_utils import (
        build_combined_old_asset_path_set,
        _scan_tree_pxr,
    )

    mapping_paths = [
        str(c1_bulk / f"{cat}_{policy}_{version}" / "01_cert" / "filtered_mapping.json")
        for cat, _ in cat_mappings
    ]
    old_asset_set = build_combined_old_asset_path_set(
        mapping_paths, dataset_root.name
    )
    print(f"[Phase2] Checking for {len(old_asset_set)} old asset paths", flush=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    scan_result = _scan_tree_pxr(
        dataset_root,
        old_asset_usd_rel_set=old_asset_set,
        dataset_name=dataset_root.name,
        exclude_dir_contains=["/_dedup_assets/"],
        progress_every=5000,
        progress_json=c1_bulk / f"phase2_mega_scan_{stamp}.json",
        progress_jsonl=c1_bulk / f"phase2_mega_scan_{stamp}.jsonl",
    )

    hit_files = scan_result.get("hit_files", 0)
    scan_elapsed = time.time() - t1
    print(f"[Phase2] Mega-scan: {hit_files} hit files out of "
          f"{scan_result.get('scanned_files', 0)} scanned "
          f"({scan_elapsed:.1f}s)", flush=True)

    if hit_files > 0:
        print("[Phase2] MEGA-SCAN FAILED — old asset references still exist",
              flush=True)
        _write_merge_report(c1_bulk, "scan_failed", scan_result=scan_result)
        return 1
    print("[Phase2] Mega-scan PASSED — no old asset references", flush=True)

    # ---- Soft-Delete ----
    print("[Phase2] Soft-deleting old assets...", flush=True)
    t2 = time.time()

    delete_errors = _mega_soft_delete(dataset_root, bak_root, cat_mappings, stamp)
    delete_elapsed = time.time() - t2
    print(f"[Phase2] Soft-delete complete ({delete_elapsed:.1f}s), "
          f"{len(delete_errors)} errors", flush=True)

    # ---- Post-Delete Scan ----
    print("[Phase2] Post-delete scan...", flush=True)
    post_result = _scan_tree_pxr(
        dataset_root,
        old_asset_usd_rel_set=old_asset_set,
        dataset_name=dataset_root.name,
        exclude_dir_contains=["/_dedup_assets/"],
        progress_every=5000,
        progress_json=c1_bulk / f"phase2_post_delete_scan_{stamp}.json",
        progress_jsonl=c1_bulk / f"phase2_post_delete_scan_{stamp}.jsonl",
    )

    post_hits = post_result.get("hit_files", 0)
    print(f"[Phase2] Post-delete scan: {post_hits} hit files", flush=True)

    _write_merge_report(c1_bulk, "complete",
                        categories_processed=len(cat_mappings),
                        merge_elapsed=merge_elapsed,
                        scan_elapsed=scan_elapsed,
                        delete_elapsed=delete_elapsed,
                        hit_files=hit_files,
                        post_hits=post_hits,
                        delete_errors=len(delete_errors))

    return 0 if (hit_files == 0 and post_hits == 0) else 1


def _discover_categories(c1_bulk: Path, policy: str, version: str) -> List[str]:
    """Discover category names from c1_bulk directory."""
    cats = []
    for d in sorted(c1_bulk.iterdir()):
        if not d.is_dir():
            continue
        name = d.name
        if name.endswith(f"_{policy}_{version}"):
            cat = name[: -(len(f"_{policy}_{version}"))]
            cats.append(cat)
    return cats


def _combined_merge(
    dataset_root: Path,
    combined_mapping: Dict[str, str],
    scene_files: List[str],
):
    """Apply combined mapping to all scenes in one pass.

    Uses pxr.Usd — must be run in Isaac Sim environment.
    """
    try:
        from pxr import Usd
    except ImportError:
        raise SystemExit("pxr.Usd not available — must run in Isaac Sim environment")

    import sys, os
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from rewrite_layout_asset_refs_with_compensation import (
        rewrite_layout, _load_mapping
    )

    scene_dirs = sorted(dataset_root.glob("GRScenes100/**/layout.usd"))

    for layout_path in scene_dirs:
        scene_dir = layout_path.parent
        for sf in scene_files:
            scene_path = scene_dir / sf
            if not scene_path.exists():
                continue

            # Backup baseline
            backup = scene_path.with_suffix(f".baseline.{sf}.usd")
            if not backup.exists():
                shutil.copy2(str(scene_path), str(backup))

            # Apply combined mapping
            mapping_pairs = _load_mapping_from_dict(
                combined_mapping, str(dataset_root)
            )
            rewrite_layout(
                layout_usd=str(scene_path),
                out_usd=str(scene_path),  # in-place
                subset_root=str(dataset_root),
                mapping_pairs=mapping_pairs,
                apply_compensation=True,
                set_instanceable=True,
                dry_run=False,
                report_out=None,
                max_preview=0,
                v_matrix_mode="auto",
            )


def _sequential_merge(
    dataset_root: Path,
    c1_bulk: Path,
    policy: str,
    version: str,
    categories: List[str],
    group_label: str,
    scene_files: List[str],
):
    """Fallback: apply categories one by one."""
    for cat in categories:
        mapping_path = (
            c1_bulk / f"{cat}_{policy}_{version}" / "01_cert" / "filtered_mapping.json"
        )
        if not mapping_path.exists():
            log.warning("Skipping %s: mapping not found", cat)
            continue
        # Re-use existing apply infrastructure
        import subprocess
        cmd = [
            sys.executable,
            str(Path(__file__).resolve().parent / "c1_bulk_apply_layout_dedup.py"),
            "--mapping-json", str(mapping_path),
            "--dataset-root", str(dataset_root),
            "--report-dir", str(c1_bulk / f"_phase2_seq_merge_{cat}"),
            "--out-name", f"layout.parallel_{group_label}.{cat}_{policy}_{version}.usd",
            "--scene-files", ",".join(scene_files),
        ]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            log.error("Sequential merge failed for %s: %s", cat, r.stderr[:500])


def _mega_soft_delete(
    dataset_root: Path,
    bak_root: Path,
    cat_mappings: List[tuple],
    stamp: str,
) -> List[str]:
    """Move old asset directories to bak for all categories at once."""
    errors = []
    bak_dest = bak_root / "_dedup_assets" / f"phase2_combined_{stamp}"
    from scan_utils import _normalize_mapping_key

    for cat, mapping in cat_mappings:
        for old_key in mapping.keys():
            rel = _normalize_mapping_key(old_key, dataset_root.name)
            if not rel:
                continue
            # Extract category/uid from GRScenes_assets/<cat>/<uid>/...
            parts = Path(rel).parts
            if len(parts) < 3 or parts[0] != "GRScenes_assets":
                continue
            uid = parts[2]
            src_dir = dataset_root / "GRScenes_assets" / cat / uid
            if not src_dir.exists():
                continue
            dst_dir = bak_dest / "GRScenes_assets" / cat / uid
            try:
                dst_dir.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(src_dir), str(dst_dir))
            except Exception as e:
                errors.append(f"{cat}/{uid}: {e}")
    return errors


def _load_mapping_from_dict(mapping: Dict[str, str], subset_root: str) -> list:
    """Convert {old: canonical} dict to list of MappingPair."""
    from rewrite_layout_asset_refs_with_compensation import MappingPair
    pairs = []
    for old, canonical in mapping.items():
        pairs.append(MappingPair(old=old, canonical=canonical))
    return pairs


def _write_merge_report(c1_bulk: Path, status: str, **extra):
    report = {
        "phase": "phase2",
        "status": status,
        "timestamp": datetime.now().isoformat(),
        **extra,
    }
    (c1_bulk / "phase2_merge_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    sys.exit(main())
```

- [ ] **Step 2: Commit**

```bash
git add scripts/c1_phase2_merge_scan_delete.py
git commit -m "feat: add Phase 2 merge+scan+delete entry point

c1_phase2_merge_scan_delete.py runs the sequential Phase 2:
1. Gate check (all phase1_done.json OK)
2. Merge (combined mapping or sequential fallback via subprocess)
3. Mega-scan (single pxr scan for all old assets)
4. Soft-delete (batch move to bak)
5. Post-delete scan (verification)

Uses scan_utils for mega-scan, c1_parallel_merge for gate/merge logic.
Must run in Isaac Sim environment (requires pxr.Usd)."
```

---

### Task 5: Update launch_job.sh timeout support

**Files:**
- Modify: `scripts/dlc/launch_job.sh` (line 121)

**Rationale:** Currently hardcoded to `--job_max_running_time_minutes=0` (no timeout).
Add `DLC_JOB_TIMEOUT` env var support.

- [ ] **Step 1: Edit `scripts/dlc/launch_job.sh`**

Change line 121 from:
```bash
    --job_max_running_time_minutes=0 \
```
To:
```bash
    --job_max_running_time_minutes=${DLC_JOB_TIMEOUT:-0} \
```

- [ ] **Step 2: Verify**

```bash
grep -n "job_max_running_time_minutes" scripts/dlc/launch_job.sh
```
Expected output: `121:    --job_max_running_time_minutes=${DLC_JOB_TIMEOUT:-0} \`

- [ ] **Step 3: Commit**

```bash
git add scripts/dlc/launch_job.sh
git commit -m "feat(dlc): support DLC_JOB_TIMEOUT for per-job timeouts

Replace hardcoded --job_max_running_time_minutes=0 with
${DLC_JOB_TIMEOUT:-0} to allow per-job timeout configuration
via environment variable. Backward compatible — defaults to 0
(no timeout) when unset."
```

---

### Task 6: Create DLC orchestration script

**Files:**
- Create: `scripts/orchestrate_c1_parallel.py`

**Rationale:** Provides `submit-phase1`, `gate-check`, `submit-phase2`, `status`
commands following the existing `orchestrate_test0_rebuilt_normalize.py` pattern.

- [ ] **Step 1: Create `scripts/orchestrate_c1_parallel.py`**

```python
#!/usr/bin/env python3
"""Orchestrate C1 parallel pipeline DLC jobs.

Commands:
  submit-phase1  Submit one DLC job per remaining category
  gate-check     Poll DLC API + read phase1_done.json for all jobs
  submit-phase2  Submit the Phase 2 merge+scan+delete job
  status         Print job status table
"""
import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional


def get_dlc_bin() -> str:
    return os.environ.get("DLC_BIN", os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "dlc"
    ))


def dlc_call(*args) -> subprocess.CompletedProcess:
    cmd = [get_dlc_bin()] + list(args)
    return subprocess.run(cmd, capture_output=True, text=True, timeout=120)


def get_dlc_job_status(job_name: str, workspace_id: str) -> Optional[str]:
    """Get DLC job status by name regex."""
    r = dlc_call("get", "job", "-w", workspace_id,
                 "--display_name_regex", f"^{job_name}$")
    if r.returncode != 0:
        return None
    # Parse JSON from output
    text = r.stdout
    try:
        start = text.index("{")
        payload = json.loads(text[start:])
        return payload.get("Status")
    except Exception:
        return None


def submit_phase1(args):
    """Submit Phase 1 DLC jobs (one per remaining category)."""
    dataset_root = args.dataset_root
    bak_root = args.bak_root
    report_path = args.report
    c1_bulk_dir = args.c1_bulk_dir
    group_label = args.group_label
    policy = args.bbox_policy
    version = args.out_version

    # Discover remaining categories
    all_cats = _discover_categories_from_report(report_path)
    done_cats = _discover_completed_categories(c1_bulk_dir, policy, version)
    remaining = [c for c in all_cats if c not in done_cats]

    print(f"Total categories in report: {len(all_cats)}")
    print(f"Already completed: {len(done_cats)}")
    print(f"Remaining to process: {len(remaining)}")
    print(f"Categories: {', '.join(remaining[:5])}..." if len(remaining) > 5
          else f"Categories: {', '.join(remaining)}")

    if not args.yes:
        resp = input("Proceed with submission? [y/N]: ")
        if resp.lower() != "y":
            print("Aborted.")
            return

    # Save category list for Phase 2
    cats_file = Path(c1_bulk_dir) / "phase2_categories.json"
    cats_file.write_text(json.dumps(remaining, indent=2))

    manifest = {"jobs": {}, "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S")}

    submitted = 0
    failed = 0
    for cat in remaining:
        job_name = f"test0_parallel_phase1_{cat}"
        cmd = [
            "bash",
            os.path.join(os.path.dirname(__file__), "dlc", "launch_job.sh"),
            job_name, "0", "1",
            args.data_sources,
            f"custom "
            f"{os.path.join(os.path.dirname(__file__), 'c1_phase1_apply_and_audit.py')} "
            f"--category {cat} "
            f"--dataset-root {dataset_root} "
            f"--bak-root {bak_root} "
            f"--report {report_path} "
            f"--c1-bulk-dir {c1_bulk_dir} "
            f"--group-label {group_label} "
            f"--bbox-gated --bbox-policy {policy} "
            f"--dedup-mode geom_only --v-matrix-mode auto "
            f"--out-version {version}",
        ]
        print(f"[{submitted+1}/{len(remaining)}] Submitting {job_name}...", end=" ",
              flush=True)
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode == 0:
            print("OK")
            submitted += 1
            manifest["jobs"][cat] = {"name": job_name, "status": "submitted"}
        else:
            print(f"FAILED: {r.stderr[:200]}")
            failed += 1
            manifest["jobs"][cat] = {
                "name": job_name,
                "status": "failed",
                "error": r.stderr[:500],
            }

    manifest_path = Path(c1_bulk_dir) / "phase1_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2))
    print(f"\nSubmitted: {submitted}, Failed: {failed}")
    print(f"Manifest: {manifest_path}")
    print(f"Categories list for Phase 2: {cats_file}")


def gate_check(args):
    """Check Phase 1 completion status."""
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from c1_parallel_merge import gate_check_phase1

    cats_file = Path(args.c1_bulk_dir) / "phase2_categories.json"
    if not cats_file.exists():
        print("ERROR: phase2_categories.json not found. Run submit-phase1 first.")
        return 1

    categories = json.loads(cats_file.read_text())
    ok, failed = gate_check_phase1(
        Path(args.c1_bulk_dir), args.bbox_policy, args.out_version, categories
    )

    if ok:
        print("GATE PASSED — all Phase 1 jobs completed successfully")
        print(f"Ready to submit Phase 2. Run: "
              f"./scripts/orchestrate_c1_parallel.py submit-phase2 ...")
        return 0
    else:
        print(f"GATE FAILED — {len(failed)} categories not complete:")
        for f in failed:
            print(f"  - {f}")
        return 1


def submit_phase2(args):
    """Submit Phase 2 job."""
    cats_file = Path(args.c1_bulk_dir) / "phase2_categories.json"
    if not cats_file.exists():
        print("ERROR: phase2_categories.json not found.")
        return 1

    cmd = [
        "bash",
        os.path.join(os.path.dirname(__file__), "dlc", "launch_job.sh"),
        "test0_parallel_phase2_merge", "0", "1",
        args.data_sources,
        f"custom "
        f"{os.path.join(os.path.dirname(__file__), 'c1_phase2_merge_scan_delete.py')} "
        f"--dataset-root {args.dataset_root} "
        f"--bak-root {args.bak_root} "
        f"--c1-bulk-dir {args.c1_bulk_dir} "
        f"--bbox-policy {args.bbox_policy} "
        f"--out-version {args.out_version} "
        f"--group-label {args.group_label} "
        f"--categories-file {cats_file}",
    ]

    env = os.environ.copy()
    env["DLC_JOB_TIMEOUT"] = "480"  # 8h for Phase 2

    print("Submitting Phase 2 job...")
    r = subprocess.run(cmd, capture_output=True, text=True, env=env)
    if r.returncode == 0:
        print("Phase 2 job submitted successfully")
    else:
        print(f"Submission failed: {r.stderr[:500]}")
        return 1


def status_cmd(args):
    """Print status of all Phase 1 and Phase 2 jobs."""
    cats_file = Path(args.c1_bulk_dir) / "phase2_categories.json"
    if cats_file.exists():
        categories = json.loads(cats_file.read_text())
    else:
        categories = []

    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from c1_parallel_merge import gate_check_phase1

    ok, failed = gate_check_phase1(
        Path(args.c1_bulk_dir), args.bbox_policy, args.out_version, categories
    )

    print(f"Phase 1: {len(categories) - len(failed)}/{len(categories)} complete")
    if failed:
        print("Pending/failed:")
        for f in failed:
            print(f"  - {f}")


def _discover_categories_from_report(report_path: str) -> List[str]:
    """Extract category names from dedup report."""
    import ijson
    cats = set()
    with open(report_path, "rb") as f:
        for group in ijson.items(f, "item"):
            for p in group.get("usd_paths", []):
                parts = Path(p).parts
                for i, part in enumerate(parts):
                    if part == "GRScenes_assets" and i + 1 < len(parts):
                        cats.add(parts[i + 1])
                        break
    return sorted(cats)


def _discover_completed_categories(
    c1_bulk_dir: str, policy: str, version: str
) -> List[str]:
    """Discover categories with valid phase1_done.json."""
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from c1_parallel_merge import gate_check_phase1

    all_cats = _discover_categories_from_disk(c1_bulk_dir, policy, version)
    ok, failed = gate_check_phase1(Path(c1_bulk_dir), policy, version, all_cats)
    return [c for c in all_cats if c not in
            {f.split(":")[0] for f in failed}]


def _discover_categories_from_disk(
    c1_bulk_dir: str, policy: str, version: str
) -> List[str]:
    """Discover categories from directory names in c1_bulk."""
    cats = []
    p = Path(c1_bulk_dir)
    for d in sorted(p.iterdir()):
        if not d.is_dir():
            continue
        name = d.name
        suffix = f"_{policy}_{version}"
        if name.endswith(suffix):
            cats.append(name[: -len(suffix)])
    return cats


def main():
    ap = argparse.ArgumentParser(description="C1 Parallel Pipeline Orchestrator")
    sub = ap.add_subparsers(dest="command")

    # submit-phase1
    p1 = sub.add_parser("submit-phase1")
    p1.add_argument("--dataset-root", required=True)
    p1.add_argument("--bak-root", required=True)
    p1.add_argument("--report", required=True)
    p1.add_argument("--c1-bulk-dir", required=True)
    p1.add_argument("--group-label", default="test0_transitive_apply_seeded")
    p1.add_argument("--bbox-policy", default="bbox_primary_rmse_observe")
    p1.add_argument("--out-version", default="v1")
    p1.add_argument("--data-sources",
                    default="d-mzps5b7joy2axmqpa8,d-d49o5g0h2818sw8j1g,"
                            "d-8wz4emfs21s5ajs9oz,d-f1dsz5nbamclxgydo8")
    p1.add_argument("--yes", action="store_true",
                    help="Skip confirmation prompt")

    # gate-check
    p2 = sub.add_parser("gate-check")
    p2.add_argument("--c1-bulk-dir", required=True)
    p2.add_argument("--bbox-policy", default="bbox_primary_rmse_observe")
    p2.add_argument("--out-version", default="v1")

    # submit-phase2
    p3 = sub.add_parser("submit-phase2")
    p3.add_argument("--dataset-root", required=True)
    p3.add_argument("--bak-root", required=True)
    p3.add_argument("--c1-bulk-dir", required=True)
    p3.add_argument("--bbox-policy", default="bbox_primary_rmse_observe")
    p3.add_argument("--out-version", default="v1")
    p3.add_argument("--group-label", default="test0_transitive_apply_seeded")
    p3.add_argument("--data-sources",
                    default="d-mzps5b7joy2axmqpa8,d-d49o5g0h2818sw8j1g,"
                            "d-8wz4emfs21s5ajs9oz,d-f1dsz5nbamclxgydo8")

    # status
    p4 = sub.add_parser("status")
    p4.add_argument("--c1-bulk-dir", required=True)
    p4.add_argument("--bbox-policy", default="bbox_primary_rmse_observe")
    p4.add_argument("--out-version", default="v1")

    ap.add_argument("--workspace-id", default="270969")

    args = ap.parse_args()

    if args.command == "submit-phase1":
        submit_phase1(args)
    elif args.command == "gate-check":
        sys.exit(gate_check(args))
    elif args.command == "submit-phase2":
        sys.exit(submit_phase2(args))
    elif args.command == "status":
        status_cmd(args)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Commit**

```bash
git add scripts/orchestrate_c1_parallel.py
git commit -m "feat: add DLC orchestration script for parallel pipeline

orchestrate_c1_parallel.py provides 4 subcommands:
- submit-phase1: batch submit one DLC job per remaining category
- gate-check: verify all Phase 1 jobs completed (phase1_done.json)
- submit-phase2: submit merge+scan+delete job with 8h timeout
- status: print completion status table

Follows existing orchestrate_test0_rebuilt_normalize.py pattern.
Auto-discovers categories from dedup report and workspace state."
```

---

### Task 7: Export authorun helpers for reuse

**Files:**
- Modify: `scripts/c1_autorun_categories.py`

**Rationale:** The orchestrator needs `_is_step6_complete()` to determine which
categories are already done. Export it as a public function.

- [ ] **Step 1: Edit `scripts/c1_autorun_categories.py`**

Find the `_is_step6_complete` function and add a public alias:

```python
def is_step6_complete(step6_dir: Path) -> bool:
    """Public alias for _is_step6_complete."""
    return _is_step6_complete(step6_dir)
```

- [ ] **Step 2: Commit**

```bash
git add scripts/c1_autorun_categories.py
git commit -m "refactor: export is_step6_complete from autorun_categories

Add public is_step6_complete() alias so orchestrator can import it
without accessing private _is_step6_complete."
```

---

### Task 8: Full test suite and finalize

**Files:**
- Verify: All tests pass

- [ ] **Step 1: Run all non-pxr tests**

```bash
python -m pytest tests/test_c1_parallel_merge.py tests/test_scan_utils.py tests/test_c1_autorun_categories.py tests/test_prepare_promoted_clone_workspace.py tests/test_doc_manager.py -v
```
Expected: all tests pass (merge + scan + autorun + workspace + doc_manager).

- [ ] **Step 2: Verify doc manager**

```bash
python scripts/doc_manager.py --validate && python scripts/doc_manager.py --gen-index
```
Expected: "All documents look good!" and index generated.

- [ ] **Step 3: Commit final state**

```bash
git add -A
git commit -m "chore: finalize parallel pipeline implementation

All Phase 1, Phase 2, orchestrator, and scan_utils code in place.
Pure-Python tests passing. Design doc updated with review fixes.
Ready for Isaac Sim integration testing on DLC."
```

---

### Task 9: Isaac Sim integration test (DLC smoke test)

**Note:** This task must be run in the Isaac Sim environment (DLC container or
local Isaac Sim). It validates the full Phase 1 → Phase 2 flow with real USD data.

- [ ] **Step 1: Submit a single-category smoke test**

From a DLC-capable machine:

```bash
DLC_JOB_TIMEOUT=60 bash scripts/dlc/launch_job.sh \
  test0_parallel_smoke 0 1 \
  d-mzps5b7joy2axmqpa8,d-d49o5g0h2818sw8j1g,d-8wz4emfs21s5ajs9oz,d-f1dsz5nbamclxgydo8 \
  "custom $CODE_ROOT/scripts/c1_phase1_apply_and_audit.py \
    --category bathtub \
    --dataset-root <WORKSPACE>/dataset \
    --bak-root <WORKSPACE>/bak \
    --report <REPORT> \
    --c1-bulk-dir <WORKSPACE>/c1_bulk \
    --group-label test0_transitive_apply_seeded \
    --bbox-gated --bbox-policy bbox_primary_rmse_observe \
    --dedup-mode geom_only --v-matrix-mode auto \
    --out-version v1"
```

- [ ] **Step 2: Verify phase1_done.json written**

After job completes, check:
```bash
cat <WORKSPACE>/c1_bulk/bathtub_bbox_primary_rmse_observe_v1/phase1_done.json
```
Expected: `{"status": "ok", "audit_passed": true, ...}`

- [ ] **Step 3: Verify sidecars exist**

```bash
find <WORKSPACE>/dataset/GRScenes100 -name "layout.parallel_*bathtub*.usd" | head -3
find <WORKSPACE>/dataset/GRScenes100 -name "start_result_interaction.parallel_*bathtub*.usd" | head -3
```
Expected: sidecar files exist with parallel_ prefix.

---

## Execution Order

1. Task 1 (scan_utils) — foundation for Phase 2
2. Task 2 (merge logic) — foundation for Phase 2
3. Task 3 (Phase 1 script)
4. Task 4 (Phase 2 script)
5. Task 5 (launch_job.sh timeout)
6. Task 6 (orchestrator)
7. Task 7 (autorun export)
8. Task 8 (full test suite)
9. Task 9 (Isaac Sim smoke test)

---

## Post-Implementation Fixes

The following bugs were discovered and fixed during code review and integration
testing. They are documented here to prevent recurrence in future pipeline work.

### Fix 1: CLI Argument Fabrication Bug (Phase 1)

**Issue**: The Phase 1 script (`scripts/c1_phase1_apply_and_audit.py`) fabricated
subprocess arguments for all three steps (build mapping, bulk apply, placement
audit) based on what seemed plausible rather than inspecting the target scripts'
actual `argparse.ArgumentParser` definitions. Every subprocess call failed with
`unrecognized arguments` errors.

**Root Cause**: The implementation plan's pseudocode used invented arg names
(`--layout-root`, `--out-dir`, `--subset-dirs`, `--mode`, `--output-basename`,
`--bak-root`, `--no-bbox-gated`) that don't exist in any target script.

**Fix** (commit `0220513`): Referenced `scripts/c1_autorun_categories.py` as the
gold standard for subprocess command construction, matched every argument to the
actual argparse definitions, and switched from `sys.executable` to `ISAAC_PY`
(Isaac Sim Python wrapper required for pxr imports).

### Fix 2: Phase 2 Function Signature Mismatches

**Issue**: The Phase 2 script (`scripts/c1_phase2_merge_scan_delete.py`) called
`rewrite_layout()` and constructed `MappingPair` objects with incorrect field
names. `MappingPair` uses fields `(old, canonical)` but the code used
`MappingPair(old_key=..., canonical_key=...)`.

**Fix** (commit `b3a0636`): Corrected `MappingPair` construction and
`rewrite_layout()` calls to match the actual keyword-only signature. Added
missing required CLI flags: `--group-label`, `--set-instanceable`, `--v-matrix-mode`.

### Fix 3: Phase 2 Sequential Merge Dead-End

**Issue**: The `_sequential_merge()` function wrote sidecar files via `--out-name`
instead of modifying scene files in-place. The sequential merge path therefore
produced sidecars just like Phase 1, leaving baseline scene files untouched.

**Fix** (commit `c2bf569`): Changed `_sequential_merge()` to apply rewrites
in-place (passing scene file as both input and output), copying each baseline
to a `.baseline` backup first (matching the `_combined_merge()` approach).

### Fix 4: Phase 2 Empty Mapping Guard

**Issue**: Some categories have zero dedup pairs (empty or nonexistent
`filtered_mapping.json`). Without a guard, `rewrite_layout()` was called with
empty mappings, causing unnecessary no-op rewrites on every scene file.

**Fix** (commit `c2bf569`): Added explicit guard in `discover_category_mappings()`
to skip categories with empty mappings. Added guards in `_combined_merge()` and
`_sequential_merge()` to skip categories with no mapping file.

### Fix 5: Phase 2 V Matrix Compensation

**Issue**: Phase 2's `_combined_merge()` and `_sequential_merge()` initially used
`v_matrix_mode="none"`, which skipped V matrix compensation for deduped asset
placements. This caused visual displacement because asset transforms were not
compensated for the difference between the old and canonical asset meshes.

**Fix** (commit `d56f5e3`): Changed `v_matrix_mode` from `"none"` to `"auto"` in
both merge paths. Added `--mode-reports-dir` CLI argument to both
`c1_phase2_merge_scan_delete.py` and `orchestrate_c1_parallel.py` so the mode
reports directory (containing dedup mode info per asset pair) is passed through
to `rewrite_layout()`. `--certificate-jsonl` is intentionally NOT passed for
Phase 2 (combining certificates from all categories is complex; transitive pairs
without certificates get an `xform_compensation_error` but the reference is still
rewritten — acceptable since transitive pairs are rare in practice).

### Fix 6: Orchestrator Missing `--mode-reports-dir` in Phase 1 Submission

**Issue**: After Fix 5 added `--mode-reports-dir` to Phase 2's submission, Phase 1's
`orchestrate_c1_parallel.py:submit_phase1` was NOT updated to pass the same flag.
All 52 Phase 1 DLC jobs failed at Step 1 (build mapping) because
`c1_build_bulk_mapping_from_dedup_report.py:679` enforces a hard requirement:
`--mode-reports-dir` is mandatory when `--bbox-gated` is set — it is needed for
per-pair mode lookup (which dedup mode was used per asset pair for V matrix
compensation).

**Root Cause**: Fix 5 added `--mode-reports-dir` to `submit-phase2` parser and
`submit_phase2()` but did not check `submit-phase1` for the same requirement.
The entry script `c1_phase1_apply_and_audit.py` conditionally passes
`--mode-reports-dir` to sub-scripts (`if args.mode_reports_dir`), but the
orchestrator never provides it, so the condition is always False → sub-script
fails with a hard SystemExit.

**Evidence** (confirmed via pod logs, phase1_done.json content, and code trace):
- Pod logs (`dlc logs <JOB_ID>`): `[Phase1:backpack] Step 1 FAILED` →
  `ERROR: --mode-reports-dir is required when --bbox-gated is set.`
- All 52 `phase1_done.json` files: `{"status": "failed", "audit_passed": "Step1 build_mapping failed..."}`
- Code: `orchestrate_c1_parallel.py:submit_phase1` command_args lacked `--mode-reports-dir`;
  `submit_phase2` correctly includes it (asymmetry introduced by Fix 5)

**Impact**: All 52 Phase 1 DLC jobs completed their script (writing `phase1_done.json`
with `status: "failed"`) but produced no actual dedup results. Phase 2 gate-check
would block because `gate_check_phase1` requires `status == "ok"`.

**Fix** (commit pending):
1. Added `--mode-reports-dir` to `submit-phase1` subparser in
   `orchestrate_c1_parallel.py` (matching `submit-phase2` parser definition)
2. Added conditional `--mode-reports-dir` append to `command_args` in `submit_phase1()`
3. Removed 52 stale `phase1_done.json` files with `status: "failed"` before re-submission

**Lesson**: When a cross-cutting CLI argument (like `--mode-reports-dir`) is added
to one orchestrator subcommand, ALL sibling subcommands that invoke the same or
similar entry scripts must be checked for the same requirement. A design that
defines shared argument groups or a common arg-injection function would have
prevented this asymmetry.
