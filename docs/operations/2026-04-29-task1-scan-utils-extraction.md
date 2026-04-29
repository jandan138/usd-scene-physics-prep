---
title: Task 1 — Extract scan_utils library from c1_bulk_step6
code_reference: scripts/scan_utils.py, scripts/c1_bulk_step6_category_promote_scan_soft_delete.py, tests/test_scan_utils.py
created_at: 2026-04-29
updated_at: 2026-04-29
maintainer: zhuzihou
status: complete
---

# Task 1 — Extract scan_utils library from c1_bulk_step6

## Summary
Extracted shared scan functions from the existing C1 bulk step6 script into a reusable `scan_utils.py` library module. This enables both the serial autorun path and the new parallel pipeline's Phase 2 mega-scan to import the same scanning logic without duplication.

## What was done
1. **Created `scripts/scan_utils.py`** — extracted 11 functions/classes:
   - `_write_json`, `_append_jsonl` (write helpers)
   - `_parse_uid_from_report_style_asset_usd` (UID extractor)
   - `_abs_from_usd_ref`, `_abs_to_subset_rel`, `_normalize_mapping_key` (path helpers)
   - `ScanHit` (dataclass)
   - `_scan_stage_for_old_assets` (single-stage pxr scanner)
   - `_emit_progress` (progress emitter)
   - `_iter_usd_files` (USD file enumerator with exclusion rules)
   - `_scan_tree_pxr` (full-tree scanner)
   - Added `.parallel_` file exclusion alongside existing `.pre_` and `.c1_` exclusions
   - Added `build_old_asset_path_set()` and `build_combined_old_asset_path_set()` helpers

2. **Modified `c1_bulk_step6_category_promote_scan_soft_delete.py`** — replaced inline function definitions with imports from `scan_utils`; added `sys.path` setup for import resolution. Remained functions: `_stamp_now`, `_derive_output_suffix`, `main()`.

3. **Created `tests/test_scan_utils.py`** — 16 tests covering:
   - UID parsing (standard, absolute, non-asset paths)
   - USD reference path resolution (relative, absolute, SDF-wrapped, empty)
   - Absolute-to-subset-relative path conversion (with/without dataset name, no match)
   - USD file enumeration with exclusion rules (`.pre_`, `.c1_`, `.parallel_`, directory exclusions)
   - Old asset path set building from mapping JSONs (single, combined)

4. **Minor bugfix in `_iter_usd_files`** — added trailing `/` to `dirpath_str` normalization so directory exclusion patterns match leaf directories (e.g., `/_dedup_assets/` matches `/data/_dedup_assets`).

## Test results
```
16 passed in 0.11s
```
All tests pass with `python -m pytest tests/test_scan_utils.py -v`.

## Decisions
- The `ScanHit` dataclass is included in scan_utils (not the original script) since `_scan_stage_for_old_assets` uses it internally and Phase 2 also needs it.
- The `_PXR_ERR` sentinel is exported from scan_utils so the original script can still check pxr availability in `main()`.
- The `dirpath_str = str(dirpath) + "/"` normalization is a forward-looking bugfix; it does not affect production behavior (where `_dedup_assets` always has subdirectories).

## Commit
`b243839 refactor: extract scan_utils library from c1_bulk_step6`
