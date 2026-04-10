---
title: "Fix: C1 Transitive Canonical Conflict (Abs/Rel Path Mismatch)"
code_reference: scripts/union_dedup_reports.py
created_at: "2026-03-11"
updated_at: "2026-03-11"
maintainer: "zhuzihou"
status: "completed"
---

# Fix: C1 Transitive Canonical Conflict

## Problem

C1 soft-delete pipeline aborted at book (7/79 categories) during the union-merged full-run. The post-promote full-tree scan found 5 layouts referencing assets that were in the "old" (to-be-deleted) set but were also canonical (representative) in other groups. The step6 script correctly treated this as a safety violation and aborted.

### Symptom

```
ABORT: post-promote full scan hit_files=5; see .../post_promote_full_usd_scan_excluding_backups_pxr.json
```

Hit assets (all book category):
- `0dda39c7617cee07ab33cd331bf1583f` -- canonical for 18 old assets, but also marked as old
- `0046e19dc7aac2a63a49e24a6fbd76d8` -- canonical for 1 old asset, but also marked as old
- `119c8dbcaaed4bf3eea89fdd9e89ced9` -- canonical for 4 old assets, but also marked as old

## Root Cause

**Path format mismatch between dedup report modes.**

- `geom_only` reports use **relative** paths: `GRScenes-test1-normalized/GRScenes_assets/book/abc.../usd/abc....usd`
- `shape_invariant` reports use **absolute** paths: `/cpfs/shared/.../GRScenes-test1-normalized/GRScenes_assets/book/abc.../usd/abc....usd`

The `union_merge()` function in `union_dedup_reports.py` uses a union-find data structure to merge groups that share assets. But since the same physical asset has two different string representations (abs vs rel), the union-find treats them as different assets, failing to merge groups that should be connected.

This causes:
1. Asset X appears as representative (position 0 = canonical) in a shape_invariant group (absolute path)
2. The same asset X appears as a non-representative member (position >0 = old) in a geom_only group (relative path)
3. The mapping builder creates entries where X is both canonical (value) AND old (key)
4. The post-promote scan finds layouts still referencing X and correctly flags it as a "hit"
5. Step6 aborts because hit_files > 0

### Data evidence (book category, pre-fix)

| Metric | Before fix | After fix |
|--------|-----------|-----------|
| Duplicate groups | 276 | 220 |
| Mapping pairs | 616 | 517 |
| Transitive conflicts | 20 | 0 |
| Scan hit_files | 5 | 0 (expected) |

56 extra groups and 99 extra mapping pairs existed due to the same assets being in separate groups under different path forms.

## Fix (3 layers of defense)

### Layer 1: Path normalization in union merge (PRIMARY FIX)

**File**: `scripts/union_dedup_reports.py`, function `normalize_path()`

Enhanced to detect absolute paths containing the `GRScenes_assets/` marker and convert them to relative report-style form by extracting `<dataset_name>/GRScenes_assets/...`.

This ensures the union-find operates on a single canonical string representation per asset, correctly merging cross-mode groups.

### Layer 2: Safety net in mapping builder

**File**: `scripts/c1_build_bulk_mapping_from_dedup_report.py`

Added post-processing step after building the `old->canonical` mapping: if any key (old asset) is also present as a value (canonical asset), it is removed with a warning. This catches any residual conflicts that slip through normalization.

### Layer 3: Safety net in step6 scan

**File**: `scripts/c1_bulk_step6_category_promote_scan_soft_delete.py`

Before scanning, filters the `old_asset_usd_rel_set` to exclude assets that are also in the canonical set (mapping values). This prevents false positive scan hits even if the mapping still contains conflicts.

## Tests

Added to `tests/test_union_merge.py` (7 new tests, 20 total):

**Path normalization (4 tests)**:
- `test_absolute_path_to_relative` -- absolute path converted to relative
- `test_relative_path_unchanged` -- relative path stays the same
- `test_abs_and_rel_normalize_to_same` -- both forms normalize identically
- `test_backslash_normalized` -- Windows backslashes converted

**Transitive canonical conflict (3 tests)**:
- `test_same_asset_abs_rel_merged_into_one_group` -- 2-way merge with mixed paths produces single group
- `test_no_canonical_in_old_after_merge` -- mapping built from merge has zero conflicts
- `test_n_way_merge_abs_rel_mixed` -- 3-way merge correctly handles abs/rel across all reports

All 20 tests pass.

## Impact

After re-running the union merge with the fix:
- All 79 categories should produce zero transitive canonical conflicts
- C1 soft-delete pipeline should complete without scan aborts
- The union merge will also be more accurate (fewer spurious groups)

## Related Documents

- [Shape-Invariant Full-Run Execution](shape_invariant_fullrun_execution.md)
- [Normalized Dedup Phase 2 Execution](normalized_dedup_phase2_execution.md)
