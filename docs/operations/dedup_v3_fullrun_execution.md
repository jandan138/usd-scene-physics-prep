---
title: "Dedup V3 Full-Run Execution: topo_filesize + 3-Way Union Merge + C1 Soft-Delete"
code_reference: scripts/union_dedup_reports.py
created_at: "2026-03-11"
updated_at: "2026-03-12"
maintainer: "claude-agent"
status: "completed"
---

# Dedup V3 Full-Run Execution

## Summary

Full-run execution of the 3-mode dedup pipeline across all 79 asset categories (52,904 assets) in the GRScenes-test1-normalized dataset. This run introduced the new `topo_filesize` dedup mode, performed a 3-way union merge, fixed the transitive canonical conflict bug, and completed C1 soft-delete for all categories.

**Final result**: 25,360 assets soft-deleted (47.9% of 52,904), 0 failures, 0 scan aborts.

---

## Phase 1: topo_filesize DLC Scan (79 Jobs)

### Background

The `topo_filesize` mode was designed to catch duplicates missed by both `geom_only` and `shape_invariant` modes. Its primary target: assets with identical topology and materials but with DCC-exporter-reordered vertices/faces that defeat hash-based and coordinate-based comparison.

### Configuration

- **Script**: `scripts/dlc/submit_topo_filesize_dedup.sh`
- **Mode**: `topo_filesize` via `scripts/dlc/dedup_by_category.py`
- **Parameters**: `--filesize-tolerance 0.02 --merge-tolerance 0.005 --float-quantize-eps 1e-2`
- **Output**: `check_reports/topo_filesize_full/`

### Results

| Metric | Value |
|--------|-------|
| Jobs submitted | 79 |
| Jobs completed | 79 |
| Per-mode removable (in 3-way context) | 7,578 |
| Unique contribution over 2-way | +3,477 assets |

---

## Phase 2: Transitive Canonical Conflict Fix

### Problem

The previous C1 soft-delete run (2-way union merge) aborted at book (7/79 categories). The post-promote scan found 5 layouts referencing assets that were in the "old" set but were also canonical in other groups.

### Root Cause

Path format mismatch between dedup report modes:
- `geom_only` reports: **relative** paths (`GRScenes-test1-normalized/GRScenes_assets/...`)
- `shape_invariant` reports: **absolute** paths (`/cpfs/shared/.../GRScenes-test1-normalized/GRScenes_assets/...`)

The `normalize_path()` function in `union_dedup_reports.py` only stripped `./` prefixes, not absolute-to-relative conversion. The union-find treated the same physical asset as two different strings, creating groups where an asset was canonical in one and old in another.

### Fix (3 layers of defense)

1. **`scripts/union_dedup_reports.py`** -- Enhanced `normalize_path()` to detect absolute paths with `GRScenes_assets/` marker and convert to relative `<dataset_name>/GRScenes_assets/...` form.
2. **`scripts/c1_build_bulk_mapping_from_dedup_report.py`** -- Post-processing safety net: removes mapping entries where an old asset is also a canonical target.
3. **`scripts/c1_bulk_step6_category_promote_scan_soft_delete.py`** -- Pre-scan safety net: excludes canonical assets from the old set.

### Impact

The old 2-way union merge number (29,313 removable / 55.4%) was **inflated by 7,662 assets** due to this bug. The corrected 2-way baseline is 21,651 (40.9%).

### Verification

- Book category: 276 groups / 20 conflicts -> 220 groups / 0 conflicts
- All 79 categories: 0 transitive canonical conflicts
- 20 unit tests pass (7 new tests in `tests/test_union_merge.py`)
- Full documentation: `docs/operations/c1_transitive_canonical_fix.md`

---

## Phase 3: 3-Way Union Merge

### Command

```bash
python scripts/union_dedup_reports.py --batch \
  --geom-dir check_reports/normalized_v2_dedup/ \
  --shape-dir check_reports/shape_invariant_full/ \
  --topo-dir check_reports/topo_filesize_full/ \
  --output-dir check_reports/union_merged_3way/ \
  --summary check_reports/union_merged_3way/summary.json
```

### Results Comparison

| Configuration | Removable | Rate | Notes |
|---|---|---|---|
| geom_only (baseline) | 8,229 | 15.6% | Hash-based vertex comparison |
| 2-way OLD (unfixed) | 29,313 | 55.4% | Inflated by abs/rel path bug |
| 2-way FIXED (geom + shape) | 21,651 | 40.9% | Corrected baseline |
| **3-way (geom + shape + topo)** | **25,128** | **47.5%** | Final merge result |

### Per-Mode Contribution

| Mode | Individual removable | Unique adds over other modes |
|---|---|---|
| geom_only | 8,229 | (baseline) |
| shape_invariant | 21,084 | Part of 2-way |
| topo_filesize | 7,578 | +3,477 over 2-way |

### Top Categories (3-way union)

| Category | Total | Union removable | Rate |
|----------|-------|-----------------|------|
| wall | 15,961 | 7,807 | 48.9% |
| other | 12,209 | 7,344 | 60.2% |
| ground | 10,107 | 5,788 | 57.3% |
| bottle | 1,698 | 1,332 | 78.4% |
| book | 1,595 | 824 | 51.7% |
| cup | 549 | 336 | 61.2% |
| plate | 426 | 329 | 77.2% |
| ceiling | 1,610 | 210 | 13.0% |
| column | 401 | 185 | 46.1% |
| pen | 414 | 167 | 40.3% |

### Top Categories by topo_filesize Contribution

| Category | 2-way fixed | 3-way | topo_filesize adds |
|----------|-------------|-------|-------------------|
| other | 5,610 | 7,344 | +1,734 |
| bottle | 812 | 1,332 | +520 |
| book | 510 | 824 | +314 |
| cup | 122 | 336 | +214 |
| pen | 24 | 167 | +143 |
| pillow | 20 | 138 | +118 |
| plate | 237 | 329 | +92 |
| toy | 10 | 65 | +55 |

---

## Phase 4: Bottle Triplet Validation

### Ground Truth

Three known-identical bottle assets with DCC-exporter-reordered vertices/faces:
- `bottle/7861bdaa89323558eb8046679f567498`
- `bottle/79088d12b87f6758da805eb64c8a3582`
- `bottle/79090fe893611281c78d1c237c2f5b64`

Properties: 779 vertices, 1,444 faces, identical materials, GLB file sizes within 4 bytes.

### Results

| Mode | Triplet grouped? | Details |
|---|---|---|
| geom_only | NO (0/3 found) | Vertex reordering prevents hash match |
| shape_invariant | NO (0/3 found) | Same vertex reordering issue |
| topo_filesize | **YES (3/3)** | All in group with 70 members |
| 3-way merged | **YES (3/3)** | All in `union_merge_49` (70 members) |

Canonical asset for the group: `0ef0ff541cbb58e437914cec5a27cfc6` (none of the triplet are canonical; all 3 would be removed).

This confirms the design rationale for `topo_filesize`: it catches duplicates that are invisible to both vertex-hash and coordinate-based methods.

---

## Phase 5: C1 Soft-Delete

### Execution

```bash
python3 scripts/c1_autorun_categories.py \
  --dataset-root GRScenes-test1-normalized \
  --bak-root GRScenes-test1-normalized_bak \
  --report check_reports/union_merged_3way/all_categories_union_merged.json \
  --group-label c1_union_3way \
  --out-version v1 \
  --no-skip-done
```

### Results

| Metric | Value |
|--------|-------|
| Categories submitted | 79 |
| Categories completed | 51 |
| Categories skipped (0 pairs) | 28 |
| Categories failed | 0 |
| Scan aborts | 0 |
| Total assets soft-deleted | 25,360 |
| Start time | 2026-03-11 14:00:31 |
| End time | 2026-03-12 00:52:54 |
| Duration | ~10h 52m |

### Top Categories by Soft-Delete Count

| Category | Assets deleted |
|----------|--------------|
| wall | 7,910 |
| other | 7,370 |
| ground | 5,856 |
| bottle | 1,336 |
| book | 843 |
| cup | 336 |
| plate | 329 |
| ceiling | 210 |
| column | 186 |
| pen | 169 |

### Safety

- All 51 active categories passed both post-promote and post-soft-delete scan gates (0 hits each)
- Zero transitive canonical conflicts thanks to the path normalization fix
- Backups stored in `GRScenes-test1-normalized_bak/_dedup_assets/c1_union_3way_*/`

---

## Key Files

| File | Purpose |
|------|---------|
| `scripts/report_asset_mesh_dedup.py` | Dedup scanner (geom_only, shape_invariant, topo_filesize) |
| `scripts/union_dedup_reports.py` | Union-find merger for N-way report merging |
| `scripts/c1_autorun_categories.py` | Automated C1 soft-delete orchestrator |
| `scripts/c1_build_bulk_mapping_from_dedup_report.py` | old->canonical mapping builder |
| `scripts/c1_bulk_step6_category_promote_scan_soft_delete.py` | Promote/scan/delete logic |
| `scripts/dlc/submit_topo_filesize_dedup.sh` | DLC batch submission for topo_filesize |
| `tests/test_union_merge.py` | Union merge tests (20 tests) |
| `tests/test_topo_filesize.py` | topo_filesize tests (19 tests) |

## Reports

| Path | Contents |
|------|----------|
| `check_reports/topo_filesize_full/` | Per-category topo_filesize dedup reports |
| `check_reports/union_merged_3way/` | Per-category 3-way merged reports |
| `check_reports/union_merged_3way/summary.json` | 3-way merge summary |
| `check_reports/union_merged_2way_fixed/` | Corrected 2-way merge (for comparison) |
| `check_reports/c1_bulk/_autorun/c1_union_3way_20260311_140031/` | C1 run ledger and logs |

## Related Documents

- [Dedup V3 Implementation (topo_filesize)](dedup_v3_implementation.md) -- Algorithm design
- [Dedup V3 Plan (topo_filesize)](dedup_v3_topo_filesize_plan.md) -- Design rationale
- [C1 Transitive Canonical Fix](c1_transitive_canonical_fix.md) -- Bug fix details
- [Shape-Invariant Full-Run](shape_invariant_fullrun_execution.md) -- Prior 2-way run
- [Re-normalization V2 Execution](renormalization_v2_execution.md) -- Dataset preparation
