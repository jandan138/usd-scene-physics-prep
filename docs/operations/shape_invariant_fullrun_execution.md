---
title: "Shape-Invariant Full-Run Execution: DLC Scan + Union Merge + C1 Soft-Delete"
code_reference: scripts/report_asset_mesh_dedup.py
created_at: "2026-03-11"
updated_at: "2026-03-11"
maintainer: "zhuzihou"
status: "in_progress"
---

# Shape-Invariant Full-Run Execution

## Background

### Prior Work

1. **geom_only dedup** (Phase 2, 2026-03-09): Full-dataset scan with `--merge-tolerance 0.005` found 8,229 removable assets (15.6% of 52,904). C1 soft-delete completed: 8,091 assets removed.
2. **shape_invariant mode** (2026-03-11): New `--mode shape_invariant` implemented in `report_asset_mesh_dedup.py` to catch scale-variant duplicates missed by hash-based geom_only matching.
3. **Key finding**: shape_invariant is COMPLEMENTARY to geom_only, not a superset. Union of both modes yields best coverage.

### Goal

Run shape_invariant dedup across all 79 categories via DLC, union-merge with existing geom_only reports, then execute C1 soft-delete pipeline on the incremental (new) duplicates.

---

## Phase 1: DLC Shape-Invariant Scan

### Setup

**Script**: `scripts/dlc/submit_shape_invariant_dedup.sh`
**Mode**: shape_invariant via `scripts/dlc/dedup_by_category.py`
**Parameters**: `--hausdorff-threshold 0.05 --merge-tolerance 0.005 --float-quantize-eps 1e-2`
**Output**: `check_reports/shape_invariant_full/`

### Job Submission (~08:33 UTC)

- 79 jobs submitted: `shape_inv_dedup_0_0_79` through `shape_inv_dedup_78_78_79`
- First JobId: `dlc102hog9eiz5nh`, last: `dlc15m9k3t787clz`

### Job Progress

| Check | Time | Succeeded | Running | Failed | Notes |
|-------|------|-----------|---------|--------|-------|
| 1st | ~08:38 | 48 | 25 | 0 | 7 EnvPreparing |
| 2nd | ~08:42 | 61 | 16 | 0 | 3 stuck in EnvPreparing (jobs 35,36,37) |
| 3rd | ~08:48 | 70+ | 10 | 3 | Retry 1: jobs 35,36,37 resubmitted |
| 4th | ~08:55 | 72 | 4 | 3 | bed/plant/shelf dead, retry 2 submitted |

### Retries

| Retry | Chunks | Categories | Reason |
|-------|--------|------------|--------|
| Retry 1 | 35,36,37 | lamp,laptop,light | EnvPreparing timeout (~8min) |
| Retry 2 | 3,52,56 | bed,plant,shelf | Job killed mid-scan |

### Current Status (as of ~08:57 UTC)

| Category | Chunk | Assets | Final Report | Notes |
|----------|-------|--------|-------------|-------|
| 71 categories | various | varies | YES | Complete |
| cabinet | 11 | 1,278 | NO | Running (600/1277) |
| light | 37 | 164 | NO | Running (100/164, retry 1) |
| other | 44 | 12,210 | NO | Running (2000/12209) |
| window | 78 | 825 | NO | Running (500/825) |
| bed | 3 | 141 | NO | Retry 2 submitted |
| plant | 52 | 355 | NO | Retry 2 submitted |
| shelf | 56 | 200 | NO | Retry 2 submitted |

---

## Phase 2: geom_only Reports Validation

**Result**: 79/79 reports complete and valid. All in `check_reports/normalized_v2_dedup/<cat>/`.

| Metric | Value |
|--------|-------|
| geom_only reports | 79/79 |
| Missing reports | 0 |
| Total assets | 52,904 |
| Removable (geom_only) | 8,229 (15.6%) |
| Categories with dupes | 26/79 |
| Errors | 0 |

---

## Phase 3: Union Merge (Partial — 70/79 categories)

### Command

```bash
python scripts/union_dedup_reports.py --batch \
  --geom-dir check_reports/normalized_v2_dedup/ \
  --shape-dir check_reports/shape_invariant_full/ \
  --output-dir check_reports/union_merged_full/ \
  --summary check_reports/union_merged_full/summary.json
```

### Partial Results (70 categories, 36,856 assets)

| Metric | geom_only | shape_invariant | Union |
|--------|-----------|-----------------|-------|
| Removable | 5,229 | 15,450 | **20,679** |
| Dedup rate | 14.2% | 41.9% | **56.1%** |
| Categories with dupes | — | — | 29/70 |

### Top Categories Comparison

| Category | Total | geom_only | shape_inv | Union | shape-only adds |
|----------|-------|-----------|-----------|-------|-----------------|
| wall | 15,961 | 2,729 | 7,776 | 10,505 (65.8%) | 7,776 |
| ground | 10,107 | 1,138 | 5,773 | 6,911 (68.4%) | 5,773 |
| bottle | 1,698 | 761 | 621 | 1,382 (81.4%) | 621 |
| book | 1,595 | 168 | 448 | 616 (38.6%) | 448 |
| plate | 426 | 203 | 232 | 435 (102.1%)* | 232 |
| column | 401 | 76 | 185 | 261 (65.1%) | 185 |
| ceiling | 1,610 | 48 | 209 | 257 (16.0%) | 209 |
| cup | 549 | 37 | 104 | 141 (25.7%) | 104 |
| picture | 273 | 15 | 18 | 33 (12.1%) | 18 |
| pen | 414 | 7 | 21 | 28 (6.8%) | 21 |
| pillow | 651 | 18 | 10 | 28 (4.3%) | 10 |
| box | 97 | 2 | 19 | 21 (21.6%) | 19 |

*plate >100% indicates union-find overlap between geom_only and shape_invariant groups

### Missing 9 Categories (pending DLC completion)

bed, cabinet, door, light, other, plant, shelf, window — total 16,048 assets.
Expected to significantly increase totals, especially `other` (12,210 assets).

**Full merge will be re-run after all 79 reports are available.**

---

## Phase 4: C1 Soft-Delete

**Status**: PENDING — blocked on Phase 3 full merge

### Planned Approach

Prerequisites (from c1-researcher findings):
```bash
# 1. Concatenate per-category union reports into single JSON for --report
python3 -c "
import json, glob
all_dupes = []
for f in sorted(glob.glob('check_reports/union_merged_full/*/*.json')):
    if '_union_merged.json' in f:
        data = json.load(open(f))
        all_dupes.extend(data.get('duplicates', []))
combined = {'meta': {'mode': 'union_merged_all'}, 'duplicates': all_dupes}
with open('check_reports/union_merged_full/all_categories_union_merged.json', 'w') as out:
    json.dump(combined, out, indent=2)
"

# 2. Delete old geom_only mappings
C1_BULK_DIR=check_reports/union_merged_full/c1_bulk
rm -f "$C1_BULK_DIR"/*_geom_only_mapping.json
rm -f "$C1_BULK_DIR"/*_geom_only_mapping.stats.json
```

Command:
```bash
python3 scripts/c1_autorun_categories.py \
  --dataset-root GRScenes-test1-normalized \
  --bak-root GRScenes-test1-normalized_bak \
  --report check_reports/union_merged_full/all_categories_union_merged.json \
  --c1-bulk-dir "$C1_BULK_DIR" \
  --group-label c1_union \
  --out-version v2 \
  --no-skip-done
```

### Key Gotchas

1. **Use a namespaced `--c1-bulk-dir`**: isolates mappings and step6 state from earlier runs or other datasets
2. **Delete old mappings**: `*_geom_only_mapping.json` files → stale mappings reused
3. **Single JSON report**: Union merge outputs per-category → must concatenate
4. **`--group-label c1_union`**: Distinguish from Phase 2's `c1_autorun` label
5. **`--out-version v2`**: Avoid overwriting Phase 2 step6 dirs

---

## Phase 5: Verification

**Status**: PENDING

---

## Summary

### Overall Dedup Improvement (partial, 70/79 categories)

| Stage | Assets | Removable | Rate |
|-------|--------|-----------|------|
| geom_only baseline | 52,904 | 8,229 | 15.6% |
| + shape_invariant (70 cats partial) | 36,856 | 20,679 | 56.1% |
| Full 79-cat union | TBD | TBD | TBD |

### Pilot vs Full-Run Comparison

| Category | Pilot Union | Full-Run Union | Match? |
|----------|-------------|----------------|--------|
| bottle | 812 (47.8%) | 1,382 (81.4%) | Different — full run finds more |
| cup | 122 (22.2%) | 141 (25.7%) | Higher |
| pen | 24 (5.8%) | 28 (6.8%) | Higher |
| plate | 237 (55.6%) | 435 (102.1%) | Higher (overlap issue) |

Note: Full-run union includes cross-group merges that pilot (4-category) couldn't detect.

---

## Timeline

| Phase | Status | Timestamp |
|-------|--------|-----------|
| Phase 1: DLC submission | 71/79 complete, 8 pending | 08:33 - ongoing |
| Phase 2: geom_only validation | Complete (79/79) | 08:33 |
| Phase 3: Union merge (partial) | 70/79 merged | 08:50 |
| Phase 3: Union merge (full) | PENDING | — |
| Phase 4: C1 soft-delete | PENDING | — |
| Phase 5: Verification | PENDING | — |

---

## Appendix

### Key Files

| File | Purpose |
|------|---------|
| `scripts/report_asset_mesh_dedup.py` | Dedup scanner (geom_only + shape_invariant modes) |
| `scripts/union_dedup_reports.py` | Union-find merger for geom_only + shape_invariant reports |
| `scripts/c1_autorun_categories.py` | Automated C1 soft-delete pipeline |
| `scripts/dlc/submit_shape_invariant_dedup.sh` | DLC batch submission for 79 shape_invariant jobs |
| `tests/test_shape_invariant.py` | Unit tests (30 tests, all pass) |

### Related Documents

- [Shape-Invariant Dedup Proposal](dedup_shape_invariant_proposal.md) — Algorithm design and rationale
- [Phase 2: Normalized Dedup Execution](normalized_dedup_phase2_execution.md) — Previous geom_only full-run
- [Re-normalization V2 Execution](renormalization_v2_execution.md) — V2 normalization pipeline

### Commits

- `ddd2675` feat(dedup): add --mode shape_invariant for scale/order-invariant asset dedup
- `578fa5d` feat(dedup): add union merge script, DLC shape_invariant support, minor fixes
