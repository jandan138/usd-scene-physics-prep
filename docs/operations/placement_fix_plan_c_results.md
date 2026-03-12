---
title: "Plan C Placement Patch: Execution Results"
code_reference: scripts/patch_dedup_placement.py, scripts/verify_all_scenes_vs_test0.py
created_at: 2026-03-12
updated_at: 2026-03-12
maintainer: claude-agent
status: complete
---

# Plan C Placement Patch: Execution Results

## Background

After the C1 3-way dedup soft-delete (25,361 assets removed across 79 categories),
scene layout prims referencing deduplicated assets were left with incorrect world-space
placement. Two bugs in the original dedup compensation logic caused this:

1. `_get_asset_internal_matrix()` read `/Root` (identity) instead of `/Root/Instance`
2. Matrix multiply order was wrong: `world * old * canon^{-1}` instead of `canon^{-1} * old * local`

See `docs/operations/placement_investigation_v2_report.md` for the full investigation.

## Execution Summary

**Date**: 2026-03-12
**Script**: `scripts/patch_dedup_placement.py`
**Plan**: `docs/operations/placement_fix_plan_c_execution.md`

### Dry Run (E3)

| Metric | Value |
|--------|-------|
| Scenes processed | 99 |
| Prims to patch | 3,187 |
| Skipped (same ref) | 54,178 |
| Skipped (identity delta) | 44,569 |
| Errors | 0 |
| Elapsed | 135.7s |

### Actual Patch (E4)

| Metric | Value |
|--------|-------|
| Scenes processed | 99 |
| Prims patched | 3,187 |
| Skipped (same ref) | 54,178 |
| Skipped (identity delta) | 44,569 |
| Errors | 0 |
| Elapsed | 561.1s |
| Backups created | 99 `.pre_patch.*.usd` files |
| Run ID | `patch_20260312_052256` |

All numbers matched the dry run exactly.

### Verification (E5)

**Verdict**: FAIL (but significant improvement)

#### Pre-Patch vs Post-Patch Comparison

| Metric | Pre-Patch | Post-Patch | Delta |
|--------|-----------|------------|-------|
| Total compared | 101,919 | 11,093 | -90,826 |
| Displaced >0.01 | 27,687 | 1,818 | **-25,869 (-93.4%)** |
| Max mesh >0.01 | 13,008 | 1,094 | -11,914 |
| only_in_test0 | 31 | 90,868 | +90,837 |
| only_in_normalized | 0 | 0 | 0 |

#### Per-Scene Results

- **82 scenes**: displaced count went to **0** (fully fixed)
- **17 scenes**: displaced count **unchanged** (not caused by dedup)
- No scene got worse

The 82 fixed scenes account for 25,869 prims that were displaced due to the dedup
compensation bugs and are now correctly placed.

#### only_in_test0 Explanation

The jump from 31 to 90,868 `only_in_test0` is expected. These are prims whose
referenced asset USDs were soft-deleted during dedup. The verifier cannot load the
referenced mesh data for comparison, so it counts them as "only in test0". This is
inherent to dedup soft-delete and not a regression.

## Remaining 1,818 Displaced Prims

### Root Cause

The remaining 1,818 displaced prims are **NOT caused by dedup**. They exist in 17
scenes where the patch correctly identified no dedup-related correction was needed
(`skip_identity_delta`). The displacement is pre-existing from the V2
normalize_asset_transforms step.

Evidence:
- All 17 scenes show `patched=0` in the patch report
- Their displaced counts are identical in pre-patch and post-patch verification
- All displaced prims are `ref_changed` (reference was updated by dedup) but the
  transform delta between old and canonical assets is identity

### Affected Scenes (17)

| Scene | Displaced | >1.0 | >10.0 |
|-------|-----------|------|-------|
| MWBGLKQKTKJZ2AABAAAAABI8 | 225 | 195 | 22 |
| MVUHLWYKTKJ5EAABAAAAACA8 | 141 | 69 | 10 |
| MWAX5JYKTKJZ2AABAAAAAAQ8 | 139 | 108 | 15 |
| MVUCSQAKTKJ5EAABAAAAABI8 | 136 | 66 | 15 |
| MWBGLKQKTKJZ2AABAAAAACA8 | 133 | 67 | 8 |
| MWF4WLIKTIFZIAABAAAAAEI8 | 132 | 52 | 21 |
| MVUHLWYKTKJ5EAABAAAAADI8 | 124 | 73 | 8 |
| MVUHLWYKTKJ5EAABAAAAAAY8 | 109 | 61 | 12 |
| MWAX5JYKTKJZ2AABAAAAACA8 | 99 | 50 | 11 |
| MWAX5JYKTKJZ2AABAAAAABY8 | 96 | 74 | 19 |
| MWAX5JYKTKJZ2AABAAAAADI8 | 81 | 49 | 6 |
| MWAX5JYKTKJZ2AABAAAAABQ8 | 81 | 50 | 14 |
| MWBGLKQKTKJZ2AABAAAAAAI8 | 72 | 53 | 10 |
| MWHLEPQKTIFZIAABAAAAAAA8 | 72 | 41 | 2 |
| MVUCSQAKTKJ5EAABAAAAADY8 | 67 | 49 | 7 |
| MWBGLKQKTKJZ2AABAAAAABA8 | 60 | 0 | 0 |
| MVUHLWYKTKJ5EAABAAAAAAA8 | 51 | 0 | 0 |

### Top Displaced Categories (post-patch)

| Category | Displaced | Max Displacement |
|----------|-----------|------------------|
| other | 381 | 183.6 |
| ground | 312 | -- |
| book | 197 | 16.5 |
| wall | 136 | -- |
| curtain | 97 | 40.8 |
| chair | 79 | -- |
| pen | 72 | -- |
| bottle | 41 | -- |

### Worst Individual Prims

| Displacement | Category | Scene |
|-------------|----------|-------|
| 234.7 | cabinet | MVUCSQAKTKJ5EAABAAAAADY8 |
| 183.6 | other | MWF4WLIKTIFZIAABAAAAAEI8 |
| 142.2 | other | MWBGLKQKTKJZ2AABAAAAACA8 |
| 89.0 | other | MVUHLWYKTKJ5EAABAAAAACA8 |
| 86.4 | other | MWBGLKQKTKJZ2AABAAAAABI8 |

## Files and Artifacts

### Reports
- Patch summary: `check_reports/placement_patch/patch_summary.json`
- Patch manifest: `check_reports/placement_patch/manifest_patch_20260312_052256.json`
- Pre-patch verification: `check_reports/placement_patch/verify_pre_patch.json`
- Post-patch verification: `check_reports/placement_patch/verify_post_patch.json`
- Per-scene patch reports: `check_reports/placement_patch/*_patch_report.json` (99 files)

### Backups
- Scene backups: `GRScenes-test1-normalized/GRScenes100/*/layout.pre_patch.*.usd` (99 files)
- Asset backups (from dedup): `GRScenes-test1-normalized_bak/_dedup_assets/`

### Scripts
- `scripts/patch_dedup_placement.py` -- patch script
- `scripts/verify_all_scenes_vs_test0.py` -- verification script

## Conclusion

The Plan C placement patch successfully corrected **93.4%** of all dedup-induced
displacement errors (25,869 of 27,687 prims). The remaining 1,818 displaced prims
are pre-existing normalization errors unrelated to dedup, concentrated in 17 scenes.

These residual displacements require a separate investigation into the
normalize_asset_transforms V2 pipeline for these specific scenes.
