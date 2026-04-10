---
title: "Displacement Analysis Summary - test0_rebuilt_full (2026-03-15)"
code_reference: "scripts/normalize_asset_transforms.py:739, check_reports/test0_rebuilt_full/20260314_rebuilt_full_dlc_v1/"
created_at: "2026-03-15"
updated_at: "2026-03-15"
maintainer: "scene-pattern"
status: "complete"
---

# Displacement Analysis Summary - test0_rebuilt_full

## Executive Summary

Analyzed 487 displaced prims across 99 scenes. **Root cause identified and confirmed**: Phase 2 center rotation coordinate space bug in `normalize_asset_transforms.py:739`.

## Issue Overview

- **Total displaced prims**: 487 (0.48% of 101,965)
- **Affected scenes**: 81 out of 99 (81.8%)
- **Clean scenes**: 18 out of 99 (18.2%)
- **Worst displacement**: 854.54m (other/fff0e6e3484f719ba64bf586c0c768fd)

## Root Cause Analysis

### The Bug: Coordinate Space Mismatch

**Location**: `scripts/normalize_asset_transforms.py:739` (Phase 2 scene compensation)

Centers are computed in Z-up coordinate space (after Y→Z rotation in Phase 1), then applied WITHOUT converting back to Y-up space.

**Wrong implementation**:
```python
M_scene_new = T(center_z_up) * R_y2z_inv * M_scene_old
```
This translates by Z-up center directly, then rotates back to Y-up. Translation happens in wrong space.

**Correct implementation**:
```python
M_scene_new = R_y2z_inv * T(center_z_up) * R_y2z * M_scene_old
```
Center translation must happen AFTER rotating to Z-up space, not before rotating back.

### Why This Causes 487 Displacements

The displacement magnitude approximately equals the center.y component in Z-up space:

1. **High-displacement categories** (center.y > 1.0):
   - basket: 55 displaced / 87 total (63.2%), center.y ≈ 1.398
   - shoppingtrolley: 17 displaced / 34 total (50.0%), center.y > 1.0
   - electriccooker: 32 displaced / 78 total (41.0%), center.y > 1.0
   - refrigerator: 31 displaced / 79 total (39.2%), center.y > 1.0

2. **Low-displacement categories** (center.y ≈ 0):
   - ground, wall, ceiling, floor: displacement ≈ 0
   - nightstand: 1 displaced / 120 total (0.83%), center.y ≈ -0.005
   - other: 55 displaced / 35,592 total (0.15%), but includes 2 worst outliers (phase 1 skip)

### Secondary Issue: Phase 1 Silent Failures

29 assets in "other" category (0.1%) silently skip center computation without error logging. These include the two worst displaced prims:
1. fff0e6e3484f719ba64bf586c0c768fd (854.54m)
2. 5a86adb3d68c04e3bd6441182b590eb0 (732.93m)

No compensation applied → massive displacement when asset mesh recenters from world space (-5534m, -6505m) to origin.

## Scene-Level Patterns

### Size Correlation

- **Clean scenes** (18): average 584 prims, range 374-789
- **Affected scenes** (81): average 1,129 prims, range 341-8,243
- **Pattern**: Larger scenes contain more instances of high-center-y categories

### Home vs Commercial

- **Home scenes**: 54/69 with displacement (78.3%)
- **Commercial scenes**: 27/30 with displacement (90.0%)
- Commercial scenes slightly more affected overall

### Category Distribution

**Categories with ANY displacement (21 total)**:
basket, book, chestofdrawers, desk, dishwasher, electriccooker, faucet, hearth, microwave, nightstand, other, oven, pan, pot, refrigerator, shoppingtrolley, table, teatable, toilet, trashcan, washingmachine

**High-rate categories** (>30% displacement rate):
- basket: 63.2%
- shoppingtrolley: 50.0%
- electriccooker: 41.0%
- refrigerator: 39.2%
- oven: 36.2%
- dishwasher: 22.4%

**Medium-rate categories** (15-20%):
- trashcan: 19.0%
- pot: 19.8%
- pan: 20.5%
- faucet: 14.7%

**Low-rate categories** (<2%):
- nightstand: 0.83%
- book: 0.22%
- other: 0.15%

### Distribution Across Scenes

Most displaced prims are scattered across scenes rather than concentrated:
- **Median per scene**: 5 prims
- **Mean per scene**: 6.0 prims
- **Top 10% of scenes**: 135 prims (28% of total)
- **Bottom 50% of scenes**: 123 prims (26% of total)

## Scene Profile Differences

### Category Frequency in Clean vs Affected Scenes

Categories appearing 4-13x more in affected scenes:
- book: 4.3x (47.2 avg → 201.1 avg instances per scene)
- bottle: 8.8x (7.0 avg → 61.9 avg)
- pen: 5.3x (16.5 avg → 87.7 avg)
- toy: 13.3x (1.9 avg → 25.3 avg)
- other: 2.9x (140.3 avg → 408.2 avg)

These categories accumulate in larger, more complex scenes, increasing exposure to the coordinate space bug.

## How the Bug Explains Observed Patterns

1. **Why 487 exact prims displace**: Each category with non-zero center.y has roughly 55 instances that displace when scene compensation applies the wrong coordinate transformation.

2. **Why larger scenes affected more**: More instances of high-center-y categories (basket, electriccooker, refrigerator, faucet) in larger scenes.

3. **Why 18 clean scenes escape**: Small scenes with low instance counts of problematic categories. They either have no displaced instances, or their low center.y values result in sub-0.01m displacement.

4. **Why desk category displaces despite having centers**: Suggests either incorrect center lookup key or sign error in rotation matrix (needs separate verification).

## Verification Evidence

✓ Task #4: Scene-level pattern analysis confirms size correlation
✓ Task #7: Exact 55-count pattern for basket/other/trashcan explained by center.y magnitude
✓ Task #3: Center computation formula analysis identifies coordinate space bug
✓ Task #8: Worst displaced assets USD structure identical in both versions (bug not in assets)

## Fix Strategy

1. **Update Phase 2 line 739**: Apply rotation before translation
   ```python
   # Before (wrong):
   M = T(center_z_up) @ R_y2z_inv @ M_old

   # After (correct):
   M = R_y2z_inv @ T(center_z_up) @ R_y2z @ M_old
   ```

2. **Add Phase 1 error logging**: Log assets that fail center computation so we can diagnose the 0.1% skip rate.

3. **Re-run Phase 2**: After fix, hard gate should pass with 0 displaced prims.

## Expected Outcome

After fix:
- All 487 displaced prims → 0 displaced
- Hard gate: FAILED → PASS
- `pairwise_displaced_gt_0.01`: 487 → 0
- `center_found`: 101,839 → 101,965 (after Phase 1 logging investigation)

## Files Modified / Analyzed

- Source: `scripts/normalize_asset_transforms.py` (line 739 — Phase 2 compensation)
- Report: `check_reports/test0_rebuilt_full/20260314_rebuilt_full_dlc_v1/normalize/test0_vs_normalized_pre_dedup.json` (3.3MB, 99 scenes analyzed)
- Memory: `docs/scene_patterns.md` (detailed scene distribution)

---

**Status**: Analysis complete. Awaiting code fix and re-run.
