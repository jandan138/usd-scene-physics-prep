---
title: Matrix Multiply Order Bug Fix in _get_chain_transform (2026-03-15)
code_reference:
- scripts/normalize_asset_transforms.py:152
created_at: '2026-03-15'
updated_at: '2026-03-15'
maintainer: team-lead
status: completed
---

# Matrix Multiply Order Bug Fix in _get_chain_transform (2026-03-15)

## Overview

This document describes a critical bug in `scripts/normalize_asset_transforms.py` where the `_get_chain_transform()` function computed transformation matrices in the wrong order, causing large displacements (up to 854.5 meters) when normalizing assets with intermediate transform prims.

## The Bug

### Location
- **File**: `scripts/normalize_asset_transforms.py`
- **Function**: `_get_chain_transform()`
- **Lines**: 152 (primary), also line 264

### Root Cause

The function was multiplying transformation matrices in the **wrong order**:

```python
# BUGGY CODE:
M_instance * M_chain  # WRONG ORDER

# CORRECT CODE:
M_chain * M_instance  # CORRECT ORDER
```

### Impact Scope

The bug only affected assets with **intermediate prims** between `/Root/Instance` and mesh geometry:

```
/Root/Instance (Xform)
  └─ Group_00 (Xform)  <-- INTERMEDIATE PRIM (triggers bug)
     └─ Mesh
```

Assets with direct Instance→Mesh structure were unaffected.

### Scale of Impact (Full Run Results)

- **Total displaced prims**: 487 of 101,919 (0.48%)
- **Affected scenes**: 37 of 99 scenes
- **Worst displacement**: 854.5 meters (in scene MVUCSQAKTKJ5EAABAAAAACY8, asset `other/fff0e6e3...`)
- **Displacement breakdown**:
  - gt_0.01m: 487 prims
  - gt_0.1m: 311 prims
  - gt_1.0m: 148 prims
  - gt_10.0m: 23 prims

### Affected Categories

21 categories had displacement (multiple prims affected across these):
- Primary offenders: other, wall, ground, refrigerator, table, nightstand, desk, etc.

## The Fix

### Code Change

**File**: `scripts/normalize_asset_transforms.py`

**Line 152** (primary):
```python
# BEFORE:
M_world = M_instance * M_chain

# AFTER:
M_world = M_chain * M_instance
```

**Line 264** (secondary):
```python
# BEFORE:
M_instance_from_chain = (inst_chain @ new_chain.Inverse()) * inst_base

# AFTER:
M_instance_from_chain = inst_base * (inst_chain @ new_chain.Inverse())
```

### Commit

- **Hash**: Applied as one-line fix (included in commit `2e0afb2` along with USD export API fix)
- **Type**: Bug fix
- **Severity**: Critical (affects output correctness)

## Why the Smoke Test Didn't Catch This

### Smoke Test Configuration

- **Scene UID**: `MV7J6NIKTKJZ2AABAAAAADA8`
- **Coverage**: Smoke scene contains 18 of 21 displaced categories (missing only: basket, shoppingtrolley, trashcan)
- **Asset count**: 1,105 prims from all 52 categories

### Smoke Test Results (Pre-Fix)

- **Displaced prims detected**: 1 of 1,105 (0.09% of scene)
- **Max displacement**: 2.13 meters
- **Category with displacement**: other (1 prim out of 149)
- **Test outcome**: Passed (no failures detected)

### Root Cause of Test Gap

The `_get_chain_transform` bug manifests **inconsistently**:

1. **Selective effect**: Not all assets with intermediate prims are displaced. It depends on the specific transformation chain structure.
2. **Lucky selection**: The smoke scene happened to sample 149 `other` prims, but only 1 was affected. The full run found 854 total `other` prims with 93 displaced (10.9% of category).
3. **No displacement check**: The smoke test ran pairwise comparison but didn't fail if displacement > 0.01 was detected. Results were silently recorded.

### Full Run vs Smoke Test Discrepancy

| Metric | Smoke Scene | Full Run |
|--------|-------------|----------|
| Displaced prims (>0.01m) | 1 | 487 |
| Pairwise comparison ran | Yes | Yes |
| Test failed | No | No (hard gate failed on other metric) |
| Scene coverage | 1 scene | 99 scenes |
| Asset coverage | ~1,105 prims | ~101,919 prims |

## Verification

### Pre-Fix Behavior

- V1 normalized assets: 52,904 assets normalized with bug present
- V1 → V2 comparison: Bug caused systematic displacement across 487 prims
- Backup location: `GRScenes-test1-normalized_v1_buggy/`

### Post-Fix Verification

- V2 normalization (re-run): Full pipeline completed successfully
- Phase 1: 79 DLC jobs, 52,904 assets, 3 data errors (known bad assets)
- Phase 2: 99 scenes compensated, 101,919 prims, 0 displacement errors
- Spot-check: 10 random assets verified with 0.0m displacement (within floating-point noise)

## Recommendations

### 1. Add Regression Test (IMPLEMENTED in Task #3)

Create a targeted test that:
- Selects 3-5 assets with intermediate prims and >100m displacement from the full run
- Includes them in a minimal test scene
- Verifies they normalize with <0.01m displacement

### 2. Improve Smoke Test Validation (FUTURE)

Options:
- **Option A**: Fail smoke test if any prim has displacement > 0.01m
- **Option B**: Use a different smoke scene with better coverage of edge-case assets
- **Option C**: Add explicit pairwise result validation step to smoke test pipeline

### 3. Code Review Process (PROCESS)

- Matrix operations in physics/geometry code should require sign-off on multiplication order
- Transformation chain operations should include unit tests with known reference values

## Technical Details

### Transformation Matrix Chain

For an asset with intermediate prims:

```
World coordinates
      ↑
      | (Scene compensation: M_scene)
      ↑
Scene Instance coordinates
      ↑
      | (Reference override: M_ref)
      ↑
Normalized Asset coordinates
      ↑
      | (Asset Instance: M_instance)
      ↑
Asset Group (intermediate): M_group
      ↑
      | (Asset Group in Instance coords)
      ↑
Asset Mesh (local)
```

The correct order for combining M_instance (instance transform) and M_chain (group+mesh chain):
- When transforming a point in mesh-local space to world: `P_world = M_chain @ M_instance @ P_local`
- When building combined transforms: `M_world = M_chain * M_instance` (Gf multiplication)

The bug reversed this to `M_instance * M_chain`, which effectively applied the transforms in the wrong order.

## Files Modified

- `scripts/normalize_asset_transforms.py` (2 locations: lines 152, 264)

## Related Issues

- **Hard gate failure** (Test 0 Rebuilt): 487 displaced prims triggered normalize gate failure
- **Placement displacement pattern** (V2 normalization): Attributed to dedup compensation, actually caused by this bug
- **Shape-invariant dedup** (C1 full-run): Required re-dedup after V2 re-normalization fix

## Testing Evidence

- **Full run pairwise report**: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_rebuilt_full/20260314_rebuilt_full_dlc_v1/normalize/test0_vs_normalized_pre_dedup.json`
- **Smoke scene analysis**: `GRScenes-test0-rebuilt/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd`
- **Asset hierarchy verification**: book assets contain `Group_default_00` intermediate prims
