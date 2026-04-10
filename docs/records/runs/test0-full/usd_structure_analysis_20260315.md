---
title: USD Structure Analysis (2026-03-15)
created_at: 2026-03-15
updated_at: 2026-03-15
maintainer: usd-structure-agent
status: completed
doc_class: record
---

# USD Structure Analysis: Displaced Assets Investigation
**Date**: 2026-03-15
**Status**: Completed
**Code Reference**: `normalize_asset_transforms.py:458`, `check_reports/test0_rebuilt_full/20260314_rebuilt_full_dlc_v1/normalize/test0_vs_normalized_pre_dedup.json`
**Maintainer**: usd-structure (agent)
**Created At**: 2026-03-15
**Updated At**: 2026-03-15

## Overview

Investigated structural differences in USD assets to explain why 487 prims (0.5% of 101,965) were displaced >0.01m during V2 re-normalization (normalized test0-rebuilt).

**Key Finding**: Displacement is NOT caused by unusual USD structure. The issue stems from **non-Animation scene categories** (Furnitures, Base, other) suffering much worse transformation errors despite simpler hierarchies.

## Scene Location Pattern

### Displaced Prims by Scene Subcategory

| Subcategory | Count | Percentage | Avg Displacement | Max Displacement |
|-------------|-------|-----------|-----------------|------------------|
| Animation | 399 | 84.9% | 3.2m | 159.8m |
| Furnitures | 69 | 14.7% | 26.2m | 854.5m |
| Base | 2 | 0.4% | 427.3m | 854.5m |

**Critical Observation**: While Animation dominates by count, Non-Animation prims have **8x worse average displacement**. The worst case (854.5m) is in the "other" category (Furnitures), not Animation.

### Category Breakdown

Top affected asset categories:
- unknown (69 displaced)
- trashcan (55 displaced)
- basket (49 displaced)
- faucet (39 displaced)
- refrigerator (31 displaced)
- electriccooker (30 displaced)
- pot (28 displaced)
- oven (25 displaced)
- desk (23 displaced)
- pan (21 displaced)

## USD Structure Analysis

### Animation Prims (Typical)
```
/Root/Meshes/Animation/desk/model_856a914fb4f34268d85458eea8f6c5f4_0
  xformOp:transform = Matrix4d with translation (e.g., 549.1, 432.6, 52.5)
  └─ Instance/
      └─ Group/
          └─ Component/
              └─ Mesh prims
```

### Furnitures/Other Prims (Typical)
```
/Root/Meshes/Furnitures/light/model_cc86f80318ecb68be567dd628fc2cd8c_0
  xformOp:transform = Matrix4d with translation (e.g., 423.2, -488.2, 191.8)
  └─ Group/
      └─ Mesh prims (or SM_* subgroups)
```

### Structural Findings

| Property | Finding |
|----------|---------|
| Transform Type | Both use `xformOp:transform` (Matrix4d) |
| Transform Present | All model instances have transforms with translation components |
| Hierarchy Depth | Animation: 5-6 levels, Furnitures: 3-4 levels |
| Geometry Preservation | Vertex counts identical (e.g., 14,704 → 14,704) |
| File Size Change | Negligible (5.3 KB → 4.5 KB for examples) |
| Internal References | None (assets are themselves referenced from scenes) |

## Implications

### What This Tells Us

1. **NOT a structural anomaly issue**: Both Animation and Furnitures prims use identical transform strategies
2. **NOT a geometry issue**: Vertices are preserved exactly
3. **Likely math/computation error**: The normalization compensation pass (`normalize_asset_transforms.py:458`) miscalculates transforms for certain categories
4. **Non-Animation categories hit harder**: Simpler hierarchy doesn't protect from errors; suggests compensation logic doesn't account for specific subtree structures

### Hypothesis

The V2 scene compensation pass may be:
- Mishandling transform chains for non-Animation prims
- Applying wrong parent transform context for some categories
- Not correctly inverting transforms for assets with specific depth patterns

The Instance/Group/Component nesting in Animation actually may be protecting those prims due to:
- More intermediate transforms to work with
- Different transform application order
- Or luck in how the math works out

## Next Steps

1. **Inspect normalize_asset_transforms.py** line 458 (scene compensation logic)
2. **Compare transform chains** for worst-case assets (other/fff0e6e3..., desk/856a914f...)
3. **Check parent transform context** calculation for Furnitures vs Animation
4. **Verify 126 missing centers** against these displacement measurements

## Data Sources

- Displaced prims JSON: `/tmp/displaced_487_full.json` (487 prims, 437 unique assets)
- Scene: `GRScenes-test0-rebuilt/GRScenes100/home/MVUCSQAKTKJ5EAABAAAAADA8_usd/start_result_interaction.usd`
- Comparison report: `check_reports/test0_rebuilt_full/20260314_rebuilt_full_dlc_v1/normalize/test0_vs_normalized_pre_dedup.json`

## Conclusion

The displaced assets show **no unusual structural differences** that would explain the displacement on their own. Both Animation and non-Animation assets use equivalent transform matrices and hierarchy patterns. The issue is a **computational/mathematical problem in the normalization compensation logic**, not a USD structure defect. Non-Animation categories are disproportionately affected, suggesting the compensation math has category-specific blind spots.
