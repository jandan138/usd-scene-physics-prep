---
title: "Matrix Multiply Order Bug Fix in normalize_asset_transforms.py"
code_reference: scripts/normalize_asset_transforms.py
created_at: 2026-03-10
updated_at: 2026-03-10
maintainer: zhuzihou
status: active
---

# Matrix Multiply Order Bug Fix

## Problem

After opening normalized scenes in Isaac Sim, **81% of objects (900/1105)** were floating/displaced from their correct positions. Displacement ranged from tens to thousands of scene units.

## Root Cause

`scripts/normalize_asset_transforms.py` lines 199 and 264 had the matrix multiplication order reversed:

```python
# BUG (old code)
M_internal = M_instance * M_chain     # Wrong: applies Instance before Chain

# FIX (new code)
M_internal = M_chain * M_instance     # Correct: USD row-vector convention p * M_chain * M_instance
```

In USD's row-vector convention (`p * M`), a mesh point transforms bottom-up through the prim hierarchy:
1. First through `M_chain` (Group_default_00 translate, etc.)
2. Then through `M_instance` (Instance orient + scale)

The buggy order caused:
1. **Wrong baked mesh coordinates** — points at completely wrong positions
2. **Wrong bbox center** — recentering used wrong centroid
3. **Wrong scene compensation** — formula was correct but received wrong center values
4. **Result**: objects displaced, with magnitude depending on intermediate transform complexity

## Investigation Details

### Affected vs Unaffected Objects

- **Unaffected (19%)**: Objects with identity/near-identity intermediate transforms (centroid already at origin). The multiply order doesn't matter for identity matrices.
- **Affected (81%)**: All objects with non-trivial intermediate transforms (rotation, translation in chain between Instance and mesh prims).

### Displacement by Category

| Category | % Displaced | Sample Distance |
|----------|------------|-----------------|
| wall | 100% | 1,383 units |
| ground | 100% | 951 units |
| ceiling | 100% | 1,666 units |
| door | 100% | 4,297 units |
| window | 100% | 5,491 units |
| cabinet | 100% | 6,460 units |
| bowl | 89% | 123 units |
| cup | 73% | varies |
| chair | 0% | 0 (identity transforms) |
| bed | 0% | 0 (identity transforms) |

### What Was NOT the Cause

- **MDL path fix**: Only modifies `Sdf.AssetPath` attributes, cannot touch transforms
- **Dedup replacement**: Post-normalization centroids match within 0.001 units between duplicate pairs
- **Compensation formula**: `M_scene_new = T(center) * R_y2z_inv * M_scene_old` is mathematically correct — it received wrong center values from the buggy bake

## Fix Applied

Three changes in `scripts/normalize_asset_transforms.py`:
- Line 199: `M_chain * M_instance` (in `compute_asset_center()`)
- Line 253: Updated comment
- Line 264: `M_chain * M_instance` (in `normalize_asset()`)

## Verification

Unit test on bowl asset `094fb8b32b3d56351cb178c5c942e16a`:
- Original world position: (664.9658, -222.0598, 80.0929)
- Re-normalized world position: (664.9658, -222.0598, 80.0929)
- **Displacement: 0.000000** (exact match)

## Re-normalization

Full pipeline re-run required:
1. Backup: `GRScenes-test1-normalized` → `GRScenes-test1-normalized_v1_buggy` (instant rename)
2. Re-run: `python scripts/normalize_asset_transforms.py --assets-root ... --output-root GRScenes-test1-normalized --symlink-copy`
3. Re-apply: Material symlink + MDL path fix + dedup

Estimated time: ~7-8 hours for 52,904 assets + 99 scenes.
