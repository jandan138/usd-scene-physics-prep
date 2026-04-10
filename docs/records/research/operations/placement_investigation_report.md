---
title: "Placement Investigation: Normalized Layout vs Original Layout"
code_reference: scripts/normalize_asset_transforms.py
created_at: 2026-03-12
updated_at: 2026-03-12
maintainer: claude-agent
status: completed
---

# Placement Investigation Report

## Summary

**Conclusion: Object geometry placement is CORRECT.** Mesh vertex world-space positions match between original and normalized layouts to within floating-point precision (max error < 0.000006 units across 6 scenes and thousands of prims).

The apparent "displacement" is a change in **transform representation**, not in actual rendered geometry.

## Investigation Method

7 agents investigated in parallel:

| Agent | Task | Finding |
|-------|------|---------|
| desk-analyst | Desk prim transform + vertex comparison | Vertices match (err < 0.000002), prim origin shifted ~8.85 units (cosmetic) |
| curtain-analyst | Curtain prim transform + vertex comparison | Zero displacement, 232,892 vertices verified |
| displacement-scanner | Full scene prim transform scan | 83% prim transforms differ (expected), NOT actual geometry displacement |
| normalization-analyst | normalize_asset_transforms.py math verification | Round-trip correct for all tested assets (max err < 0.00005) |
| multi-scene-sampler | 5 additional scenes vertex comparison | Zero vertex displacement across all 5 scenes |
| child-prim-inspector | Child prim structure + transform analysis | Group_00 offset absorbed into vertices, prim origin shifted but geometry correct |
| dedup-pivot-checker | C1 dedup compensation code review | Compensation is no-op (reads /Root identity), but doesn't cause issues because duplicate assets share the same bbox center |

## Why Transforms Look Different (Expected Behavior)

### Original Layout
- Prim transforms: **identity** (no rotation, no translation)
- Asset mesh vertices: world-space coordinates baked in
- upAxis: Y

### Normalized Layout
- Prim transforms: **Y→Z rotation + translation** (compensating for asset recentering)
- Asset mesh vertices: centered at origin, Z-up
- upAxis: Z

These two representations produce **identical rendered geometry** in world space. The transform values differ because the coordinate system changed, not because objects moved.

## Specific Prims Investigated

### Desk: `model_64a72e14.../Instance/Group_00/Component_9`
- Reference: `desk/08331d5e...` (same in both, NOT changed by dedup)
- Vertex world positions: **identical** (max err 0.000002)
- Prim origin shifted ~8.85 units because Group_00's translate(0.0005, -8.668, 59.117) was absorbed into vertex data
- **Geometry is correctly placed**

### Curtain: `model_f2732ecb.../Instance/SM_6KBZZ2.../obj4`
- Reference: `curtain/66af8bda...` (same in both, NOT changed by dedup)
- Vertex world positions: **identical** (max err 0.000003)
- Child prims had identity transforms, so no prim origin shift
- **Geometry is correctly placed**

## Multi-Scene Verification

| Scene | Prims Sampled | Max Displacement | Count >0.1 |
|-------|--------------|-----------------|------------|
| MV7J6..ADI8 | 39 | 0.000002 | 0 |
| MVUCS..AAA8 | 39 | 0.000002 | 0 |
| MVUCS..ACQ8 | 45 | 0.000003 | 0 |
| MVUHL5..AA8 | 43 | 0.000001 | 0 |
| MVUHLW..AQ8 | 48 | 0.000006 | 0 |

## Noted Issues (Non-Critical)

### 1. Dedup Compensation Is No-Op
`rewrite_layout_asset_refs_with_compensation.py` reads `/Root` (identity) instead of `/Root/Instance` for the asset internal matrix. The compensation formula produces identity × identity⁻¹ = identity, doing nothing. This is currently harmless because duplicate assets share the same bbox center (same mesh → same center after normalization), but should be fixed for correctness.

### 2. Child Prim Transforms Absorbed
Normalization absorbs child prim transforms (e.g., Group_00's translate) into vertex data and clears the xformOps. This shifts the prim origin but not the geometry. Could affect tools that rely on prim origin rather than mesh vertices.

### 3. Prim-Level Transform Comparison Is Misleading
Comparing `ComputeLocalToWorldTransform` between original and normalized layouts shows large "displacements" (mean ~900 units) because the transform representation intentionally changed. The correct comparison method is world-space vertex position matching.

## Verification Commands

```python
# Correct way to verify placement
from pxr import Usd, UsdGeom, Gf
stage_orig = Usd.Stage.Open("original/layout.usd")
stage_norm = Usd.Stage.Open("normalized/layout.usd")

# Compare actual vertex world positions, NOT prim transforms
xform_cache_o = UsdGeom.XformCache()
xform_cache_n = UsdGeom.XformCache()
for prim_o in stage_orig.Traverse():
    mesh = UsdGeom.Mesh(prim_o)
    if not mesh: continue
    points_o = mesh.GetPointsAttr().Get()
    world_o = xform_cache_o.GetLocalToWorldTransform(prim_o)
    # Transform points to world space and compare...
```
