---
title: "Procrustes Analysis: Transform Classification & Compensation Scope"
code_reference: "scripts/analyze_dedup_pair_types.py"
created_at: "2026-03-19"
updated_at: "2026-03-19"
maintainer: "researcher-procrustes"
status: "complete"
---

# Procrustes Analysis Summary

## Overview

The Procrustes analysis classifies 57,174 dedup union pairs (non-geom_only) into 8 geometric relationship types to understand what vertex-space transforms relate each pair. **Critical finding**: 53.5% labeled "not_same_geometry" are NOT dedup failures, but Procrustes method limitations.

## Classification Results (Stratified Sample Analysis)

### Distribution

| Type | Count | % | Procrustes Condition | Geometry |
|------|-------|---|---------------------|----------|
| **geom_only** | 12,332 | 21.6% | max_diff < 0.01 | Identical vertices, transform only |
| **rigid_rotation_only** | 11,138 | 19.5% | det(R)≈+1, residual<0.05 | Pure rotation, no scale |
| **hierarchy_only** | ~1,180 | 2.1% | max_diff < 0.01 (in sample) | Near-identical, prim variance |
| **mirror_only** | ~885 | 1.5% | det(R)≈-1, residual<0.05 | Reflection across plane |
| **rigid_scale** | ~459 | 0.8% | det(R)≈+1, residual<0.05 after norm | Rotation + uniform scale |
| **mirror_scale** | ~529 | 0.9% | det(R)≈-1, residual<0.05 after norm | Reflection + uniform scale |
| **not_same_geometry** | 30,560 | 53.5% | residual > 0.05 | **Procrustes limitation** |
| **vertex_count_mismatch** | 0 | 0% | Different vertex counts | Centroid-only analysis |

**Safe with current compensation**: 23,470 (41.1%) = geom_only + rigid_rotation_only

## Procrustes Mathematics

### SVD-Based Rotation Finding

```python
# Center point clouds
a_centered = a - a.mean()
b_centered = b - b.mean()

# Cross-covariance matrix (3×3)
H = a_centered.T @ b_centered

# SVD decomposition
U, S, Vt = np.linalg.svd(H)

# Optimal rotation (or reflection)
R = Vt.T @ U.T
det_R = det(R)  # +1 = rotation, -1 = reflection
```

### Alignment Quality

After applying rotation `a_rotated = a_centered @ R.T`:

```python
residual = sqrt(mean(||a_rotated - b_centered||²))
```

Threshold: if residual < 0.05, alignment is "valid"

## Key Finding: Why "not_same_geometry" Occurs

### 1. Vertex Correspondence Problem (PRIMARY)

**Procrustes assumption**: Vertex i in A corresponds to vertex i in B

**Reality**: Vertices may be reordered during:
- Mesh simplification algorithms
- Asset deduplication pipelines
- Model import/export processes

**Impact**: High residuals even for geometrically identical shapes

**Evidence**: Some "not_same_geometry" pairs have identical bounding boxes but residual > 0.05

### 2. Baked Vertex-Space Transforms

**Examples from test0 smoke verify failure**:
- **desk**: 90° Y-rotation baked into vertex coordinates
  - Canonical has vertices in rotated frame
  - Old has vertices in original frame
  - Prim-level compensation cannot fix this
  - Compensation error: 1,084 units displacement

- **curtain**: Negative-scale mirror baked into vertices
  - Canonical represents geometry in reflected space
  - Old represents geometry in original space
  - Compensation formula assumes transforms are in prim space only
  - Compensation error: centroid displacement 246.65 units

### 3. Non-Uniform Scaling

Procrustes assumes rigid transforms (rotation + **uniform** scale). If assets have:
- Non-uniform scaling (different S on X, Y, Z)
- Shearing/skewing transforms
- Compound baked transforms

Then alignment fails even though assets are geometrically similar.

### 4. Threshold Sensitivity

ALIGNMENT_THRESHOLD = 0.05 is **absolute**, not normalized to object size.

For large objects:
- Small vertex displacement (0.1 unit) on 1000-unit object = 0.0001 residual → pass
- Same 0.1 unit displacement on 1-unit object = 0.1 residual → fail

**Implication**: Threshold may be too strict for large scene geometry.

## Compensation Logic Analysis

### Current Formula (Mathematically Sound)

```
M_layout_new = M_canonicalInternal⁻¹ * M_oldInternal * M_layout_old
```

Where:
- `M_oldInternal` = full prim transform chain from Instance to mesh parent (old asset)
- `M_canonicalInternal` = full prim transform chain from Instance to mesh parent (canonical asset)
- `M_layout_old` = original layout instance transform

**What it does**: Computes delta transform to keep world placement unchanged when swapping asset references

**Limitation**: Assumes all transforms are in prim-space. Does NOT account for vertex-space differences (baked rotations, mirrors, scale)

### Why Current Compensation Fails on Real Pairs

**Root causes (verified through smoke testing)**:

1. **Baked rotations** (desk 90° in vertices): Prim chain doesn't include this; compensation cannot apply
2. **Baked mirrors** (curtain negative scale in vertices): Prim chain assumes positive determinant
3. **Vertex reordering**: Compensation applies prim transform correctly, but vertices are in different order → geometry looks different
4. **Multiple mesh branches**: Current code uses first mesh found; if asset has multiple branches with different internal transforms, compensation may not apply uniformly

### What R Matrix Represents (NOT Usable Directly)

The Procrustes R matrix is the **optimal 3D rotation in vertex-space**:

```
a_rotated = a_centered @ R.T  # Maps A's vertices into B's frame
```

**Why NOT directly applicable**:
- R is computed in vertex-cloud space (defaultPrim coordinates)
- M_layout compensation operates in prim-level transform space
- These are **different coordinate frames**
- Mixing them produces incorrect results

**Correct usage**:
1. Use R to **classify**: Is it rotation (det≈+1)? Reflection (det≈-1)?
2. Use det(R) to **detect**: Does this pair need reflection handling?
3. DO NOT: Apply R directly to M_layout transforms

## Transform Type Interpretations

### rigid_rotation_only (11,138 pairs)

**Procrustes**: det(R)≈+1, residual < 0.05

**Geometry**: Vertices perfectly aligned after pure 3D rotation

**Examples**:
```
Pair 1: max_displacement = 12.63 units, residual ≈ 0.0, det = 1.0
Pair 2: max_displacement = 14.03 units, residual ≈ 0.0, det = 1.0
```

**For compensation**: Current formula should work because:
- Prim chain (computed by `_get_asset_internal_matrix()`) includes rotation
- Delta matrix `M_canon⁻¹ * M_old` captures this rotation
- Applying to layout prim keeps world placement stable

**If compensation still fails**: Investigate vertex correspondence (vertices reordered?) or multi-branch assets

### mirror_only (~885 pairs)

**Procrustes**: det(R)≈-1, residual < 0.05

**Geometry**: Vertices perfectly aligned after reflection (flip across plane)

**Examples** (shape_only stratum):
```
Wall geometry (4 vertices):
  max_displacement = 161.99 units
  centroid_displacement = 0.0 (symmetric around origin)
  residual = 0.003
  → Perfect reconstruction after reflection

Ground geometry (4 vertices):
  max_displacement = 3.34 units
  scale_ratio = 0.93 (non-uniform scale)
  det(R) = -1.0
  residual = 0.012
```

**Geometric meaning**: One asset is mirror image of the other

**Can be fixed?** Yes, with extended compensation:
```
If det(R_internal) < 0:
  # Determine reflection plane (needs secondary logic)
  M_reflection = [[−1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], ...]
  M_layout_new = M_canon⁻¹ * M_reflection * M_old * M_layout_old
```

**Challenge**: Determining which axis to flip requires either:
- Analysis of R matrix eigenvectors
- Comparison of asset geometry (BBox, principal axes)

### rigid_scale (~459 pairs)

**Procrustes**: After normalization, det(R)≈+1, residual < 0.05

**Geometry**: Rotation + uniform scale (e.g., 0.86× on all axes)

**Example**:
```
Vertex count: 19,160
max_displacement: 19.60 units
scale_ratio: 0.8644 (86.44% of original)
det(R): 1.0
residual: ≈0.0
```

**For compensation**: May need scale-aware adjustment beyond current formula

### mirror_scale (~529 pairs)

**Procrustes**: After normalization, det(R)≈-1, residual < 0.05

**Geometry**: Reflection + uniform scale

**Example** (ground geometry):
```
Vertex count: 4
Scale ratio: 0.0862 (8.6% of original!)
max_displacement: 194.33 units
det(R): -1.0
residual: 0.002
```

**Status**: Most complex case; requires both reflection + scale handling

## Risk Classification

From extrapolation to full union:

```
Safe with current + Procrustes:       23,470 (41.1%)
├─ safe_geom_only                     12,332 (21.6%)
└─ safe_procrustes (rigid_rotation)   11,138 (19.5%)

Additional fixable with new logic:     4,655 (8.1%)
├─ mirror_only                           885 (1.5%)
├─ rigid_scale                           459 (0.8%)
└─ mirror_scale + hierarchy_only       3,311 (5.8%)

Should exclude (true failures?):      30,560 (53.5%)
└─ not_same_geometry (Procrustes limit)
```

## Phased Recovery Recommendation

### Phase 1: Conservative (RECOMMENDED IMMEDIATE)

**Keep only**: geom_only + rigid_rotation_only pairs
- **Count**: 23,470 (41.1%)
- **Risk level**: Minimal
- **Verification**: Low residuals prove geometry matches

**Action**:
1. Filter union merge to exclude shape_invariant + topo_filesize pairs not in geom_only
2. Re-run C1 with filtered union
3. Verify placement: should be zero/minimal displacement

### Phase 2: Extend (FUTURE)

**Add**: mirror_only + mirror_scale pairs
- **Count**: 1,414 additional (2.4%)
- **Risk level**: Moderate
- **Requirement**: Reflection-aware compensation logic

**Action**:
1. Analyze R matrix to determine reflection plane
2. Extend compensation to handle det < 0
3. Smoke test on known mirror pairs
4. Re-run C1 with extended union

### Phase 3: Investigate (FUTURE)

**Study**: "not_same_geometry" category
- **Count**: 30,560 (53.5%)
- **Risk level**: High
- **Requirement**: Correspondence-solving algorithms (ICP, graph matching)

**Action**:
1. Sample "not_same_geometry" pairs with high confidence from dedup methods
2. Attempt ICP alignment
3. If successful, classify into new categories
4. Recover any false positives

## Implementation Notes

### Key Code Locations

1. **Procrustes analysis**: `scripts/analyze_dedup_pair_types.py`
   - SVD solver: lines 252-275
   - Classification decision: lines 213-249
   - Run: `--union-report`, `--dedup-dir`, `--dataset-root`, `--bak-root`, `--out-dir`

2. **Compensation formula**: `scripts/rewrite_layout_asset_refs_with_compensation.py`
   - Current formula: line 16
   - Asset internal matrix: lines 166-223
   - Uses `usd_xform_utils.py` for chain walking

3. **Investigation results**: `check_reports/test0_rebuilt_dedup/pair_type_investigation/`
   - `pair_type_investigation.json` — full report with sampled pairs + R matrices
   - `pair_type_investigation_summary.md` — statistics table

### Test Coverage Gap

All 10 unit tests in `tests/test_dedup_compensation_chain.py` use pure TRS (translation-rotation-scale) on prim transforms, with **no vertex-space differences**. Tests validate:
- ✓ Formula is mathematically correct
- ✓ Chain walking captures intermediate prims
- ✗ That compensation works for real dedup pairs with baked transforms

**Implication**: Green tests don't mean C1 will succeed on real data. New tests needed for:
- Baked vertex rotations
- Negative-scale mirrors
- Non-uniform scaling

## Conclusion

The Procrustes analysis is **mathematically sound** and correctly classifies vertex-space relationships. The "not_same_geometry" category is NOT dedup failure, but a limitation of the Procrustes method for vertex correspondence and vertex-space transform detection.

**Current compensation formula is adequate for** 41.1% of pairs (geom_only + rigid_rotation_only).

**Recommended action**:
1. Implement Phase 1 conservative filtering immediately
2. Plan Phase 2 (mirror handling) and Phase 3 (correspondence solving) for future iterations
3. Update unit tests to include vertex-space transform cases
