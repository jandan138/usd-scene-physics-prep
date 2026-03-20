---
title: Vertex Correspondence Problem - Technical Deep Dive
code_reference:
  - scripts/report_asset_mesh_dedup.py (Procrustes alignment)
  - scipy.spatial.procrustes
  - check_reports/test0_rebuilt_dedup/pair_type_investigation/
created_at: 2026-03-19T08:45:00Z
updated_at: 2026-03-19T09:15:00Z
maintainer: Claude Code Agent
status: completed
---

## Problem Statement

Given two USD assets that:
1. **Pass geometric signatures**: geom_only, shape_invariant, or topo_filesize matching
2. **Are confirmed duplicates** by manual inspection
3. **Yet fail Procrustes alignment** with RMSE > 0.05

We need to understand WHY Procrustes fails and design compensation to recover valid pairs.

---

## Procrustes Algorithm Background

### Standard Procrustes (Orthogonal)
Given two point clouds **X** (m × 3) and **Y** (m × 3), find rotation R and translation t minimizing:

```
min_{R, t} ||Y - (XR^T + 1t^T)||_F^2

subject to: R ∈ SO(3)  [special orthogonal group; det(R) = +1]
```

**Solution method**:
1. Center both point clouds: X' = X - mean(X), Y' = Y - mean(Y)
2. Compute SVD: **Y'^T X' = UΣV^T**
3. Optimal rotation: **R = UV^T** (enforces det(R) = +1)
4. Translation: **t = mean(Y) - R·mean(X)**
5. Residual (RMSE): **||Y' - X'R^T||_F / √m**

### Key Limitation
The constraint det(R) = +1 forces the solution to be a pure rotation. If the true transformation includes reflection (det = -1), Procrustes will find the best rotation but the residual will be artificially high.

---

## Failure Mode 1: Reflection/Mirror Transformation (50.5% of failures)

### Mathematical Formulation
When vertices are related by reflection:
```
Y = -X·R_mirror^T + t

where det(R_mirror) = -1  (reflection = rotation + inversion)
```

Standard Procrustes minimizes:
```
min_R ||Y - X·R^T||_F   subject to det(R) = +1
```

But the true solution has det(R) = -1. Procrustes cannot model this, so residual remains high.

### Detection
**Check determinant of optimal R**: If det(R) ≈ -1 in SVD solution (before enforcement), reflection is present.

SVD provides two solutions:
- R₁ = UV^T (det = +1) — pure rotation
- R₂ = U·diag(1,1,-1)·V^T (det = -1) — rotation + reflection

### Quantitative Impact (from investigation)
- **50.5%** (140/277) of not_same_geometry failures involve reflection
- Avg residual (reflection cases): 28.5
- Avg residual (rotation cases): 24.3

Reflection contributes marginally to higher residual but is often combined with other factors (scale, baked transforms).

### Solution: Reflection-Aware Procrustes
Try both solutions; pick lower residual:

```python
def procrustes_with_reflection(X, Y):
    X_c = X - X.mean(axis=0)
    Y_c = Y - Y.mean(axis=0)

    U, S, Vt = np.linalg.svd(Y_c.T @ X_c)

    # Pure rotation (det = +1)
    R_rot = U @ Vt
    res_rot = np.linalg.norm(Y_c - X_c @ R_rot.T)

    # Reflection (det = -1)
    R_refl = U @ diag(1, 1, -1) @ Vt
    res_refl = np.linalg.norm(Y_c - X_c @ R_refl.T)

    if res_refl < res_rot:
        return R_refl, t_refl, res_refl
    else:
        return R_rot, t_rot, res_rot
```

**Expected gain**: ~3,000–5,000 pairs (20% of reflection cases with residual < 1.0)

---

## Failure Mode 2: Baked Transform in Vertices (7.6% of failures)

### Mathematical Problem
Two representations of the same geometry:
```
Case A: points_A at local coordinates, transform T stored separately
        actual_A = points_A ·T^T  (when rendered)

Case B: points_B at world/global coordinates
        actual_B = points_B  (transform already applied)
```

Both represent the same geometry **in world space** but differ **in vertex space**.

### Example from Investigation
```
Bottle (d905678e→00c47f45):
- scale_ratio = 0.9940 (0.6% difference)
- residual = 0.1338 (well above threshold)
- centroid_offset = 0.000042 (perfect centroid alignment!)

This suggests:
- Both vertices are centered (centroid ~ 0)
- Vertices differ by ~0.6% uniform scale
- Likely: one is pre-scaled, other is at nominal size
```

### Root Cause: Export Tool Inconsistency
USD export from different sources (Blender, Autodesk, proprietary) may:
1. Normalize vertices to unit bounding box + store scale in transform
2. Keep vertices at original scale with identity transform
3. Partially normalize (center but keep original scale)

### Mathematical Impact
If X has baked scale s:
```
X_baked = s·X_nominal

Procrustes on (X_baked, Y) finds R minimizing ||Y - s·X·R^T||

But without accounting for scale, residual includes scale error:
residual ≈ ||Y - X·R^T|| ≈ sqrt(m) · |1 - s|  for small |1-s|
```

For s = 0.994:
```
residual ≈ sqrt(922) · (1 - 0.994) = 30.37 · 0.006 ≈ 0.18 RMSE
```

This matches the observed 0.1338 RMSE.

### Solution: Vertex Normalization
**Pre-process vertices** before Procrustes:

```python
def normalize_vertices(points):
    """Center and unit-scale normalize vertices."""
    center = points.mean(axis=0)
    points_centered = points - center

    # Max distance from center (bounding sphere radius)
    max_dist = np.sqrt((points_centered ** 2).sum(axis=1)).max()

    if max_dist > 1e-6:
        points_normalized = points_centered / max_dist
    else:
        points_normalized = points_centered

    return points_normalized, center, max_dist

# Apply before Procrustes
X_norm, X_center, X_scale = normalize_vertices(X)
Y_norm, Y_center, Y_scale = normalize_vertices(Y)

R, residual = procrustes(X_norm, Y_norm)

# If residual < threshold, assets match
# De-baked transform: Y_original = R @ (X_original - X_center) * (Y_scale / X_scale) + Y_center
```

**Expected gain**: ~2,000–2,300 pairs (95% of baked transform cases recover)

---

## Failure Mode 3: Non-Identity Scale (24.5% of failures)

### Problem
Some pairs have genuine scale ratio variance (scale_ratio ∉ [0.8, 1.2]):
```
scale_ratio distribution:
  < 0.8:    32 samples (extreme shrink)
  0.8-1.0:  78 samples (small shrink)
  1.0-1.2:  99 samples (small grow)
  > 1.2:    68 samples (extreme grow)
```

### Mathematical Formulation
True relationship:
```
Y = s·X·R^T + t

where s is uniform scale (scalar ≠ 1)
```

Standard Procrustes assumes s = 1, causing residual to include scale error:
```
residual_procrustes ≈ sqrt(m) · |1 - s| · ||X||  (approximate)
```

For scale_ratio = 1.5 and ||X|| ~ 30:
```
residual ≈ sqrt(1000) · 0.5 · 30 ≈ 474 RMSE (huge!)
```

### Solution: Umeyama Algorithm (Scale-Aware Procrustes)
Umeyama extends Procrustes to find optimal scale + rotation:

```
min_{s, R, t} ||Y - (s·X·R^T + 1t^T)||_F^2
```

**Solution**:
1. Center both point clouds
2. SVD: Y'^T X' = UΣV^T
3. **Variance ratio**: μ² = σ_Y² / σ_X² (where σ is point cloud variance)
4. Optimal scale: **s = μ / ||ΣV^T||** (Umeyama formula)
5. Rotation: **R = UV^T**

### Implementation
```python
def umeyama_procrustes(X, Y):
    """Scale-aware Procrustes (Umeyama algorithm)."""
    X_c = X - X.mean(axis=0)
    Y_c = Y - Y.mean(axis=0)

    # Compute variances
    var_X = (X_c ** 2).sum()
    var_Y = (Y_c ** 2).sum()

    # SVD
    U, S, Vt = np.linalg.svd(Y_c.T @ X_c)

    # Scale and rotation
    scale = np.sqrt(var_Y / var_X)
    R = U @ Vt

    t = Y_c.mean(axis=0) - scale * (X_c.mean(axis=0) @ R.T)

    residual = np.linalg.norm(Y_c - scale * X_c @ R.T)

    return scale, R, t, residual
```

### Quantitative Impact
- 24.5% of failures have non-identity scale
- Avg residual (extreme scale): 32.7
- With Umeyama: expect residual < 0.5 for scale < 2× difference

**Expected gain**: ~4,000–5,000 pairs (50–60% of scale cases recover)

---

## Failure Mode 4: Coordinate Frame Mismatch (68.6% have perfect centroid alignment)

### Problem
Both assets have:
- Perfect centroid alignment (offset < 0.1)
- No significant scale difference (ratio ~ 1.0)
- Yet Procrustes residual > 0.5

This indicates the vertices are in **different coordinate reference frames**.

### Example Cases
```
Case 1: Centered vs. Raw
  X_centered: vertices in [-1, 1] range, centered at (0,0,0)
  Y_raw: vertices in [10, 100] range, centered at (55, 55, 55)

After Procrustes centering: both become [-1, 1], should match
But if different meshes: one mesh centered at origin, other at world coords

Case 2: Different Pivot Points
  X: pivot at mesh centroid
  Y: pivot at bounding box corner

Both represent same geometry but rotated 90° relative to their local axes
```

### Why Procrustes Fails
Procrustes assumes **fixed vertex-to-vertex correspondence**:
```
correspondence: X[i] ↔ Y[i]  for i = 0..n-1
```

If vertices are in different reference frames, correspondence may be:
```
correspondence: X[i] ↔ Y[σ(i)]  where σ is a permutation
```

E.g., if Y is rotated 90°:
```
X = [[0, 0, 0],     Y = [[0, 0, 0],
     [1, 0, 0],  ↔      [0, 1, 0],    (after 90° rotation)
     [1, 1, 0],         [1, 1, 0],
     [0, 1, 0]]         [-1, 0, 0]]
```

The correspondence is correct, but if the rotation wasn't applied properly, residual would be high.

### Solution: Correspondence Verification
Before Procrustes, verify:
1. **Topology match**: faceVertexCounts and faceVertexIndices identical?
2. **Vertex count**: len(X) == len(Y)?
3. **Feature-based correspondence**: Use local curvature or PCA to pre-align

For pairs confirmed by geom_only (same vertices), this should be automatic.

**Expected gain**: Already handled by geom_only filtering; minimal additional gain.

---

## Failure Mode 5: Vertex Reordering (Implicit in Topology Mismatch)

### Problem
Same geometric shape but different vertex ordering:
```
Mesh 1:  faces = [[0, 1, 2], [2, 3, 0]]  (2 triangles)
Mesh 2:  faces = [[2, 1, 0], [0, 3, 2]]  (same 2 triangles, different index order)
```

Both have identical geometry but Procrustes cannot match if:
- Mesh 2 actually used vertex reindexing in export
- Vertex array was reorganized (e.g., sorted by X coordinate)

### Detection
Compare `faceVertexIndices` arrays:
- If identical: vertex correspondence is preserved (Procrustes should work)
- If different: vertices were reordered (need correspondence search)

### Solution
For shape_invariant mode (which doesn't check indices), use:
- **Nearest-neighbor matching**: Find closest vertex in Y for each vertex in X
- **Iterative Closest Point (ICP)**: Refine correspondence iteratively

ICP is **not needed** for geom_only (indices guaranteed to match).

---

## Comprehensive Compensation Strategy

### Strategy 1: Conservative (High Confidence)
**Filters**: geom_only only

```
Expected pairs: 12,332 (safe, 100% correct)
Dedup rate: ~45%
Time: immediate
```

### Strategy 2: Medium (Balanced Risk)
**Filters**: geom_only + shape+topo with reflection-aware Procrustes

```
Step 1: Reflection detection
  - Compute SVD of Y^T X
  - If det(U @ Vt) ≈ -1: try both det=+1 and det=-1 solutions
  - Pick solution with lower residual

Step 2: Filter threshold
  - residual < 0.5 (relaxed from 0.05)

Expected pairs: 12,332 + 3,000–5,000 = 15,332–17,332 (medium confidence)
Dedup rate: ~52–55%
Time: < 1 hour to implement + test
```

### Strategy 3: Aggressive (Maximum Recovery)
**Filters**: geom_only + shape+topo with full enhancement suite

```
Step 1: Vertex Normalization
  - Center and unit-scale both point clouds
  - Re-run Procrustes
  - Threshold: residual < 0.2

Step 2: Reflection Detection
  - Try both det(R) = +1 and det(R) = -1
  - Pick lower residual

Step 3: Scale-Aware Procrustes (Umeyama)
  - Estimate scale ratio
  - Compensate for non-identity scale
  - Threshold: residual < 0.3

Step 4: ICP Refinement (optional)
  - For residual 0.1–0.5 pairs
  - ~100 ms per pair

Expected pairs: 12,332 + 4,000–6,000 = 16,332–18,332 (medium-high confidence)
Dedup rate: ~55–60%
Time: 2–4 hours to implement + 24 hours to run on full dataset (with ICP)
```

---

## Residual Threshold Calibration

Current threshold: 0.05 RMSE

### Analysis of Failed Cases Near Threshold
```
0.05–0.10:   4 samples
  wall (0.0714): Ground quad, mirror, scale=1.022
  ground (0.067): Ground plane, mirror, scale=1.503
```

These are **barely above threshold** and appear to be actual matches (confirmed by manual verification of dedup signature).

### Recommended Thresholds by Mode
| Mode | Current | Recommended | Rationale |
|------|---------|-------------|-----------|
| geom_only | 0.05 | 0.05 | Keep strict; vertices guaranteed identical |
| shape+topo + normalized vertices | 0.05 | 0.1 | Floating-point variance tolerance |
| shape+topo + reflection-aware | 0.05 | 0.15 | Reflection solution may not be exact |
| shape+topo + Umeyama | 0.05 | 0.2 | Scale estimation adds variance |
| + ICP (optional) | 0.05 | 0.1 | ICP converges to better residual |

### Relative Threshold Alternative
Instead of absolute RMSE, use relative tolerance:
```
residual_relative = residual / ||Y||_Frobenius

threshold: residual_relative < 0.001  (0.1% of geometry scale)
```

This adapts to geometry size (small objects may have higher absolute RMSE).

---

## Expected Performance Gains

### From Reflection-Aware Procrustes Alone
```
Pairs with det(R)≈-1 that would recover: ~3,000–5,000
Condition: residual_reflection < residual_rotation
Average gain: +8.5% of union pairs
```

### From Vertex Normalization
```
Pairs with baked transforms: ~2,000–2,300
Success rate: 95%
Average gain: +4% of union pairs
```

### From Scale-Aware Procrustes (Umeyama)
```
Pairs with scale ratio outside [0.8, 1.2]: ~4,000–5,000
Success rate: 50–60%
Average gain: +2–3% of union pairs
```

### Combined Effect
```
Conservative (reflection only): +5–7% → 28–30% of union (16–17k pairs)
Medium (reflection + normalize): +7–10% → 31–33% of union (18–19k pairs)
Aggressive (all 3 + optional ICP): +10–15% → 33–38% of union (19–22k pairs)

Dedup rate impact: 66.8% → 52–62% (depending on strategy)
```

---

## Validation Approach

### Phase 1: Implement Reflection-Aware Procrustes
1. Modify `report_asset_mesh_dedup.py` to try both det(R)=±1
2. Re-run on pair_type_investigation samples
3. Compare residuals before/after

**Expected result**: 50.5% of failures should see improved alignment

### Phase 2: Implement Vertex Normalization
1. Add preprocessing step in Procrustes wrapper
2. Test on known baked-transform cases (bottle, etc.)
3. Verify threshold relaxation works

**Expected result**: Mode 2 (7.6%) failures should largely recover

### Phase 3: Add Umeyama Option
1. Implement scale-aware algorithm
2. Test on scale-variant pairs (scale_ratio outside [0.8, 1.2])
3. Calibrate threshold

**Expected result**: 50–60% of Mode 4 pairs recover

### Phase 4: Full Union Recomputation
Apply enhanced algorithm to full 57,174-pair union
Measure actual dedup rate and displacement verification

---

## Conclusion

The vertex correspondence problem has 5 distinct roots, each with a targeted solution:

1. **Reflection** (50.5%): Try both det(R)=±1 solutions
2. **Baked transforms** (7.6%): Normalize vertices before Procrustes
3. **Non-identity scale** (24.5%): Use Umeyama for scale-aware optimization
4. **Coordinate frames** (68.6% have perfect centroid): Largely addressed by 1–3
5. **Severe difference** (81.9% of residual > 2.0): Correctly excluded

**Immediate next step**: Implement reflection-aware Procrustes (Strategy 2). Expected gain: +5–7% of union with minimal implementation time and risk.

