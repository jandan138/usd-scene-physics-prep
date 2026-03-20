---
title: Alignment Algorithm Comparison - ICP and Alternatives
code_reference:
  - scripts/report_asset_mesh_dedup.py
  - scipy.spatial.procrustes
  - open3d.pipelines.registration
created_at: 2026-03-19T09:20:00Z
updated_at: 2026-03-19T10:00:00Z
maintainer: Claude Code Agent
status: completed
---

## Executive Summary

For the dedup use case, we need to align two point clouds and verify they represent the same geometry. We have 4 main algorithm families, each with trade-offs:

| Algorithm | Time Complexity | Accuracy | Reflection Support | Scale Support | Vertex Reordering | Best For |
|-----------|-----------------|----------|-------------------|---------------|-------------------|----------|
| **Procrustes** | O(m²) + SVD | High (optimal) | No | No | Requires fixed correspondence | Known vertex order |
| **Procrustes + Reflection** | O(m²) + SVD | High | Yes | No | Requires fixed correspondence | Mirror/flipped assets |
| **Umeyama** | O(m²) + SVD | High | Yes | Yes | Requires fixed correspondence | Scale variant assets |
| **ICP** | O(m·n·k) | High (iterative) | Yes | Yes | Handles reordering | Unknown correspondence |
| **ICP + Scale** | O(m·n·k) + scale estimation | Very High | Yes | Yes | Handles reordering | Complex cases |

Where m, n = point cloud sizes, k = ICP iterations (typically 20–50).

---

## Algorithm 1: Standard Procrustes (Baseline)

### Mathematical Formulation
```
min_{R, t} ||Y - (X·R^T + 1·t^T)||_F²

subject to:  R ∈ SO(3)  [special orthogonal group, det(R)=+1]
             R^T·R = I  [orthonormality constraint]
```

### Algorithm
1. Center both point clouds
2. SVD: Y_c^T·X_c = U·Σ·V^T
3. R = U·V^T (enforces orthonormality)
4. t = mean(Y) - R·mean(X)
5. Residual = ||Y_c - X_c·R^T||_F / √m

### Complexity
- Time: O(m²) for covariance + O(m³) for SVD ≈ **O(m³)**
- Space: O(m²) for covariance matrix

For typical mesh sizes (m=100–5000), this is instant (< 1ms).

### Advantages
- **Optimal solution**: Minimizes exact objective
- **No iterations**: Closed-form solution
- **Numerically stable**: SVD is well-conditioned

### Disadvantages
- **No reflection support**: Forced det(R)=+1, fails for mirrors
- **No scale compensation**: Assumes uniform scale = 1
- **Fixed correspondence**: Requires points in same order

### Performance on Test Data
From pair_type_investigation:
- Success (residual < 0.05): 12,332/57,174 (21.6% — geom_only pairs)
- Failure (residual ≥ 0.05): 44,842/57,174 (78.4%)

---

## Algorithm 2: Reflection-Aware Procrustes (Minimal Enhancement)

### Mathematical Formulation
Try both constrained solutions:
```
Solution A: min_{R} ||Y - X·R^T||_F²  subject to det(R)=+1  [rotation]
Solution B: min_{R} ||Y - X·R^T||_F²  subject to det(R)=-1  [reflection]

Best: argmin(residual_A, residual_B)
```

### Algorithm
1. Center both point clouds
2. SVD: Y_c^T·X_c = U·Σ·V^T
3. **R₁ = U·V^T** (det ≈ +1, pure rotation)
4. **R₂ = U·diag(1,1,-1)·V^T** (det ≈ -1, rotation + reflection)
5. Compute residuals for both
6. Return solution with lower residual

### Complexity
- Time: O(m³) + one extra matrix multiply ≈ **O(m³)**
- Space: O(m²)

No significant overhead vs. standard Procrustes.

### Advantages
- **Handles reflection**: Can find mirror transformations
- **Closed-form**: Still optimal within reflection category
- **Minimal code change**: ~10 lines to standard Procrustes
- **No hyperparameters**: No tuning needed

### Disadvantages
- **Still no scale**: Assumes uniform scale = 1
- **Still needs correspondence**: Points must be in same order
- **Slightly higher residual**: Reflection may not be exact fit

### Expected Performance Gain
From investigation data:
- 140/277 failures (50.5%) involve reflection
- Of these, ~26 have residual in "recoverable" range (0.1–1.0)
- **Expected recovery**: 3,000–5,000 pairs (~10% improvement)

### Implementation
```python
def procrustes_reflection_aware(X, Y, reflection_cost_factor=1.0):
    """
    Procrustes with reflection option.

    Args:
        X, Y: point clouds (n × 3)
        reflection_cost_factor: penalty for reflection (default 1.0 = no penalty)

    Returns:
        R: optimal rotation/reflection matrix
        t: translation
        residual: RMSE
        used_reflection: bool indicating if reflection was used
    """
    X_c = X - X.mean(axis=0)
    Y_c = Y - Y.mean(axis=0)

    # SVD
    U, S, Vt = np.linalg.svd(Y_c.T @ X_c)

    # Solution 1: Pure rotation (det = +1)
    R_rot = U @ Vt
    res_rot = np.linalg.norm(Y_c - X_c @ R_rot.T)

    # Solution 2: Rotation + reflection (det = -1)
    R_refl = U @ np.diag([1, 1, -1]) @ Vt
    res_refl = np.linalg.norm(Y_c - X_c @ R_refl.T)

    # Apply penalty factor (optional)
    res_refl_penalized = res_refl * reflection_cost_factor

    # Choose best solution
    if res_refl_penalized < res_rot:
        return R_refl, Y_c.mean(axis=0) - R_refl @ X_c.mean(axis=0), res_refl, True
    else:
        return R_rot, Y_c.mean(axis=0) - R_rot @ X_c.mean(axis=0), res_rot, False
```

### When to Use
- First enhancement after base Procrustes
- For pairs with observed det(R) ≈ -1 (mirror assets)
- When speed is critical (< 1ms per pair)

---

## Algorithm 3: Umeyama (Scale-Aware Procrustes)

### Mathematical Formulation
```
min_{s, R, t} ||Y - (s·X·R^T + 1·t^T)||_F²

where s is uniform scale (scalar > 0)
```

### Algorithm
1. Center: X_c = X - mean(X), Y_c = Y - mean(Y)
2. Compute variances:
   - σ²_X = mean(||X_c||²)
   - σ²_Y = mean(||Y_c||²)
3. SVD: Y_c^T·X_c = U·Σ·V^T
4. Optimal scale: **s = √(σ²_Y / σ²_X)** (or s = tr(Σ)/tr(X_c^T·X_c))
5. R = U·V^T or U·diag(1,1,-1)·V^T
6. t = mean(Y) - s·R·mean(X)
7. Residual = ||Y_c - s·X_c·R^T||_F / √m

### Complexity
- Time: O(m³) (same SVD) + O(m) for scale estimation ≈ **O(m³)**
- Space: O(m²)

Still instantaneous.

### Advantages
- **Handles scale**: Can match geometry at different scales
- **Closed-form**: Optimal solution
- **Reflection capable**: Can use det(R)=-1 variant
- **Minimal overhead**: Negligible time penalty

### Disadvantages
- **Assumes uniform scale**: Non-uniform scaling will fail
- **Still needs correspondence**: Points must be ordered identically
- **Two parameters to solve**: scale + rotation (but decoupled)

### Expected Performance Gain
From investigation:
- 68/277 failures (24.5%) have scale outside [0.8, 1.2]
- Success rate: 50–60% with Umeyama
- **Expected recovery**: 2,000–3,000 pairs (~5% improvement)

### Implementation
```python
def umeyama_procrustes(X, Y):
    """
    Scale-aware Procrustes (Umeyama algorithm).

    Returns:
        s: optimal scale
        R: optimal rotation
        t: optimal translation
        residual: RMSE
    """
    X_c = X - X.mean(axis=0)
    Y_c = Y - Y.mean(axis=0)

    # Variances
    var_X = (X_c ** 2).sum() / len(X_c)
    var_Y = (Y_c ** 2).sum() / len(Y_c)

    # SVD
    U, S, Vt = np.linalg.svd(Y_c.T @ X_c)

    # Compute scale
    covariance = np.diag(S).sum()
    s = np.sqrt(var_Y / var_X)

    # Rotation
    R = U @ Vt

    # Translation
    t = Y_c.mean(axis=0) - s * R @ X_c.mean(axis=0)

    # Residual
    residual = np.linalg.norm(Y_c - s * X_c @ R.T) / np.sqrt(len(X_c))

    return s, R, t, residual
```

### When to Use
- For pairs with suspected scale differences
- When geom_only + reflection-aware Procrustes insufficient
- As second enhancement step

---

## Algorithm 4: Iterative Closest Point (ICP)

### Mathematical Formulation
```
Iterative minimize:
  min_{R, t} Σ_i ||Y[i] - (X[π(i)]·R^T + t)||²

where π(i) is correspondence (initially nearest-neighbor search)
```

### Algorithm (Point-to-Point ICP)
```
Initialize: R = I, t = 0
for iteration = 1 to max_iter:
    1. Find nearest neighbor: π(i) = argmin_j ||Y[i] - X[j]||
    2. Compute Procrustes on matched pairs
    3. Update R, t
    4. Recompute residual
    5. If residual converges: break
```

### Variants
1. **Point-to-Point ICP** (above): Simplest, fastest
2. **Point-to-Plane ICP**: Uses plane normals for better convergence
3. **Symmetric ICP**: Both X→Y and Y→X correspondences
4. **ICP with Scale**: Add scale update in each iteration

### Complexity
- Time: O(m·n·k·log(n)) where k = iterations (typically 20–50)
  - Nearest-neighbor search: O(log(n)) with KD-tree
  - Procrustes: O(m²)
  - Per iteration: O(m·n·log(n))
- Space: O(n) for KD-tree

For typical sizes: **~10ms–100ms per pair**

### Advantages
- **Handles unknown correspondence**: Automatically finds best matching
- **Handles vertex reordering**: Robust to permuted vertices
- **Can find better local optimum**: Iterative refinement
- **Reflection & scale capable**: Extend Procrustes step

### Disadvantages
- **Not globally optimal**: Converges to local minimum
- **Requires initialization**: Bad initial guess = wrong solution
- **Slower**: ~1,000× slower than Procrustes
- **Hyperparameters**: Iteration count, convergence threshold
- **Correspondence changes**: Non-deterministic in some cases

### Expected Performance
From literature (not measured on our data):
- **Recovery rate**: 80–95% for residual 0.1–1.0 range
- **Typical residual drop**: 30–50% reduction from initial
- **Convergence**: 20–50 iterations for typical meshes

### Implementation Options

#### Option 1: Use Open3D (Recommended)
```python
import open3d as o3d

def icp_align(X, Y, max_iterations=50):
    """ICP using Open3D."""
    X_cloud = o3d.geometry.PointCloud()
    X_cloud.points = o3d.utility.Vector3dVector(X)

    Y_cloud = o3d.geometry.PointCloud()
    Y_cloud.points = o3d.utility.Vector3dVector(Y)

    # ICP registration
    result = o3d.pipelines.registration.registration_icp(
        Y_cloud, X_cloud,
        max_correspondence_distance=10.0,  # Tunable
        init=np.eye(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        criteria=o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=max_iterations,
            relative_fitness=1e-6,
            relative_rmse=1e-6
        )
    )

    R = result.transformation[:3, :3]
    t = result.transformation[:3, 3]
    residual = result.inlier_rmse

    return R, t, residual
```

**Pros**: Robust, well-tested, fast C++ backend
**Cons**: External dependency (o3d), larger overhead

#### Option 2: Pure Python Implementation
```python
def icp_point_to_point(X, Y, max_iter=50, tol=1e-6):
    """Bare-bones point-to-point ICP in pure Python."""
    R = np.eye(3)
    t = np.zeros(3)
    X_transformed = X.copy()

    prev_residual = float('inf')

    for iteration in range(max_iter):
        # Find nearest neighbors
        distances = np.linalg.norm(Y[:, None, :] - X_transformed[None, :, :], axis=2)
        correspondences = np.argmin(distances, axis=1)

        # Procrustes on matched pairs
        X_matched = X_transformed[correspondences]
        R_new, t_new, residual = procrustes_reflection_aware(X_matched, Y)

        # Update transformation
        R = R_new @ R
        t = t_new + t
        X_transformed = X @ R.T + t

        # Check convergence
        if abs(prev_residual - residual) < tol:
            break
        prev_residual = residual

    return R, t, residual
```

**Pros**: No dependencies, fully under control
**Cons**: Slower (Python overhead), needs tuning

### When to Use ICP
1. **Moderate uncertainty** (residual 0.1–2.0): ICP can refine alignment
2. **Unknown correspondence**: Vertices reordered or non-matching
3. **Mixed-mode dedup**: Combine with Procrustes for robustness
4. **Second-pass verification**: Run on pre-aligned pairs

---

## Algorithm Comparison Table (Detailed)

| Criterion | Procrustes | + Reflection | Umeyama | ICP |
|-----------|-----------|-------------|---------|-----|
| **Time/pair** | <1ms | <1ms | <1ms | 10–100ms |
| **Global optimality** | Yes | Yes* | Yes | No |
| **Handles reflection** | No | Yes | Can extend | Yes |
| **Handles scale** | No | No | Yes | Can extend |
| **Handles reordering** | No | No | No | Yes |
| **Requires tuning** | No | No | No | Yes (3–5 params) |
| **Deterministic** | Yes | Yes | Yes | No** |
| **For identical vertices** | ✓ | ✓ | ✓ | ✓ |
| **For near-identical** | ✗ | ~ | ~ | ✓ |
| **For scale-variant** | ✗ | ✗ | ✓ | ✓ |
| **For reordered** | ✗ | ✗ | ✗ | ✓ |

*Optimal within rotation class or reflection class, but only one is returned
**Small random variations possible depending on KD-tree structure

---

## Strategy: Phased Approach

### Phase 1: Reflection-Aware Procrustes (Recommended First Step)
**Effort**: 2–3 hours
**Expected gain**: 5–7% of union (3,000–5,000 pairs)
**Risk**: Low (closed-form, no new failure modes)

```python
# In report_asset_mesh_dedup.py, around line XXX:
if pair_type in ['geom_only', 'shape_invariant']:
    R, t, residual, used_reflection = procrustes_reflection_aware(X, Y)
    if residual < threshold:
        mark_as_safe(pair)
```

### Phase 2: Add Vertex Normalization
**Effort**: 1–2 hours
**Expected gain**: 4–6% of union (2,000–3,000 pairs)
**Risk**: Very low (preprocessing only)

```python
# Pre-process before Procrustes:
X_norm, X_center, X_scale = normalize_vertices(X)
Y_norm, Y_center, Y_scale = normalize_vertices(Y)
R, t, residual, _ = procrustes_reflection_aware(X_norm, Y_norm)

if residual < threshold:
    # De-normalize transformation for validation
    ...
```

### Phase 3: Scale-Aware Procrustes (Umeyama)
**Effort**: 1–2 hours
**Expected gain**: 2–3% of union (1,000–2,000 pairs)
**Risk**: Low (closed-form, but adds scale parameter)

```python
# For shape+topo pairs with scale variance:
if scale_ratio < 0.8 or scale_ratio > 1.2:
    s, R, t, residual = umeyama_procrustes(X_norm, Y_norm)
    if residual < threshold and 0.7 < s < 1.4:  # Sanity check
        mark_as_safe(pair)
```

### Phase 4: ICP for Refinement (Optional, Lower Priority)
**Effort**: 3–4 hours (with Open3D)
**Expected gain**: 2–4% of union (1,000–2,000 pairs)
**Risk**: Medium (iterative, non-deterministic)

```python
# For residual 0.1–2.0 pairs that failed Procrustes variants:
if 0.1 < residual < 2.0:
    try:
        R_icp, t_icp, residual_icp = icp_point_to_point(X_norm, Y_norm)
        if residual_icp < 0.2:  # Significantly improved
            mark_as_safe(pair)
    except:
        pass  # ICP diverged; keep original classification
```

---

## Threshold Calibration Strategy

### Observation: Residuals are Bimodal
From investigation:
- **Mode 1** (recoverable): 0.05–1.0 RMSE → real matches
- **Mode 2** (true differences): 2.0+ RMSE → distinct geometry

Clear gap between modes → can use simple threshold

### Recommended Thresholds
| Algorithm | Threshold | Reasoning |
|-----------|-----------|-----------|
| Procrustes base | 0.05 | Geom_only only; identical vertices |
| + Reflection | 0.10 | Reflection introduces ~5% variance |
| + Normalize | 0.15 | Floating-point tolerance + scale variance |
| + Umeyama | 0.20 | Scale estimation adds variance |
| + ICP | 0.10 | ICP converges to better optimum |

### Adaptive Threshold Option
```python
# Scale-relative threshold
threshold = 0.001 * bounding_sphere_radius(Y)

# This adapts to geometry size
# Large objects can have higher absolute RMSE but same relative quality
```

---

## Summary & Recommendations

### Best Path Forward
1. **Immediate (< 1 day)**:
   - Implement reflection-aware Procrustes
   - Re-run on test0_rebuilt-normalized
   - Measure actual dedup rate improvement

2. **Short-term (1–2 days)**:
   - Add vertex normalization
   - Relax threshold to 0.10
   - Run on full C1 union

3. **Medium-term (optional, 1 week)**:
   - Add Umeyama if gain is significant
   - Profile: if ICP useful, consider Open3D integration

4. **Long-term (optional, 2+ weeks)**:
   - Integrate ICP for edge cases
   - Per-category threshold tuning
   - Create dedup quality metrics dashboard

### Expected Final Results
- **Conservative (reflection + normalize)**: 52–55% dedup (down from 66.8%)
- **Medium (+ Umeyama)**: 55–60% dedup
- **Aggressive (+ ICP)**: 60–65% dedup (approaching original with higher confidence)

All strategies maintain **significantly higher confidence** than current 66.8% (which includes 30k uncertain pairs).

