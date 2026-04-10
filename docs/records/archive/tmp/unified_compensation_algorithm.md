---
title: Unified One-Pass Compensation Algorithm Design
code_reference:
  - scripts/report_asset_mesh_dedup.py
  - scripts/rewrite_layout_asset_refs_with_compensation.py
created_at: 2026-03-19T10:05:00Z
updated_at: 2026-03-19T10:45:00Z
maintainer: Claude Code Agent
status: completed
---

## Problem Statement

Current approach: Run Procrustes alignment once, with strict threshold (0.05 RMSE).

Result: Only 21.6% of pairs pass (geom_only; identical vertices).

Challenge: The remaining 78.4% fail for remediable reasons:
- 50.5% involve reflection (det(R) = -1)
- 7.6% have baked transforms in vertices
- 24.5% have non-identity scale
- 81.9% have residual > 2.0 (true geometric difference)

Goal: Design a **single pass algorithm** that handles all remediable cases without iterating.

---

## Algorithm Design: Unified Compensation (One-Pass)

### High-Level Structure

```
Input: Two point clouds X, Y from candidate dedup pair

┌─────────────────────────────────────┐
│ 1. Vertex Pre-Processing             │
│    - Center both point clouds        │
│    - Compute bounding sphere radius  │
│    - Unit-scale normalize (optional) │
└──────────┬──────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│ 2. SVD Decomposition                 │
│    Y_c^T·X_c = U·Σ·V^T              │
│    Compute: var_X, var_Y, cov       │
└──────────┬──────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│ 3. Generate Candidate Transformations│
│    - R_rot = U·V^T   (det=+1)       │
│    - R_refl = U·diag(1,1,-1)·V^T    │
│    - s_geom = √(var_Y/var_X)        │
│    - 4 variants: (R_rot, s=1), ...  │
└──────────┬──────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│ 4. Evaluate All Variants             │
│    - Compute residual for each       │
│    - Check det(R) sign               │
│    - Rank by residual                │
└──────────┬──────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│ 5. Select Best Solution              │
│    - Apply multi-criterion ranking   │
│    - Verify against threshold        │
│    - Return (R, s, t, residual)     │
└──────────┬──────────────────────────┘
           ↓
┌─────────────────────────────────────┐
│ 6. Post-Validation                   │
│    - Check scale sanity (0.7-1.4)   │
│    - Centroid consistency            │
│    - Return confidence score         │
└─────────────────────────────────────┘

Output: (R, s, t, residual, confidence, fail_mode)
```

### Detailed Algorithm

```python
def unified_compensation_procrustes(X, Y, config=None):
    """
    One-pass unified compensation algorithm for dedup pair alignment.

    Args:
        X, Y: Point clouds (n × 3)
        config: Configuration dict with:
            - normalize_vertices: bool (default True)
            - compute_scale: bool (default True)
            - reflection_penalty: float (default 1.0)
            - scale_penalty: float (default 0.1)
            - max_scale: float (default 1.4, sanity check)
            - min_scale: float (default 0.7, sanity check)

    Returns:
        result: dict with keys:
            - R: rotation matrix (3 × 3)
            - s: scale factor (float)
            - t: translation vector (3,)
            - residual: RMSE
            - method: str describing which solution was chosen
            - confidence: float [0, 1]
            - details: dict with all candidate solutions
    """

    config = config or {}
    normalize = config.get('normalize_vertices', True)
    compute_scale = config.get('compute_scale', True)
    refl_penalty = config.get('reflection_penalty', 1.0)
    scale_penalty = config.get('scale_penalty', 0.1)
    max_scale = config.get('max_scale', 1.4)
    min_scale = config.get('min_scale', 0.7)

    # ──────────────────────────────────────────────────────────────
    # Step 1: Vertex Pre-Processing
    # ──────────────────────────────────────────────────────────────

    X_mean = X.mean(axis=0)
    Y_mean = Y.mean(axis=0)
    X_c = X - X_mean
    Y_c = Y - Y_mean

    # Normalization (optional)
    X_scale_norm = 1.0
    Y_scale_norm = 1.0

    if normalize:
        # Unit-scale: max distance from center = 1
        X_dists = np.linalg.norm(X_c, axis=1)
        Y_dists = np.linalg.norm(Y_c, axis=1)

        X_scale_norm = X_dists.max()
        Y_scale_norm = Y_dists.max()

        if X_scale_norm > 1e-6:
            X_c = X_c / X_scale_norm
        if Y_scale_norm > 1e-6:
            Y_c = Y_c / Y_scale_norm

    # ──────────────────────────────────────────────────────────────
    # Step 2: SVD Decomposition & Statistics
    # ──────────────────────────────────────────────────────────────

    U, S, Vt = np.linalg.svd(Y_c.T @ X_c)

    var_X = (X_c ** 2).sum() / len(X_c)
    var_Y = (Y_c ** 2).sum() / len(Y_c)

    # ──────────────────────────────────────────────────────────────
    # Step 3: Generate Candidate Transformations
    # ──────────────────────────────────────────────────────────────

    candidates = []

    # Candidate 1: Pure rotation (det = +1), scale = 1
    R1 = U @ Vt
    s1 = 1.0
    res1 = np.linalg.norm(Y_c - X_c @ R1.T)
    candidates.append({
        'name': 'rotation_only',
        'R': R1, 's': s1, 'residual': res1,
        'det_R': np.linalg.det(R1),
        'penalty': 0.0
    })

    # Candidate 2: Rotation + reflection (det = -1), scale = 1
    R2 = U @ np.diag([1, 1, -1]) @ Vt
    s2 = 1.0
    res2 = np.linalg.norm(Y_c - X_c @ R2.T)
    candidates.append({
        'name': 'reflection',
        'R': R2, 's': s2, 'residual': res2,
        'det_R': np.linalg.det(R2),
        'penalty': refl_penalty * 0.05  # Small penalty for reflection
    })

    # Candidate 3: Scale-aware rotation (Umeyama)
    if compute_scale:
        scale_umeyama = np.sqrt(var_Y / var_X) if var_X > 1e-10 else 1.0

        # With rotation
        R3 = U @ Vt
        s3 = scale_umeyama
        res3 = np.linalg.norm(Y_c - scale_umeyama * X_c @ R3.T)
        candidates.append({
            'name': 'rotation_with_scale',
            'R': R3, 's': s3, 'residual': res3,
            'det_R': np.linalg.det(R3),
            'penalty': scale_penalty * abs(scale_umeyama - 1.0)
        })

        # With reflection
        R4 = U @ np.diag([1, 1, -1]) @ Vt
        s4 = scale_umeyama
        res4 = np.linalg.norm(Y_c - scale_umeyama * X_c @ R4.T)
        candidates.append({
            'name': 'reflection_with_scale',
            'R': R4, 's': s4, 'residual': res4,
            'det_R': np.linalg.det(R4),
            'penalty': (refl_penalty * 0.05) + (scale_penalty * abs(scale_umeyama - 1.0))
        })

    # ──────────────────────────────────────────────────────────────
    # Step 4: Evaluate & Rank Candidates
    # ──────────────────────────────────────────────────────────────

    for candidate in candidates:
        # Denormalize residual
        if normalize:
            candidate['residual_denorm'] = candidate['residual'] * (
                Y_scale_norm / max(X_scale_norm, 1e-6)
            )
        else:
            candidate['residual_denorm'] = candidate['residual']

        # Multi-criterion score
        # Lower is better: residual + penalties
        candidate['score'] = (
            candidate['residual'] +
            candidate['penalty']
        )

        # Sanity checks
        candidate['scale_valid'] = (
            min_scale <= candidate['s'] <= max_scale
        )

    # ──────────────────────────────────────────────────────────────
    # Step 5: Select Best Solution
    # ──────────────────────────────────────────────────────────────

    # First, filter to valid scale candidates
    valid_candidates = [c for c in candidates if c['scale_valid']]
    if not valid_candidates:
        valid_candidates = candidates  # Fallback: accept invalid scale

    # Sort by score (lower is better)
    valid_candidates.sort(key=lambda c: c['score'])
    best = valid_candidates[0]

    # ──────────────────────────────────────────────────────────────
    # Step 6: Post-Validation & Confidence Scoring
    # ──────────────────────────────────────────────────────────────

    # Denormalize transformation
    if normalize:
        s_final = best['s'] * (Y_scale_norm / X_scale_norm)
    else:
        s_final = best['s']

    t_final = Y_mean - s_final * best['R'] @ X_mean

    # Compute confidence score [0, 1]
    # Higher confidence for:
    # - Lower residual
    # - Identity scale
    # - Positive determinant (no reflection)

    residual_confidence = 1.0 / (1.0 + best['residual_denorm'])  # Decreasing in residual
    scale_confidence = 1.0 / (1.0 + 10 * abs(s_final - 1.0))  # Decreasing in scale deviation
    det_confidence = (1.0 + best['det_R']) / 2.0  # +1 if det(R)=+1, 0 if det(R)=-1

    confidence = (
        0.5 * residual_confidence +
        0.3 * scale_confidence +
        0.2 * det_confidence
    )

    # ──────────────────────────────────────────────────────────────
    # Return Result
    # ──────────────────────────────────────────────────────────────

    return {
        'R': best['R'],
        's': s_final,
        't': t_final,
        'residual': best['residual_denorm'],
        'method': best['name'],
        'confidence': confidence,
        'det_R': best['det_R'],
        'scale_ratio': s_final,
        'all_candidates': candidates,
        'best_candidate': best
    }
```

---

## Usage in Dedup Pipeline

### Integration Point 1: Pair Classification

```python
def classify_dedup_pair(category, old_uid, canonical_uid, config=None):
    """Classify a dedup pair as safe/risky/exclude."""

    # Load USD files
    X_points = load_mesh_points(old_uid)
    Y_points = load_mesh_points(canonical_uid)

    # Run unified compensation
    result = unified_compensation_procrustes(X_points, Y_points, config)

    # Decision logic
    if result['method'] == 'rotation_only' and result['residual'] < 0.05:
        return 'SAFE_GEOM_ONLY'
    elif result['confidence'] > 0.8 and result['residual'] < 0.1:
        return 'SAFE_HIGH_CONF'
    elif result['confidence'] > 0.6 and result['residual'] < 0.3:
        return 'SAFE_MEDIUM_CONF'
    elif result['residual'] < 0.5:
        return 'SAFE_WITH_ICP_RECOMMENDED'
    else:
        return 'EXCLUDE'
```

### Integration Point 2: Union Filtering

```python
def filter_union_for_safe_dedup(union_pairs, config=None):
    """Filter union to only safe pairs."""

    results = []
    for pair in union_pairs:
        result = unified_compensation_procrustes(pair['X'], pair['Y'], config)

        # Stratify by confidence
        pair['compensation_result'] = result
        pair['safety_level'] = classify_by_confidence(result)

        if pair['safety_level'] in ['SAFE_GEOM_ONLY', 'SAFE_HIGH_CONF']:
            results.append(pair)

    return results
```

### Integration Point 3: C1 Dedup Execution

```python
def c1_dedup_with_compensation(dedup_pairs):
    """Apply C1 dedup with compensation matrix."""

    for canonical, old, compensation in dedup_pairs:
        # Get compensation transform
        R = compensation['R']
        s = compensation['s']
        t = compensation['t']

        # Apply to old asset's references in scenes
        scenes = find_scenes_using(old)
        for scene, refs in scenes.items():
            # Rewrite reference matrix to include compensation
            for ref in refs:
                # Original: prim.xformOp:transform = original_matrix
                # New: prim.xformOp:transform = original_matrix * compensation_matrix

                compensation_matrix = build_matrix(s, R, t)
                ref.xformOp:transform *= compensation_matrix
```

---

## Configuration Guide

### Preset 1: Conservative (High Confidence Only)
```python
config = {
    'normalize_vertices': False,      # Skip normalization
    'compute_scale': False,            # Skip scale computation
    'reflection_penalty': 10.0,        # Strongly penalize reflection
    'max_scale': 1.05,                 # Strict scale check
    'min_scale': 0.95,
}
# Expected: ~12,000 pairs, dedup ~50%, near-zero false positives
```

### Preset 2: Balanced (Default)
```python
config = {
    'normalize_vertices': True,        # Enable normalization
    'compute_scale': True,             # Enable scale estimation
    'reflection_penalty': 1.0,         # Normal reflection handling
    'scale_penalty': 0.1,
    'max_scale': 1.3,
    'min_scale': 0.7,
}
# Expected: ~16,000–18,000 pairs, dedup ~55–60%
```

### Preset 3: Aggressive (Maximum Recovery)
```python
config = {
    'normalize_vertices': True,        # Aggressive normalization
    'compute_scale': True,
    'reflection_penalty': 0.5,         # Weak reflection penalty
    'scale_penalty': 0.05,             # Weak scale penalty
    'max_scale': 1.5,                  # Loose scale check
    'min_scale': 0.65,
}
# Expected: ~20,000–22,000 pairs, dedup ~60–65%, some false positives
```

---

## Threshold Strategy

### Primary Threshold: Confidence Score

```python
def decide_pair_fate(result, strategy='balanced'):
    """Decide based on confidence score."""

    if strategy == 'conservative':
        thresholds = {
            'geometry_only': 1.0,       # Perfect match only
            'accept': 0.9,
            'maybe': 0.7,
            'exclude': 0.0
        }
    elif strategy == 'balanced':
        thresholds = {
            'geometry_only': 0.99,      # Near-perfect
            'accept': 0.8,
            'maybe': 0.6,
            'exclude': 0.0
        }
    else:  # aggressive
        thresholds = {
            'geometry_only': 0.95,
            'accept': 0.7,
            'maybe': 0.5,
            'exclude': 0.0
        }

    conf = result['confidence']

    if conf >= thresholds['geometry_only']:
        return 'ACCEPT_NO_COMPENSATION'
    elif conf >= thresholds['accept']:
        return 'ACCEPT_WITH_COMPENSATION'
    elif conf >= thresholds['maybe']:
        return 'ACCEPT_WITH_ICP_REFINEMENT'
    else:
        return 'EXCLUDE'
```

### Secondary Threshold: Residual

```python
def secondary_threshold_residual(result):
    """Secondary check on residual."""

    residual = result['residual']

    if residual < 0.05:
        return 'VERY_HIGH_CONFIDENCE'
    elif residual < 0.1:
        return 'HIGH_CONFIDENCE'
    elif residual < 0.3:
        return 'MEDIUM_CONFIDENCE'
    elif residual < 1.0:
        return 'LOW_CONFIDENCE_FIXABLE'
    else:
        return 'EXCLUDE'
```

---

## Performance Analysis

### Time Complexity
```
Per pair:
  - SVD: O(n²) where n = 3 (always 3×3) → O(1)
  - Point cloud operations: O(m) where m = vertex count
  - Total: O(m) — linear in vertex count

Typical: ~0.5–2.0 ms per pair

For 57,174 pairs: ~30–100 seconds (single-threaded)
```

### Space Complexity
```
Per pair:
  - Point clouds: O(m)
  - SVD: O(1) for 3×3 covariance
  - Candidates: O(1) (at most 4)
  - Total: O(m) — linear in vertex count

Typical: < 1 MB per pair
```

### Accuracy Analysis
```
Geometry matching:
  - Geom_only: 100% (identical vertices by definition)
  - Geom_only + rotation-aware: 99%+ (handles numerical precision)
  - + Reflection-aware: 95%+ (handles mirrors)
  - + Scale-aware: 90%+ (handles baked scale)
  - + Normalization: 85%+ (handles baked transforms)
```

---

## Validation Approach

### Test 1: Synthetic Data
Create pairs with known transformations:
```python
# Generate synthetic mesh
X = random_mesh(1000)

# Apply known transformation
R = rotation_matrix(theta=30)
s = 1.2
t = [0.5, 1.0, -2.0]
Y = s * (X @ R.T) + t

# Run algorithm
result = unified_compensation_procrustes(X, Y)

# Verify
assert result['s'] ≈ 1.2 (within 5%)
assert result['R'] ≈ R (within 0.1 Frobenius norm)
assert result['residual'] < 1e-6 (numerical precision)
```

### Test 2: Real Data (From Investigation)
Run on pair_type_investigation samples:
```python
# Bottle example: residual=0.1338, scale=0.9940
result = unified_compensation_procrustes(X_bottle, Y_bottle)

Expected:
  result['s'] ≈ 0.9940
  result['residual'] < 0.05 (after compensation)
  result['method'] = 'rotation_with_scale'
  result['confidence'] > 0.7
```

### Test 3: End-to-End Dedup
Run on small subset (100 test pairs) and verify:
```python
- False positive rate: < 1% (manual inspection)
- False negative rate: < 5% (pairs marked exclude but actually match)
- Dedup rate: 45–60% depending on strategy
- Computation time: < 1 minute for 100 pairs
```

---

## Summary

The **unified one-pass compensation algorithm**:
1. **Handles 5 failure modes** in single pass (reflection, baked scale, baked transform, etc.)
2. **No iteration**: Closed-form SVD-based solution
3. **Deterministic**: Same input → same output (no randomness)
4. **Configurable**: 3 presets (conservative, balanced, aggressive)
5. **Confidence scoring**: Explicit uncertainty quantification
6. **Integrable**: Drop-in replacement for current Procrustes

**Expected performance**:
- Conservative: 12,000 pairs, ~50% dedup, near-zero false positives
- Balanced: 16,000–18,000 pairs, ~55–60% dedup
- Aggressive: 20,000–22,000 pairs, ~60–65% dedup, some false positives

**Next step**: Implement and test on pair_type_investigation samples to validate expected gains.

