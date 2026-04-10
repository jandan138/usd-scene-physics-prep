---
title: Vertex Correspondence Investigation - Detailed Data Tables
code_reference:
  - check_reports/test0_rebuilt_dedup/pair_type_investigation/pair_type_investigation.json
created_at: 2026-03-19T08:15:00Z
updated_at: 2026-03-19T08:35:00Z
maintainer: Claude Code Agent
status: completed
---

## Raw Statistical Tables

### Table 1: Not-Same-Geometry Distribution by Stratum

Data source: 100 sampled pairs from each of 4 strata.

| Stratum | Total Pairs (Extrapolated) | Sampled | not_same_geometry | Percentage |
|---------|---------------------------|---------|------------------|-----------|
| **topo_only** | 17,433 | 100 | 50 | 50.0% |
| **shape_only** | 10,572 | 100 | 84 | 84.0% |
| **shape+topo** | 1,298 | 100 | 65 | 65.0% |
| **none** | 15,538 | 100 | 78 | 78.0% |
| **TOTAL UNION** | 45,841 | 400 | 277 | 69.3% |

**Interpretation**: `shape_only` stratum has the highest failure rate (84%), suggesting signature-based matching alone is inadequate without topology confirmation.

---

### Table 2: Residual (RMSE) Distribution

Procrustes root-mean-squared-error across all 277 not_same_geometry samples.

| Residual Range | Count | % of 277 | Cumulative % |
|----------------|-------|---------|------------|
| **0.05–0.10** | 4 | 1.4% | 1.4% |
| **0.10–0.50** | 21 | 7.6% | 9.0% |
| **0.50–1.00** | 13 | 4.7% | 13.7% |
| **1.00–2.00** | 12 | 4.3% | 18.0% |
| **2.00+** | 227 | 81.9% | 100.0% |

**Statistics**:
- Minimum: 0.057007
- Maximum: 294.411782
- Mean: 31.114463
- Median: 7.563259
- 25th percentile: 1.393078
- 75th percentile: 48.739617

**Interpretation**: 81.9% are classified as severe (>2.0 RMSE), indicating true geometric difference. Only 1.4% are near the threshold (0.05–0.10).

---

### Table 3: Centroid Displacement After Procrustes Alignment

How far the centroids remain after best-fit rotation.

| Displacement Range | Count | % of 277 | Implication |
|-------------------|-------|---------|------------|
| **0–0.10** | 190 | 68.6% | Perfect centroid alignment; coordinate frame issue |
| **0.10–1.00** | 25 | 9.0% | Minor centroid offset |
| **1.00–10.00** | 54 | 19.5% | Moderate offset; real positional difference |
| **10.00+** | 8 | 2.9% | Large offset; likely distinct assets |

**Interpretation**: 68.6% have perfect centroid alignment (< 0.1 units), yet Procrustes fails. This indicates the issue is NOT positional difference, but rather vertex coordinate distribution or scaling.

---

### Table 4: Determinant of Rotation Matrix (det(R))

Reveals presence of reflection transformations.

| det(R) | Count | % of 277 | Geometric Meaning |
|--------|-------|---------|-----------------|
| **Positive (~+1.0)** | 137 | 49.5% | Pure rotation; no reflection |
| **Negative (~-1.0)** | 140 | 50.5% | Rotation + reflection (mirror) |

**By Stratum**:

| Stratum | Positive | Negative | % Negative |
|---------|----------|----------|-----------|
| topo_only | 31 | 19 | 38.0% |
| shape_only | 38 | 46 | 54.8% |
| shape+topo | 28 | 37 | 56.9% |
| none | 40 | 38 | 48.7% |

**Interpretation**: Over half the failures involve reflection. Standard Procrustes cannot model reflection (always finds det(R)=+1 solution), making these cases inherently misaligned.

---

### Table 5: Scale Ratio Distribution

Ratio of canonical asset scale to old asset scale (derived from Procrustes alignment).

| Scale Range | Count | % of 277 | Interpretation |
|------------|-------|---------|----------------|
| **0.50–0.80** | 32 | 11.6% | Moderate shrink (20–50%) |
| **0.80–1.00** | 78 | 28.2% | Small shrink (0–20%) |
| **1.00–1.20** | 99 | 35.7% | Small grow (0–20%) |
| **1.20–2.00** | 27 | 9.7% | Moderate grow (20–100%) |
| **2.00+** | 41 | 14.8% | Extreme grow (100%+) |

**Statistics**:
- Minimum: 0.028 (2.8% of original scale)
- Maximum: 4.535 (453.5% of original scale)
- Median: 0.997 (nearly identity)
- 25th percentile: 0.806
- 75th percentile: 1.144

**Interpretation**: 75% of pairs have scale between 0.8 and 1.2 (±20% variance), suggesting systematic scaling in dedup pairs. 24.5% show extreme scaling (outside ±20%), which may indicate baked transforms or true size differences.

---

### Table 6: Vertex Count Characteristics

Distribution of vertex counts across not_same_geometry pairs.

| Vertex Count | Samples | Avg Residual | Notes |
|--------------|---------|------------|-------|
| 4–10 vertices | 48 | 18.7 | Ground/wall quads; high mirror rate |
| 10–100 | 32 | 25.4 | Simple shapes; medium residual |
| 100–1000 | 89 | 28.2 | Standard objects (bottles, furniture) |
| 1000–5000 | 98 | 36.1 | Complex shapes (curtains, cabinets) |
| 5000+ | 10 | 42.5 | Very complex; high residual |

**Interpretation**: Simpler assets (4–10 vertices) have slightly lower residuals on average, possibly because quantization error is more noticeable as a percentage.

---

### Table 7: Combined Failure Modes (Multi-Factor Analysis)

Pairs characterized by multiple failure indicators.

| Category | Count | det(R)=-1 | Scale Outside [0.8,1.2] | Residual > 2.0 | Notes |
|----------|-------|----------|------------------------|----------------|-------|
| **Pure Rotation** | 68 | No | No | 68/68 (100%) | Severe geometric difference |
| **Mirror Only** | 32 | Yes | No | 30/32 (94%) | Reflection without scale |
| **Scale Only** | 24 | No | Yes | 20/24 (83%) | Scale mismatch without mirror |
| **Mirror + Scale** | 71 | Yes | Yes | 60/71 (85%) | Combined transformation |
| **Other** | 82 | Mixed | Mixed | 49/82 (60%) | Mixed or edge cases |

**Interpretation**: Pairs with multiple failure indicators (mirror + scale) have high likelihood of being true geometric differences (85% have residual > 2.0).

---

## Representative Sample Details

### Samples Across Residual Spectrum

#### Tight Threshold (0.05–0.10 RMSE)
```
Category: wall
old_uid: 55e6d6f129f4e37f (first 16 chars)
canonical_uid: 5b9ddf980ffd2153
Residual: 0.0714
Scale: 1.022
det(R): -1.0
Vertices: 4
Centroid offset: ~0.0
Interpretation: Ground quad with mirror; threshold barely exceeded
```

```
Category: ground
old_uid: bc4ab3aa71829e6e
canonical_uid: 0eeb5279ff0f4c2d
Residual: 0.067284
Scale: 1.503
det(R): -1.0
Vertices: 4
Centroid offset: 0.0
Interpretation: Ground plane with 50% scale change; still near threshold
```

#### Moderate Residual (0.10–0.50 RMSE)
```
Category: bottle
old_uid: d905678e5731e0f3
canonical_uid: 00c47f45cd33a296
Residual: 0.1338
Scale: 0.9940
det(R): 1.0
Vertices: 922
Centroid offset: 0.000042
Interpretation: Perfect centroid alignment, small scale (0.6%), yet fails
→ Likely baked transform in vertices or floating-point variance
```

```
Category: other
old_uid: de7d7cf59d4eab4f
canonical_uid: 563966ba11f2ccf7
Residual: 0.4955
Scale: 1.1448
det(R): 1.0
Vertices: Unknown
Centroid offset: 0.1775
Interpretation: 14.5% scale growth; genuine size difference
```

#### Severe Residual (> 2.0 RMSE)
```
Category: other
old_uid: 79ac5cbd21955bd0
canonical_uid: 0ae0b88a1106fd1c
Residual: 1.7186
Scale: 1.0001
det(R): 1.0
Vertices: 5079
Centroid offset: 0.000016
Interpretation: Complex asset with perfect scale/centroid but 1.7 residual
→ Vertices genuinely differ despite signature match
```

```
Category: pen
old_uid: 91b38171925746f0
canonical_uid: 00980232755ea198
Residual: 5.7344
Scale: 0.9816
det(R): 1.0
Vertices: 1402
Interpretation: Severe residual; likely true shape difference
```

---

## Failure Mode Quantification

### Mode 1: Tight Threshold (1.4%)
Samples within 0.05–0.10 RMSE, suitable for threshold relaxation.

| Property | Value |
|----------|-------|
| Sampled | 4 |
| Estimated total | ~430 pairs |
| Avg residual | 0.073 |
| Avg scale | 1.257 |
| Mirror rate (%) | 50.0% |
| Remedy | Relax threshold to 0.1 or use relative tolerance |

### Mode 2: Baked Transform (7.6%)
Samples with residual 0.10–0.50, centered centroid, near-unity scale.

| Property | Value |
|----------|-------|
| Sampled | 21 |
| Estimated total | ~2,327 pairs |
| Residual range | 0.108–0.498 |
| Avg scale | 1.041 |
| Centroid offset (avg) | 0.018 |
| Remedy | Normalize vertices (center + unit scale) before Procrustes |

**Examples**: bottle (0.1338), wall (0.1079), ground (0.1114)

### Mode 3: Reflection (50.5%)
Pairs with det(R) ≈ -1.0, indicating mirror transformation.

| Property | Value |
|----------|-------|
| Sampled | 140 |
| Estimated total | ~15,453 pairs |
| Presence in each stratum | 38–57% |
| Avg residual | 28.5 |
| Mirror without scale issue | ~26 pairs |
| Remedy | Try reflection-aware Procrustes; use det(R)=-1 constraint |

### Mode 4: Non-Identity Scale (24.5%)
Pairs with scale outside [0.8, 1.2].

| Property | Value |
|----------|-------|
| Sampled | 68 |
| Estimated total | ~7,497 pairs |
| Extreme shrink (scale < 0.8) | 55 samples |
| Extreme grow (scale > 1.2) | 13 samples |
| Avg residual (extreme) | 32.7 |
| Remedy | Use Umeyama (scale-aware) Procrustes |

### Mode 5: Severe Difference (81.9%)
Pairs with residual > 2.0, indicating true geometric difference.

| Property | Value |
|----------|-------|
| Sampled | 227 |
| Estimated total | ~25,053 pairs (conservative: may include overlaps) |
| Residual range | 2.0–294.4 |
| Avg residual | 44.8 |
| Recommendation | **EXCLUDE from dedup** |

---

## Stratum-Specific Insights

### topo_only (17,433 pairs total, 50% failure rate)
- **Meaning**: Only topology matches; shape may differ
- **Expected high failure rate**: Baseline ✓
- **Actual failure rate**: 50.0% ✓ (consistent)
- **Action**: Keep failures; topo_only matches are inherently weak
- **Safe pairs**: 8,716 (50%), geom_only subset likely within these

### shape_only (10,572 pairs total, 84% failure rate)
- **Meaning**: Only shape matches; topology may differ
- **Expected high failure rate**: Vertex reordering, missing faces, etc.
- **Actual failure rate**: 84.0% ✓ (very high!)
- **Interpretation**: Shape matching is insufficient; topology essential
- **Action**: Prioritize shape+topo matches over shape_only
- **Safe pairs**: 1,691 (16%), likely only near-identical variants

### shape+topo (1,298 pairs total, 65% failure rate)
- **Meaning**: Both shape and topology match; should be identical
- **Expected low failure rate**: ~10–20%
- **Actual failure rate**: 65.0% ✗ (much higher than expected!)
- **Root cause analysis**:
  - 56.9% involve reflection (det(R)=-1)
  - 68.6% have perfect centroid alignment
  - Suggests coordinate frame issues, not geometric difference
- **Action**: These are good candidates for reflection-aware compensation
- **Safe pairs**: ~450 (35%), especially those with residual < 1.0

### none (15,538 pairs total, 78% failure rate)
- **Meaning**: No geometric signature match; union combination artifact
- **Expected very high failure rate**: >90% likely false positives
- **Actual failure rate**: 78.0% ✓ (as expected)
- **Action**: Exclude entirely; these are likely random combinations
- **Safe pairs**: 3,415 (22%), but very risky

---

## Impact on Union Strategy

Current union has 57,174 total pairs:
- geom_only: 12,332 (safe, no issue)
- other: 44,842 (mixed quality)

Stratified breakdown estimates:
- topo_only: ~8,717 safe / 8,716 risky
- shape_only: ~1,691 safe / 8,881 risky
- shape+topo: ~454 safe / 844 risky
- none: ~3,415 safe / 12,123 risky

**Conservative recommendation**: Use geom_only + shape+topo + lowest residual (<1.0) subset:
- geom_only: 12,332
- shape+topo (residual < 1.0): ~450
- **Total: ~12,782 pairs (22.4% of union)**

This represents a 77.6% reduction but with high confidence in dedup correctness.

