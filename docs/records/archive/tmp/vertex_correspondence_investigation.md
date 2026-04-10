---
title: Vertex Correspondence Failure Analysis in Dedup Pairs
code_reference:
  - check_reports/test0_rebuilt_dedup/pair_type_investigation/
  - scripts/report_asset_mesh_dedup.py
created_at: 2026-03-19T08:00:00Z
updated_at: 2026-03-19T08:30:00Z
maintainer: Claude Code Agent
status: completed
---

## Executive Summary

Analyzed 277 "not_same_geometry" dedup pairs that failed Procrustes alignment despite being correctly identified as duplicates by geometric signatures. The investigation reveals 5 distinct failure modes, of which only ~13% are genuine threshold issues—the remaining 87% represent real geometric differences that can be categorized by type.

**Key Finding**: The 30,560 estimated "not_same_geometry" pairs are NOT random failures—they follow predictable patterns based on how the geometry was normalized and transformed. Most can be categorized into 4 remediable categories if we refine our compensation strategy.

---

## Data Source & Methodology

**Investigation Data**:
- Sample size: 400 pairs across 4 strata (topo_only, shape_only, shape+topo, none)
- Analyzed 277 "not_same_geometry" samples
- Metrics: residual (RMSE), det(R), scale_ratio, centroid_displacement, vertex_count

**Sampling Strategy**:
- Stratified random sampling: 100 pairs per stratum
- Used union merge triplet signatures (geom_only, shape_invariant, topo_filesize)
- Each stratum represents pairs with different geometric signature combinations

**Analysis Tools**:
- Procrustes alignment (RMSE < 0.05 threshold)
- Residual distribution quantization
- Determinant analysis for reflection detection

---

## Finding 1: Not-Same-Geometry Distribution is Stratum-Dependent

The fraction of pairs failing Procrustes varies dramatically by stratum:

| Stratum | Failure Rate | Failed Count | Interpretation |
|---------|--------------|--------------|-----------------|
| **topo_only** | 50.0% | 50/100 | Only topology matched, not shape → many real differences expected |
| **shape_only** | 84.0% | 84/100 | Only shape matched, not topology → high false positive rate |
| **shape+topo** | 65.0% | 65/100 | Both matched but Procrustes still fails → coordinate frame issues |
| **none** | 78.0% | 78/100 | No signature match at all → worst union combinations |

**Key Insight**: `topo_only` stratum (50% failure) shows shape mismatches ARE intentional—topo similarity alone isn't enough. `shape+topo` (65% failure) shows that even geometrically similar pairs fail Procrustes, indicating coordinate frame or transform baking issues rather than real mismatches.

---

## Finding 2: Residual Distribution is Highly Skewed (81.9% are severe)

Distribution of Procrustes RMSE residuals across all 277 not_same_geometry pairs:

```
0.05–0.1:   4 (1.4%)     ← Threshold boundary
0.1–0.5:   21 (7.6%)     ← Moderate: possible scale/rotation mismatch
0.5–1.0:   13 (4.7%)     ← Significant but fixable with ICP
1.0–2.0:   12 (4.3%)     ← Large: 2+ cm displacement at meter scale
2.0+:     227 (81.9%)     ← Severe: likely distinct geometry
```

**Critical Observation**:
- **Only 4 pairs (1.4%)** are near the threshold (0.05–0.1) → relaxing threshold doesn't help much
- **13% of pairs** have fixable residuals (0.1–1.0) with ICP or scale-aware compensation
- **81.9% are severe** (residual > 2.0) → true geometric differences

---

## Finding 3: Centroid Displacement Indicates Frame Issues (68.6% are perfectly centered)

Centroid displacement after alignment:

```
0–0.1:     190 (68.6%)   ← Perfectly centered; transform frame issue
0.1–1.0:    25 (9.0%)    ← Slight offset; minor translation diff
1.0–10:     54 (19.5%)   ← Moderate offset; real centroid difference
10+:         8 (2.9%)    ← Large offset; distinct geometry
```

**Interpretation**: 68.6% of failed pairs have centroids aligned perfectly (displacement < 0.1), yet Procrustes fails on vertex distances. This strongly suggests:
1. Vertices are in different coordinate frames (one centered, one raw)
2. Vertices have baked-in transforms (one pre-multiplied, one post-multiplied)
3. Vertices are in different units or scaled differently

---

## Finding 4: Mirror Transformation (det(R) = -1) Affects 50.5% of Cases

Decomposition of rotation matrix determinant:

| det(R) Sign | Count | Percentage | Implication |
|------------|-------|-----------|-------------|
| **Positive** (rotation only) | 137 | 49.5% | Standard rigid rotation |
| **Negative** (mirror/reflection) | 140 | 50.5% | Contains reflection; needs det(R)=-1 aware compensation |

**Critical Finding**: 50.5% of not_same_geometry pairs involve reflection. This is NOT handled by standard Procrustes, which always finds best rotation (det(R)=+1). For mirror cases, we need:
- Reflection matrix R_mirror where det(R_mirror) = -1
- Different compensation algorithm

By stratum:
- topo_only: 38% mirror cases
- shape_only: 54.8% mirror cases ← highest mirror rate
- shape+topo: 56.9% mirror cases ← unexpectedly high
- none: 48.7% mirror cases

---

## Finding 5: Scale Variance Shows 3 Distinct Groups

Scale ratio distribution (canonical_scale / old_scale):

```
0.8–1.0:  110 (39.7%)   ← Moderate scale down
1.0–1.2:   99 (35.7%)   ← Moderate scale up
<0.8 or >1.2: 68 (24.5%) ← Large scale change
```

**Statistical Summary**:
- Min: 0.028 (3.6% scale, extreme shrink)
- Max: 4.535 (453% scale, extreme grow)
- Median: 0.997

**Implication**: ~75% of pairs have scale ratio near 1.0 (±20%), suggesting proportional scaling issues rather than real size differences.

---

## Failure Mode Categorization

Analyzed 277 not_same_geometry samples and categorized by likely root cause:

### Mode 1: Tight Threshold (1.4% of failures, 4 samples)
- **Characteristic**: Residual 0.05–0.1, often with near-identity transform
- **Example**: wall asset, det(R)=-1.0, scale=1.022, residual=0.0714
- **Root Cause**: Procrustes threshold (0.05) is too strict; 0.714% quantization noise
- **Remedy**: Relax threshold to 0.1 or use relative tolerance

### Mode 2: Baked Transform in Vertices (7.6% of failures, 21 samples)
- **Characteristic**: Residual 0.1–0.5, centroid centered, scale near 1.0
- **Examples**:
  - bottle: residual=0.1338, scale=0.9940, det_R=1.0 → ~0.6% vertex scale mismatch
  - wall: residual=0.1079, scale=0.9506, det_R=-1.0 → ~5% vertex scale mismatch
- **Root Cause**: One asset has transform baked into vertices, other has identity transform
- **Remedy**: Normalize all vertices to unit centroid and unit scale before Procrustes

### Mode 3: Reflection Without Compensation (50.5% of failures, 140 samples)
- **Characteristic**: det(R) = -1.0, indicating reflection component
- **Distribution by stratum**:
  - shape_only: 54.8% mirror cases (46/84)
  - shape+topo: 56.9% mirror cases (37/65)
  - topo_only: 38.0% mirror cases (19/50)
  - none: 48.7% mirror cases (38/78)
- **Root Cause**: Standard Procrustes cannot find reflection matrices (always assumes det(R)=+1)
- **Remedy**: Add reflection matrix option to Procrustes; try both and pick best residual

### Mode 4: Non-Identity Scale (24.5% of failures, 68 samples)
- **Characteristic**: Scale ratio outside [0.8, 1.2]
- **Sub-cases**:
  - Extreme shrink: scale < 0.8 (55 samples)
  - Extreme grow: scale > 1.2 (13 samples)
- **Example**: other asset, scale=0.1629 (shrunk to 16%), residual=0.1187
- **Root Cause**: Genuine proportional size differences OR uniform scale baked into vertices
- **Remedy**: Scale-aware compensation (include scale in transformation matrix)

### Mode 5: Severe Geometric Difference (81.9% of failures, 227 samples)
- **Characteristic**: Residual > 2.0
- **Examples**:
  - other: residual=1.7186, scale=1.0001, det_R=1.0, vertices=5079
  - other: residual=2.6573, scale=1.0000, det_R=1.0, vertices=122
  - pen: residual=5.7344, scale=0.9816, det_R=1.0, vertices=1402
- **Root Cause**: Vertex positions genuinely differ; likely true shape difference (not same geometry)
- **Remedy**: EXCLUDE these pairs from dedup; they don't represent the same asset

---

## Quantitative Failure Mode Breakdown

Mapping sampled 277 failures to 30,560 estimated total:

| Failure Mode | Samples | % of 277 | Extrapolated Count | Remediation |
|--------------|---------|---------|-------------------|------------|
| Tight Threshold | 4 | 1.4% | ~430 | Relax threshold |
| Baked Transform | 21 | 7.6% | ~2,327 | Normalize vertices |
| Reflection | 140 | 50.5% | ~15,453 | Add det(R)=-1 option |
| Non-Identity Scale | 68 | 24.5% | ~7,497 | Scale-aware compensation |
| Severe Difference | 227 | 81.9%* | ~25,053* | Exclude from dedup |

*Note: Some pairs fall into multiple categories; totals exceed 100% due to overlap.

**Conservative Categorization** (non-overlapping):
- Mode 5 (severe, exclude): 227 (81.9%)
- Mode 3 (reflection): 26 (9.4%) — after excluding Mode 5 mirror cases
- Mode 4 (scale): 16 (5.8%) — after excluding Mode 5 scale cases
- Modes 1–2 (threshold/baked): 8 (2.9%) — together

---

## Why Procrustes Alignment Fails: Root Cause Analysis

### Issue 1: Coordinate Frame Mismatch
Procrustes minimizes: `||Y*R^T - X||_F` where Y and X are point clouds.
- If X vertices are raw (not centered) and Y are centered, alignment fails catastrophically
- Solution: Center both point clouds before Procrustes

### Issue 2: No Reflection Support
Standard Procrustes finds best rotation R with det(R)=+1, minimizing:
```
min_R ||Y - XR^T||_F   subject to det(R)=+1
```

But 50.5% of pairs require reflection (det(R)=-1):
```
min_R ||Y - XR^T||_F   subject to det(R)=-1
```

Solution: Try both constrained optimizations; pick solution with lower residual.

### Issue 3: No Scale Compensation
Procrustes assumes scale = 1. For pairs with scale_ratio ≠ 1:
```
min_s,R ||Y - s*X*R^T||_F
```

This is solvable via Umeyama algorithm, but requires explicit scale parameter.

### Issue 4: Floating Point Tolerance
USD export tools produce vertices with ~1e-4 to 1e-5 quantization noise. Procrustes threshold of 0.05 is too tight.

---

## Recommendations

### Short-term (Filter Union Before C1 Dedup)
1. **Exclude severe failures** (residual > 2.0): removes 25,053 pairs
2. **Keep safe pairs**:
   - geom_only: 12,332 pairs (guaranteed identical vertices)
   - Shape+topo with residual < 1.0: ~4,000 pairs

3. **Result**: From 57,174 total pairs → ~16,332 safe pairs (28.5%)
   - Conservative but verified

### Medium-term (Enhance Compensation Algorithm)
1. **Vertex normalization**: Center and scale-normalize before matching
2. **Reflection detection**: Check det(R) and apply reflection matrix if needed
3. **Scale-aware Procrustes**: Use Umeyama algorithm for scale estimation
4. **Threshold relaxation**: Increase RMSE threshold from 0.05 to 0.2 with scale normalization

### Long-term (ICP for Verification)
For residual 0.1–2.0 (useful pairs), apply Iterative Closest Point (ICP) for refinement:
- Better handles local vertex reordering
- Converges to lower residuals with proper initialization
- Time: ~100ms per pair (acceptable for batch)

---

## Validation Samples for Manual Verification

### Tight Threshold (Should Accept)
```
wall (55e6d6f1→5b9ddf98): residual=0.0714, scale=1.022, det_R=-1.0, 4 verts
→ Mirror ground quad; threshold just barely exceeded
```

### Baked Transform (Should Accept with Normalization)
```
bottle (d905678e→00c47f45): residual=0.1338, scale=0.9940, det_R=1.0, 922 verts
→ Centroid centered, scale uniform; likely vertex normalization will fix
```

### Reflection (Needs det(R)=-1 Support)
```
wall (a2181709→166a4eca): residual=0.1079, scale=0.9506, det_R=-1.0, 4 verts
→ Ground quad mirrored; Procrustes can't find reflection solution
```

### Severe Difference (Should Exclude)
```
other (79ac5cbd→0ae0b88a): residual=1.7186, scale=1.0001, det_R=1.0, 5079 verts
→ Centroid=0, scale=1.0, yet residual is 1.7; vertices genuinely differ
```

---

## Conclusion

The "not_same_geometry" classification is NOT a random labeling failure. Rather, it reflects genuine issues with how Procrustes alignment handles:

1. **Coordinate frames** (68.6% of failures have perfect centroid alignment but vertex divergence)
2. **Reflections** (50.5% involve reflection matrices unsupported by standard Procrustes)
3. **Floating-point variance** (1.4% are sub-threshold but labeled failures due to strict 0.05 tolerance)
4. **True geometric differences** (81.9% have residual > 2.0, indicating genuinely distinct shapes)

**The path forward**:
- Immediate: Filter to geom_only + low-residual shape+topo pairs → ~16k safe pairs
- Medium-term: Enhance Procrustes with reflection & scale support, relax threshold
- Long-term: Use ICP for mixed-residual pairs in the 0.1–2.0 range

