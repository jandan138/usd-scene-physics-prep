---
title: "Shape-Invariant Dedup Mode Implementation"
code_reference: "scripts/report_asset_mesh_dedup.py"
created_at: "2026-03-11"
updated_at: "2026-03-11"
maintainer: "zhuzihou"
status: "implemented"
---

# Shape-Invariant Dedup Mode Implementation

## Overview

A new `--mode shape_invariant` option was added to `scripts/report_asset_mesh_dedup.py` that detects duplicate assets invariant to vertex ordering, face winding, uniform scale, and translation. This complements the existing `geom_only` mode by catching scale-variant duplicates that hash-based matching misses.

## What Was Implemented

### New Functions

| Function | Purpose | Location |
|----------|---------|----------|
| `_normalize_to_unit_bbox(points)` | AABB normalization to [0,1] range, preserving aspect ratio | `report_asset_mesh_dedup.py` |
| `_compute_shape_descriptor(points, face_count, eps)` | Shape signature with vertex count, face count, and quantized aspect ratio hash | `report_asset_mesh_dedup.py` |
| `_hausdorff_distance(pts_a, pts_b)` | Bidirectional Hausdorff metric with fast path (lexsort) and slow path (KDTree) | `report_asset_mesh_dedup.py` |
| `_shape_invariant_merge(records, tolerance)` | Union-find grouping with Hausdorff distance threshold | `report_asset_mesh_dedup.py` |

### New Dataclass

- `ShapeDescriptor`: frozen dataclass with `vertex_count`, `face_count`, `aspect_ratio_hash` fields

### New CLI Parameters

```
--mode {all,shape_invariant}
    Default "all" generates only the existing 3 reports.
    "shape_invariant" additionally generates a 4th report.

--hausdorff-threshold FLOAT
    Hausdorff distance threshold for shape_invariant mode (unit-normalized).
    Default 0.05.
```

### Output

```
<out-dir>/<dataset>_asset_mesh_dedup_shape_invariant.json
```

Same JSON format as existing reports, with `meta.mode = "shape_invariant"` and `meta.hausdorff_threshold` fields.

## Test Results

### Unit Tests: 30/30 PASS

File: `tests/test_shape_invariant.py`

All unit tests covering normalization, shape descriptors, Hausdorff distance computation, and the ShapeDescriptor dataclass pass.

### Regression Test: PASS

All 3 existing modes (`geom_only`, `scale_only`, `full_matrix`) produce identical output compared to baseline. No `shape_invariant` report is generated with default `--mode=all`, confirming backward compatibility.

### Code Review: APPROVED

Two minor issues documented:
1. No negative threshold validation on `--hausdorff-threshold` (low risk -- negative values simply produce no matches)
2. O(n^2) pairwise comparison within pre-filter groups (documented as acceptable given pre-filter reduces group sizes)

### Cross-Category Test Results

| Category | Total | Baseline (geom_only) | shape_invariant | Delta |
|----------|-------|----------------------|-----------------|-------|
| pen | 414 | 7 removable (1.7%) | 21 removable (5.1%) | +14 (+3.4%) |
| plate | 426 | 203 removable (47.7%) | 232 removable (54.5%) | +29 (+6.8%) |
| cup | 549 | 37 removable (6.7%) | 104 removable (18.9%) | +67 (+12.2%) |

## Key Finding: Complementary Mode, Not Superset

The shape_invariant mode is **complementary** to geom_only, not a superset. This was the most important discovery during validation.

### Bottle Category Detailed Analysis

| Metric | geom_only | shape_invariant | Union |
|--------|-----------|-----------------|-------|
| Groups | 175 | 244 | -- |
| Removable | 761 | 621 | 812 |
| Rate | 44.8% | 36.6% | 47.8% |

- **shape_invariant finds 56 new assets** that geom_only misses (scale variants with different absolute coordinates)
- **shape_invariant loses 127 assets** that geom_only catches (Hausdorff distance is stricter than hash matching at bucket boundaries)
- **Union of both modes** yields the best coverage: 812 removable assets (47.8%)

### Triplet Validation

The known bottle triplet (`7861bd`, `79088d`, `79090f`) was correctly NOT grouped by shape_invariant mode, confirming these geometrically distinct assets are properly handled.

## Recommendations

1. **Use union strategy**: Run both `geom_only` (with `--merge-tolerance`) and `shape_invariant` modes, then merge their results for maximum dedup coverage
2. **Default threshold**: `--hausdorff-threshold 0.05` works well for the GRScenes dataset; adjust lower for stricter matching
3. **DLC integration**: The mode can be integrated into the existing `scripts/dlc/dedup_by_category.py` chunk dispatch framework for distributed execution

## Known Limitations

1. No negative threshold validation on `--hausdorff-threshold` CLI parameter
2. O(n^2) pairwise comparison within pre-filter groups (mitigated by pre-filter reducing group sizes by ~90-95%)
3. Only handles uniform scale invariance; non-uniform (anisotropic) scale variants are not detected
4. Vertex count must match exactly between compared meshes (by design, for meaningful Hausdorff comparison)

## Algorithm Summary

1. **Normalize** each mesh's vertices to unit bounding box (preserving aspect ratio)
2. **Pre-filter** by `(vertex_count, aspect_ratio_hash)` to reduce pairwise comparisons
3. **Hausdorff distance** between normalized vertex sets (lexsort-based fast path for same vertex count)
4. **Union-Find** grouping when distance < threshold
5. Output report in standard dedup JSON format
