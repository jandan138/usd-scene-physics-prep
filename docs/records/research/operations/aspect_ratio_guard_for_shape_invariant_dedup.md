---
title: "Aspect Ratio Guard for Shape-Invariant Dedup"
code_reference: scripts/compute_vertex_transform.py, scripts/rewrite_layout_asset_refs_with_compensation.py
created_at: 2026-03-24
updated_at: 2026-03-24
maintainer: claude-agent
status: active
---

# Aspect Ratio Guard for Shape-Invariant Dedup

## Problem

Shape-invariant dedup pairs with significantly different bounding box proportions
produce incorrect V matrices via `compute_V_shape_invariant()`. The bbox
normalization step maps both meshes to [0,1] unit cubes, but if one mesh is tall
and narrow while the other is short and wide, the normalization distorts the
alignment and the resulting V matrix causes displacement when applied.

## Solution

Added an **aspect ratio guard** that rejects pairs whose sorted bbox extents
differ by more than a configurable threshold (default: 2.0x).

### Changes

1. **`compute_V_shape_invariant()`** (`scripts/compute_vertex_transform.py`):
   - New parameter: `aspect_ratio_threshold: float = 2.0`
   - Guard checks sorted bbox extents before `_normalize_to_unit_bbox()` calls
   - Raises `RuntimeError("aspect ratio mismatch: ...")` if any dimension
     ratio exceeds threshold or one dimension is degenerate while other is not

2. **`_get_V_cached()`** (`scripts/rewrite_layout_asset_refs_with_compensation.py`):
   - Catches `RuntimeError` with "aspect ratio mismatch" specifically
   - Returns `(None, "aspect_ratio_rejected")` instead of falling back to Identity
   - Other exceptions still fall back to Identity V

3. **Pre-filter in `rewrite_layout()`** (`scripts/rewrite_layout_asset_refs_with_compensation.py`):
   - Before the traversal loop, pre-computes V for all mapping pairs
   - Removes aspect-ratio-rejected pairs from `mapping_by_old`
   - This ensures rejected pairs never get their references rewritten
   - Logs rejected pairs in the changes list with kind="aspect_ratio_rejected"

### Why pre-filter (not post-filter)?

The reference rewrite happens at line 579 (SetMetadata), before V computation
at line 608. If we only caught the rejection in the compensation block, the
reference would already be rewritten to canonical but with no valid V to
compensate the transform — resulting in wrong shape at the same position.
Pre-filtering removes rejected pairs from the mapping entirely.

## Ground Verification (2026-03-24)

Tested on 16 displaced ground prims from the V5 bbox verify report.

Results with threshold=2.0:
- **2 REJECTED** (would be caught): `2d9dfa0f...` (ratio=2.09, disp=4.77) and
  `fcf5811c...` (ratio=2.01, disp=2.50)
- **14 PASSED** (not caught): most have max_ratio=2.00 exactly

Key observation: many ground tile pairs have exactly 2:1 Y-extent ratio (24cm
vs 12cm tiles). The `> 2.0` threshold misses these by design. The displacement
in these 14 pairs ranges from 1.33 to 5.09, suggesting the root cause is not
aspect ratio mismatch but rather the plane-normal or scale compensation path.

## Threshold Discussion

- `threshold=2.0` is conservative — catches only grossly mismatched pairs
- Lowering to `1.5` would catch more (ratio=2.00 pairs with 2:1 scaling)
- The 2:1 ground tile pairs may be better addressed by improving the
  flat-asset scale compensation path rather than rejecting them
