---
title: "BBox-Gated Multi-Mode Dedup: Change Plan"
created_at: "2026-03-31"
updated_at: "2026-03-31"
maintainer: "Claude Code"
status: "planned"
code_reference:
  - "scripts/report_asset_mesh_dedup.py"
  - "scripts/compute_vertex_transform.py"
  - "scripts/rewrite_layout_asset_refs_with_compensation.py"
  - "scripts/placement_pairwise_compare.py"
---

# Background

v6 A/B evaluation revealed two bugs and a strategy shift. Policy B correctly
blocked 2 categories (bottle, other) due to RMSE hard failures caused by
tolerance_merge groups contaminating the geom_only report. Investigation
documented in `docs/operations/grscenes_test0_bbox_gated_v6_rmse_bbox_investigation.md`.

## Strategy Shift

**Old**: Only geom_only mode enabled + RMSE as blocking criterion.

**New**: All three dedup modes (geom_only, topo_filesize, shape_invariant)
enabled + bbox-only gating. RMSE is recorded but not blocking.

Rationale: any pair that matches any of the three dedup modes AND passes the
bbox gate should be eligible for scene replacement. The bbox gate already
catches real geometry problems; RMSE adds noise when mesh paths differ between
old and new assets.

# Bugs Found

## Bug 1: tolerance_merge Contaminates geom_only Report

**File**: `scripts/report_asset_mesh_dedup.py`

- **Line 1046**: `_make_duplicates_map(records, "asset_geom_sig_hex")` creates
  true geom_only groups (identical geometry hash)
- **Line 1053**: `_tolerance_merge(records, dups, merge_tolerance)` mixes
  tolerance_merge groups (same topology, different geometry, vertex distance
  within threshold) into the same dict
- **Line 1078**: The mixed dict is written to the geom_only report unfiltered

Contamination rate: bottle 66%, other 16%.

These tolerance_merge pairs have different `asset_geom_sig` hashes and are NOT
true geom_only duplicates. Their inclusion caused cert to label them as
`mode: geom_only, rmse: 0.0` (false), then rewrite applied wrong V matrices.

## Bug 2: build_mode_index Only Indexes paths[0]

**File**: `scripts/compute_vertex_transform.py`, lines 636-641

For a group `[A, B, C]`, only creates index entries:
`(A,B)`, `(B,A)`, `(A,C)`, `(C,A)`.

Missing: `(B,C)`, `(C,B)`.

When cert selects a canonical that isn't paths[0], lookup fails and falls back
to transitive mode, which gets rejected in bbox-gated path.

# Change Plan (6 Changes)

## Change 1: Clean geom_only Report

**File**: `scripts/report_asset_mesh_dedup.py`, lines 1051-1053

**Current**:
```python
if merge_tolerance > 0 and mode == "geom_only" and mode != "shape_invariant":
    hash_only_count = len(dups)
    dups = _tolerance_merge(records, dups, merge_tolerance)
    tolerance_merged_count = len(dups) - hash_only_count
```

**Fix**: Do NOT overwrite `dups`. Keep geom_only output clean. Either:
- Remove the `_tolerance_merge` call from geom_only mode entirely, OR
- Run tolerance_merge separately and write to a dedicated `tolerance_merge`
  report file

Reports must be independent: geom_only contains only hash-identical pairs,
topo_filesize contains only topo+filesize matches, etc.

## Change 2: Expand BBOX_GATED_ALLOWED_MODES

**File**: `scripts/compute_vertex_transform.py`, line 70

**Current**: `BBOX_GATED_ALLOWED_MODES = ("geom_only",)`

**Fix**: `BBOX_GATED_ALLOWED_MODES = ("geom_only", "topo_filesize", "shape_invariant")`

## Change 3: Remove Hardcoded Cert Gate

**File**: `scripts/compute_vertex_transform.py`, lines 853-856

**Current**:
```python
if mode != "geom_only":
    cert["reject_reason"] = f"unsupported_certificate_mode_{mode}"
    cert["rmse_unavailable_reason"] = f"unsupported_certificate_mode_{mode}"
    return cert
```

This fires AFTER the `allowed_modes` check at line 848, making the allowlist
ineffective. Remove or replace with `if mode not in allowed_modes`.

## Change 4: Expand Apply Stage Whitelist

**File**: `scripts/rewrite_layout_asset_refs_with_compensation.py`, line 793

**Current**: `if dedup_mode not in ("geom_only", "identity"):`

**Fix**: `if dedup_mode not in ("geom_only", "identity", "topo_filesize", "shape_invariant"):`

## Change 5: Add Mode-Aware Branching to Bbox-Gated Path

**File**: `scripts/rewrite_layout_asset_refs_with_compensation.py`, lines 802-804

**Current** (hardcoded internal formula only):
```python
old_internal = _get_internal_cached(old_abs)
canonical_internal = _get_internal_cached(new_abs)
new_local = canonical_internal.GetInverse() * old_internal * old_local
```

**Fix** (mirror the legacy path at lines 868-873):
```python
if dedup_mode in ("geom_only", "identity"):
    old_internal = _get_internal_cached(old_abs)
    canonical_internal = _get_internal_cached(new_abs)
    new_local = canonical_internal.GetInverse() * old_internal * old_local
else:
    V, _ = _get_V_cached(old_abs, new_abs)
    new_local = V * old_local
```

Without this change, topo_filesize/shape_invariant pairs would pass the
whitelist but use the wrong compensation formula (internal-only instead of
instance-space V), producing incorrect transforms.

## Change 6: Fix build_mode_index to Index All Pairs

**File**: `scripts/compute_vertex_transform.py`, lines 636-641

**Current**: Only indexes `(paths[0], paths[i])` for i > 0.

**Fix**: Index all `(paths[i], paths[j])` combinations for i != j within each
group. This ensures any canonical selection finds a direct match instead of
falling back to transitive.

# NOT Changing (Deliberate)

## Transitive Mode

Not enabling transitive in bbox-gated path. Reasons:
- Requires `group_members` parameter not available in bbox-gated path
- BFS accumulated V matrices have compounding error risk
- Most current transitive fallbacks are caused by Bug 2 (incomplete index);
  fixing Bug 2 should drastically reduce transitive hits

## Audit Policy

Using `bbox_primary_rmse_observe` (already bbox-only semantics — RMSE recorded
but not blocking). No need for a new policy name.

# Verification Plan

After implementing changes:
1. Regenerate geom_only reports for bottle and other — verify 0 tolerance_merge
   groups
2. Run bbox-gated pipeline on bottle and other with expanded modes — verify
   mode_not_enabled rejections drop to near 0
3. Run pairwise compare audit — verify bbox pass rate and RMSE distribution
4. Compare total mapping pairs vs v6 to quantify yield improvement

# Risk Assessment

| Risk | Severity | Mitigation |
|------|----------|-----------|
| Wrong V formula for new modes | HIGH | Change 5 adds mode-aware branching |
| Stale reports with tolerance_merge | MEDIUM | Regenerate after Change 1 |
| Transitive fallback rejected | LOW | Bug 2 fix reduces transitive hits |
| Shape_invariant V quality | LOW | Bbox gate catches bad transforms |

# Related Documents

- `docs/operations/grscenes_test0_bbox_gated_v6_rmse_bbox_investigation.md`
- `docs/operations/grscenes_test0_bbox_gated_ab_eval_v6_status.md`
- `docs/operations/grscenes_test0_bbox_gated_dedup_status_20260327.md`
- `docs/operations/v_compensation_translation_scaling_bug.md`
