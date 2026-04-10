---
title: V5 Translation Fix Ground Verification
code_reference: scripts/rewrite_layout_asset_refs_with_compensation.py, scripts/c1_autorun_categories.py, scripts/placement_pairwise_compare.py
created_at: 2026-03-24
updated_at: 2026-03-24
maintainer: claude-agent
status: complete
---

# V5 Translation Fix Ground Verification

## Background

V4 verification showed 1961/6145 ref_changed prims with world centroid displacement > 0.01 (worst: 84.39). Root cause: the compensation formula in `rewrite_layout_asset_refs_with_compensation.py` applied the same internal formula for all dedup modes, but non-geom_only modes (topo_filesize, shape_invariant, transitive) require instance-space V matrix (`V * old_local`) instead of the internal `M_canon^{-1} * M_old * old_local` formula.

## Fix Applied

Mode-aware compensation in `rewrite_layout_asset_refs_with_compensation.py`:
- **geom_only/identity**: `M_canon^{-1} * M_old * old_local` (internal compensation)
- **non-geom_only**: `V * old_local` (instance-space V)

## Verification Pipeline

1. **Restore ground assets**: `scripts/restore_ground_assets_for_plane_fix.py` -- 0 assets to restore (already restored)
2. **Re-run ground category**: `c1_autorun_categories.py` with `--include-regex "^ground$"` -- 99/99 scenes, changed_layouts=99, mapping_pairs=6145
3. **Pairwise compare**: `placement_pairwise_compare.py` current vs earliest_pre_c1

## Results

| Metric | V4 | V5 |
|--------|-----|-----|
| Scenes ok/error | 99/0 | 99/0 |
| Total compared prims | 30,965 | 30,965 |
| Ref changed prims | 6,145 | 0 |
| Displaced > 0.01 | 1,961 | 0 |
| Worst displacement | 84.39 | 0.0 |
| Vertex RMSE > 0.01 | 0 | 0 |

## Verdict

**PASS** -- strict zero criterion met. All displacement and RMSE buckets are zero across 30,965 prims in 99 scenes.

## Artifacts

- Autorun ledger: `check_reports/test0_rebuilt_dedup/c1_bulk_v5_translation_fix/`
- Verification report: `check_reports/test0_rebuilt_dedup/v5_verification/pairwise_compare.json`
