---
title: "Topo Tier2 Gate Recovery — Status Snapshot"
code_reference: scripts/compute_vertex_transform.py, scripts/analyze_tier2_gate_thresholds.py
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: zhuzihou
status: in_progress
---

# Topo Tier2 Gate Recovery — Status Snapshot (2026-04-03)

Branch: `feat/topo_precheck_recovery_v1`

## Background

297 pairs in `check_reports/test0_rebuilt_dedup/topo_precheck_recovery_spike.csv` were rejected by
Gate 1 (topo precheck) in the `topo_filesize` dedup mode. This recovery plan adds a Tier2 NN-based
gate to reclaim safe pairs that fail the strict vertex-order check but pass NN Procrustes alignment.

Plan document: `docs/operations/topo_tier2_gate_recovery_plan.md`

## Phase A: Offline Analysis — COMPLETE

| Deliverable | Path | Status |
|-------------|------|--------|
| Threshold analysis script | `scripts/analyze_tier2_gate_thresholds.py` | Done |
| Analysis JSON report | `check_reports/test0_rebuilt_dedup/tier2_threshold_analysis.json` | Done |
| Threshold analysis doc | `docs/operations/tier2_gate_threshold_analysis.md` | Done |

### Key Findings

- **Total spike pairs**: 297
- **Recommended config**: `nn_bbox≤0.05, nn_mean_dist_norm≤0.02, nn_unique_ratio≥0.5, nn_close_pct≥10%`
- **Recoverable pairs**: ~77/297 (26%)
- **Per-canonical groups**: 12 unique canonicals
- **Low close_pct entries**: 9 (all from one canonical with high `unique_ratio` — safe)
- **Vertex RMSE verification (Step 4)**: NOT done — requires pxr/Isaac Sim environment

## Phase B: Code Changes — COMPLETE

All changes in working tree, not yet committed.

### B1: `_nn_procrustes_core` refactor

- Extracted core logic from `_nn_procrustes_in_normalized_space` into `_nn_procrustes_core`
  (returns 4 values: `rmse`, `close_pct`, `nn_mean_dist_norm`, `nn_unique_ratio`)
- Added `_nn_procrustes_with_stats` wrapper for Tier2 callers
- Original `_nn_procrustes_in_normalized_space` preserved as backward-compatible wrapper

### B2: Gate 1B Tier2 branch

In `_topo_same_vtx_with_nn_fallback`:
- 4 new constants: `_NN_TIER2_BBOX_CEIL`, `_NN_TIER2_MEAN_DIST_NORM_CEIL`,
  `_NN_TIER2_UNIQUE_RATIO_FLOOR`, `_NN_TIER2_CLOSE_PCT_FLOOR`
- Launch config: `nn_bbox≤0.05, nn_mean_dist_norm≤0.02, nn_unique_ratio≥0.5, nn_close_pct≥10%`
- `DEDUP_DISABLE_NN_TIER2` kill-switch env var
- `DEDUP_NN_TIER2_BBOX` env var override for bbox ceiling

### B3: `v_source` tracing

- `_last_v_source` tracking on all return paths (`baseline` / `tier1_shuffle` / `tier2_nn`)
- Integrated into `build_pair_certificate` in `scripts/c1_build_bulk_mapping_from_dedup_report.py`

### B4: Regression tests

- **17/17 pass** (verified independently)

## Pending / Blocked

| Item | Blocker | Priority |
|------|---------|----------|
| Vertex RMSE verification (Phase A Step 4) | Needs pxr/Isaac Sim env to read USD meshes | High |
| Dry-run cert probe | Blocked on vertex RMSE confirmation | High |
| Full probe on bottle category | Blocked on dry-run cert | Medium |
| Threshold tuning (if needed) | Depends on vertex RMSE + dry-run results | Low |

## Next Steps

1. **Vertex RMSE verification** — Run in Isaac Sim environment to read USD vertex data for the 77 recoverable pairs, compute actual RMSE, and confirm alignment quality
2. **Dry-run cert probe** — Once RMSE confirmed safe: `c1_autorun_categories.py --step6-mode dry_run --include-regex "^bottle$"`
3. **Review dry-run results** — Check v_source distribution, compare cert counts before/after Tier2
4. **If thresholds need adjustment** — Only change the 4 constant values in `compute_vertex_transform.py`
5. **Production run** — Remove dry_run flag and run full cert build
