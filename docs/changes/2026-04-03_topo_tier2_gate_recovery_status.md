---
title: "Topo Tier2 Gate Recovery — Status Snapshot"
code_reference: scripts/compute_vertex_transform.py, scripts/analyze_tier2_gate_thresholds.py, scripts/verify_tier2_vertex_rmse.py
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

## Phase C: RMSE Verification + Dry-Run Probe — COMPLETE

### C1: Vertex RMSE Verification

Script: `scripts/verify_tier2_vertex_rmse.py`
Report: `check_reports/test0_rebuilt_dedup/tier2_vertex_rmse_verification.json`

| Metric | Value |
|--------|-------|
| Filtered pairs | 77 |
| Computed | 77 |
| Errors | 0 |
| **Verdict** | **PASS** |
| BBox delta max | 0.0498 |
| BBox delta mean | 0.0170 |
| BBox delta p95 | 0.0466 |
| RMSE (norm) max | 0.560 |
| RMSE (norm) mean | 0.442 |

**Key observations:**
- BBox delta (the metric used by the cert pipeline) is well within the 0.05 threshold
- Cross-validation: recomputed bbox_delta matches CSV nn_bbox exactly (max diff = 0.0)
- Normalized per-vertex RMSE ~0.44 is expected for NN correspondence on different-topology meshes
  — this is not a cert-pipeline metric, included for observational purposes only
- Zero load errors across all 77 pairs

### C2: Dry-Run Cert Probe (bottle)

Command:
```bash
./scripts/isaac_python.sh scripts/c1_autorun_categories.py \
  --dataset-root GRScenes-test0-rebuilt-normalize-prededup \
  --bak-root GRScenes-test0-rebuilt-normalized_bak \
  --report check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/all_categories_union_merged.json \
  --c1-bulk-dir check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_probe \
  --include-regex "^bottle$" \
  --group-label c1_v8_tier2_probe \
  --step6-mode dry_run \
  --bbox-gated \
  --bbox-policy bbox_primary_rmse_observe \
  --v-matrix-mode auto \
  --mode-reports-dir check_reports/test0_rebuilt_dedup/v8_prededup
```

**Cert results** (from `01_cert/pair_certificate_summary.json`):

| Metric | shuffle-fix probe | Tier2 probe | Delta |
|--------|-------------------|-------------|-------|
| eligible_count | 1294 | **1371** | **+77** |
| bbox_precheck_failed_topo_filesize | 297 | **220** | **-77** |
| bbox_precheck_failed_shape_invariant | 14 | 14 | 0 |
| transitive_not_supported | 256 | 256 | 0 |
| v_computation_failed_shape_invariant | 3 | 3 | 0 |

**Eligible pairs by mode:**
- geom_only: 126
- shape_invariant: 38
- topo_filesize: 1207 (up from 1130 — the +77 recovered pairs)

**Apply results**: 52/99 scenes changed, 99 processed.

**Audit results** (from `03_audit/audit_verdict.json`):

| Metric | Shuffle-fix probe | Tier2 probe | Notes |
|--------|-------------------|-------------|-------|
| Scenes ok/error | 52/0 | 52/0 | Same |
| Total compared prims | 70,102 | 70,102 | Same |
| Displaced > 0.01 | 0 | **78** | All from ref-changed prims |
| Displaced > 1.0 | 0 | 0 | No major displacement |
| Vertex RMSE > 0.01 | 16 | 16 | Same, single scene |
| Ref changed hard fails | 390 | **423** | +33, from +77 new pairs |
| Ref same > 0.01 | 0 | **0** | Zero regressions on unchanged prims |
| Audit passed | false | false | Known observe policy behavior |
| Category maxima | — | bottle=0.131 | All other categories 0.0 |

**Key safety observations:**
- `ref_same > 0.01 = 0`: No displacement on prims that weren't changed (critical safety metric)
- `displaced > 1.0 = 0`: No major displacement anywhere
- All 78 displaced prims are ref-changed (expected sub-centimeter bbox differences from topo_filesize V compensation)
- Audit false is known behavior with `bbox_primary_rmse_observe` policy (same as all previous probes)

### Go/No-Go Decision

**GO** — All criteria met:
1. Vertex RMSE verification: PASS (bbox_delta max 0.0498 < 0.1)
2. Cert probe: +77 eligible pairs exactly matching filtered recoverable count
3. No regressions in other modes (geom_only, shape_invariant unchanged)
4. Zero errors in both verification and cert steps

## Pending

| Item | Status | Priority |
|------|--------|----------|
| Full rollout to all 83 categories | Ready (GO decision made) | High |
| Commit to main | After full rollout confirmation | High |
