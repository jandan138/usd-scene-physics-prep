---
title: "Topo Tier2 Gate Recovery: NN fallback + v_source tracing"
code_reference: scripts/compute_vertex_transform.py
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: TraeAI
status: active
---

# Topo Tier2 Gate Recovery

## Summary

Added a Tier2 relaxed NN gate for topo_filesize pairs that fail the strict Tier1 shuffle gate
(nn_close_pct > 95%) but still have acceptable NN correspondence quality. Also added v_source
tracing so certificates record which V computation path was used.

## Changes

### 1. Refactored `_nn_procrustes_in_normalized_space` (Phase B1)

Split into three functions:
- `_nn_procrustes_core()` — returns `(V_4x4, nn_close_pct, nn_unique_ratio, nn_mean_dist_norm)`
- `_nn_procrustes_in_normalized_space()` — backward-compatible wrapper, returns first 3 values
- `_nn_procrustes_with_stats()` — returns all 4 values including `nn_mean_dist_norm`

The new `nn_mean_dist_norm` metric is `float(np.mean(dists))` from the NN tree query in
bbox-normalized space.

### 2. Gate 1B Tier2 branch (Phase B2)

New constants:
```python
_NN_TIER2_BBOX_THRESHOLD = 0.05
_NN_TIER2_UNIQUE_THRESHOLD = 0.5
_NN_TIER2_MEAN_DIST_THRESHOLD = 0.02
_NN_TIER2_CLOSE_PCT_FLOOR = 10.0
```

Flow in `_topo_same_vtx_with_nn_fallback`:
1. Baseline procrustes (index-aligned)
2. If baseline bbox > 0.15, attempt NN fallback
3. **Gate 1 (Tier1)**: nn_close_pct > 95% → Gate 2 (candidate_bbox <= 0.01) → accept as `tier1_shuffle`
4. **Gate 1B (Tier2)**: if nn_close_pct >= 10%, candidate_bbox <= 0.05, nn_mean_dist_norm <= 0.02, nn_unique_ratio >= 0.5 → accept as `tier2_nn`
5. Otherwise → keep baseline

Env controls:
- `DEDUP_DISABLE_NN_TIER2=1` — kill-switch for Tier2
- `DEDUP_NN_TIER2_BBOX=<float>` — override bbox threshold

### 3. v_source tracing (Phase B3)

Module-level `_last_v_source: Dict[str, Any]` is set before each return in
`_topo_same_vtx_with_nn_fallback`. Values: `"baseline"`, `"tier1_shuffle"`, `"tier2_nn"`.

In `build_pair_certificate`:
- geom_only: `cert["v_source"] = "identity"`
- Other modes: `cert["v_source"]` read from `_last_v_source` after `compute_V_for_pair`
- For tier2_nn, additional fields: `tier2_nn_bbox`, `tier2_nn_unique_ratio`, `tier2_nn_mean_dist_norm`

## Tests

All 17 existing tests pass (no regressions). Test command:
```bash
python -m pytest tests/test_compute_vertex_transform.py -v
```
