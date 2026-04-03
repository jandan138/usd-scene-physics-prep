---
title: "Phase 2 Spike: Topo NN Correspondence Recovery — Implementation Notes"
code_reference: scripts/debug_topo_nn_correspondence_spike.py
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: zhuzihou
status: active
---

# Phase 2 Spike: Topo NN Correspondence Recovery — Implementation

## Script

`scripts/debug_topo_nn_correspondence_spike.py`

## Design Decisions

1. **Import strategy**: Tries `from scripts.compute_vertex_transform import ...` first.
   If pxr dependency blocks the import, falls back to local copies of:
   - `_normalize_to_unit_bbox` (L346-357 of compute_vertex_transform.py)
   - `_try_single_procrustes` (L198-274)
   - `procrustes_full` (L277-339)
   - `extract_instance_space_vertices` (L138-191)

   The local copies are verbatim from the source (no modifications).

2. **Candidate A algorithm** follows the plan exactly:
   - bbox normalize both point sets via `_normalize_to_unit_bbox`
   - Build `cKDTree` on normalized old points
   - Query NN for each normalized canon point -> reorder old points
   - Run 4-combo Procrustes in normalized space
   - Denormalize using the same formula as `compute_V_shape_invariant`

3. **Sampling**: Reads cert JSONL, filters by mode and `bbox_delta.max_abs` into 4 buckets
   (poor >0.5, gap 0.15-0.5, good <=0.01, geom_only). Uses `np.random.RandomState(42)` for
   reproducible sampling.

4. **CSV columns**: Match the plan specification exactly:
   `old_usd, canon_usd, mode, bucket, n_verts, baseline_rmse, baseline_bbox_max_abs,
   candidate_rmse, candidate_bbox_max_abs, nn_mean_dist, nn_close_pct, nn_unique_ratio,
   is_shuffle_heuristic, wall_time_s`

## CLI

```bash
python3 scripts/debug_topo_nn_correspondence_spike.py \
  --cert-jsonl check_reports/test0_rebuilt_dedup/c1_bulk_v8_multimode/bottle_bbox_primary_rmse_observe_v1/01_cert/pair_certificates.jsonl \
  --dataset-root GRScenes-test0-rebuilt-normalize-prededup \
  --out-csv check_reports/test0_rebuilt_dedup/topo_nn_spike_results.csv
```

## Verification

- `python3 scripts/debug_topo_nn_correspondence_spike.py --help` — runs successfully
- AST parse clean, no syntax errors
- Requires: numpy, scipy (for cKDTree), pxr (for vertex extraction)
