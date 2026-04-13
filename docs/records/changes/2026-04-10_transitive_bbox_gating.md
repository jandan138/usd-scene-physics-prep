---
title: "Transitive BBox Gating"
code_reference:
  - scripts/compute_vertex_transform.py
  - scripts/c1_build_bulk_mapping_from_dedup_report.py
  - scripts/placement_pairwise_compare.py
  - scripts/c1_autorun_categories.py
  - tests/test_compute_vertex_transform.py
  - tests/test_bbox_gated_mapping.py
  - tests/test_placement_pairwise_compare_bbox_gate.py
  - tests/test_c1_autorun_categories.py
created_at: 2026-04-10
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Transitive BBox Gating

## Summary

This note consolidates the transitive bbox-gating feature landing across the
certificate builder, mapping builder, audit path, and autorun wiring.

The landed capability is now end-to-end in code:

- transitive pairs can compose a deterministic witness-path `V`
- the endpoint pair is certified directly before admission
- certified transitive pairs can enter the mapping builder and filtered mapping
- audit can consume transitive certificate semantics instead of falling back to
  strict raw `mode_index` behavior
- bbox autorun passes per-category `pair_certificates.jsonl` into audit so the
  real certificate semantics are available during rollout
- pair certificate evaluation now computes `V` through a pure numpy helper, so
  bbox-gated certificate tests do not require `pxr` just to evaluate candidate
  pairs

## Landed Slices

### 1. Witness-path certificates

- `scripts/compute_vertex_transform.py` now builds transitive certificates via
  `build_transitive_pair_certificate(...)`.
- witness selection is deterministic and prefers lower-risk shortest paths.
- the certificate records `transitive_witness_*` metadata and explicit endpoint
  verification fields.
- transitive admission fail-closes when the composed endpoint bbox precheck
  rejects the pair.

### 2. Mapping admission

- `scripts/c1_build_bulk_mapping_from_dedup_report.py` no longer hard-rejects
  `pair_mode == "transitive"`.
- eligible transitive rows now flow through the same certified-graph and
  `filtered_mapping` pipeline as other certified edges.
- rejected transitive rows now keep their real reject reason instead of being
  collapsed into `transitive_not_supported`.

### 3. Audit semantics

- `scripts/placement_pairwise_compare.py` can ingest
  `--certificate-jsonl <pair_certificates.jsonl>` and resolve effective mode
  from certified rows.
- certified transitive rows can therefore inherit soft/hard semantics from
  certificate metadata under bbox policies such as
  `bbox_primary_rmse_observe`.
- rejected rows are excluded from lookup semantics so failed certification does
  not accidentally soften audit behavior.

### 4. Autorun wiring

- `scripts/c1_autorun_categories.py` now threads the category-local
  `pair_certificates.jsonl` artifact into the bbox audit command.
- this makes the shipped rollout path certificate-aware without changing the
  broader autorun execution order.

## Rollout Status

- code support for certifying and auditing transitive edges is now landed
- historical GRScenes-test0 rollout totals are still the pre-rerun state from
  the earlier non-transitive bundle until a fresh real-data rerun is executed
- this change updates capability, not the stored historical run outputs

## Verification

Final verification commands run in this worktree:

```bash
python -m pytest tests/test_compute_vertex_transform.py -q
python -m pytest tests/test_bbox_gated_mapping.py -q
python -m pytest tests/test_c1_autorun_categories.py -q
./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py -q
```

Observed results:

- `python -m pytest tests/test_compute_vertex_transform.py -q`
  - `20 passed`
- `python -m pytest tests/test_bbox_gated_mapping.py -q`
  - `4 passed`
- `python -m pytest tests/test_c1_autorun_categories.py -q`
  - `1 passed`
- `./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py -q`
  - `11 passed`

Conclusion:

- all targeted checks in the required feature test set passed

## Related Notes

- `docs/records/changes/2026-04-13_transitive_bbox_witness_certificates.md`
- `docs/records/changes/2026-04-13_bbox_gated_transitive_mapping.md`
- `docs/records/changes/2026-04-13_transitive_bbox_audit_certificate_semantics.md`
- `docs/records/changes/2026-04-13_pair_certificate_numpy_v_path.md`
- `docs/records/changes/2026-04-13_task4_transitive_bbox_gating.md`
- `docs/records/research/operations/grscenes_test0_dedup_rollout_status_20260410.md`

## Concerns

- no fresh GRScenes-test0 rerun is included in this task, so rollout counts and
  policy totals remain historical rather than transitive-refreshed
