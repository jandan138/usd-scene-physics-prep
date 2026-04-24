---
title: "GRScenes-test0 Dedup Rollout Status"
created_at: 2026-04-10
updated_at: 2026-04-21
maintainer: OpenCode
status: active
code_reference:
  - scripts/compute_vertex_transform.py
  - scripts/c1_build_bulk_mapping_from_dedup_report.py
  - scripts/placement_pairwise_compare.py
  - scripts/c1_autorun_categories.py
  - check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/summary.json
  - check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout/
doc_class: record
---

# GRScenes-test0 Dedup Rollout Status

## Summary

This note now serves as the historical baseline-status record for the
bbox-gated GRScenes-test0 dedup path.

Two facts are now true at the same time:

- code support for certifying and auditing transitive edges is now landed in the
  repo
- the table below still preserves the old historical pre-rerun totals from the
  `test0_rebuilt_dedup` artifact family, even though a fresh transitive-capable
  full rerun has now completed successfully

## Landed Capability Update

The repo can now do all of the following for transitive edges:

- build deterministic witness-path certificates
- admit certified transitive edges into the filtered mapping graph
- audit with certificate-aware transitive semantics
- pass `pair_certificates.jsonl` into bbox autorun audit

This closes the earlier code gap where transitive rows were either unsupported
at mapping time or had to fall back to stricter legacy audit semantics.

## Historical Rollout Totals

The most recent repository-backed historical rollout state comes from:

- `check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/summary.json`
- `check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout/`

Preserved historical totals:

| Metric | Historical value |
| :--- | ---: |
| union_removable | 56,376 |
| candidate_pairs | 56,960 |
| eligible_pairs | 30,372 |
| mapping_pairs | 30,372 |
| rejected_pairs | 26,588 |
| categories done | 74 |
| categories skipped (`mapping_pairs=0`) | 9 |

Additional historical rollout facts:

- the completed rerun state ends with no remaining failed categories in the
  final rerun ledger
- Step 6 is still evidenced only as `dry_run` in the rerun ledgers
- later investigation showed the dry-run apply outputs shared one suffixed
  layout filename per scene, so the rerun should not be read as a completed
  promoted duplicate-removed baseline

These numbers remain useful as rollout history, but they are no longer the most
current rerun result.

For the final transitive-capable rerun conclusion, see:

- `docs/records/research/operations/2026-04-18_transitive_full_rerun_conclusion.md`

## Interpretation

This status note is now historical background rather than the latest state.

The fresh transitive-capable full rerun should be treated as the current
repository-backed dry-run result for metrics and certification evidence, but not
as a confirmed promoted duplicate-removed baseline. See the full rerun
conclusion for the overwrite caveat and dry-run reuse boundary.

Reference:

- `docs/records/research/operations/2026-04-18_transitive_full_rerun_conclusion.md`
