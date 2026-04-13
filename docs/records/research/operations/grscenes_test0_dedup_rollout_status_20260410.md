---
title: "GRScenes-test0 Dedup Rollout Status"
created_at: 2026-04-10
updated_at: 2026-04-13
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

This note captures the rollout status for the bbox-gated GRScenes-test0 dedup
path after the transitive bbox-gating code landing.

Two facts are true at the same time:

- code support for certifying and auditing transitive edges is now landed in the
  repo
- the currently cited rollout totals still reflect the historical pre-rerun
  state from the `test0_rebuilt_dedup` artifact family, because no fresh
  GRScenes-test0 rerun has been executed yet with the transitive-capable stack

## Landed Capability Update

The repo can now do all of the following for transitive edges:

- build deterministic witness-path certificates
- admit certified transitive edges into the filtered mapping graph
- audit with certificate-aware transitive semantics
- pass `pair_certificates.jsonl` into bbox autorun audit

This closes the earlier code gap where transitive rows were either unsupported
at mapping time or had to fall back to stricter legacy audit semantics.

## Current Rollout Totals Remain Historical

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

These numbers are still useful as rollout history, but they should not be
presented as transitive-refreshed totals.

## Interpretation

- the feature landing changes what the stack is capable of certifying and
  auditing
- it does not retroactively change artifacts already written by earlier runs
- any rollout report, yield comparison, or promotion decision that needs
  transitive coverage must come from a fresh rerun

## Next Required Operational Step

Run a fresh GRScenes-test0 bbox-gated rerun with the transitive-capable stack,
then replace the historical totals above with rerun-derived totals and verdicts.

Until that rerun exists, the correct status statement is:

> Transitive certification and audit support are landed in code, but GRScenes-test0
> rollout totals still reflect the pre-rerun historical state.
