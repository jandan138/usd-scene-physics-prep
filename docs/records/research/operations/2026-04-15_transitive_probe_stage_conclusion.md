---
title: "Transitive Probe Stage Conclusion"
code_reference:
  - scripts/c1_autorun_categories.py
  - scripts/c1_build_bulk_mapping_from_dedup_report.py
  - scripts/placement_pairwise_compare.py
  - scripts/dlc/launch_job.sh
  - scripts/dlc/run_task.sh
created_at: 2026-04-15
updated_at: 2026-04-18
maintainer: OpenCode
status: active
doc_class: record
---

# Transitive Probe Stage Conclusion

> Superseded note: this document remains the probe-stage historical record, but
> the final authoritative full rerun is now complete. See
> `docs/records/research/operations/2026-04-18_transitive_full_rerun_conclusion.md`
> for the final metrics and rollout conclusion.

## Summary

This document records the intermediate probe-stage validation that happened
before the final authoritative full rerun completed.

Key outcome:

- the six-category transitive probe retry (`book`, `bottle`, `box`, `ground`,
  `other`, `pen`) completed successfully
- the remaining `wall` category also completed successfully once isolated into a
  dedicated wall-only DLC job
- this confirms the transitive cert/mapping/audit path works on real probe data
- the remaining operational issue is throughput strategy, not correctness

## Completed Probe Evidence

Probe retry root:

- `check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout_transitive_probe_dlc_retry_20260414`

Completed categories from the probe retry ledger:

- `book`
- `bottle`
- `box`
- `ground`
- `other`
- `pen`

Aggregated totals across those six completed categories:

- `candidate_pairs=40430`
- `eligible_pairs=25037`
- `mapping_pairs=25037`
- `rejected_pairs=15393`
- `transitive_not_supported=0`
- `bbox_precheck_failed_transitive=2647`
- `v_computation_failed_transitive=2145`

Historical baseline for the same six categories:

- `candidate_pairs=40430`
- `mapping_pairs=23240`
- `rejected_pairs=17190`
- `transitive_not_supported=6589`

Interpretation:

- for the completed probe categories, transitive edges are no longer blocked by
  `transitive_not_supported`
- the recovered path shifts the failure frontier into real geometry rejection
  buckets rather than blanket unsupported rejection
- `mapping_pairs` increased by `1797` across the completed six-category probe
  slice

## Wall-Only Validation

Wall-only debug root:

- `check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout_transitive_wall_debug_dlc_20260415`

Wall-only DLC job:

- name: `test0_transitive_wall_debug_0_1`
- job_id: `dlcb4geimt0nqzba`
- final status: `Succeeded`
- runtime: about `7h38m`

Wall cert summary:

- `candidate_pairs=8217`
- `eligible_pairs=1272`
- `rejected_pairs=6945`
- `transitive_not_supported=0`
- `bbox_precheck_failed_transitive=9`
- `v_computation_failed_transitive=4303`

Historical wall baseline:

- `mapping_pairs=1214`
- `transitive_not_supported=4370`

Wall-only outcome:

- wall is not blocked by unsupported transitive edges anymore
- wall completed successfully through `01_cert -> 02_apply -> 03_audit -> 04_step6`
- `audit_verdict.json` reports:
  - `passed=true`
  - `scenes_error=0`
  - `ref_changed_fail_count=0`

## What We Learned

### 1. The transitive feature is working

- real rerun data now shows transitive coverage being converted into certified
  and mapped pairs
- the key success condition is not hypothetical anymore; it is observed in real
  probe outputs

### 2. Missing asset warnings are noisy but not currently blocking

- the normalized/prededup rerun chain still contains layout references to assets
  that are absent from that chain
- these warnings remain visible in both the six-category retry and wall-only
  debug jobs
- however, the completed categories still reach `audit passed` with no hard
  audit failures

### 3. Wall is slow, but not actually broken

- earlier evidence made `wall` look stalled inside the multi-category probe
- the isolated wall-only run proved that `wall` is a very slow but viable case,
  not an inherently failing one

### 4. Single-job full serial execution is the wrong operational shape

- `other` took about `5.21h` in the probe retry
- `wall` took about `7.64h` in the wall-only debug run
- a fully serial full rerun would therefore be unnecessarily slow and fragile

## Recommended Next Strategy

Do **not** continue with one giant fully serial full rerun job.

Use a conservative low-parallelism batching strategy instead:

- batch A: `wall` alone
- batch B: `other` alone
- batch C: `book` alone or with one small companion only if quota pressure is
  low
- batch D+: remaining medium/light categories grouped into one or two serial
  jobs

This is still conservative because:

- each job remains internally serial
- heavy categories are isolated for observability
- the number of concurrent jobs stays small
- failures will be easier to attribute and rerun

## Bottom Line

The probe stage is successful enough to justify moving forward.

- correctness is good enough to proceed
- unsupported transitive edges are being converted into real certified outcomes
- the next problem is execution strategy and wall-clock time, not feature
  correctness
