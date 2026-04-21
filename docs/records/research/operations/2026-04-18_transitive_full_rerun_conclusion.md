---
title: "Transitive Full Rerun Conclusion"
code_reference:
  - scripts/c1_autorun_categories.py
  - scripts/c1_build_bulk_mapping_from_dedup_report.py
  - scripts/placement_pairwise_compare.py
  - scripts/dlc/launch_job.sh
  - scripts/dlc/run_task.sh
created_at: 2026-04-18
updated_at: 2026-04-18
maintainer: OpenCode
status: active
doc_class: record
---

# Transitive Full Rerun Conclusion

## Summary

The authoritative GRScenes-test0 transitive-capable full rerun has completed
successfully.

Full rerun job:

- name: `test0_transitive_full_0_1`
- job_id: `dlcve680agoitv7g`
- final status: `Succeeded`
- runtime: about `65h11m`

Operational result:

- `74` categories finished as `category_done`
- `9` categories finished as `category_skip`
- `0` categories failed
- Step 6 remained `dry_run`

Feature result:

- `transitive_not_supported` was reduced from `11399` to `0`
- `mapping_pairs` increased from `30372` to `32445`
- the rerun recovered `2073` additional mapping pairs relative to the previous
  historical rollout

## Historical Baseline vs Full Rerun

Historical baseline root:

- `check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout`

Transitive full rerun root:

- `check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout_transitive_full_dlc_20260415`

### Shared scope

- `categories=83`
- `candidate_pairs=56960`
- `groups_seen=586146`
- `groups_included=7062`
- `revoked_edge_count=0`

### Historical baseline totals

- `eligible_pairs=30372`
- `mapping_pairs=30372`
- `rejected_pairs=26588`

Reject reason totals:

- `transitive_not_supported=11399`
- `bbox_precheck_failed_topo_filesize=6177`
- `bbox_precheck_failed_shape_invariant=2097`
- `v_computation_failed_shape_invariant=6915`

### Full rerun totals

- `eligible_pairs=32445`
- `mapping_pairs=32445`
- `rejected_pairs=24515`

Reject reason totals:

- `bbox_precheck_failed_topo_filesize=6177`
- `bbox_precheck_failed_shape_invariant=2097`
- `bbox_precheck_failed_transitive=2850`
- `v_computation_failed_shape_invariant=6915`
- `v_computation_failed_transitive=6476`

### Net deltas

- `mapping_pairs: +2073`
- `eligible_pairs: +2073`
- `rejected_pairs: -2073`
- `transitive_not_supported: -11399`

Interpretation:

- the old unsupported-transitive bucket did not disappear into nothing; it was
  redistributed into real geometric outcomes
- `9326` formerly unsupported transitive pairs now fail in more informative
  buckets:
  - `v_computation_failed_transitive=6476`
  - `bbox_precheck_failed_transitive=2850`
- the remaining `2073` were recovered as valid mapping pairs

## Operational Completion Evidence

Fresh ledger summary from:

- `check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout_transitive_full_dlc_20260415/_autorun/c1_v8_tier2_rollout_transitive_full_dlc_bbox_primary_rmse_observe_20260416_025705/ledger.jsonl`

Observed completion state:

- `category_done=74`
- `category_skip=9`
- `category_fail=0`
- final event = `run_done`

The last completed categories include:

- `tv`
- `tv_stand`
- `tvstand`
- `wall`
- `washingmachine`
- `window`

Heavy categories also closed cleanly with `01_cert`, `03_audit`, and
`04_step6` outputs present, including:

- `other`
- `pen`
- `person`
- `wall`
- `window`

## Warning Interpretation

The rerun still emits many missing-asset warnings such as `Could not open
asset ...` for references preserved in layout USDs but absent from the current
normalized/prededup asset chain.

Observed behavior across probe, wall-only debug, and the final full rerun shows
these warnings are currently noisy but non-blocking:

- the full run still completed successfully
- the heavy categories still reached `audit passed`
- no category-level failures were recorded in the final ledger

This means the current transitive-capable rerun is operationally viable even
with those warnings present, though the dataset hygiene problem still exists.

## What Changed In Practice

Before the transitive-capable rerun:

- the rollout was already operationally complete, but its main limitation was a
  large `transitive_not_supported` bucket

After the transitive-capable rerun:

- unsupported transitive edges are no longer a terminal bucket in this rollout
- the system now distinguishes between:
  - transitive V computation failures
  - transitive bbox precheck failures
  - genuinely accepted transitive-driven mappings

This is the key evidence that the feature moved the pipeline from
"transitive edges are blocked" to "transitive edges are evaluated under the same
certificate and audit contract as direct edges".

## Bottom Line

The transitive-capable GRScenes-test0 full rerun succeeded and produced a real
improvement over the historical rollout.

- the rerun is complete
- the output remains dry-run authoritative rather than promote-applied
- the transitive feature recovered `2073` additional mapping pairs
- the old unsupported bucket was eliminated and replaced with more informative
  transitive-specific rejection buckets

The next decision is no longer "does the feature work?"

The next decision is whether to treat this dry-run rerun as the accepted new
rollout baseline and what follow-up, if any, is required for the underlying
missing-reference noise in the dataset chain.
