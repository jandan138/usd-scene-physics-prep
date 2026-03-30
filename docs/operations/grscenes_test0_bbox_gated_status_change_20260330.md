---
title: "GRScenes-test0 BBox-Gated Status Change Archive"
created_at: "2026-03-30"
updated_at: "2026-03-30"
maintainer: "Codex"
status: "active"
code_reference:
  - "scripts/c1_autorun_categories.py"
  - "scripts/c1_build_bulk_mapping_from_dedup_report.py"
  - "check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/summary/dlc_status_summary.json"
  - "check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/run_manifest.json"
---

# Summary

This note archives the status change for
`check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/`.

The important change is simple:

- on 2026-03-27, `v5` was still in the "submitted / queuing" state
- by the time of this archive on 2026-03-30, all `16` DLC shard jobs had
  already reached terminal state
- all `16` terminal jobs failed

So the current situation is not "still waiting for cards". The current
situation is "the fresh A/B run already failed and needs diagnosis / rerun".

# What Changed

## Previously archived state

The previous in-repo status snapshot for `v5` recorded:

- fresh run root created
- fresh merged report created
- fresh shard files created
- `16` DLC shard jobs submitted
- first post-submit status:
  - `16 Queuing`
  - `0 terminal`

## Current state

Current repo status summary:

- source:
  - `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/summary/dlc_status_summary.json`
- archived observation time inside that summary:
  - `2026-03-27T12:55:14Z`
- status counts:
  - `16 Failed`
- terminal count:
  - `16`
- non-terminal count:
  - `0`

Live DLC query on 2026-03-30 matches the same conclusion:

- all `16` listed job ids report `JobStatus = Failed`

# Failure Shape

The failures are highly uniform.

## 1. Both policies failed

Failed display-name families:

- Policy A:
  - `t0bboxabv5a_<shard>_8`
- Policy B:
  - `t0bboxabv5b_<shard>_8`

There is no evidence here that one policy passed further than the other.

## 2. Every shard died on the first category

Evidence source:

- `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/policy_a/_autorun/*/ledger.jsonl`
- `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/policy_b/_autorun/*/ledger.jsonl`

Observed pattern:

- every ledger has:
  - `run_start`
  - `category_start`
  - `category_fail`
- no ledger has:
  - `run_done`

Representative examples:

- Policy A shard:
  - category `bicycle`
  - failed at `step = cert`
  - `rc = 1`
- Policy B shard:
  - category `Musical_instrument`
  - failed at `step = cert`
  - `rc = 1`

## 3. The pipeline never reached meaningful A/B comparison stages

Current `policy_a/` and `policy_b/` trees under `v5` contain only `_autorun`
ledger outputs, not completed category-stage artifacts.

That means the run did **not** get far enough to produce trustworthy:

- cert outputs for the full shard set
- apply outputs
- audit verdicts
- step6 gate decisions
- final A/B comparison data

# Root Cause Visible In Logs

Representative cert logs:

- `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/policy_a/_autorun/c1_autorun_bbox_primary_rmse_observe_20260327_125015/bicycle/01_cert.log`
- `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/policy_b/_autorun/c1_autorun_bbox_primary_rmse_harder_20260327_125126/Musical_instrument/01_cert.log`

Both show the same failure:

```text
ModuleNotFoundError: No module named 'ijson'
```

The exception is raised when
`scripts/c1_build_bulk_mapping_from_dedup_report.py` imports `ijson`.

# Interpretation

The failure should be understood as a shared runtime dependency problem, not as
a data-quality verdict and not as a policy-A vs policy-B outcome.

In plain terms:

- the jobs did start running
- they immediately tried to enter the cert stage
- the cert-stage script needed Python package `ijson`
- that package was not available in the DLC runtime
- so every shard crashed before the actual A/B evaluation could begin

This means:

- we still do **not** have a trustworthy Policy A / Policy B result bundle
- the next real step is to fix the runtime dependency issue, then rerun from a
  fresh root

# Next Recommended Step

The next action should be one of:

1. install / provide `ijson` in the DLC runtime used by `isaac_python.sh`
2. or patch `scripts/c1_build_bulk_mapping_from_dedup_report.py` to fall back
   cleanly when `ijson` is unavailable

After that, rerun from a new fresh run id instead of trying to treat `v5` as a
valid finished A/B evaluation.
