---
title: "GRScenes-test0 BBox-Gated DLC ijson Remediation Plan"
created_at: "2026-03-30"
updated_at: "2026-03-30"
maintainer: "Codex"
status: "active"
code_reference:
  - "scripts/dlc/run_task.sh"
  - "scripts/isaac_python.sh"
  - "scripts/c1_build_bulk_mapping_from_dedup_report.py"
  - "check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/summary/dlc_status_summary.json"
  - "check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/policy_a/_autorun/c1_autorun_bbox_primary_rmse_observe_20260327_125015/bicycle/01_cert.log"
---

# Summary

This document defines the remediation plan for the failed
`20260327_bbox_ab_eval_v5` bbox-gated real-data A/B run.

The immediate problem is not policy logic. The immediate problem is runtime
availability of `ijson` inside DLC cert-stage jobs.

Observed failure in representative cert logs:

```text
ModuleNotFoundError: No module named 'ijson'
```

This failure occurs before the pipeline can produce meaningful Policy A /
Policy B outputs.

# Problem Statement

## What failed

The fresh `v5` run launched `16` DLC shard jobs:

- `8` shards for Policy A
- `8` shards for Policy B

All `16` jobs reached terminal state and all `16` failed.

## Where they failed

Every shard failed in the same place:

- first category in the shard
- `cert` step
- inside `scripts/c1_build_bulk_mapping_from_dedup_report.py`

Representative evidence:

- `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/policy_a/_autorun/c1_autorun_bbox_primary_rmse_observe_20260327_125015/bicycle/01_cert.log`
- `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/policy_b/_autorun/c1_autorun_bbox_primary_rmse_harder_20260327_125126/Musical_instrument/01_cert.log`

## Why this matters

`cert` is the gate that builds:

- pair certificates
- certified graph
- filtered mapping

Without those outputs, the run cannot proceed to:

- apply
- audit
- Step6 dry-run
- final A/B comparison

So the runtime failure blocks the whole real-data evaluation.

# Evidence

## Historical failure evidence

`v5` cert logs show:

```text
ModuleNotFoundError: No module named 'ijson'
```

`v5` autorun ledgers show a uniform failure pattern:

- `run_start`
- `category_start`
- `category_fail`

No shard reaches:

- `run_done`
- apply-stage outputs
- audit-stage outputs
- step6 outputs

## Current local verification

On 2026-03-30, local verification in this checkout shows:

- repo `python` can currently import `ijson`
- `./scripts/isaac_python.sh` can currently import `ijson`
- observed import location:
  - `/isaac-sim/kit/python/lib/python3.10/site-packages/ijson/__init__.py`

This means the failure should not be interpreted as:

- "the repository can never import `ijson`"

It should be interpreted as:

- the failed DLC runtime did not reliably expose `ijson` at job execution time
- relying on ambient image state is not sufficiently robust for this pipeline

# Goals

## Primary goal

Make `ijson` reliably available to DLC bbox-gated cert jobs that run through:

- `scripts/dlc/launch_job.sh`
- `scripts/dlc/run_task.sh`
- `scripts/isaac_python.sh`

## Secondary goals

- make failures explicit earlier when the runtime is missing required packages
- avoid repeating a full fresh A/B submission only to fail at the first cert
  import again
- keep the fix repo-controlled rather than dependent on hidden machine state

## Non-goals

- changing bbox-gated policy logic
- changing Policy A / Policy B semantics
- changing the execution shape of the real-data rerun

# Remediation Options

## Option 1. Repo-controlled runtime bootstrap in `run_task.sh`

Approach:

- add a preflight before dispatching Isaac Python work
- verify `ijson` import explicitly
- if missing, install `ijson` into a repo-controlled runtime dependency
  directory, then prepend that directory to `PYTHONPATH`

Pros:

- directly targets the actual DLC job entrypoint
- repo-controlled and reproducible
- does not depend on ambient image drift
- benefits every future DLC job launched through the same wrapper

Cons:

- adds a small runtime bootstrap path to job startup
- requires deciding where the runtime dependency cache should live

Assessment:

- best primary fix

## Option 2. Preflight-only hard fail with better logging

Approach:

- add an explicit `ijson` import precheck in `run_task.sh` or
  `scripts/isaac_python.sh`
- if missing, fail immediately with a targeted diagnostic that prints:
  - Python executable
  - `sys.prefix`
  - `site-packages`
  - import failure

Pros:

- simple
- improves debuggability

Cons:

- does not actually repair the runtime
- still requires a separate manual environment fix before rerun

Assessment:

- good guardrail, but insufficient as the only fix

## Option 3. Script-level fallback in `c1_build_bulk_mapping_from_dedup_report.py`

Approach:

- change the script to fall back to stdlib `json` streaming alternatives or a
  less memory-efficient full load when `ijson` is unavailable

Pros:

- removes the immediate hard dependency from the cert script

Cons:

- treats a runtime problem as an application-level workaround
- may change memory/performance behavior for large reports
- does not help other future runtime dependencies

Assessment:

- useful as a resilience improvement later, but not the preferred primary fix
  for DLC runtime correctness

# Recommended Plan

Use a two-layer remediation:

1. primary fix:
   - repo-controlled runtime bootstrap in `scripts/dlc/run_task.sh`
2. guardrail:
   - explicit dependency preflight and better diagnostics in
     `scripts/isaac_python.sh` and/or `run_task.sh`

Optional later hardening:

3. fallback behavior in `scripts/c1_build_bulk_mapping_from_dedup_report.py`
   only if we decide we want defense in depth against missing `ijson`

This ordering keeps the main fix aligned with the actual failure surface:

- DLC jobs launched correctly
- same wrappers used in future reruns
- reduced dependence on whatever the base image happens to contain

# Proposed Implementation Slice

## Files to update

- `scripts/dlc/run_task.sh`
- `scripts/isaac_python.sh`

Optional later hardening:

- `scripts/c1_build_bulk_mapping_from_dedup_report.py`

## Concrete behavior to add

### 1. Runtime dependency cache location

Choose a repo-controlled path such as:

```text
<repo_root>/.runtime_deps/isaac_py310/
```

Requirements:

- writable by DLC jobs
- stable across retries
- safe to prepend to `PYTHONPATH`

### 2. `run_task.sh` preflight

Before dispatching any Isaac Python command:

- check whether `ijson` imports under the actual Isaac Python runner
- if import succeeds:
  - continue immediately
- if import fails:
  - bootstrap install `ijson==3.5.0` into the runtime dependency cache
  - prepend the cache path to `PYTHONPATH`
  - re-check import
- if the second check still fails:
  - abort with a focused diagnostic

Important packaging rule:

- do **not** make the bootstrap depend on public internet availability at job
  runtime
- prefer one of:
  - a pre-vendored wheel stored in the repo or adjacent project-controlled path
  - an internal package source already reachable from the DLC environment
  - a deterministic copy from a known-good local Isaac Python site-packages tree

This keeps the remediation reproducible and avoids replacing one runtime
uncertainty with another.

### 3. `isaac_python.sh` guardrail

Add optional support for a repo-local dependency directory:

- if the runtime dependency cache exists, prepend it to `PYTHONPATH`
- keep this logic lightweight and deterministic

This ensures local and DLC execution paths converge on the same import behavior.

### 4. Logging

On missing dependency or bootstrap:

- print the Python executable path
- print `sys.prefix`
- print the final `ijson` import location when successful
- print the cache path used

This is important so future failures do not look like generic cert crashes.

# Validation Plan

## Local validation

1. run a targeted import check through:
   - `./scripts/isaac_python.sh`
2. simulate bootstrap path behavior by forcing the runtime dependency cache path
   into `PYTHONPATH`
3. run:
   - `scripts/c1_build_bulk_mapping_from_dedup_report.py --help`
   or a tiny category-local cert invocation

## DLC-surface validation

Before resubmitting the full A/B rerun:

1. launch one minimal DLC validation job through the existing wrappers
2. that job should only print:
   - Python executable
   - `sys.prefix`
   - `ijson` import path
3. require a successful import in that validation job before launching a fresh
   A/B rerun

## Full rerun validation

After the runtime fix is validated:

1. choose a fresh run id, not `v5`
2. resubmit Policy A / Policy B shards in the same agreed execution shape
3. confirm at least one shard progresses beyond:
   - `cert`
   - into `apply`

# Documentation Linkage

This plan should be linked from:

- `docs/operations/grscenes_test0_bbox_gated_dedup_status_20260327.md`
- `docs/operations/grscenes_test0_bbox_gated_status_change_20260330.md`

Suggested link text:

- `DLC ijson remediation plan`

# Exit Criteria

This remediation plan is considered complete when:

1. the plan is reviewed and accepted
2. the linked status docs point to it
3. the runtime fix is implemented
4. a DLC validation job proves `ijson` import under the real job entrypoint
5. a new fresh A/B rerun is launched from the validated runtime
