---
title: DLC Quota Migration Analysis (smartbot → less_gpu / more_gpu)
code_reference: scripts/dlc/launch_job.sh
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: TraeAI
status: draft
doc_class: archive
---

# DLC Quota Migration Analysis

## Summary

The `smartbot` parent partition is being split into `less_gpu` (≤4 GPU jobs) and `more_gpu` (≥8 GPU jobs). All current submission scripts use the old parent `resource_id` and a 1-GPU template with CPU/memory values that do **not** match the new templates. This document records the gaps and required changes.

## Current vs Required (Typical 1-GPU Job)

| Parameter | Current Value | New Template (`less_gpu` 1 GPU) | Action |
|-----------|---------------|----------------------------------|--------|
| `worker_gpu` | `1` | `1` | No change |
| `worker_cpu` | `16` | `14` | **Change** |
| `worker_memory` | `118Gi` | `100Gi` | **Change** |
| `worker_shared_memory` | `118Gi` | `?` | **Clarify** |
| `resource_id` | `quotalplclkpgjgv` (parent smartbot) | `TBD` (`less_gpu`) | **Change** |
| `oversold_type` | `ForbiddenQuotaOverSold` | `?` | **Clarify** |
| `priority` | `7` | `?` | **Clarify** |

## Files That Must Be Changed

Only **`scripts/dlc/launch_job.sh`** hardcodes resource flags. All submit wrappers delegate to it unchanged.

- `scripts/dlc/launch_job.sh`
  - `RESOURCE_ID` default
  - `--worker_cpu`
  - `--worker_memory`
  - `--worker_shared_memory` (pending clarification)

**Submit wrappers examined (all call `launch_job.sh` without overriding resource flags):**

- `scripts/dlc/submit_batch.py`
- `scripts/dlc/submit_normalize_phase1.sh`
- `scripts/dlc/submit_normalize_phase2.sh`
- `scripts/dlc/submit_shape_invariant_dedup.sh`
- `scripts/dlc/submit_topo_filesize_dedup.sh`
- `scripts/dlc/submit_test0_rebuilt_geom_only_dedup.sh`
- `scripts/dlc/submit_test0_rebuilt_shape_invariant_dedup.sh`
- `scripts/dlc/submit_test0_rebuilt_topo_filesize_dedup.sh`
- `scripts/dlc/submit_v8_prededup_geom_only_dedup.sh`
- `scripts/dlc/submit_v8_prededup_shape_invariant_dedup.sh`
- `scripts/dlc/submit_v8_prededup_topo_filesize_dedup.sh`

## Parameters with Unclear Mappings

1. **`worker_shared_memory`** — Not listed in the template image. Current code sets it equal to `worker_memory` (`118Gi`). We need confirmation whether it should:
   - Match `worker_memory` exactly (`100Gi`), or
   - Follow a different rule (e.g., fixed ratio or capped ceiling).

2. **`oversold_type=ForbiddenQuotaOverSold`** — Included historically but not mentioned in the new policy. Need confirmation if it remains valid under `less_gpu` / `more_gpu`.

3. **`priority 7`** — Also not mentioned in the policy. Need confirmation if the same priority range applies.

## Questions for Admin

1. **Exact `resource_id` strings**
   - What is the `resource_id` for `less_gpu`?
   - What is the `resource_id` for `more_gpu`?

2. **`worker_shared_memory` rule**
   - Should it equal `worker_memory` (e.g., `100Gi` for 1-GPU)?
   - Or is there a separate mapping table?

3. **`--oversold_type` and `--priority` validity**
   - Do `ForbiddenQuotaOverSold` and `priority 7` remain accepted under the new sub-quotas?
   - If not, what are the replacements or defaults?

4. **Rejection behavior for non-matching specs**
   - Will a 1-GPU job specifying `16C/118Gi` be actively rejected under `less_gpu`?
   - If yes, should we proactively switch **all** 1-GPU templates to `14C/100Gi` now, or wait until the old parent quota is fully disabled?
