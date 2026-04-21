---
title: "DLC Wall Probe Stall Decision"
code_reference:
  - scripts/dlc/launch_job.sh
  - scripts/dlc/run_task.sh
  - scripts/c1_autorun_categories.py
created_at: 2026-04-15
updated_at: 2026-04-18
maintainer: OpenCode
status: active
doc_class: record
---

# DLC Wall Probe Stall Decision

## Summary

The transitive probe retry job `dlc1qeonwq3u626g` completed `book`, `bottle`,
`box`, `ground`, `other`, and `pen`, but `wall` remained in `category_start`
without creating the expected `wall_bbox_primary_rmse_observe_v1/` outputs.

> Superseded note: this document records the decision made while the wall stage
> still appeared stalled inside the multi-category probe. The later isolated
> wall-only rerun completed successfully, and the final full rerun also
> succeeded. See
> `docs/records/research/operations/2026-04-18_transitive_full_rerun_conclusion.md`
> for the current conclusion.

## Decision

Use the conservative low-risk path:

- stop the current multi-category DLC job once the stalled state is confirmed
- keep the completed category outputs for later comparison
- isolate the remaining problem with a `wall`-only DLC debug submission
- keep execution serial rather than introducing multi-job category parallelism

## Why

- historical rollout data shows `wall` is not the heaviest category in this
  pipeline
- historical `wall` runtime is about `0.75h`, while the current retry spent
  more than `6h` inside `wall` without stage outputs
- the behavior therefore looks more like a `wall`-specific stall than normal
  heavy-category runtime

## Caveat

The current rerun dataset copy already contains modifications from the completed
probe categories. A `wall`-only debug run on the same dataset root is suitable
for stall diagnosis, but it is not a pristine replay from the original
`prededup` state.
