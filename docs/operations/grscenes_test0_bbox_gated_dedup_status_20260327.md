---
title: "GRScenes-test0 BBox-Gated Dedup Status and Handoff"
created_at: "2026-03-27"
updated_at: "2026-03-27"
maintainer: "Codex"
status: "active"
code_reference:
  - "scripts/compute_vertex_transform.py"
  - "scripts/c1_build_bulk_mapping_from_dedup_report.py"
  - "scripts/rewrite_layout_asset_refs_with_compensation.py"
  - "scripts/c1_bulk_apply_layout_dedup.py"
  - "scripts/placement_pairwise_compare.py"
  - "scripts/c1_bulk_step6_category_promote_scan_soft_delete.py"
  - "scripts/c1_autorun_categories.py"
  - "tests/test_bbox_gated_mapping.py"
  - "tests/test_bbox_gated_rewrite.py"
  - "tests/test_placement_pairwise_compare_bbox_gate.py"
---

# Summary

This note records the current implementation state of the bbox-gated dedup
landing for `GRScenes-test0`, the real-data execution attempts already made,
the reasons those attempts were stopped, and the exact execution shape that the
next Codex session should continue from.

This document is intentionally operational. It is not a policy discussion note.

# What Has Been Landed

The repo is no longer at a pure planning stage. The following implementation
slices have already been landed in code.

## 1. Pair certification and filtered mapping

Implemented in:

- `scripts/compute_vertex_transform.py`
- `scripts/c1_build_bulk_mapping_from_dedup_report.py`

Current behavior:

- a structured pair-certificate layer exists
- early rollout keeps the certificate surface narrow:
  - proof-backed `geom_only` only
- the mapping build can now emit:
  - `pair_certificates.jsonl`
  - `pair_certificate_summary.json`
  - `certified_graph.json`
  - `filtered_mapping.json`
  - `filtered_mapping.stats.json`
- attributable edge revocation is wired in at the mapping layer:
  - `revoked_edges.jsonl`
  - rebuild certified graph / canonical selection from remaining valid edges

## 2. Fail-closed rewrite and bulk apply

Implemented in:

- `scripts/rewrite_layout_asset_refs_with_compensation.py`
- `scripts/c1_bulk_apply_layout_dedup.py`

Current behavior:

- bbox-gated rewrite is fail-closed instead of permissive
- early rollout rejects unsupported rewrite shapes before mutation:
  - payload rewrites
  - asset-valued attribute rewrites
  - multi-ref changed prims
- would-be ref changes that fail preflight do not mutate the scene
- reject records are surfaced into rewrite reports
- bulk apply threads through bbox-gated metadata and can aggregate reject
  ledgers

## 3. Authoritative audit and Step6 gate

Implemented in:

- `scripts/placement_pairwise_compare.py`
- `scripts/c1_bulk_step6_category_promote_scan_soft_delete.py`

Current behavior:

- `placement_pairwise_compare.py` now emits an authoritative
  `audit_verdict.json`
- the validator now records bbox / footprint / centroid hard-fail evidence for
  `ref_changed` scope
- the validator can write a separate verdict artifact and exit non-zero on hard
  failures
- Step6 can now require an audit verdict input and block promotion / soft-delete
  when the verdict fails

## 4. BBox-gated autorun flow

Implemented in:

- `scripts/c1_autorun_categories.py`

Current behavior:

- there is a dedicated bbox-gated path, distinct from the old permissive C1 flow
- the execution order is now:
  - cert
  - apply
  - audit
  - Step6 gate / dry-run
- real-data execution ergonomics were improved during this turn:
  - `--scene-list-json`
    - audit only changed scenes
  - `--category-list-json`
    - category sharding
  - `--scene-files layout.usd`
    - keep A/B focused on the actual contract surface

# Tests and Validation Already Passed

## Focused tests

Passed:

- `python -m pytest tests/test_compute_vertex_transform.py tests/test_bbox_gated_mapping.py -q`
  - `17 passed`
- `./scripts/isaac_python.sh -m pytest tests/test_bbox_gated_rewrite.py tests/test_placement_pairwise_compare_bbox_gate.py tests/test_dedup_compensation_chain.py tests/test_patch_placement.py -q`
  - `37 passed, 4 skipped`

## Synthetic end-to-end smokes

Confirmed:

- bbox-gated cert -> apply -> audit -> Step6 dry-run can run end-to-end
- a synthetic `ref_changed` path can produce a clean authoritative verdict when
  the replacement is actually safe

# Real-Data Run Attempts Already Made

The following run roots exist and should be treated as historical attempts, not
final evaluation outputs.

## `20260327_bbox_ab_eval_v1`

Root:

- `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v1/`

Purpose:

- first real-data smoke against a real category

Why it is not final:

- it was used to validate the path shape and artifact segregation
- it predated later runtime optimizations

## `20260327_bbox_ab_eval_v2`

Root:

- `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v2/`

Purpose:

- first full A/B attempt

Why it was stopped:

- audit still scanned full scene scope instead of changed-scene scope
- continuing would have produced a slower, now-outdated execution shape

## `20260327_bbox_ab_eval_v3`

Root:

- `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v3/`

Purpose:

- rerun after adding changed-scene audit

Why it was stopped:

- category execution was still effectively too broad / not sharded tightly
- a cleaner sharded shape became available

## `20260327_bbox_ab_eval_v4`

Root:

- `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v4/`

Purpose:

- sharded full A/B attempt

Why it was stopped:

- after v4 started, a further execution-shape improvement landed:
  - keep A/B on `layout.usd` only
- leaving v4 running would have mixed old and new execution shapes

## `20260327_bbox_ab_eval_v5`

Root:

- `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/`

Purpose:

- fresh real-data A/B rerun from commit `94659ba`
- keep the latest agreed execution shape in one isolated bundle

Current state:

- fresh merged report materialized at:
  - `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/summary/merged_geom_only.json`
- fresh clean non-door shard files materialized at:
  - `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/summary/shard_0.json`
  - `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/summary/shard_1.json`
  - `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/summary/shard_2.json`
  - `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/summary/shard_3.json`
  - `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/summary/shard_4.json`
  - `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/summary/shard_5.json`
  - `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/summary/shard_6.json`
  - `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/summary/shard_7.json`
- isolated workroots prepared at:
  - `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/workroots/policy_a_dataset`
  - `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/workroots/policy_b_dataset`
- both policies were submitted to DLC as `8` shard jobs each:
  - `t0bboxabv5a_<shard>_8`
  - `t0bboxabv5b_<shard>_8`
- machine-readable live status is tracked at:
  - `check_reports/test0_bbox_gated/20260327_bbox_ab_eval_v5/summary/dlc_status_summary.json`
- archived status-change note:
  - `docs/operations/grscenes_test0_bbox_gated_status_change_20260330.md`
- remediation plan:
  - `docs/operations/grscenes_test0_bbox_gated_dlc_ijson_remediation_plan_20260330.md`
- remediation validation status:
  - `docs/operations/grscenes_test0_bbox_gated_dlc_ijson_validation_status_20260331.md`

Why it is not final yet:

- all `16` fresh-run DLC shard jobs are now terminal and all `16` failed
- every shard failed on its first category at `step = cert`
- representative cert logs show the common runtime error:
  - `ModuleNotFoundError: No module named 'ijson'`
- because the shared failure happens before cert completes, `v5` still does not
  provide a trustworthy Policy A / Policy B result bundle

# What Is Blocking Final A/B Completion

The blocker is no longer missing implementation.

The blocker is that the fresh real-data A/B run has not yet fully reached
terminal outputs from the latest isolated run root after the latest
runtime-shape optimizations were all in place at the same time.

Concretely:

- core code is implemented
- focused tests pass
- synthetic smokes pass
- `v5` was launched in the right shape, but the full run failed at a shared
  cert-stage runtime dependency issue before meaningful A/B outputs were
  produced

That is why it would be misleading to claim that the A/B evaluation is already
complete.

# Latest Recommended Execution Shape

The next session should start a fresh run root and not resume `v1` to `v4`.

The run should use all of the following together:

1. fresh run id under `check_reports/test0_bbox_gated/<new_run_id>/`
2. isolated policy workroots:
   - `workroots/policy_a_dataset`
   - `workroots/policy_b_dataset`
3. merged `geom_only` report input only
4. bbox-gated autorun
5. `--scene-files layout.usd`
6. changed-scene-only audit
7. category sharding via `--category-list-json`
8. Step6 in `dry_run` mode until the final A/B bundle is assembled

# Do Not Mix These Things

To avoid confusing valuable baseline data, intermediate outputs, and obsolete
run attempts, keep the following rules strict.

## Baseline

Authoritative upstream baseline:

- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup`

Do not treat any A/B workroot as the baseline.

## Historical runs

These are not final evaluation outputs:

- `20260327_bbox_ab_eval_v1`
- `20260327_bbox_ab_eval_v2`
- `20260327_bbox_ab_eval_v3`
- `20260327_bbox_ab_eval_v4`

They are useful only as execution history and partial diagnostics.

## Final evaluation target

The next fresh run should produce the only bundle that counts as the final A/B
evaluation for this implementation wave.

# Exact Next-Window Objective

The next Codex window should do exactly this:

1. start from the already-landed code in the current repo
2. ignore old incomplete A/B roots except for diagnostics
3. create a fresh `v5`-style run root
4. run Policy A and Policy B in sharded parallel execution
5. wait for every shard to finish
6. aggregate the per-policy outputs into:
   - A/B eligible pair deltas
   - mapping/yield deltas
   - audit verdict summary deltas
   - worst-category and worst-scene concentration
7. write the final comparison summary back into repo docs

# References

- `docs/operations/grscenes_test0_bbox_gated_dedup_implementation.md`
- `docs/operations/grscenes_test0_bbox_gated_dedup_task_breakdown.md`
- `docs/operations/grscenes_test0_bbox_gated_ab_eval_v6_status.md` — final A/B evaluation results
- `docs/operations/grscenes_test0_bbox_gated_status_change_20260330.md`
- `docs/operations/grscenes_test0_bbox_gated_dlc_ijson_remediation_plan_20260330.md`
- `docs/operations/grscenes_test0_bbox_gated_dlc_ijson_validation_status_20260331.md`
- `.codex/worklogs/main/2026-03-27/bbox-gated-ab-eval-plan.md`
- `.codex/worklogs/main/2026-03-27/bbox-gated-ab-eval-research.md`
- `.codex/worklogs/main/2026-03-27/bbox-gated-ab-eval-handoff.md`
