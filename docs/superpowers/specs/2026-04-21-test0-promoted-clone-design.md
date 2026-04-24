---
title: GRScenes Test0 Promoted Clone Design
code_reference:
  - scripts/c1_autorun_categories.py
  - scripts/c1_bulk_apply_layout_dedup.py
  - scripts/c1_bulk_step6_category_promote_scan_soft_delete.py
  - scripts/rewrite_layout_asset_refs_with_compensation.py
  - check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout_transitive_full_dlc_20260415
created_at: 2026-04-21
updated_at: 2026-04-21
maintainer: OpenCode
status: active
doc_class: record
---

# GRScenes Test0 Promoted Clone Design

## Goal

Produce a true duplicate-removed promoted clone of
`GRScenes-test0-rebuilt-normalize-prededup-transitive-rerun-20260413` under
`/cpfs/user/zhuzihou/` without mutating the shared repo-side dataset root.

This design targets a clone whose `layout.usd` files have accumulated category
rewrites through real Step 6 promotion and whose old duplicate assets have been
soft-deleted into a colocated backup tree.

## Current Context

The existing transitive full rerun established valid dry-run certification and
mapping evidence, but it did not produce a true promoted clone.

Confirmed constraints:

- `01_cert` artifacts are reusable inputs
- `02_apply` dry-run suffixed layouts are not cumulative final outputs
- `03_audit` and `04_step6` dry-run artifacts are evidence only, not completion
  markers for a promoted clone
- Step 6 must be rerun in `apply` mode so each category promotes back into the
  clone's `layout.usd`

## Product Decision

Create a new user-side clean workspace and run a fresh promoted-clone pass
there.

Recommended root pattern:

- `/cpfs/user/zhuzihou/dedup_workspaces/test0_transitive_apply_<stamp>/`

Fallback root pattern when `/cpfs/user/zhuzihou` hits an effective writable
space or quota blocker:

- `/shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_<stamp>/`

This workspace keeps the clone, backup tree, and fresh autorun artifacts
together while leaving the shared repo-side dataset untouched.

## Workspace Layout

The workspace should be structured as:

```text
/cpfs/user/zhuzihou/dedup_workspaces/test0_transitive_apply_<stamp>/
  dataset/
    GRScenes100/
    GRScenes_assets/
    Material/
  bak/
    _dedup_assets/
  c1_bulk/
    <category>_bbox_primary_rmse_observe_v1/
      01_cert/
      02_apply/
      03_audit/
      04_step6/
    _autorun/
  notes/
```

Directory semantics:

- `dataset/` is a full clone of
  `GRScenes-test0-rebuilt-normalize-prededup-transitive-rerun-20260413`
- `bak/` receives old assets moved by Step 6 soft-delete
- `c1_bulk/` is a fresh workspace for this promoted-clone run
- `notes/` is optional scratch space for run summaries or handoff notes

## Reuse Boundary

Only the old `01_cert` layer should be seeded into the new workspace.

Seed from:

- `check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout_transitive_full_dlc_20260415/*/01_cert/`

Do not seed from the old run:

- `02_apply/`
- `03_audit/`
- `04_step6/`
- suffixed scene outputs inside the shared rerun dataset root

Rationale:

- `filtered_mapping.json` stores dataset-relative `GRScenes_assets/...` paths,
  so apply can resolve them under the new clone root
- old dry-run apply outputs were overwritten across categories because they
  shared one suffixed layout filename per scene
- old audit and Step 6 records describe dry-run state in the old workspace and
  cannot certify completion for the new clone

## Execution Flow

### 1. Create the clone workspace

- create the new user-side workspace root
- copy the full source dataset into `dataset/`
- create an empty `bak/`
- create a fresh `c1_bulk/`

### 2. Seed reusable cert artifacts

- copy each category's old `01_cert/` directory into the fresh `c1_bulk/`
- preserve `filtered_mapping.json`, `filtered_mapping.stats.json`,
  `pair_certificates.jsonl`, and `certified_graph.json`

### 3. Run bbox autorun on the clone

Run `scripts/c1_autorun_categories.py` against the clone with:

- `--bbox-gated`
- `--bbox-policy bbox_primary_rmse_observe`
- `--dedup-mode geom_only`
- `--v-matrix-mode auto`
- `--scene-files layout.usd`
- `--step6-mode apply`

The run should point `--dataset-root`, `--bak-root`, and `--c1-bulk-dir` at
the new user-side workspace.

### 4. Category accumulation contract

For each category, the expected sequence is:

1. load seeded `01_cert/filtered_mapping.json`
2. rerun `02_apply` against the clone's current `layout.usd`
3. rerun `03_audit` on the clone-local apply result
4. rerun `04_step6` in `apply` mode so the suffixed output is promoted back to
   the clone's `layout.usd`
5. soft-delete old assets for that category into `bak/_dedup_assets/...`

This is the mechanism that makes category rewrites cumulative in the clone.

## Runtime Strategy

Start locally rather than on DLC.

Why local first:

- the expensive transitive certification work already exists in the seeded
  `01_cert` artifacts
- the remaining work is apply, audit, promote, and soft-delete on the clone
- local execution avoids another full remote rerun unless the local Isaac Sim
  environment proves unusable or too slow

DLC becomes a fallback only if:

- local Isaac Sim is unavailable
- the local pass is operationally unstable
- throughput is clearly too low for the remaining apply/audit workload

Workspace-root fallback:

- if `/cpfs/user/zhuzihou` cannot hold a full writable clone because of an
  effective quota or writable-space limit, move the workspace root to
  `/shared/smartbot/zhuzihou/assets/dedup_workspaces/` before considering DLC

## Resume And Failure Policy

Use stop-on-first-failure behavior.

Resume must be conservative because bbox autorun's skip-done behavior is tied to
audit outputs rather than verified Step 6 completion.

Policy:

- run with a fresh workspace for the initial pass
- if a category fails, stop and inspect that category before continuing
- use the autorun ledger plus Step 6 reports to decide what is actually done
- do not blindly trust prior dry-run `audit_verdict.json` files as completion
  markers

If resume is needed, rerun only after confirming whether the last attempted
category completed Step 6 apply.

## Verification Requirements

The promoted clone is acceptable only if all of the following hold.

### 1. Per-category Step 6 completion

For each processed category, `04_step6/` must include:

- `promote_to_layout_usd_report.json`
- `post_promote_full_usd_scan_excluding_backups_pxr.json` with `hit_files == 0`
- `soft_delete_old_assets_report.json`
- `post_soft_delete_layout_scan_pxr.json` with `hit_layouts == 0`

### 2. Clone mutation verification

- the clone's `layout.usd` files must be the promoted outputs, not the original
  shared-root copies
- `GRScenes_assets/` count in the clone should drop materially relative to the
  source dataset
- moved old assets must appear under `bak/_dedup_assets/...`

### 3. Spot checks on known categories

Sample categories such as `book`, `bottle`, and `window` should be checked to
confirm:

- old refs are no longer present in the clone's `layout.usd`
- canonical refs now resolve inside the clone's `GRScenes_assets/`
- expected old assets were moved into the backup tree

### 4. Readiness for later export

Only after the promoted clone passes the checks above should it be used as the
source for a later slim delivery export.

## Non-Goals

This design does not:

- mutate the shared repo-side dataset root in place
- reuse old `02_apply~04_step6` artifacts as final state
- promise zero dangling references in the clone
- define the later slim delivery export in detail
- require DLC unless local execution proves unsuitable

## Bottom Line

The correct next step is not to reuse the old dry-run suffixed layouts as if
they were a promoted baseline. The correct next step is to create a user-side
clean workspace, seed only the reusable `01_cert` layer, and rerun
`02_apply -> 03_audit -> 04_step6 apply` so the clone's `layout.usd` files and
asset tree become a true duplicate-removed promoted clone.
