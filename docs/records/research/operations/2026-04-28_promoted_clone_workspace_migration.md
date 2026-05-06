---
title: Promoted Clone Workspace Migration and DLC Resume Plan
code_reference:
  - scripts/c1_autorun_categories.py
  - scripts/c1_bulk_step6_category_promote_scan_soft_delete.py
  - scripts/dlc/launch_job.sh
  - scripts/dlc/run_task.sh
  - /shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/workspace_manifest.json
  - /shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/c1_bulk/_autorun/test0_transitive_apply_seeded_bbox_primary_rmse_observe_20260424_052851/ledger.jsonl
created_at: 2026-04-28
updated_at: 2026-04-28
maintainer: OpenCode
status: active
doc_class: record
---

# Promoted Clone Workspace Migration and DLC Resume Plan

## Summary

The promoted-clone apply run was interrupted after the SmartBot-side workspace hit
an effective space limit. The safe recovery path is to migrate the whole mutable
workspace to a user-side CPFS path, verify that the migrated workspace preserves
the current cumulative state, then submit a new single DLC resume job against the
new absolute paths.

Do not continue from the existing delivery bundle at
`/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418`.
That directory is a slim delivery baseline, not the cumulative promoted-clone
workspace.

## Current Evidence

Current mutable workspace:

- `/shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046`

Fresh observed state before migration planning:

- workspace size observed by `du -sh`: at least `155G` before the command timed out
- `dataset/GRScenes100`, `dataset/GRScenes_assets`, `bak`, and `c1_bulk` are real directories, not symlinks
- non-empty completed categories: `24`
- non-empty remaining categories: `50`
- latest completed category in the active DLC ledger: `desk`
- active interrupted category: `dishwasher`

Interrupted `dishwasher` evidence:

- `02_apply/batch_summary.json` exists
- `03_audit/audit_verdict.json` exists
- `04_step6/progress.json` reports `phase = scan`, `processed = 53000`, `total = 72765`, `hits = 0`
- `04_step6/post_promote_full_usd_scan_excluding_backups_pxr.json` is missing
- `04_step6/post_soft_delete_layout_scan_pxr.json` is missing

Because `scripts/c1_autorun_categories.py` treats `step6_mode=apply` as done
only when both final Step 6 scan reports exist with zero hits, a new autorun will
skip completed categories through `desk` and rerun `dishwasher` from the category
level. It will not resume from the `53000/72765` scan offset.

## Existing Target Baseline Disposition

The existing target directory:

- `/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418`

is not suitable as the resume workspace because:

- it was exported as a slim delivery dataset with `GRScenes100/`, `GRScenes_assets/`, `Material/`, `README.md`, `MANIFEST.json`, and dangling-reference reports
- it has only `83` top-level asset categories under `GRScenes_assets`, while the current mutable workspace has `114`
- sampled `layout.usd` files differ from the current mutable workspace by MD5
- current sidecar apply outputs such as `layout.test0_transitive_apply_seeded_bbox_primary_rmse_observe_v1.usd` are absent there
- it lacks the required mutable workspace structure: `dataset/`, `bak/`, `c1_bulk/`, `workspace_manifest.json`, and current autorun ledgers

Cleanup was approved on 2026-04-28. Before deletion, these provenance files were
archived under
`/cpfs/user/zhuzihou/assets/_archived_metadata/GRScenes-test0-transitive-full-baseline-20260418_20260428/`:

- `README.md` (`630` bytes)
- `MANIFEST.json` (`7704` bytes)
- `dangling_references.json` (`11833` bytes)

The old delivery baseline root was then deleted with a path-guarded cleanup
script. Independent verification after cleanup showed:

- `/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418` no longer exists
- the three archived metadata files exist at the archive path above

## Migration Target

Recommended new mutable workspace root:

```text
/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046
```

Recommended copy command:

```bash
mkdir -p /cpfs/user/zhuzihou/assets/dedup_workspaces
rsync -aH --partial --info=progress2 \
  /shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/ \
  /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/
```

Run this only after confirming the old DLC process is not still writing to the
source workspace.

Actual migration result on 2026-04-28:

- old DLC job `dlcs95vua6bok17u` was confirmed `Failed` in workspace `270969`
- no local `c1_autorun_categories.py` process was writing the workspace
- a cross-path directory `mv` attempt degraded into a partial copy, leaving the
  source intact and a partial target containing only `.dataset_copy_started` and
  `dataset/`
- migration was completed with resumable `rsync -aH --partial --delete` from
  source to target
- `rsync -aHn --delete --itemize-changes` then produced no output, indicating no
  remaining source-to-target differences under the rsync metadata contract
- after target verification, the old SmartBot workspace source directory was
  removed with a path-guarded cleanup script
- independent verification showed the old SmartBot path no longer exists and the
  user-side target path still contains the expected workspace

## Post-Copy Verification

Minimum checks before submitting DLC from the new path:

```bash
test -d /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/dataset/GRScenes100
test -d /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/dataset/GRScenes_assets
test -d /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/bak
test -d /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/c1_bulk
test -f /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/workspace_manifest.json
```

Then verify the migrated `c1_bulk` completion state still reports `desk` as done
and `dishwasher` as incomplete by reading:

- `c1_bulk/desk_bbox_primary_rmse_observe_v1/04_step6/post_promote_full_usd_scan_excluding_backups_pxr.json`
- `c1_bulk/desk_bbox_primary_rmse_observe_v1/04_step6/post_soft_delete_layout_scan_pxr.json`
- `c1_bulk/dishwasher_bbox_primary_rmse_observe_v1/04_step6/progress.json`

## DLC Resume Command Contract

The resume job must use the new absolute paths and must be submitted only after a
mock submit is shown and explicitly approved.

### Submission Attempt 1 (2026-04-29) — Failed

- **Job Name**: `test0_transitive_apply_resume_usercpfs_0_1`
- **Job ID**: `dlcxmlbt64kka4rl`
- **Status**: `Failed` after ~5 minutes
- **Root cause**: `FileNotFoundError: Missing assets dir: /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/dataset/GRScenes_assets`
- **Diagnosis**: The job's `--data_sources` parameter only included the 3 data
  sources for the old `/shared/smartbot/...` path, but not the data source
  `d-f1dsz5nbamclxgydo8` that mounts `/cpfs/user/zhuzihou/` into the DLC
  container. Without this mount, the container could not access the migrated
  workspace.
- **Fix**: Add `d-f1dsz5nbamclxgydo8` to the `--data_sources` parameter.
- **Prevention**: This pitfall has been added to the DLC skill
  `.claude/skills/dlc/SKILL.md` under the "Data Source Path Mounting" section
  to catch similar mistakes in future submissions.

### Submission Attempt 2 (2026-04-29) — Running

- **Job Name**: `test0_transitive_apply_resume_usercpfs_v2_0_1`
- **Job ID**: `dlcoh2xvu5efzkyo`
- **Status**: `Running` (confirmed after ~5 minutes)
- **Data Sources**: `d-mzps5b7joy2axmqpa8,d-d49o5g0h2818sw8j1g,d-8wz4emfs21s5ajs9oz,d-f1dsz5nbamclxgydo8`
- **Fix applied**: Added `d-f1dsz5nbamclxgydo8` to mount `/cpfs/user/zhuzihou/`

The job successfully transitioned from `EnvPreparing` to `Running`, confirming
that the container can now access the migrated workspace at
`/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046`.
It will automatically skip completed categories (`desk` and earlier) and resume
from `dishwasher`.

### Mock Submit Pattern

```bash
DLC_BIN=/bin/echo bash scripts/dlc/launch_job.sh \
  test0_transitive_apply_resume_usercpfs 0 1 \
  d-mzps5b7joy2axmqpa8,d-d49o5g0h2818sw8j1g,d-8wz4emfs21s5ajs9oz \
  "custom /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/c1_autorun_categories.py --dataset-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/dataset --bak-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/bak --report /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/all_categories_union_merged.json --c1-bulk-dir /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/c1_bulk --group-label test0_transitive_apply_seeded --bbox-gated --bbox-policy bbox_primary_rmse_observe --step6-mode apply --dedup-mode geom_only --v-matrix-mode auto --mode-reports-dir /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_rebuilt_dedup/v8_prededup --scene-files layout.usd"
```

Observed mock-submit resource resolution on 2026-04-28:

- `GPU=1`
- `CPU=14`
- `Memory=100Gi`
- `SharedMem=100Gi`
- `Resource=quota1r947pmazvk`
- workspace: `270969`

## Operational Risks

- The copied `run_promoted_clone.sh` still points at the old `/shared/smartbot/...` paths; do not run it directly unless it is rewritten or replaced.
- DLC container access to `/cpfs/user/zhuzihou/assets` still needs live verification by the submitted job; DSW-side access has been verified.
- `dishwasher` will rerun from the category level, so the partial Step 6 scan work is lost.
- Cleaning the delivery baseline before confirming downstream usage can remove a separately packaged snapshot, even though it is not useful for this resume.
