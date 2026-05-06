---
title: Promoted Clone Workspace Preparation
code_reference:
  - scripts/prepare_promoted_clone_workspace.py
  - tests/test_prepare_promoted_clone_workspace.py
  - docs/superpowers/plans/2026-04-21-test0-promoted-clone.md
created_at: 2026-04-21
updated_at: 2026-04-28
maintainer: OpenCode
status: active
doc_class: record
---

# Promoted Clone Workspace Preparation

## Summary

Implemented the first execution chunk for the GRScenes-test0 promoted-clone helper.

Added `scripts/prepare_promoted_clone_workspace.py` to:

- clone a source dataset into `workspace_root/dataset`
- create `bak/`, `c1_bulk/`, and `notes/`
- seed only reusable `01_cert/` artifacts from prior bbox-gated category roots
- write `workspace_manifest.json`
- write `run_promoted_clone.sh` with the local `c1_autorun_categories.py --step6-mode apply` wrapper command

Follow-up hardening kept the same workflow but tightened correctness:

- the emitted autorun wrapper now uses the caller-provided `policy_tag`
- the wrapper includes `--out-version` when a non-default version is requested
- incomplete `01_cert` bundles now fail fast instead of being partially reused
- inferred `source_report` and `mode_reports_dir` are validated before workspace creation starts

Resumable-copy hardening kept the workspace contract narrow while addressing the CPFS-scale clone timeout:

- dataset cloning now uses `rsync -a --partial` instead of monolithic `shutil.copytree`
- the helper writes `.dataset_copy_started` before the clone and `.dataset_copy_complete` after success
- retries are accepted only for the recognized interrupted-clone state: the workspace root contains exactly `dataset/` and `.dataset_copy_started`, with no extra root artifacts and no complete marker
- arbitrary pre-existing workspace roots are still rejected

Added `tests/test_prepare_promoted_clone_workspace.py` to cover:

- dataset cloning and workspace directory creation
- seeding only `01_cert/` inputs
- refusal to overwrite an existing workspace root
- emitted autorun command flags for apply mode and `layout.usd`
- non-default policy/version propagation into the run wrapper
- early failure on incomplete cert bundles
- early failure when inferred report inputs are missing
- resumable rsync copy path and copy-state markers
- acceptance of the recognized interrupted-clone state only
- rejection of interrupted-copy roots that contain unrelated extra root artifacts

## TDD Record

Red phase:

```bash
python -m pytest tests/test_prepare_promoted_clone_workspace.py -q
```

- first failed with `ModuleNotFoundError: No module named 'prepare_promoted_clone_workspace'`
- after adding the import stub, failed with `NotImplementedError`

Green phase:

```bash
python -m pytest tests/test_prepare_promoted_clone_workspace.py -q
```

- initial chunk passed with `4 passed`
- hardening follow-up passed with `7 passed`
- resumable-copy follow-up passed with `10 passed`
- resumable-state tightening follow-up passed with `11 passed`

## Notes

- Followed the plan's first execution chunk only
- Kept the implementation minimal and aligned with the command contract expected by `scripts/c1_autorun_categories.py`

## Runtime Progress

Real promoted-clone workspace creation was attempted first under:

- `/cpfs/user/zhuzihou/dedup_workspaces/test0_transitive_apply_20260421_093709`

Observed result:

- dataset clone reached a partial state and then failed with `No space left on
  device (28)` during `rsync`
- mount-wide `df -h` still showed free capacity, so this was treated as an
  effective user-side writable-space or quota blocker rather than a repository
  logic problem

Fallback decision:

- move the real promoted-clone workspace root to
  `/shared/smartbot/zhuzihou/assets/dedup_workspaces/`

Successful workspace creation root:

- `/shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046`

Observed readiness after clone completion:

- `dataset/`, `bak/`, `c1_bulk/`, `notes/`, `workspace_manifest.json`, and
  `run_promoted_clone.sh` all exist
- `.dataset_copy_complete` exists
- `seeded_category_count = 83`

Promoted-clone autorun was then launched from:

- `/shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/run_promoted_clone.sh`

Current runtime tracking:

- PID: `418214`
- log: `/shared/smartbot/zhuzihou/assets/.opencode_logs/test0_transitive_apply_20260421_103046_run.log`
- observed startup state: initial `mapping_pairs=0` categories skipped cleanly,
  then `basket` entered `02_apply`

Later runtime correction:

- the first long promoted-clone autorun stopped at `bed` Step 6 because one old
  bed asset remained referenced after promote
- root cause was fixed in the bbox-gated rewrite layer by enabling certified
  `transitive` rewrites during `02_apply`
- a bed-only rerun then completed successfully with:
  - `category_done` in
    `c1_bulk/_autorun/test0_transitive_apply_seeded_bbox_primary_rmse_observe_20260422_044747/ledger.jsonl`
  - `hit_files = 0` in `bed/04_step6/post_promote_full_usd_scan_excluding_backups_pxr.json`
  - `hit_layouts = 0` in `bed/04_step6/post_soft_delete_layout_scan_pxr.json`

Resume state after the bed fix:

- the main promoted-clone autorun was restarted with default `skip_done`
- new runtime log:
  `/shared/smartbot/zhuzihou/assets/.opencode_logs/test0_transitive_apply_20260421_103046_resume.log`
- new autorun dir:
  `c1_bulk/_autorun/test0_transitive_apply_seeded_bbox_primary_rmse_observe_20260422_112450`
- verified startup behavior:
  - `basket` and `bed` are no longer in the category list because their bbox
    category roots now satisfy the repo's current done predicate
  - the resumed run continues from later categories, starting with quick
    `mapping_pairs_0` skips such as `Musical_instrument`, `backpack`,
    `bathtub`, and `bicycle`, then advancing onward

Subsequent runtime note:

- the resumed autorun later stopped at `blanket` with
  `category_fail step=step6 rc=1`
- this was not a new blanket mapping bug; the sole residual hit in
  `blanket/04_step6/post_promote_full_usd_scan_excluding_backups_pxr.json`
  pointed at:
  - `.../MVUHLWYKTKJ5EAABAAAAACQ8_usd/layout.bed_transitive_fix_probe.usd`
- that file was a manual bed-debug probe artifact created during targeted
  verification and unintentionally left under the dataset tree, so blanket's
  full-tree Step 6 scan correctly treated it as a live USD file
- remediation taken:
  - deleted `layout.bed_transitive_fix_probe.usd`
  - restarted `blanket` Step 6 directly using the existing passed audit verdict
- current blanket rerun log:
  `/shared/smartbot/zhuzihou/assets/.opencode_logs/test0_transitive_apply_20260421_103046_blanket_step6_rerun.log`

Fresh verification after the blanket rerun:

- `blanket/04_step6/post_promote_full_usd_scan_excluding_backups_pxr.json`
  now reports `hit_files = 0`
- `blanket/04_step6/post_soft_delete_layout_scan_pxr.json` now reports
  `hit_layouts = 0`
- `blanket/04_step6/soft_delete_old_assets_report.json` exists
- the blanket-specific rerun process exited and the category is now complete

Autorun skip semantics follow-up:

- the repo-side bbox autorun logic previously treated a category as done when
  `03_audit/audit_verdict.json` said `passed`, even if `04_step6` later failed
- this was corrected so `step6_mode=apply` now requires `04_step6` to satisfy
  the same completion contract as `_is_step6_complete()` before a category is
  skipped on resume
- after this fix, the resumed main autorun started a new run dir at
  `c1_bulk/_autorun/test0_transitive_apply_seeded_bbox_primary_rmse_observe_20260423_091341`
  and correctly excluded already-complete `bed` and `blanket` from its category
  list while continuing from `book` onward

DSW-to-DLC migration note:

- the DSW sequential autorun reached the next safe boundary after `bottle`
  completed with `category_done` at `2026-04-24 05:06:44`
- `bottle/04_step6/post_promote_full_usd_scan_excluding_backups_pxr.json`
  reported `hit_files = 0` over `77041` scanned USD files
- `bottle/04_step6/post_soft_delete_layout_scan_pxr.json` reported
  `hit_layouts = 0` over `99` scanned layouts
- after `bottle`, DSW had just started `bowl` `02_apply`; no `bowl/04_step6`
  artifacts existed, so stopping there is resumable because no `bowl` promote
  had occurred
- local DSW autorun PIDs were terminated with SIGTERM after the `bottle`
  boundary
- a single sequential DLC resume job was submitted after a mock submit and
  explicit confirmation:
  - job name: `test0_transitive_apply_resume_0_1`
  - job id: `dlcs95vua6bok17u`
  - resource template: `DLC_GPU_COUNT=1`, `CPU=14`, `Memory=100Gi`,
    `SharedMem=100Gi`, `Resource=quota1r947pmazvk`
- DLC created a new autorun ledger at
  `c1_bulk/_autorun/test0_transitive_apply_seeded_bbox_primary_rmse_observe_20260424_052851/ledger.jsonl`
  whose category list correctly skips completed `bed`, `blanket`, `book`,
  `bookshelf`, and `bottle`, and resumes at `bowl`

SmartBot-space interruption and user-side migration plan:

- the DLC resume run later advanced through `desk` and then stopped during
  `dishwasher` Step 6 after SmartBot-side storage became unavailable or full
- fresh ledger evidence shows `desk` has `category_done`, while `dishwasher` has
  `category_start` without `category_done`
- `dishwasher/04_step6/progress.json` shows the interrupted phase was the full
  USD scan with `processed = 53000`, `total = 72765`, and `hits = 0`
- final Step 6 reports for `dishwasher` are absent, so the corrected
  `skip_done` predicate will not skip it on resume
- the safe continuation path is to migrate the whole mutable workspace from
  `/shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046`
  to
  `/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046`
  and submit a new single DLC job using the new absolute paths
- this migration was completed on 2026-04-28 with `rsync`; the old SmartBot
  source workspace was removed after a no-difference dry-run verification
- the existing
  `/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418`
  directory is a slim delivery baseline, not a resumable promoted-clone
  workspace; sampled `layout.usd` hashes differ from the mutable workspace and
  current sidecar outputs are absent there
- detailed migration and cleanup notes are recorded in
  `docs/records/research/operations/2026-04-28_promoted_clone_workspace_migration.md`
- the DLC resume job was submitted on 2026-04-29:
  - job name: `test0_transitive_apply_resume_usercpfs_0_1`
  - job id: `dlcxmlbt64kka4rl`
  - workspace: `270969`
  - **status**: `Failed` after ~5 minutes
  - **root cause**: missing data source mount for `/cpfs/user/zhuzihou/`
  - **fix**: add data source `d-f1dsz5nbamclxgydo8` to `--data_sources`
  - second attempt job name: `test0_transitive_apply_resume_usercpfs_v2_0_1`
  - second attempt job id: `dlcoh2xvu5efzkyo`
  - using new user-side workspace paths under `/cpfs/user/zhuzihou/assets/`
  - will skip completed categories through `desk` and resume from `dishwasher`
