---
title: Promoted Clone Workspace Copy Blocker Investigation
code_reference:
  - scripts/prepare_promoted_clone_workspace.py
  - tests/test_prepare_promoted_clone_workspace.py
created_at: 2026-04-21
updated_at: 2026-04-21
maintainer: OpenCode
status: active
doc_class: record
---

# Promoted Clone Workspace Copy Blocker Investigation

## Scope

Investigated the real operational blocker in `scripts/prepare_promoted_clone_workspace.py` for the user-side CPFS rollout under `/cpfs/user/zhuzihou/dedup_workspaces/...`, where the initial dataset copy timed out after roughly one hour and left a partial workspace.

## Evidence Gathered

Commands and inspections performed:

```bash
python -m pytest tests/test_prepare_promoted_clone_workspace.py -q
```

Result:

- not rerun in this investigation because the request was diagnostic-only and existing context already reported the suite green

Repository inspection:

- `scripts/prepare_promoted_clone_workspace.py` shows a single `_copytree()` wrapper around `shutil.copytree(..., symlinks=True)`
- `prepare_promoted_clone_workspace()` rejects any pre-existing `workspace_root` before attempting the dataset copy
- `bak/`, `c1_bulk/`, `notes/`, `run_promoted_clone.sh`, and `workspace_manifest.json` are only created after the dataset copy finishes

Workspace inspection:

- partial workspace exists at `/cpfs/user/zhuzihou/dedup_workspaces/test0_transitive_apply_20260421_074220`
- top level contains only `dataset/`
- `dataset/` currently contains `GRScenes_assets/` and `Material/`
- `GRScenes100/` is not yet present in the partial destination, so the copy did not complete

Operational probe:

```bash
du -sh /cpfs/user/zhuzihou/dedup_workspaces/test0_transitive_apply_20260421_074220/dataset
```

Result:

- command itself exceeded a 120 second inspection timeout, which is consistent with a very large destination tree on CPFS

## Root Cause

The blocker is the interaction of two current behaviors in `scripts/prepare_promoted_clone_workspace.py`:

1. `scripts/prepare_promoted_clone_workspace.py:144-145` rejects any existing `workspace_root`.
2. `scripts/prepare_promoted_clone_workspace.py:158-166` performs the entire dataset population as one `shutil.copytree()` call before creating any other workspace artifacts.

In a large CPFS run, an external timeout or operator interruption during `shutil.copytree()` leaves behind a partially populated `workspace_root/dataset/`. On retry, the helper refuses to run because the workspace already exists, so the copy cannot be resumed and the finished portion cannot be reused.

This matches the observed state exactly: only `dataset/` exists, later artifacts were never written, and the helper's current contract provides no recovery path.

## Minimal Safe Change

Recommended helper change:

- replace the dataset copy implementation with a resumable `rsync`-based sync for `dataset/`
- allow retry only for an explicitly incomplete workspace state instead of blanket-refusing any existing root
- gate completion with a marker file written only after the dataset sync succeeds

Minimal behavior sketch:

1. Create `workspace_root` if missing.
2. Treat `workspace_root / ".dataset_copy_in_progress"` or the absence of a completion marker as resumable state.
3. Sync `source_dataset_root/` into `workspace_root/dataset/` using `rsync -a --partial`.
4. After sync success, remove the in-progress marker and write a completion marker such as `.dataset_copy_complete`.
5. Only then create `bak/`, `c1_bulk/`, `notes/`, seed certs, and write the run script and manifest.
6. If `workspace_root` exists with unexpected extra contents and no in-progress marker, still fail fast.

Why this is the smallest safe change:

- it changes only the dataset copy path, which is the actual blocker
- it preserves the existing post-copy workspace layout and seeding logic
- it avoids re-copying already synchronized files on retry
- it distinguishes safe-to-resume partial state from arbitrary pre-existing directories

## Why `dirs_exist_ok=True` Is Not Enough

Using `shutil.copytree(..., dirs_exist_ok=True)` would make the helper re-enter an existing destination, but it would still walk and re-copy the tree through Python file copies. That is not a good fit for the reported CPFS scale and does not provide a strong interrupted-copy contract on its own.

`rsync` is already available in this environment (`/usr/bin/rsync`, version `3.2.7`) and is a better minimal operational primitive for this case.

## Operational Workaround If Code Is Not Changed

For the existing partial workspace, the lowest-risk workaround is:

1. Resume the dataset copy directly with `rsync -a --partial <source_dataset_root>/ /cpfs/user/zhuzihou/dedup_workspaces/test0_transitive_apply_20260421_074220/dataset/`
2. After the dataset is complete, either:
   - finish the remaining helper actions manually once, or
   - move the finished dataset aside, recreate a fresh workspace, and populate it without re-copying from the original source only if a clean operational recipe exists

This workaround unblocks the current one-off run, but it should not be treated as the long-term answer because the helper will still fail on the next interruption.

## Recommendation

- Modify the helper now if this promoted-clone preparation is expected to be retried or reused on large CPFS datasets.
- Use the `rsync` workaround only to salvage the already-partial workspace immediately.

Rationale:

- the failure mode is already proven in a real run
- the fix is narrow and localized to the copy contract
- the current tests do not cover resumable behavior, so the helper is green only for small all-or-nothing fixtures

## Test Gaps To Add With The Fix

Relevant current tests:

- `tests/test_prepare_promoted_clone_workspace.py:116-130` currently locks in unconditional `FileExistsError`
- `tests/test_prepare_promoted_clone_workspace.py:62-113` covers only the small happy path

Additions needed when implementing the fix:

- retry succeeds when `workspace_root` exists with `dataset/` plus an in-progress marker
- retry refuses an existing workspace with unexpected extra files and no resume marker
- resume path writes the completion marker and then the manifest/run script
- resumed sync does not destroy already-seeded data outside the copy contract

## Decision

- Recommended path: implement a small resumable copy contract in the helper, using `rsync` plus explicit copy-state markers
- Immediate stopgap: manually resume the current partial dataset with `rsync`
