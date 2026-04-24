---
title: Promoted Clone Workspace Resumable Copy Re-review
code_reference:
  - scripts/prepare_promoted_clone_workspace.py
  - tests/test_prepare_promoted_clone_workspace.py
  - docs/records/changes/2026-04-21_promoted_clone_workspace.md
created_at: 2026-04-21
updated_at: 2026-04-21
maintainer: OpenCode
status: active
doc_class: record
---

# Promoted Clone Workspace Resumable Copy Re-review

## Scope

Reviewed the resumable-copy hardening for correctness, operational safety on the intended CPFS rollout path, and whether the recognized resume state is narrow enough.

## Verification

Command run:

```bash
python -m pytest tests/test_prepare_promoted_clone_workspace.py -q
```

Result:

- passed with `10 passed in 0.24s`

## Findings

1. `scripts/prepare_promoted_clone_workspace.py:67-81`

The recognized resumable state is broader than the doc claims. `_is_recognized_resumable_workspace()` only requires `workspace_root/`, `dataset/`, and `.dataset_copy_started` to exist, and forbids a short allowlist of later-stage outputs. It does not reject unexpected extra files or directories already present in `workspace_root`, and it does not require the root contents to match a strict interrupted-copy shape. In practice, any pre-existing tree can become resumable if someone drops the started marker into it, which is wider than the advertised "recognized interrupted-clone state only" contract.

2. `tests/test_prepare_promoted_clone_workspace.py:281-331`

The resumable tests cover the accepted happy-path partial clone and one rejected unrecognized state, but they do not assert rejection when unrelated extra root-level artifacts are present alongside `dataset/` and `.dataset_copy_started`. That leaves the state-machine boundary above untested.

## Decision

- changes required
