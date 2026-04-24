---
title: Promoted Clone Workspace Final Re-review
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

# Promoted Clone Workspace Final Re-review

## Scope

Final code-quality re-review of the promoted-clone workspace helper after the resumable-state allowlist fix, with emphasis on the narrowness of the resume state machine for the user-side CPFS workspace rollout.

## Verification

Command run:

```bash
python -m pytest tests/test_prepare_promoted_clone_workspace.py -q
```

Result:

- passed with `11 passed in 0.34s`

## Outcome

- approved

## Notes

- The resumable-state recognizer now accepts only a workspace root whose top-level entries are exactly `dataset/` and `.dataset_copy_started`, and still requires the complete marker to be absent.
- The test suite now includes explicit rejection of an interrupted-copy root that contains an unrelated extra root artifact.

## Residual Risks

- The real CPFS run still depends on `rsync` being present in the execution environment; if it is missing, preparation fails at copy start.
- An interrupted `rsync` can still leave a partial dataset tree by design; the helper now constrains when resume is allowed, but it does not independently validate dataset completeness before later stages.
