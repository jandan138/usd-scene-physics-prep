---
title: Promoted Clone Workspace Helper Re-review
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

# Promoted Clone Workspace Helper Re-review

## Scope

Re-reviewed the promoted-clone workspace helper after the hardening follow-up, focusing on the four previously reported issues: policy/version propagation, incomplete cert bundle rejection, early inferred-input validation, and test coverage.

## Verification

Commands run:

```bash
python -m pytest tests/test_prepare_promoted_clone_workspace.py -q
```

```bash
grep -nE -- '--out-version|bbox-policy|out_version' scripts/c1_autorun_categories.py
```

Results:

- focused test suite passed with `7 passed in 0.05s`
- downstream autorun CLI still defines and uses `--out-version` with bbox-gated category roots keyed by `{category}_{policy}_{out_version}`

## Review Outcome

- approved

## Notes

- The helper now threads `policy_tag` into the wrapper command and emits `--out-version` for non-default versions.
- Seedability now requires the full `01_cert` bundle and fails before workspace creation on partial bundles.
- Inferred `source_report` and `mode_reports_dir` are validated before any copy begins.
- Tests now cover each of those hardened behaviors directly.

## Residual Risks

- The helper still does a full `shutil.copytree` of the dataset, so real CPFS runs remain sensitive to available quota, copy duration, and interruption mid-copy.
- Validation checks presence and type for the report inputs, but not semantic correctness of their contents; stale-but-existing upstream reports would still pass preparation and fail later during autorun if inconsistent.
