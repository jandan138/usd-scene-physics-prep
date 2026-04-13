---
title: "Task 2 transitive bbox gating code re-review"
code_reference:
  - scripts/c1_build_bulk_mapping_from_dedup_report.py
  - tests/test_bbox_gated_mapping.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Task 2 transitive bbox gating code re-review

## Scope

- Reviewed worktree: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.worktrees/feat-transitive-bbox-gating`
- Reviewed files:
  - `scripts/c1_build_bulk_mapping_from_dedup_report.py`
  - `tests/test_bbox_gated_mapping.py`
- Re-review target: confirm the prior `mode_index` fallback bug and weak transitive tests were fixed, then identify any remaining actionable issues.

## Findings

- Prior issue resolved: `scripts/c1_build_bulk_mapping_from_dedup_report.py:386-393` now keys pair-mode routing off `args.mode_reports_dir` instead of `mode_index` truthiness, so an empty index still routes unresolved pairs to `transitive`.
- Prior issue resolved: `tests/test_bbox_gated_mapping.py:207-342` now exercises the real transitive certificate path through `build_transitive_pair_certificate()` by stubbing `find_transitive_V()` and verifying `group_members`, empty `mode_index`, and `return_witness=True` inputs.
- Remaining issue: `tests/test_bbox_gated_mapping.py` still could not be executed in the current environment because importing `scripts/c1_build_bulk_mapping_from_dedup_report.py` requires `ijson`, which is missing here. The command failed during collection with `ModuleNotFoundError: No module named 'ijson'`.

## Verification

- Ran:

```bash
python -m pytest tests/test_bbox_gated_mapping.py
```

- Result: collection failed before running tests due to missing `ijson`.

## Decision

- Previous quality issues are fixed.
- Remaining actionable issue is test collectability in environments without `ijson`.
