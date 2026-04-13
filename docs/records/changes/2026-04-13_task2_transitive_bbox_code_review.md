---
title: "Task 2 transitive bbox gating code review"
code_reference:
  - scripts/c1_build_bulk_mapping_from_dedup_report.py
  - tests/test_bbox_gated_mapping.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Task 2 transitive bbox gating code review

## Scope

- Reviewed worktree: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.worktrees/feat-transitive-bbox-gating`
- Reviewed files:
  - `scripts/c1_build_bulk_mapping_from_dedup_report.py`
  - `tests/test_bbox_gated_mapping.py`
- Review focus: correctness, mapping/certified-graph regressions, `group_members`/`mode_index` fragility, test quality, and complexity risk for Task 3.

## Findings

1. High: `scripts/c1_build_bulk_mapping_from_dedup_report.py:386-393` only uses per-pair mode lookup when `mode_index` is truthy. If `--mode-reports-dir` is provided but `build_mode_index()` returns `{}` because reports are missing, filtered away, or unreadable for the current category, the code silently falls back to `report_mode` for every pair instead of classifying unresolved pairs as `transitive`. That reintroduces pre-Task-2 direct-mode behavior and can incorrectly admit mappings / certified edges that should have gone through the transitive witness path.

2. Medium: the new transitive tests in `tests/test_bbox_gated_mapping.py:228-237` and `:348-357` stub both `build_mode_index()` and `_uid_from_path()` with synthetic values, then stub `build_transitive_pair_certificate()` itself. They only prove branch dispatch, not the real integration between UID extraction, `mode_index` lookup, and `group_members`-driven witness construction. This leaves the empty-index regression above uncovered and would also miss real lookup bugs caused by production path parsing.

## Test Attempt

- `python -m pytest tests/test_bbox_gated_mapping.py`
- Result: failed during collection because `ijson` is not installed in the current environment (`ModuleNotFoundError: No module named 'ijson'`).

## Decision

- Review outcome: issues found.
