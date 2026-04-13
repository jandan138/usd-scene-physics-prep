---
title: "Task 4 transitive bbox gating code review"
code_reference:
  - scripts/c1_autorun_categories.py
  - tests/test_c1_autorun_categories.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Task 4 transitive bbox gating code review

## Scope

- Reviewed worktree: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.worktrees/feat-transitive-bbox-gating`
- Reviewed files:
  - `scripts/c1_autorun_categories.py`
  - `tests/test_c1_autorun_categories.py`
- Review focus: extracted audit command builder correctness, preservation of existing audit arguments, helper minimality/testability, and regression-test strength for the wiring change.

## Findings

1. Medium: `tests/test_c1_autorun_categories.py:25-56` only asserts the newly added `--certificate-jsonl` and optional `--mode-reports-dir` flags. It does not lock down the pre-existing audit wiring (`--right-layout-name`, `--label`, `--out`, `--verdict-out`, `--scene-list-json`, `--bbox-policy`, tolerance flags, `--allow-no-mesh`, or the left/right roots and modes), so a future edit could accidentally drop or rewrite existing audit arguments without failing this test. For an extraction whose main risk is argument drift, the regression should assert the full command shape or at least every previously existing audit flag/value pair.

## Verification

- `python -m pytest tests/test_c1_autorun_categories.py -q`
- Result: `1 passed in 0.03s`
- `python scripts/doc_manager.py --find-refs scripts/c1_autorun_categories.py`
- Result: existing references include `docs/records/changes/2026-04-13_task4_transitive_bbox_gating.md` and related task records.

## Decision

- Review outcome: issues found.
