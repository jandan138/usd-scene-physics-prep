---
title: "Task 3 transitive bbox gating code review"
code_reference:
  - scripts/placement_pairwise_compare.py
  - tests/test_placement_pairwise_compare_bbox_gate.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Task 3 transitive bbox gating code review

## Scope

- Reviewed worktree: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.worktrees/feat-transitive-bbox-gating`
- Reviewed files:
  - `scripts/placement_pairwise_compare.py`
  - `tests/test_placement_pairwise_compare_bbox_gate.py`
- Review focus: certificate lookup loading, mode resolution fallback order, rejected-row handling, direct-mode stability, and test coverage strength.

## Findings

1. High: `scripts/placement_pairwise_compare.py:133-142` only checks certificate entries in the `(canonical, old)` direction derived from `right_refs[0]` and `left_refs[0]`. The CLI/tool itself is direction-agnostic, but the fallback `mode_index` lookup at `:145-149` is symmetric. If the comparison is run with canonical assets on the left and old assets on the right, certificate lookup is skipped and the code silently falls back to `mode_index`, breaking the intended certificate-over-`mode_index` precedence and potentially choosing the wrong tier (`geom_only`/`topo_filesize`/`shape_invariant`).

2. Medium: `tests/test_placement_pairwise_compare_bbox_gate.py:225-518` only exercises certificate precedence with old-left/canonical-right transitive fixtures. There is no regression covering the reversed lookup direction from finding 1, and no direct-mode certificate case to prove that direct rows stay stable when certificate lookup is enabled. The current 9 passing tests would not catch either class of regression.

## Verification

- `./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py -q`
- Result: `9 passed in 0.34s`
- `python -m pytest tests/test_placement_pairwise_compare_bbox_gate.py -q`
- Result: collection failed outside Isaac Python because `scripts/placement_pairwise_compare.py` exits when `pxr` is unavailable.

## Decision

- Review outcome: issues found.
