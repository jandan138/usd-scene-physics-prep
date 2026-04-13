---
title: "Task 3 transitive bbox gating code rereview"
code_reference:
  - scripts/placement_pairwise_compare.py
  - tests/test_placement_pairwise_compare_bbox_gate.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Task 3 transitive bbox gating code rereview

## Scope

- Reviewed worktree: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.worktrees/feat-transitive-bbox-gating`
- Reviewed files:
  - `scripts/placement_pairwise_compare.py`
  - `tests/test_placement_pairwise_compare_bbox_gate.py`
- Rereview focus: lookup symmetry, certificate-over-`mode_index` precedence, rejected-row handling, direct-mode stability, and regression coverage for the prior findings.

## Findings

- No remaining quality issues found in the scoped changes.
- `scripts/placement_pairwise_compare.py:122-178` now resolves certificate lookup through `_lookup_certificate_mode()`, checking both path and UID pairs in both directions before falling back to `mode_index`.
- `tests/test_placement_pairwise_compare_bbox_gate.py:275-374` now covers reversed lookup order and direct-mode certified rows, closing the prior regression gaps.

## Verification

- `./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py -q`
- Result: `11 passed in 0.37s`

## Decision

- Review outcome: quality ok.
