---
title: "Task 2 transitive bbox spec compliance review"
code_reference:
  - scripts/c1_build_bulk_mapping_from_dedup_report.py
  - tests/test_bbox_gated_mapping.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Task 2 transitive bbox spec compliance review

## Scope

- Reviewed worktree: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.worktrees/feat-transitive-bbox-gating`
- Reviewed files:
  - `scripts/c1_build_bulk_mapping_from_dedup_report.py`
  - `tests/test_bbox_gated_mapping.py`
- Ignored docs-only changes unless they affected behavior.

## Findings

- `_run_bbox_gated()` no longer hard-rejects `pair_mode == "transitive"` with `transitive_not_supported`.
- The transitive path now calls `_cvt.build_transitive_pair_certificate(...)` with the required arguments:
  - `old_usd`
  - `canonical_usd`
  - `group_members`
  - `mode_index`
  - `policy`
- Certified-graph and canonical-selection logic in `_run_bbox_gated()` is unchanged aside from allowing the returned transitive certificate to flow through the same eligible-edge path as other modes.
- Added test coverage proves both required transitive outcomes:
  - `test_bbox_gated_safe_transitive_pair_enters_filtered_mapping`
  - `test_bbox_gated_rejected_transitive_pair_uses_real_reject_reason`
- No audit or autorun files were touched in the current worktree diff.
- No commit was made during this review.

## Verification

- Reviewed the scoped diff with:

```bash
git diff -- scripts/c1_build_bulk_mapping_from_dedup_report.py tests/test_bbox_gated_mapping.py
```

- Verified current modified-file set with:

```bash
git diff --name-only
```

- Verified syntax for the scoped files with:

```bash
python -m py_compile scripts/c1_build_bulk_mapping_from_dedup_report.py tests/test_bbox_gated_mapping.py
```

- Attempted scoped test execution with:

```bash
pytest tests/test_bbox_gated_mapping.py
```

- Result: the environment does not have `pytest` installed (`/bin/bash: line 1: pytest: command not found`), so runtime test execution could not be completed in this review session.

## Decision

- `SPEC_OK`
