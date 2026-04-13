---
title: "Task 3 transitive bbox spec compliance rereview"
code_reference:
  - scripts/placement_pairwise_compare.py
  - tests/test_placement_pairwise_compare_bbox_gate.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Task 3 transitive bbox spec compliance rereview

## Scope

- Reviewed worktree: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.worktrees/feat-transitive-bbox-gating`
- Reviewed files:
  - `scripts/placement_pairwise_compare.py`
  - `tests/test_placement_pairwise_compare_bbox_gate.py`

## Findings

- The prior spec gap is resolved in `scripts/placement_pairwise_compare.py:79-100`.
- `_load_certificate_lookup(...)` now filters rows via `_is_certified_certificate_row(...)`, which requires `eligible` truthy and `reject_reason` falsy before any certificate-derived mode enters the audit lookup.
- Certificate-derived mode lookup still correctly takes priority over `mode_index` fallback in `_lookup_prim_dedup_mode(...)`.
- Test coverage now includes rejected-row protections:
  - `test_rejected_certificate_row_does_not_override_certified_row`
  - `test_rejected_transitive_certificate_row_is_ignored_by_audit`
- No remaining spec gaps were found in the scoped files.

## Verification

- Reviewed the scoped diff with:

```bash
git diff -- scripts/placement_pairwise_compare.py tests/test_placement_pairwise_compare_bbox_gate.py
```

- Verified syntax for the scoped files with:

```bash
python -m py_compile scripts/placement_pairwise_compare.py tests/test_placement_pairwise_compare_bbox_gate.py
```

- Ran the scoped test suite with the repository-required Isaac Sim wrapper:

```bash
./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py
```

- Result: `9 passed in 0.38s`.

## Decision

- `SPEC_OK`
