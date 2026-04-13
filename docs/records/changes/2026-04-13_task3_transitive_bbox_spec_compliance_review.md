---
title: "Task 3 transitive bbox spec compliance review"
code_reference:
  - scripts/placement_pairwise_compare.py
  - tests/test_placement_pairwise_compare_bbox_gate.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Task 3 transitive bbox spec compliance review

## Scope

- Reviewed worktree: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.worktrees/feat-transitive-bbox-gating`
- Reviewed files:
  - `scripts/placement_pairwise_compare.py`
  - `tests/test_placement_pairwise_compare_bbox_gate.py`
- Ignored docs-only changes unless they affected behavior.

## Findings

- `scripts/placement_pairwise_compare.py:95-108` loads certificate semantics for any JSONL row with a derivable mode and valid asset paths, but does not require `eligible == True` and does not reject rows with a non-null `reject_reason`.
- This violates the Task 3 design constraint that audit semantics apply only to certified / eligible certificate rows, not rejected ones.
- The risk is concrete for rejected transitive certificates because emitted rows can still carry `transitive_witness_modes`, so `_certificate_mode_from_row(...)` can derive a tier-2 mode from a rejected row and `_lookup_prim_dedup_mode(...)` will prefer it over the `mode_index` fallback.
- The required positive-path coverage is otherwise present:
  - certificate lookup is preferred over `mode_index`
  - a certified transitive pair under observe policy soft-fails at tier 2
  - without certificate lookup, legacy strict behavior remains
- No commit was made during this review.

## Verification

- Reviewed the scoped diff with:

```bash
git diff -- scripts/placement_pairwise_compare.py tests/test_placement_pairwise_compare_bbox_gate.py
```

- Confirmed current modified-file set with:

```bash
git status --short
```

- Verified syntax for the scoped files with:

```bash
python -m py_compile scripts/placement_pairwise_compare.py tests/test_placement_pairwise_compare_bbox_gate.py
```

- Direct pytest execution is not a valid verifier here because importing `placement_pairwise_compare.py` exits when `pxr` is unavailable:

```bash
python -m pytest tests/test_placement_pairwise_compare_bbox_gate.py
```

- Ran the scoped test suite with the repository-required Isaac Sim wrapper:

```bash
./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py
```

- Result: `7 passed in 0.34s`.

## Decision

- `SPEC_ISSUES`
