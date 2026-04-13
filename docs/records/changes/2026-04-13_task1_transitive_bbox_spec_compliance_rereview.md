---
title: "Task 1 transitive bbox spec compliance re-review"
code_reference:
  - scripts/compute_vertex_transform.py
  - tests/test_compute_vertex_transform.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Task 1 transitive bbox spec compliance re-review

## Scope

Re-reviewed only:

- `scripts/compute_vertex_transform.py`
- `tests/test_compute_vertex_transform.py`

Ignored docs except for documentation bookkeeping, and ignored the known out-of-scope
baseline failure in the final spec verdict.

## Verification

Focused Task 1 checks:

```bash
python -m pytest tests/test_compute_vertex_transform.py -k 'witness_path_prefers_lower_risk_shortest_path or transitive_certificate_records_witness_metadata or transitive_certificate_fail_closes_when_endpoint_bbox_precheck_fails'
```

Result:

- `3 passed, 17 deselected`

Full scoped file run:

```bash
python -m pytest tests/test_compute_vertex_transform.py
```

Result:

- `19 passed, 1 failed`
- Remaining failure is the known out-of-scope baseline failure:
  `tests/test_compute_vertex_transform.py::TestPairCertificate::test_topo_filesize_with_v_computation`

Worktree diff check:

```bash
git diff --name-only
```

Result:

- `docs/INDEX.md`
- `scripts/compute_vertex_transform.py`
- `tests/test_compute_vertex_transform.py`

No mapping, audit, or autorun files were modified.

## Findings

- Deterministic witness-path selection is implemented via `_find_transitive_witness_path(...)`, `_sorted_graph_neighbors(...)`, and `_path_witness_key(...)`.
- Shared endpoint evaluation helper `_evaluate_certificate_with_V(...)` is present and populates bbox, centroid, and RMSE certificate fields.
- `build_transitive_pair_certificate(...)` is present.
- `find_transitive_V(...)` remains available and is implemented through the witness helper path.
- Transitive certificate metadata now uses a `transitive_*` field family.
- Endpoint-verification semantics are recorded under that family using:
  - `transitive_endpoint_pair_directly_verified`
  - `transitive_endpoint_verification_kind`
  - `transitive_endpoint_verification_passed`
- Required Task 1 tests are present and pass.

## Decision

Task 1 is spec compliant in the current worktree state.
