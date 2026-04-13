---
title: "Task 1 transitive bbox spec compliance review"
code_reference:
  - scripts/compute_vertex_transform.py
  - tests/test_compute_vertex_transform.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Task 1 transitive bbox spec compliance review

## Scope

Reviewed only Task 1 in worktree
`/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.worktrees/feat-transitive-bbox-gating`.

Scoped files:

- `scripts/compute_vertex_transform.py`
- `tests/test_compute_vertex_transform.py`

Ignored by request:

- `docs/INDEX.md`
- doc note files unless they changed behavior

## Verification

Focused Task 1 tests:

```bash
python -m pytest tests/test_compute_vertex_transform.py -k 'witness_path_prefers_lower_risk_shortest_path or transitive_certificate_records_witness_metadata or transitive_certificate_fail_closes_when_endpoint_bbox_precheck_fails'
```

Result:

- `3 passed, 17 deselected`

Broader file run:

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

- Deterministic shortest-path witness selection is implemented.
- Shared endpoint evaluation helper `_evaluate_certificate_with_V(...)` is implemented and used.
- `build_transitive_pair_certificate(...)` is implemented.
- `find_transitive_V(...)` remains available and now reuses the witness-path helper.
- Required Task 1 tests are present and pass.

Spec gaps found:

- Transitive certificate metadata does not use the approved `transitive_*` field family.
  Current fields are `witness_path_uids`, `witness_path_modes`, and `witness_path_length`.
- The certificate does not expose endpoint-verification semantics under `transitive_*`
  metadata fields, even though endpoint verification is performed directly.
- Witness metadata from `_witness_metadata_from_path(...)` includes `usd_paths`, but the
  transitive certificate does not persist that witness-path path metadata.

## Decision

Task 1 is not fully spec compliant in its current worktree state because the transitive
certificate metadata shape does not match the approved design.
