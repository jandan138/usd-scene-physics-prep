---
title: "Transitive BBox Witness Certificates"
code_reference: scripts/compute_vertex_transform.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
---

# Transitive BBox Witness Certificates

## Summary

Task 1 added deterministic transitive witness-path selection and a shared endpoint
certificate evaluator in `scripts/compute_vertex_transform.py`.

The transitive flow now:
- selects a shortest witness path deterministically
- prefers lower-risk mode chains among equal-length paths
- composes witness-path `V`
- verifies the endpoint pair directly before certifying the transitive pair

## Code Changes

### Witness selection

- Added `_TRANSITIVE_WITNESS_MODE_RISK` to rank direct modes for witness selection.
- Added `_sorted_graph_neighbors`, `_path_witness_key`, `_witness_metadata_from_path`,
  and `_find_transitive_witness_path`.
- `find_transitive_V(...)` remains available, but now reuses the witness-path helper and
  can optionally return witness metadata.

### Shared endpoint evaluation

- Added `_compute_edge_V(...)` so direct edge evaluation is reused when composing a
  witness-path transform.
- Added `_evaluate_certificate_with_V(...)` to evaluate a provided `V` against the
  endpoint pair and populate bbox, centroid, and RMSE fields.
- Refactored `build_pair_certificate(...)` to use the shared evaluator for non-identity
  direct modes.

### Transitive certificate builder

- Added `build_transitive_pair_certificate(...)`.
- The certificate records transitive witness metadata via:
  - `transitive_witness_uids`
  - `transitive_witness_modes`
  - `transitive_witness_length`
- The certificate records endpoint verification semantics via:
  - `transitive_endpoint_pair_directly_verified`
  - `transitive_endpoint_verification_kind`
  - `transitive_endpoint_verification_passed`
- The transitive builder fail-closes if the endpoint bbox precheck rejects the composed
  witness transform.

## Review Follow-Up

Task 1 review feedback required the transitive certificate schema to use the approved
`transitive_*` field family instead of the earlier `witness_path_*` names.

Follow-up changes:
- renamed the stored witness metadata to the approved `transitive_*` names
- added explicit endpoint-verification semantics for the composed endpoint check
- updated Task 1 tests to assert the approved contract only

## Tests

Added targeted tests in `tests/test_compute_vertex_transform.py` for:
- lower-risk shortest witness preference
- witness metadata recording on transitive certificates
- fail-closed transitive bbox precheck behavior

## TDD Notes

Red phase command:

```bash
python -m pytest tests/test_compute_vertex_transform.py -k 'witness_path_prefers_lower_risk_shortest_path or transitive_certificate_records_witness_metadata or transitive_certificate_fail_closes_when_endpoint_bbox_precheck_fails'
```

Observed red reason:
- test collection failed because `build_transitive_pair_certificate` did not exist yet

Green phase command:

```bash
python -m pytest tests/test_compute_vertex_transform.py -k 'witness_path_prefers_lower_risk_shortest_path or transitive_certificate_records_witness_metadata or transitive_certificate_fail_closes_when_endpoint_bbox_precheck_fails'
```

Green result:
- `3 passed, 17 deselected`

Broader verification:

```bash
python -m pytest tests/test_compute_vertex_transform.py
```

Result:
- `19 passed, 1 failed`
- The remaining failure is the known baseline failure:
  `tests/test_compute_vertex_transform.py::TestPairCertificate::test_topo_filesize_with_v_computation`

Review follow-up red/green commands:

```bash
python -m pytest tests/test_compute_vertex_transform.py -k 'transitive_certificate_records_witness_metadata or transitive_certificate_fail_closes_when_endpoint_bbox_precheck_fails'
```

Red result:
- `2 failed, 18 deselected`
- failure reason: missing approved `transitive_*` fields on the transitive certificate

Green result after the schema fix:
- `2 passed, 18 deselected`

## Concerns

- `find_transitive_V(...)` now supports `return_witness=True` for the transitive
  certificate flow. Existing callers keep the original matrix-only behavior.
