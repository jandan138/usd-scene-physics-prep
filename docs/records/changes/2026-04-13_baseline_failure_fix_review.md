---
title: "Baseline Failure Fix Review"
code_reference:
  - scripts/compute_vertex_transform.py
  - tests/test_compute_vertex_transform.py
  - docs/records/changes/2026-04-13_pair_certificate_numpy_v_path.md
  - docs/records/changes/2026-04-10_transitive_bbox_gating.md
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Baseline Failure Fix Review

## Scope

Reviewed the baseline-failure fix in `feat-transitive-bbox-gating` with focus on:

- `scripts/compute_vertex_transform.py`
- `tests/test_compute_vertex_transform.py`
- `docs/records/changes/2026-04-13_pair_certificate_numpy_v_path.md`
- `docs/records/changes/2026-04-10_transitive_bbox_gating.md`

## Findings

- no actionable bugs or regressions found in the targeted numpy `V` path split
- `build_pair_certificate()` now evaluates candidate pairs through `_compute_numpy_V_for_pair(...)` instead of routing through `compute_V_for_pair()` and its `Gf.Matrix4d` conversion path
- `compute_V_for_pair()` remains available for rewrite-time callers and still converts the computed numpy matrix to `Gf.Matrix4d` at its API boundary
- the baseline test `TestPairCertificate::test_topo_filesize_with_v_computation` now exercises the pure-numpy path and is aligned with the intended fix

## Evidence Reviewed

- targeted diff for the four requested files
- targeted unit coverage in `tests/test_compute_vertex_transform.py`
- controller-provided verification evidence:
  - `python -m pytest tests/test_compute_vertex_transform.py tests/test_bbox_gated_mapping.py tests/test_c1_autorun_categories.py -q` -> `25 passed`
  - `./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py -q` -> `11 passed`
  - `python scripts/doc_manager.py --validate && python scripts/doc_manager.py --gen-index` -> success

## Residual Risk

- `build_pair_certificate()` still depends on USD vertex extraction for real asset files, so plain-Python execution without `pxr` remains limited to test/mocked extraction scenarios; this is consistent with the stated intent because the removed dependency is the unnecessary `Gf` conversion in the `V` evaluation path, not USD mesh loading itself
