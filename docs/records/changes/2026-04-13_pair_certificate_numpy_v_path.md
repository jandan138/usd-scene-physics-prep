---
title: "Pair Certificate Numpy V Path"
code_reference:
  - scripts/compute_vertex_transform.py
  - tests/test_compute_vertex_transform.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Pair Certificate Numpy V Path

## Summary

`build_pair_certificate()` no longer depends on `pxr` just to evaluate bbox-gated
pair certificates.

The root cause of the remaining baseline failure was that certificate evaluation
called `compute_V_for_pair()`, which eagerly required `pxr/Gf` and therefore
failed closed in the plain Python test environment before the computed `V` was
ever evaluated.

## What Changed

- added `_compute_numpy_V_for_pair(...)` in `scripts/compute_vertex_transform.py`
- `build_pair_certificate()` now uses that pure-numpy helper for
  `topo_filesize` / `shape_invariant` certificate evaluation
- `compute_V_for_pair()` still exists for callers that need `Gf.Matrix4d`, but
  it now converts to pxr only at the outermost boundary

## Why This Is Correct

- certificate evaluation only needs a 4x4 numeric transform and mesh vertices
- it does not need USD/pxr matrix types
- the pxr dependency belongs at the rewrite/apply boundary, not in the pure
  certificate path

## Verification

```bash
python -m pytest tests/test_compute_vertex_transform.py -q
```

Observed result:

- `20 passed`

## Impact

- the previous baseline failure
  `tests/test_compute_vertex_transform.py::TestPairCertificate::test_topo_filesize_with_v_computation`
  is now fixed
- bbox-gated certificate tests can run in the plain Python environment without
  failing on `No module named 'pxr'`
