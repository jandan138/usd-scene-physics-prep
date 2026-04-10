---
title: "Bbox-gated multi-mode dedup: Changes 2, 3, 6"
code_reference: scripts/compute_vertex_transform.py
created_at: 2026-04-01
updated_at: 2026-04-01
maintainer: TraeAI
status: implemented
---

# Bbox-gated multi-mode dedup: Changes 2, 3, 6

Implements three changes from the bbox-gated multi-mode dedup plan in
`scripts/compute_vertex_transform.py`.

## Change 2: Expand BBOX_GATED_ALLOWED_MODES (line 70)

**Before**: `BBOX_GATED_ALLOWED_MODES = ("geom_only",)`
**After**: `BBOX_GATED_ALLOWED_MODES = ("geom_only", "topo_filesize", "shape_invariant")`

This allows `topo_filesize` and `shape_invariant` pairs to pass the allowed-modes
gate in `build_pair_certificate`.

## Change 3: Remove hardcoded cert gate + mode-aware proof fields

### 3a: Removed hardcoded `mode != "geom_only"` rejection (was lines 853-856)

The block:
```python
if mode != "geom_only":
    cert["reject_reason"] = f"unsupported_certificate_mode_{mode}"
    cert["rmse_unavailable_reason"] = f"unsupported_certificate_mode_{mode}"
    return cert
```
was removed. This block made the `allowed_modes` parameter ineffective by
rejecting all non-geom_only modes regardless of the parameter value.

### 3b: Updated docstring

The docstring now documents all three eligible modes (geom_only, topo_filesize,
shape_invariant) instead of claiming only geom_only is supported.

### 3c: Mode-aware proof fields

Proof fields (`alternate_proof_kind`, `proof_source`) are now mode-dependent:
- `geom_only` -> `"geom_only_exact_world_proof"` (unchanged)
- `topo_filesize` -> `"topo_filesize_bbox_gated_proof"`
- `shape_invariant` -> `"shape_invariant_bbox_gated_proof"`

## Change 6: Fix build_mode_index to index all pairs (lines 636-640)

**Before**: Star-topology indexing from `paths[0]` to all others. For a group
`[A, B, C]`, only `(A,B)` and `(A,C)` were indexed; `(B,C)` was missing.

**After**: Full pairwise indexing using nested loops over all `(i, j)` with
`i < j`, producing both orderings. For `[A, B, C]`, all six directed pairs
`(A,B), (B,A), (A,C), (C,A), (B,C), (C,B)` are now indexed.

This fixes lookup misses when the canonical asset in a compensation pair is not
the first element in the dedup report's `usd_paths` array.
