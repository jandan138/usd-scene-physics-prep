---
title: "Bbox-gated multi-mode: Changes 4 & 5 — expand whitelist and add mode-aware branching"
code_reference: scripts/rewrite_layout_asset_refs_with_compensation.py
created_at: 2026-04-01
updated_at: 2026-04-01
maintainer: TraeAI
status: complete
---

## Summary

Implemented Changes 4 and 5 from the bbox-gated multi-mode dedup plan in the
bbox-gated compensation path of `rewrite_layout_asset_refs_with_compensation.py`.

## Change 4: Expand apply-stage whitelist (line 793)

The bbox-gated path previously only allowed `geom_only` and `identity` dedup
modes through to the compensation stage. All other modes were rejected with
`mode_not_enabled_<mode>`.

**Before:**
```python
if dedup_mode not in ("geom_only", "identity"):
```

**After:**
```python
if dedup_mode not in ("geom_only", "identity", "topo_filesize", "shape_invariant"):
```

This enables `topo_filesize` and `shape_invariant` dedup pairs to proceed
through the bbox-gated compensation path.

## Change 5: Mode-aware branching in bbox-gated path (lines 802-807)

Previously the bbox-gated path used only the internal-matrix formula
(`M_canon_inv * M_old * old_local`), which is correct for `geom_only`/`identity`
but produces the V-compensation translation scaling bug for other modes (see
`docs/operations/v_compensation_translation_scaling_bug.md`).

**Before:**
```python
old_internal = _get_internal_cached(old_abs)
canonical_internal = _get_internal_cached(new_abs)
new_local = canonical_internal.GetInverse() * old_internal * old_local
```

**After:**
```python
if dedup_mode in ("geom_only", "identity"):
    old_internal = _get_internal_cached(old_abs)
    canonical_internal = _get_internal_cached(new_abs)
    new_local = canonical_internal.GetInverse() * old_internal * old_local
else:
    new_local = V * old_local
```

This mirrors the existing correct logic in the legacy path (lines 868-873),
where `V` is the instance-space compensation matrix from `_get_V_cached()`.

## Rationale

- For `geom_only`/`identity`: V=Identity, so internal-matrix formula is correct
  (and more precise, as it avoids floating-point accumulation in V).
- For `topo_filesize`/`shape_invariant`: V is a Procrustes/ICP alignment in
  instance space. Applying `M_canon_inv * V * M_old * old_local` would sandwich
  V between internal transforms, scaling its translation component incorrectly
  (the root cause of the V5 translation scaling bug). Using `V * old_local`
  directly avoids this.
