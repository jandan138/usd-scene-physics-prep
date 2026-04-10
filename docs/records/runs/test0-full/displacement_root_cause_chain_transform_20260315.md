---
title: "Root Cause: 487 Displaced Prims — _get_chain_transform Matrix Multiply Order Bug"
code_reference: "scripts/normalize_asset_transforms.py:137-153"
created_at: "2026-03-15"
updated_at: "2026-03-15"
maintainer: "team-lead"
status: "verified"
---

# Root Cause: 487 Displaced Prims — `_get_chain_transform` Matrix Multiply Order Bug

## Executive Summary

The hard gate failure (487 displaced prims out of 101,965) in the `20260314_rebuilt_full_dlc_v1` normalize run is caused by a **matrix multiplication order bug** in `_get_chain_transform()` at `scripts/normalize_asset_transforms.py:152`.

The function accumulates transforms in ancestor-first order, but USD's row-vector convention requires descendant-first order.

**This is NOT a Phase 2 compensation formula bug** — the formula `M_scene_new = T(center) * R_y2z_inv * M_scene_old` at line 739 is mathematically correct and was verified to be applied correctly.

## The Bug

### Location

`scripts/normalize_asset_transforms.py:137-153`

```python
def _get_chain_transform(ancestor, descendant) -> Gf.Matrix4d:
    chain = []
    cur = descendant
    while cur and cur.GetPath() != ancestor.GetPath():
        chain.append(cur)
        cur = cur.GetParent()
    chain.reverse()  # top-down order: [ancestor_child, ..., descendant]

    result = Gf.Matrix4d(1.0)
    for p in chain:
        result = result * _get_local_matrix(p)  # <-- BUG: wrong order
    return result
```

### Problem

For a hierarchy `Instance > Group_00 > Component_6`:

- **Current code** produces: `M_chain = M_group00 * M_component6`
- **Row-vector convention** requires: `M_chain = M_component6 * M_group00`

In USD row-vector convention, a point transforms as `p_world = p * M_local * M_parent * M_grandparent`. The current code reverses this ordering.

### Fix

One-line change at line 152:

```python
# WRONG:
result = result * _get_local_matrix(p)

# CORRECT:
result = _get_local_matrix(p) * result
```

## Numerical Verification

Tested with asset `desk/5e55f06f3732989070508a1615154c55` which has hierarchy `/Root/Instance/Group_00/Component_6`:

| Method | Point position | Error |
|--------|---------------|-------|
| Wrong order (current code) | (32.91, 66.00, **30.02**) | **18.618m** |
| Fixed order | (32.91, 66.00, **48.63**) | **0.000000m** |
| Ground truth (`GetLocalToWorldTransform`) | (32.91, 66.00, **48.63**) | — |

The fix produces **exact** match with USD's own `GetLocalToWorldTransform`.

## Phase 2 Formula Verification

The Phase 2 compensation formula at line 739 was independently verified to be **correct**:

```python
# Line 739 (CORRECT — no change needed):
M_scene_new = T_center * _R_Y2Z_INV * M_scene_old
```

Verification: For the same desk prim in scene `MVUHLWYKTKJ5EAABAAAAADQ8_usd`:
- `M_expected = T(center) * R_y2z_inv * M_src_local`
- `M_actual = dst_layout local matrix`
- **Max element difference: 0.000000** — formula applied correctly

The displacement comes from the **wrong center** (computed using wrong `_get_chain_transform`), not from wrong application.

## Scope of Impact

### Why only 487 prims?

The bug only manifests for assets with **intermediate prims** between `/Root/Instance` and mesh:
- `/Root/Instance/Group_00/Component_6` — **affected** (2+ prims in chain, order matters)
- `/Root/Instance/SM_mesh` — **unaffected** (1 prim in chain, order irrelevant)

Out of 101,965 scene prims, only 487 (0.48%) reference assets with intermediate Group nodes.

### Category breakdown

| Category | Displaced | Total | Rate | Max displacement |
|----------|-----------|-------|------|-----------------|
| basket | 55 | 87 | 63.2% | 3.9m |
| other | 55 | 22,329 | 0.2% | 854.5m |
| trashcan | 55 | 277 | 19.9% | 2.5m |
| faucet | 39 | 266 | 14.7% | 6.5m |
| pot | 32 | 162 | 19.8% | 2.1m |
| electriccooker | 32 | 78 | 41.0% | 1.7m |
| refrigerator | 31 | 79 | 39.2% | 11.7m |
| book | 30 | — | — | 15.6m |
| oven | 25 | 69 | 36.2% | 0.8m |
| desk | 23 | — | — | 159.8m |
| (11 more) | 110 | — | — | — |

### Separate issue: 126 missing centers

126 prims have missing centers (from 36 failed book assets + known-bad assets). This is a **data quality** issue, not a code bug. Zero overlap with the 487 displaced prims.

## Why Agents Initially Identified Wrong Root Cause

The investigation agents proposed a "coordinate space mismatch" at line 739 — that `T(center_z_up)` was applied in the wrong frame. This hypothesis was **disproved** by:

1. Mathematical derivation showing `M_scene_new = T(center) * R_y2z_inv * M_scene_old` is correct
2. Numerical verification showing the exact matrix was applied with 0.000000 element difference
3. World-space vertex comparison showing the actual displacement matches the error from wrong `_get_chain_transform`, not from wrong formula

The true root cause (line 152) was found by tracing why the stored center value differed from the actual asset bounding box center.

## Recovery Plan

1. Apply one-line fix at line 152
2. Re-run Phase 1 (all 114 categories) — centers will be recomputed correctly
3. Re-run Phase 2 (99 scenes) — compensation will use correct centers
4. Hard gate should pass with 0 displaced prims

## Key Files

- Bug location: `scripts/normalize_asset_transforms.py:152`
- Pairwise report: `check_reports/test0_rebuilt_full/20260314_rebuilt_full_dlc_v1/normalize/test0_vs_normalized_pre_dedup.json`
- Audit report: `check_reports/test0_rebuilt_full/20260314_rebuilt_full_dlc_v1/normalize/test0_normalize_phase2_audit.json`
- Gate verdict: `check_reports/test0_rebuilt_full/20260314_rebuilt_full_dlc_v1/summary/normalize_gate_verdict.json`
