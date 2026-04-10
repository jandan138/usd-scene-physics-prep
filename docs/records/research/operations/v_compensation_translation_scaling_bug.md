---
title: "V Compensation Translation Scaling Bug"
code_reference:
  - scripts/rewrite_layout_asset_refs_with_compensation.py
  - scripts/compute_vertex_transform.py
created_at: 2026-03-24
updated_at: 2026-03-24
maintainer: team-lead
status: active
---

# V Compensation Translation Scaling Bug

## Problem

After dedup V-matrix compensation, ground prims have up to 84.39 units of world-space centroid displacement. The V matrix correctly maps canonical→old in instance space (error=0), but the compensation formula produces wrong world positions.

## Root Cause

### The V matrix operates in instance-space

`compute_V_shape_invariant()` uses `extract_instance_space_vertices()` which returns vertices in **defaultPrim space** (already includes the 0.1 Instance scale). So V's 4x4 matrix:

```
V = [sR  | 0]    ← sR in instance-space coordinates
    [t   | 1]    ← t in instance-space coordinates (e.g. 93.7)
```

### The compensation formula assumes mesh-local space

`rewrite_layout_asset_refs_with_compensation.py:606`:
```python
new_local = canonical_internal.GetInverse() * V * old_internal * old_local
```

Where `M_internal = 0.1 * Identity` (Instance prim's scale).

### The math breakdown

```
M_canon_inv * V * M_old = (10*I) * V * (0.1*I)

For 3x3 block (rows 0-2):
  10 * sR * 0.1 = sR  ← correct, scales cancel

For translation row (row 3):
  Row 3 of (10*I @ V) = [t | 1]  (unchanged, because M_inv[3][3]=1)
  Row 3 of ((10*I @ V) @ (0.1*I)) = [t*0.1 | 1]  ← WRONG! t shrunk by 10x
```

V's translation (93.7 instance-space units) becomes 9.37 — a 10x error.

### Verified numbers

| Quantity | Value |
|----------|-------|
| V translation | (93.71, 0, -3.42) |
| M_inv * V * M * M_layout_old translation | (-190.75, 183.56, 280.00) |
| V * M_layout_old translation (correct) | (-106.42, 180.49, 280.00) |
| Translation difference | 84.33 ≈ displacement 84.39 |

## Fix Options

### Option A: Change V to mesh-local space (fix V computation)

Modify `compute_V_for_pair()` to extract vertices in **mesh-local space** (without M_internal), then compute V. The formula `M_canon_inv * V * M_old * M_layout` would then be correct.

Pros: Formula semantics become consistent.
Cons: Need to modify `extract_instance_space_vertices` or add a new extraction mode.

### Option B: Use V directly (fix formula)

Since V already works in instance space:

```python
# When V is in instance-space (which it always is from compute_V_for_pair):
new_local = V * old_local
```

This is correct because: `world = pts_instance * M_layout`, and V maps instance→instance, so `pts_old_instance * M_layout_old = pts_canon_instance * V * M_layout_old = pts_canon_instance * new_local`.

Pros: Simplest fix, 1 line change.
Cons: Only correct when M_canon_internal == M_old_internal (which is true for all normalized assets with same Instance scale).

### Option C: Correct the formula for translation (fix formula precisely)

Convert V from instance-space to mesh-local-space before applying the formula:

```python
V_mesh_local = old_internal * V * canonical_internal.GetInverse()
new_local = canonical_internal.GetInverse() * V_mesh_local * old_internal * old_local
# Simplifies to: new_local = V * old_local (same as Option B when internals are equal)
```

### Recommended: Option B with fallback

For the common case (M_canon == M_old), use `V * old_local`. For the rare case where internals differ, keep the current formula but pre-convert V:

```python
V_local = old_internal * V * canonical_internal.GetInverse()
new_local = canonical_internal.GetInverse() * V_local * old_internal * old_local
```

Which simplifies to `new_local = V * old_local` regardless.

Actually the universal correct formula is simply: `new_local = V * old_local`.

Proof: V maps instance→instance. Vertices are stored in mesh-local space in the USD.
```
world = p_mesh * M_internal * M_layout
      = p_instance * M_layout  (since p_instance = p_mesh * M_internal)

For old:   world_old = p_old_inst * M_layout_old
For canon: world_new = p_canon_inst * M_layout_new

We want world_old == world_new:
  p_old_inst * M_layout_old = p_canon_inst * M_layout_new
  (p_canon_inst * V) * M_layout_old = p_canon_inst * M_layout_new
  V * M_layout_old = M_layout_new  ✓
```

## Affected Code

- `scripts/rewrite_layout_asset_refs_with_compensation.py:606`
- Fix: change `canonical_internal.GetInverse() * V * old_internal * old_local` to `V * old_local`
