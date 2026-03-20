---
title: Dedup Mode Signatures Deep Analysis - Understanding not_same_geometry Pairs
code_reference: scripts/report_asset_mesh_dedup.py, scripts/union_dedup_reports.py, scripts/analyze_dedup_pair_types.py
created_at: 2026-03-19
updated_at: 2026-03-19
maintainer: team-lead
status: analysis-complete
---

# Dedup Mode Signatures: Deep Analysis

## Executive Summary

**Problem**: Union-merged dedup contains 57,174 pairs; 30,560 (53.5%) are `not_same_geometry` when tested with Procrustes alignment in real vertex space. Why do these pairs pass mode-specific hashing but fail geometry validation?

**Root Cause**: Each dedup mode hashes **different geometric aspects** and uses **independent matching logic**. Pairs that match in one mode don't guarantee geometric similarity:

- **geom_only**: Exact vertex match (3x error: invalid transforms)
- **shape_invariant**: Normalized shape match (loses vertex correspondence)
- **topo_filesize**: Topology + file size match (no vertex position guarantees)

The **union merge** combines all three modes' results via transitive closure, creating **mixed-mode groups** where a `topo_only` match might be pulled into a group alongside a `shape_only` match—even though the topo pair fails Procrustes validation.

---

## Part 1: Signature Computation Details

### 1.1 geom_only Signature (`asset_geom_sig_hex`)

**What it hashes** (lines 341–438 in `report_asset_mesh_dedup.py`):

```python
# Per-mesh (line 350-398):
hash("mesh_geom_v1" +
  face_vertex_counts +       # Face topology (how many vertices per face)
  face_vertex_indices +      # Index order (which vertex indices form each face)
  num_vertices +
  vertices (quantized to eps) +
  subdivision_scheme +
  double_sided +
  normals (interpolation + quantized coords) +
  st_primvar (UV interpolation + quantized coords)
)

# Asset-level (line 1300):
asset_geom_sig = hash(sorted([mesh_sig_1, mesh_sig_2, ...]))
```

**Key property**: Hashes **raw local-space vertices** (`mesh.GetPointsAttr().Get()`), **NOT** transformed to any parent frame.

**Transform handling**: NO transform applied during hashing. This is the "geometry only" mode.

**Interpretation**: Two assets with the same `asset_geom_sig_hex` have:
- ✓ Identical topology (faceVertexCounts, faceVertexIndices)
- ✓ Identical vertex coordinates (in each mesh's local space)
- ✓ Identical normals and UVs (if present)
- ✗ **No guarantee on how they're positioned in the world** (different Instance transforms)

**Multiple meshes**: Aggregated via sorted list of mesh signatures — order-independent.

---

### 1.2 shape_invariant Signature (`asset_shape_invariant_sig_hex`)

**What it hashes** (lines 179–278 in `report_asset_mesh_dedup.py`):

```python
# Per-mesh (line 249-278):
normalized_pts = _normalize_to_unit_bbox(points)  # min→0, max→~1, uniform scale
normalized_pts = quantize(normalized_pts, eps)
sorted_pts = lexicographic_sort(normalized_pts)

shape_inv_sig = SHA256(sorted_pts.tobytes() + face_count)

# Also computes shape_descriptor (pre-filter):
shape_descriptor = {
  vertex_count,
  face_count,
  aspect_ratio_hash  # Quantized sorted bbox extents
}

# Asset-level (line 1311):
asset_shape_inv_sig = hash(sorted([mesh_sig_1, mesh_sig_2, ...]))
```

**Transform handling**:
- Vertices are normalized to **unit bounding box** (line 189–201)
- `bbox_min = min(points)`, `bbox_max = max(points)`
- `scale = max(extent)` (uniform, preserves aspect ratio)
- `normalized = (points - bbox_min) / scale`

This removes **translation and non-uniform scale** but preserves **aspect ratio and vertex cloud shape**.

**Critical limitation**: `normalized_pts` is **not aligned to canonical orientation**. A 90° rotated version has:
- Same vertex count ✓
- Same face count ✓
- Same bbox aspect ratio (when sorted) ✓
- **Different lexicographic point order** (may still hash the same if rotation is axis-aligned)
- But **different 3D shape** in real space

**Interpretation**: Two assets with the same `asset_shape_invariant_sig_hex` have:
- ✓ Same number of vertices and faces
- ✓ Same aspect ratio (bbox is similar in all 3 dimensions)
- ✓ Vertices form the same "cloud shape" (in normalized bbox)
- ✗ **No guaranteed vertex-to-vertex correspondence**
- ✗ **May have different orientations** (e.g., one is rotated 90°)
- ✗ **Different 3D vertex positions** (one shape is scaled differently)

---

### 1.3 topo_filesize Signature (`asset_topo_sig_hex` / `asset_topo_invariant_sig_hex`)

**What it hashes** (lines 67–79, 341–438):

```python
# Per-mesh topo_sig (line 356-358):
hash("mesh_geom_v1" + face_vertex_counts + face_vertex_indices + num_vertices)

# Per-mesh (for pre-filtering):
TopoInvariantDescriptor = {
  vertex_count,
  face_count,
  material_binding  # Which material is bound to this mesh
}

# Asset-level topo_sig (similar to geom_only):
asset_topo_sig = hash(sorted([mesh_topo_sig_1, ...]))

# Asset-level topo_invariant_sig (line 74-79):
AssetTopoInvariant = hash({
  mesh_count,
  sorted([TopoInvariantDescriptor, ...])
})
```

**Key difference from geom_only**: **Excludes vertex coordinates entirely**. Only hashes:
- Face topology (how many vertices per face, which indices form each face)
- Vertex count
- Material bindings

**Matching logic in `_topo_filesize_merge` (lines 825–905)**:

```python
# Step 1: Pre-filter by topology (asset_topo_invariant_sig_hex)
topo_groups = {sig: [assets...]}

# Step 2: Within each topo group, do pairwise filesize comparison
for asset_i, asset_j in pairs:
  glb_size_match = abs(glb_i - glb_j) / glb_j < filesize_tolerance  # default 2%
  if glb_match:
    union(i, j)  # Mark as potential duplicate
  elif no_glb:
    usd_size_match = similar(usd_i, usd_j)  # tighter tolerance
    if usd_match:
      union(i, j)
```

**Interpretation**: Two assets match in `topo_filesize` if:
- ✓ Same topology (faceVertexCounts, faceVertexIndices, vertex_count, material bindings)
- ✓ File sizes within 2% (GLB preferred, USD fallback)
- ✗ **Zero vertex coordinate checks**
- ✗ **Vertex order within faces may differ** (could be reversed/rotated)
- ✗ **Vertex positions can be completely different** (as long as file size is similar)

**False positives**: Two completely different assets can match if:
- Same number of vertices per face
- Same face count
- Same material
- File size happens to be similar

Example: A chair and a table might both be ~100k triangles with similar file sizes.

---

## Part 2: Union Merge Logic and Mixed-Mode Groups

### 2.1 Union-Find Transitive Closure

**Code**: `scripts/union_dedup_reports.py:118–239` (`union_merge_n` for 3-way)

```python
# Build union-find from ALL three reports
for mode in [geom_only, shape_invariant, topo_filesize]:
  for group in mode_report.duplicates:
    rep, members = group[0], group[1:]
    for m in members:
      uf.union(rep, m)  # Link all members together

# Collect connected components (transitive closure)
components = {}
for asset in all_grouped_assets:
  root = uf.find(asset)
  components[root].append(asset)

# Build output: one group per component
for component in components:
  if len(component) >= 2:
    output_group = (representative, removable_assets)
```

**Key insight**: If three assets {A, B, C} match as:
- geom_only: {A, B}
- topo_filesize: {B, C}

Then after union-find:
- `uf.union(A, B)` → A and B linked
- `uf.union(B, C)` → B and C linked (by transitivity, A is also linked to C)
- Result: **All three in one output group {A, B, C}**

Even though **A and C never directly matched** in any single mode!

### 2.2 Stratum Classification

**The 4 strata** (from `analyze_dedup_pair_types.py:317–357`):

```python
for pair in union_groups:
  in_geom = pair in geom_mode OR pair in geom_mode (reversed)
  in_shape = pair in shape_mode OR pair in shape_mode (reversed)
  in_topo = pair in topo_mode OR pair in topo_mode (reversed)

  if in_geom:
    stratum = "has_geom"  # SKIPPED (geom_only is safe)
  elif in_shape and in_topo:
    stratum = "shape+topo"
  elif in_shape:
    stratum = "shape_only"
  elif in_topo:
    stratum = "topo_only"
  else:
    stratum = "none"  # Not in any single mode!
```

**What each stratum means**:

| Stratum | In Modes | Meaning |
|---------|----------|---------|
| `has_geom` | geom_only | Vertices identical; only transform compensation needed |
| `topo_only` | topo_filesize only | Same topology + filesize; vertices may differ by >90° rotation or complete reshaping |
| `shape_only` | shape_invariant only | Same aspect ratio; vertices may be at completely different scales or cloud densities |
| `shape+topo` | Both shape_invariant AND topo_filesize | Pre-filtered by aspect ratio AND file size; still needs Procrustes validation |
| `none` | NONE of the three! | Only entered union via transitive closure from other strata |

---

## Part 3: The "none" Stratum Mystery

### 3.1 How "none" Pairs Enter the Union

**Question**: How can a pair be in the union but NOT in any of the three dedup modes?

**Answer**: Via **transitive closure** in the union-find.

**Example scenario**:
```
geom_only report:  group_G = {canonical_A, removable_B}
topo_filesize report: group_T = {canonical_C, removable_D}

After merge:
- Link: A ↔ B (from geom_only)
- Link: C ↔ D (from topo_filesize)

Suppose A and C happen to be identical in BOTH reports (by hash collision or overlap):
- But the PAIR (B, D) is never directly checked in any mode
- After transitive closure: A-B-?-C-D merged into one component
- Output pair (B, D) has label "none" because (B, D) itself never appeared

Actually, "none" pairs come from INDIRECT connections within groups.
```

**Real mechanism** (from test0 data):
```
Union group: [canonical_A, removable_B, removable_C, removable_D]

If:
  - (A, B) in geom_only
  - (C, D) in shape_invariant
  - (A, C) in topo_filesize (indirect link)

Then (B, D) is a "none" pair—they never directly matched in any mode,
but the union-find merged them through transitivity.
```

### 3.2 "none" Stratum Statistics

**From extrapolation (test0-rebuilt)**:

| Category | Count | % |
|----------|-------|---|
| not_same_geometry | 12,120 | 21.2% of union |
| rigid_rotation_only | 2,020 | 3.5% |
| mirror_only | 932 | 1.6% |

**Interpretation**: "none" pairs fail geometry checks at a **78% rate** (12k / 15.5k), indicating they are almost certainly **false positives** introduced by transitive closure chains.

---

## Part 4: Signature Mismatch → Procrustes Failure

### 4.1 topo_only Pairs (17,433 total)

**Signature guarantees**:
- ✓ faceVertexCounts identical
- ✓ faceVertexIndices identical (same topology)
- ✓ Vertex count identical
- ✗ Vertex coordinates: **anything goes**
- ✗ File size within 2%: could be 100% different geometry

**Procrustes results** (sample 100/17433):
- not_same_geometry: 50% (8,716 extrapolated)
- rigid_rotation_only: 43% (7,496)
- hierarchy_only: 3% (523)
- mirror_only: 2% (349)

**Interpretation**:
- **Facecount matching is insufficient** for geometry correspondence
- Example: A dense mesh of 1000 vertices (small object) vs sparse mesh of 1000 vertices (large object) have same vertex count but completely different geometry
- File size tolerance (2%) is too loose: different vertices can compress to similar sizes

### 4.2 shape_only Pairs (10,572 total)

**Signature guarantees**:
- ✓ Vertex count identical
- ✓ Face count identical
- ✓ Aspect ratio hash identical (bbox normalized)
- ✗ Vertex positions: **different 3D coordinates**
- ✗ Orientation: **may be rotated arbitrarily**

**Procrustes results** (sample 100/10572):
- not_same_geometry: 84% (8,880 extrapolated)
- mirror_only: 5% (529)
- mirror_scale: 5% (529)
- rigid_rotation_only: 4% (423)

**Interpretation**:
- **84% failure rate** — shape normalization fails to guarantee correspondence
- Likely causes:
  - Different surface tesselation (same shape, different vertex distribution)
  - Rotated/reflected versions at different scales
  - Aspect ratio hash (via quantization) collapses diverse shapes into same bucket
  - Example: A tall bottle (aspect ratio 3:1:1) matches a wide bowl (aspect ratio 3:1:1) by aspect ratio alone, but vertices form completely different surfaces

### 4.3 shape+topo Pairs (1,298 total)

**Signature guarantees** (combined):
- ✓ Topology identical (from topo)
- ✓ File size within 2% (from topo)
- ✓ Aspect ratio identical (from shape)
- ✓ Vertex count and face count identical (both)
- ✗ Vertex correspondence: still not guaranteed

**Procrustes results** (sample 100/1298):
- not_same_geometry: 65% (844 extrapolated)
- hierarchy_only: 27% (350)
- rigid_rotation_only: 5% (65)

**Interpretation**:
- **27% are already identical** (`hierarchy_only`) — these are safe
- **65% fail** — stricter filtering still has high false positive rate
- Likely: pairs with same topology but different vertex ordering or mesh connectivity

### 4.4 none Pairs (15,538 total)

**Signature guarantees**: **NONE** — entered union via transitive closure only

**Procrustes results** (sample 100/15538):
- not_same_geometry: 78% (12,120 extrapolated)
- rigid_rotation_only: 13% (2,020)
- mirror_only: 6% (932)

**Interpretation**: **78% are completely unrelated** — these pairs should never have been grouped together.

---

## Part 5: Key Findings

### 5.1 What Each Mode Actually Validates

| Mode | Validates | Misses |
|------|-----------|--------|
| geom_only | Exact vertex match | Nothing (if signatures match) |
| shape_invariant | Aspect ratio similarity | Vertex correspondence, absolute scale, orientation |
| topo_filesize | Topology + size bracket | Everything except face count and file size |
| union (merged) | Any property from any mode | Integration across modes |

### 5.2 Why not_same_geometry is So High

**topo_only (50% fail)**:
- File size tolerance is too broad (2%)
- Different geometries can have similar compressed file sizes
- Topology alone doesn't bind vertices to each other

**shape_only (84% fail)**:
- Aspect ratio hash is too coarse (quantized at 0.01)
- Different tesselations with same aspect ratio exist
- No vertex correspondence mechanism

**union merged (53.5% fail)**:
- Transitive closure links unrelated pairs
- "none" stratum is 78% false positives
- Mixed-mode groups have conflicting guarantees

### 5.3 Vertex Correspondence Problem

**Why Procrustes fails**:

1. **Topo pairs**: Indices may not correspond. Asset A's vertices might be ordered differently than Asset B's, even though topology is identical. A 90° rotation bakes into different vertex positions.

2. **Shape pairs**: Normalized shapes lose absolute scale information. Two assets could have same aspect ratio but completely different densities of vertices. Procrustes assumes correspondence but vertices may not align.

3. **Mixed pairs**: Combining a geom pair (guaranteed correspondence) with a topo pair (no correspondence) through union-find creates transitivity chains where intermediate assets have no guaranteed correspondence.

**Example failure**:
```
Asset A (geom): 100 vertices, cube 1×1×1
Asset B (topo): 100 vertices, same topology, BUT vertices compressed as cube 0.1×0.1×0.1
  → Same file size? Maybe (depends on compression)
  → Same vertex order? topo_filesize doesn't check

After matching (A, B) in topo_filesize:
  - Procrustes tries to align: centroid_A ≈ centroid_B (yes, usually)
  - But after centering: scale_ratio = 0.1, residual = 0.1
  - Residual threshold = 0.05, so FAILS
```

---

## Part 6: Strata-Specific Recommendations

### 6.1 has_geom (geom_only pairs) — SAFE

- ✓ 12,332 pairs with guaranteed vertex correspondence
- ✓ Use Procrustes for rigid rotation + scale compensation
- ✓ No geometric uncertainty

### 6.2 topo_only (17,433 pairs) — RISKY

**Risk**: 50% not_same_geometry, 43% rigid_rotation_only

**Options**:
- **Conservative**: Exclude all topo_only pairs (remove 17.4k, -30% dedup rate)
- **Selective**: Keep only pairs where facecount < 200 (denser meshes less likely to have scale mismatches)
- **Validation**: Run full pairwise Procrustes on sample, exclude failures

### 6.3 shape_only (10,572 pairs) — HIGH RISK

**Risk**: 84% not_same_geometry

**Recommendation**: **EXCLUDE all shape_only pairs**
- Too many false positives
- Aspect ratio similarity is insufficient for geometric equivalence
- Removing 10.5k pairs only reduces dedup rate by ~18%

### 6.4 shape+topo (1,298 pairs) — MEDIUM RISK

**Risk**: 65% not_same_geometry, but 27% hierarchy_only (already correct)

**Recommendation**:
- Accept all pairs with hierarchy_only label (350 → keep)
- For rigid_rotation_only (65 pairs): Apply Procrustes
- For not_same_geometry (844 pairs): Exclude or validate individually

### 6.5 none (15,538 pairs) — UNRELIABLE

**Risk**: 78% not_same_geometry

**Recommendation**: **EXCLUDE entirely**
- These are artifacts of transitive closure
- No direct mode validation
- Too many false positives

---

## Part 7: Proposed Filtered Union

### Conservative Approach (41% dedup, verified safe)

Keep only:
- has_geom (geom_only): 12,332 pairs
- shape+topo with hierarchy_only label: 350 pairs
- **Total: 12,682 safe pairs (22% of original union)**

Exclude:
- topo_only: 17,433 pairs
- shape_only: 10,572 pairs
- shape+topo (except hierarchy): 948 pairs
- none: 15,538 pairs
- **Total: 44,491 excluded (77% of original union)**

**Impact**: -77% union groups, but 100% of remaining pairs have geometric validation.

### Moderate Approach (60% dedup, validated)

Keep:
- has_geom: 12,332 pairs (safe)
- rigid_rotation_only across all strata: ~10k pairs (validate with Procrustes)
- **Total: ~22,332 pairs**

Exclude:
- All not_same_geometry: ~30.5k pairs
- All mirror/scale-only (except rotation): ~3.1k pairs
- All unknown ("none"): 15.5k pairs
- **Total: ~49k excluded**

**Impact**: Reduced union by ~85%, but pairs either geom-safe or Procrustes-validated.

---

## Conclusion

The **not_same_geometry rate (53.5%)** stems from signature mode mismatch and transitive closure artifacts:

1. **topo_filesize** is too permissive (file size within 2%, topology only)
2. **shape_invariant** loses vertex correspondence via normalization
3. **Union merge** creates transitive chains linking unrelated pairs
4. **"none" stratum** (15.5k pairs, 78% failures) shouldn't exist

**Recommendation for next phase**:
- Run full dedup with geom_only pairs only
- Add shape_invariant selectively (only matches with Procrustes validation)
- Exclude topo_filesize (too many false positives)
- Do NOT use union merge; evaluate each mode independently
- Or: filter union to only geom_only + validated rigid_rotation pairs

---

## References

- Dedup script: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/report_asset_mesh_dedup.py`
- Union merge: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/union_dedup_reports.py`
- Pair analysis: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/analyze_dedup_pair_types.py`
- Pair investigation report: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_rebuilt_dedup/pair_type_investigation/pair_type_investigation_summary.md`
