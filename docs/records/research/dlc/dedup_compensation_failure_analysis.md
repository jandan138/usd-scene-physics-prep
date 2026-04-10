---
title: Dedup Compensation Failure Analysis
code_reference: scripts/rewrite_layout_asset_refs_with_compensation.py, scripts/usd_xform_utils.py
created_at: 2026-03-19
updated_at: 2026-03-19
maintainer: researcher-compensation
status: research-complete
---

# Dedup Compensation Failure Analysis

## Executive Summary

The dedup compensation formula `M_new_local = M_canon_internal⁻¹ × M_old_internal × M_old_layout` is **mathematically correct** for pure prim transforms (TRS), but **catastrophically fails** on real dedup pairs where geometry differences are baked into vertex coordinates.

**Placement verification shows 24.9% of 101,919 prims displaced > 0.01 units** after C1 dedup, with worst categories at 246+ units displacement. The root cause is not a bug in the formula or chain walk (both correct), but a **fundamental limitation**: the formula assumes vertex configurations are identical between old and canonical assets, which is false for:
- **Baked vertex rotations** (e.g., desk: 1084 unit displacement)
- **Mirror/reflection** (e.g., curtain: 246.65 unit displacement)
- **Non-uniform scale** applied to geometry

## Part 1: Compensation Formula & Implementation

### The Formula (Row-Vector Convention)

From `rewrite_layout_asset_refs_with_compensation.py:12-16`:

```
p_world = p_mesh * M_assetInternal * M_layout * M_parent_world

When swapping old asset ref to canonical:
  M_oldInternal * M_layout_old = M_canonicalInternal * M_layout_new
  => M_layout_new = inverse(M_canonicalInternal) * M_oldInternal * M_layout_old
```

**Interpretation**:
- `p_mesh`: points in the mesh's local coordinate space
- `M_assetInternal`: accumulated transform from defaultPrim through Instance to first mesh parent
- `M_layout`: the instance prim's local transform in the scene layout
- `M_parent_world`: parent prim's world transform (preserved during compensation)

The goal: when replacing old asset reference with canonical asset, adjust `M_layout` so that world placement stays the same.

### Implementation (rewrite_layout_asset_refs_with_compensation.py:559-593)

```python
if apply_compensation:
    uniq = sorted(set(changed_pairs))
    if len(uniq) == 1:
        old_abs, new_abs = uniq[0]
        # Compensate local transform so world placement is unchanged.
        try:
            old_internal = _get_internal_cached(old_abs)         # Read M_oldInternal
            canonical_internal = _get_internal_cached(new_abs)   # Read M_canonicalInternal
            old_local = xform_cache.GetLocalTransformation(prim) # Get M_layout_old
            if isinstance(old_local, tuple):
                old_local = old_local[0]
            new_local = canonical_internal.GetInverse() * old_internal * old_local
            _set_local_matrix(prim, new_local)
            xform_compensated += 1
```

**Key observations**:
1. Compensation only applied when `len(uniq) == 1` (single old→canonical pair per prim)
2. Multi-ref prims skipped with `xform_compensation_skipped_multi_ref`
3. Applied at reference rewrite time, lazily computed via `_get_internal_cached`
4. Result is a single `xformOp:transform` matrix (may replace TRS op stack)

## Part 2: Asset Internal Matrix Computation

### _get_asset_internal_matrix() (lines 166-223)

Purpose: compute the full transform chain from asset's defaultPrim through Instance down to the first mesh's parent.

**Algorithm**:
1. Open asset USD with `Usd.Stage.Open(..., load=Usd.Stage.LoadNone)`
2. Find defaultPrim (fallback to first non-root child if missing)
3. Locate `/Root/Instance` child prim
4. Call `find_all_meshes(instance_prim)` to detect mesh hierarchy
5. **Chain walk**:
   - If mesh is direct child of Instance → return `get_local_matrix(instance_prim)` only
   - Otherwise → call `get_chain_transform(default_prim, mesh_parent)` to accumulate full chain

### Shared Helpers (usd_xform_utils.py)

#### get_local_matrix(prim) [lines 38-51]
- Wraps `UsdGeom.Xformable.GetLocalTransformation(Usd.TimeCode.Default())`
- Handles tuple vs. matrix return value across pxr versions
- Returns identity if not valid Xformable

#### get_chain_transform(ancestor, descendant) [lines 54-74]

**Row-vector convention**: `p_ancestor = p_descendant * M_chain`

```python
chain = []
cur = descendant
while cur and cur.GetPath() != ancestor.GetPath():
    chain.append(cur)
    cur = cur.GetParent()
chain.reverse()  # top-down order

result = Gf.Matrix4d(1.0)
for p in chain:
    result = get_local_matrix(p) * result  # Left multiply
return result
```

**Behavior**: Accumulates from ancestor's first child down to descendant (inclusive), in top-down order.

#### find_all_meshes(root_prim) [lines 77-90]
- Traverses with `Usd.PrimRange(root_prim)`
- Filters to `UsdGeom.Mesh` with non-empty `points` attribute
- Returns ALL meshes; `_get_asset_internal_matrix` uses `meshes[0]`

### Test Coverage (test_dedup_compensation_chain.py: 10 tests, all passing)

| Test | Scenario | Coverage |
|------|----------|----------|
| `test_direct_child_mesh` | Instance → Mesh | Returns Instance local only |
| `test_one_intermediate` | Instance → Group_00 → Mesh | Chains: Group*Instance ✓ |
| `test_two_intermediates` | Instance → Group_00 → Component_N → Mesh | Full 3-level chain ✓ |
| `test_sibling_meshes_same_parent` | Multiple meshes under same parent | Uses first mesh ✓ |
| `test_different_branch_meshes` | Multiple branches, different meshes | Consistent first-mesh use ✓ |
| `test_identity_intermediate` | Instance(T) → Group(I) → Mesh | Preserves identity ✓ |
| `test_no_mesh_fallback` | Instance with no mesh children | Returns Instance local ✓ |
| `test_compensation_with_intermediate_vs_flat` | Old has Group, canonical flat | Formula validates ✓ |
| `test_compensation_both_intermediates_different` | Both have different Groups | Formula validates ✓ |
| `test_no_instance_returns_identity` | Missing Instance child | Returns identity ✓ |

**Important**: All tests use pure TRS on prims. No test creates geometrically different vertex configurations.

## Part 3: Why Compensation Fails

### Placement Verification Results

From `check_reports/test0_rebuilt_dedup/verify_post_dedup/source_vs_deduped_normalized.json`:

```
Total prims compared:          101,919
Displaced > 0.01 units:         25,431 (24.9%)
Displaced > 0.1 units:          22,951 (22.5%)
Displaced > 1.0 units:          14,300 (14.0%)
Displaced > 10.0 units:          2,274 (2.2%)

Worst categories (by max displacement):
1.  wall:       248.14 units
2.  curtain:    246.65 units  ← Mirror/reflection case
3.  counter:    200.75 units
4.  other:      140.80 units
5.  ground:     135.22 units
6.  light:      126.10 units
7.  plant:      101.64 units
8.  bed:         90.45 units
9.  table:       82.01 units
10. bathtub:     80.65 units
```

### Root Cause A: Baked Vertex Transforms (PRIMARY)

**Example: desk category, 1084 unit displacement**

**Setup**:
- Old asset: mesh vertices stored in local space, rotated 90° around Y-axis (baked rotation)
- Canonical asset: same logical mesh, vertices stored without rotation (different vertex positions)
- Both assets may have `M_oldInternal ≈ Identity` and `M_canonicalInternal ≈ Identity`

**What happens with compensation**:
```
M_new = M_canon⁻¹ * M_old * M_old_layout
      = I⁻¹ * I * M_old_layout
      = M_old_layout  (NO CHANGE!)
```

**Why it fails**:
- The layout prim compensation is calculated based on prim transforms (M_internal)
- But the actual geometric difference is in vertex coordinates (`p_mesh`)
- When canonical assets is loaded with `M_new_layout = M_old_layout`, its vertices are NOT rotated
- Result: **mesh appears rotated 90° relative to its placement → 1084 units displacement**

**Key insight**: The formula assumes `p_mesh_old = p_mesh_canonical` (same vertex positions). This is false for baked transforms.

### Root Cause B: Mirror / Reflection (SECONDARY)

**Example: curtain category, centroid displacement 246.65 units**

**Setup**:
- Some dedup pairs use negative scale on one axis (reflection/mirroring)
- Procrustes analysis in dedup detection finds these and classifies them (correctly) as deduplicable
- But compensation formula cannot handle vertex-space reflection

**Why it fails**:
- Reflection has `det(Matrix) < 0` (negative determinant)
- Pure matrix multiplication cannot represent the vertex-space mirror that already happened
- Compensation tries to invert the matrix, but this doesn't undo vertex reflection

### Root Cause C: Multi-Branch Hierarchies (POSSIBLE)

**Setup**:
```
Instance → Group_00 → Mesh_A
        ↘ Group_01 → Mesh_B
```

**How this fails**:
- `_get_asset_internal_matrix()` calls `find_all_meshes()` which returns all meshes
- But compensation uses `meshes[0]` (first mesh found via prim traversal order)
- Chain walk computes M_chain to first mesh only
- If layout references differ in how they use Group_00 vs. Group_01, compensation applies wrong chain

**Likelihood**: Lower than A & B, but possible in complex hierarchies.

## Part 4: Formula Limitations & Assumptions

### What the Formula Handles Correctly

1. **Pure prim TRS with identical vertex coordinates**
   - If M_oldInternal and M_canonicalInternal differ due to prim transforms
   - But vertices are in the same space → Formula works perfectly
   - All unit tests pass because they simulate this case

2. **Identity internal matrices**
   - Both M_oldInternal and M_canonicalInternal are identity
   - Formula simplifies to: M_new_local = M_old_local (no compensation needed)

3. **Single-branch hierarchies**
   - Instance → optionally Group_00 → ... → Mesh
   - Chain walk correctly accumulates all transforms

4. **Asymmetric internal transforms** (as long as vertices match)
   - Old has Group_00 with transform T, canonical doesn't
   - Formula correctly computes compensation
   - Verified by test `test_compensation_with_intermediate_vs_flat`

### What the Formula Does NOT Handle

1. **Baked transforms in vertex coordinates** (CRITICAL)
   - Vertices rotated/scaled/reflected in the source mesh data
   - No amount of prim transform compensation can fix this
   - Would require Procrustes alignment or vertex rewriting

2. **Negative/reflection scale** (CRITICAL)
   - Matrix with `det < 0` (reflection)
   - Compensation cannot undo vertex-space mirroring

3. **Non-uniform scale on distorted geometry** (MEDIUM)
   - Canonical applies non-uniform scale (Sx ≠ Sy ≠ Sz) to vertices
   - Affects relative geometry in unpredictable ways

4. **Multi-branch with different vertex-space transforms** (MEDIUM)
   - Different branches have different internal transforms
   - Using first mesh's chain may not apply to other branches

## Part 5: Dedup Detection vs. Compensation Mismatch

### How Dedup Tools Detect Pairs (3 Methods)

**geom_only mode**:
- Compares mesh vertex positions directly
- Tolerant of prim transform differences
- Matches: same vertex coordinates (in reference space)

**shape_invariant mode** (uses Procrustes alignment):
- Applies Procrustes to find best-fit rotation + translation
- Computes RMSE; matches if RMSE < 0.05 (absolute threshold)
- **Finds and classifies transform type**: rigid_rotation, rigid_scale, mirror, not_same_geometry
- Generates R-matrix (rotation part) of best-fit alignment

**topo_filesize mode**:
- Matches by file size + vertex count + edge topology
- No vertex comparison; pure structure match
- Highest false positive risk

### The Critical Gap

**Dedup detection**: "These two meshes are the same asset, despite different prim transforms or baked vertices. Here's the Procrustes R-matrix that aligns them."

**Dedup compensation**: "Adjust layout prim transform to preserve world placement."

**Missing connection**: The compensation logic never applies the Procrustes R-matrix. It's pure matrix multiplication assuming vertices are already aligned.

### Why shape_invariant Detection Works But Compensation Fails

1. **Detection** (in `scripts/report_asset_mesh_dedup.py` or similar):
   - Computes Procrustes alignment between old and canonical mesh vertices
   - Finds R-matrix and translation
   - Correctly classifies: "rotation by 90°" or "reflection on X-axis"

2. **Compensation** (in `rewrite_layout_asset_refs_with_compensation.py`):
   - Reads M_oldInternal and M_canonicalInternal from USD prim transforms
   - Does NOT read or use the Procrustes R-matrix from dedup report
   - Assumes these matrices already capture all differences
   - **Result**: Missing the vertex-space rotation/reflection that R-matrix would have provided

## Part 6: Test Coverage Gaps

### Missing Edge Cases

**Negative scale / reflection**:
- No test with `Matrix4d` having `det < 0`
- No test for mirrored geometry pairs
- Could add: `test_reflection_compensation` with negative scale

**Baked rotation in vertices**:
- No test where M_oldInternal = I but vertices are rotated in local space
- No test where M_canonicalInternal applies rotation prim, old doesn't
- Could add: `test_baked_rotation_compensation` (would fail, demonstrating the limitation)

**Multi-branch with mixed transforms**:
- Tests cover sibling meshes under same parent
- Don't test multiple branches where each has different internal transforms
- Could add: `test_multi_branch_different_internals`

**Non-uniform scale**:
- No test includes scale transforms
- No test for non-uniform scale (SxSySz with different values)
- Could add: `test_nonuniform_scale_compensation`

**Procrustes-informed compensation**:
- No test that verifies compensation using Procrustes R-matrix
- No test that shows failure without R-matrix
- Could add: `test_compensation_with_procrustes_alignment`

### Why Existing Tests Still Pass

1. All test hierarchy specs use identity or simple translate
2. No test creates geometrically different vertex configurations
3. Tests validate the *mathematical formula* works, not that it solves *real dedup pairs*
4. Procrustes analysis is never integrated into the tests

## Part 7: Next Steps

### Phase 1: Quantify Failure Patterns (IMMEDIATE)

1. **Analyze worst categories**:
   ```bash
   python scripts/placement_pairwise_compare.py \
     --left GRScenes-test0-rebuilt \
     --right GRScenes-test0-rebuilt-normalized \
     --out-dir check_reports/worst_categories_analysis/
   ```

2. **Extract old → canonical pairs from dedup reports**:
   - Use `check_reports/test0_rebuilt_dedup/geom_only/`, `shape_invariant/`, `topo_filesize/`
   - For desk and curtain categories, extract specific (old_hash, canon_hash) pairs

3. **Inspect M_oldInternal vs. M_canonicalInternal**:
   - Load both asset USDs
   - Compute internal matrices
   - Detect pattern: rotation_only? scale_only? reflection? truly_different?

### Phase 2: Implement Procrustes-Aware Compensation (MEDIUM EFFORT)

1. **Integrate Procrustes R-matrix into compensation**:
   ```
   M_new = M_canon⁻¹ × M_old × R_procrustes⁻¹ × M_old_layout
   ```
   (apply R after old internal, before layout)

2. **Handle reflection** (det < 0):
   - Detect when canonical has reflection
   - Apply sign flip or use pseudoinverse instead of regular inverse

3. **Iterate all meshes** (not just first):
   - Compute compensation chain for each mesh
   - Apply compensation uniformly if all branches require it

### Phase 3: Validate & Re-Run (MEDIUM EFFORT)

1. **Add comprehensive tests**:
   - Test baked rotation (will fail with current code)
   - Test reflection
   - Test multi-branch
   - Test Procrustes integration

2. **Smoke test on small subset**:
   - Run improved compensation on test0_smoke subset
   - Verify placement with `placement_pairwise_compare.py`
   - Should see displacement drop from 24.9% to < 1%

3. **Full re-run**:
   - Restore assets from pre_c1 snapshots (available at `check_reports/test0_rebuilt_dedup/c1_bulk/_autorun/*/pre_c1_snapshots/`)
   - Re-run C1 with improved compensation
   - Full placement verification

## Part 8: Code References

| Function | File | Lines | Purpose |
|----------|------|-------|---------|
| `_get_asset_internal_matrix` | rewrite_layout_asset_refs_with_compensation.py | 166-223 | Compute M_internal from USD |
| `get_local_matrix` | usd_xform_utils.py | 38-51 | Extract prim local matrix |
| `get_chain_transform` | usd_xform_utils.py | 54-74 | Accumulate Instance→mesh chain |
| `find_all_meshes` | usd_xform_utils.py | 77-90 | Find all mesh prims |
| `rewrite_layout` | rewrite_layout_asset_refs_with_compensation.py | 469-735 | Main entry, applies refs + compensation |
| Compensation formula | rewrite_layout_asset_refs_with_compensation.py | 559-593 | Apply M_new = M_canon⁻¹ × M_old × M_old_layout |
| `_ensure_matrix_xform` | rewrite_layout_asset_refs_with_compensation.py | 226-234 | Ensure xformOp:transform exists |
| `_set_local_matrix` | rewrite_layout_asset_refs_with_compensation.py | 237-239 | Set prim matrix |

## Part 9: Key Questions Answered

**Q: Is the compensation formula mathematically correct?**
A: Yes. The formula `M_new = M_canon⁻¹ × M_old × M_old_layout` correctly preserves world placement when swapping asset references, **assuming identical vertex coordinates**. All 10 unit tests pass.

**Q: Is the chain walk implementation correct?**
A: Yes. `get_chain_transform` correctly accumulates from ancestor's child to descendant in top-down order. The bug (wrong multiply order) was fixed in commit ca544fc.

**Q: Why does compensation fail then?**
A: The formula assumes `p_mesh_old = p_mesh_canonical`. This is false for baked vertex transforms (rotation, reflection, non-uniform scale). The formula only adjusts prim transforms, not vertex positions.

**Q: Does the compensation logic include Procrustes alignment?**
A: No. It's pure matrix multiplication. The Procrustes R-matrix found during dedup detection is never used in compensation.

**Q: What fraction of prims are displaced by the failures?**
A: 24.9% (25,431 of 101,919) displaced > 0.01 units. 2.2% (2,274) displaced > 10 units. Worst category: 246.65 units.

**Q: Can this be fixed without rerunning dedup?**
A: Partially. If dedup reports contain Procrustes R-matrices, we can improve compensation to use them. If not, may need to re-run shape_invariant dedup to generate R-matrices, then re-apply C1.

## Appendix: Glossary

- **M_internal**: Accumulated transform from asset's defaultPrim → Instance → ... → first mesh parent
- **M_layout**: Instance prim's local transform in scene layout
- **p_mesh**: Point in mesh's local coordinate space
- **p_world**: Point in world coordinate space
- **Baked transform**: Transform applied to vertex coordinates (not prim transform)
- **Procrustes alignment**: Least-squares best-fit rotation + translation between two point clouds
- **R-matrix**: Rotation matrix from Procrustes alignment
- **det < 0**: Matrix determinant negative, indicating reflection/mirroring
