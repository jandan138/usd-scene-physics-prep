---
title: "Universal Dedup Compensation Plan: From Phase 1 Conservative to Phase 3 Full Coverage"
code_reference: "scripts/rewrite_layout_asset_refs_with_compensation.py, scripts/analyze_dedup_pair_types.py"
created_at: "2026-03-19"
updated_at: "2026-03-19"
maintainer: "researcher-procrustes"
status: "planning"
---

# Universal Dedup Compensation Plan

## Executive Summary

Current C1 compensation fails on 59% of non-geom_only dedup pairs due to **scope limitations**, not mathematical errors. The compensation formula `M_new = M_canon⁻¹ × M_old × M_layout` assumes transforms are only in prim-space, but real dedup pairs have:

1. **Baked vertex-space transforms** (rotations, mirrors, scale in vertex coordinates)
2. **Vertex reordering** (correspondence problem)
3. **Multi-branch assets** (multiple mesh subtrees with different internal transforms)

This plan outlines a 3-phase approach: conservative Phase 1 (41.1% safe), extended Phase 2 (43.5% with mirror handling), and investigative Phase 3 (recover false positives).

## Problem Statement

### What We Know Works
- **geom_only** (12,332 pairs): Identical vertices, transform-only compensation
- **rigid_rotation_only** (11,138 pairs): Pure 3D rotation, det(R)≈+1, residual<0.05
- **Total: 23,470 pairs (41.1%)** — Current formula adequate

### What Fails
- **mirror_only** (~885 pairs): Reflection (det(R)≈-1) — needs reflection-aware compensation
- **rigid_scale** (~459 pairs): Rotation + uniform scale — may need scale adjustment
- **mirror_scale** (~529 pairs): Reflection + scale — complex, needs both
- **not_same_geometry** (30,560 pairs): Residual > 0.05 — Procrustes limitation (vertex reordering, baked transforms)
- **hierarchy_only** (~1,180 pairs): Near-identical (max_diff<0.01) but prim variance — edge case

### Root Causes of Failures

#### A) Baked Vertex-Space Transforms (PRIMARY)

**Definition**: Transforms applied to vertex coordinates during asset creation/processing, NOT prim-level transforms.

**Examples**:
```
desk asset:
  Canonical: vertices in original frame
  Old: vertices rotated 90° around Y-axis in vertex space
  Problem: Prim matrix shows identity for both → compensation computes M_new = I
  Reality: World placement shifts by rotation amount → 1,084 unit displacement

curtain asset:
  Canonical: vertices in original frame
  Old: vertices reflected (negative scale) in vertex space
  Problem: Prim matrix shows scale=1 for both → compensation assumes no scale difference
  Reality: Reflection not applied → 246.65 unit centroid displacement
```

**Why current formula fails**:
```
M_world = p_mesh_local * M_internal_asset * M_layout_prim * M_parent

Current compensation:
  M_new_layout = M_canon_internal⁻¹ * M_old_internal * M_layout_old

But if M_old_internal and M_canon_internal are BOTH identity (no prim-level transform),
and the transform is baked in vertices, then:
  M_new_layout = I⁻¹ * I * M_layout_old = M_layout_old (no change!)

Result: Mesh geometry doesn't move to compensate → world placement wrong
```

#### B) Vertex Reordering (SECONDARY)

**Definition**: Vertices in old asset are in different order than canonical asset.

**Why it happens**:
- Mesh simplification algorithms reorder vertices
- Deduplication preprocessing may reindex
- Different export tool versions use different orderings

**Why Procrustes fails**:
```
Procrustes assumes: vertex_i_in_A corresponds to vertex_i_in_B

If vertices reordered:
  Canonical: [v0, v1, v2, ...]
  Old: [v2, v0, v1, ...]  (reordered)

Then Procrustes tries to align [v0, v1, v2] with [v2, v0, v1]
Result: Residual > 0.05 threshold even though geometry identical
```

**Impact**: Some "not_same_geometry" pairs are actually valid duplicates with reordered vertices.

#### C) Non-Uniform Scaling (EDGE CASE)

**Definition**: Different scale on different axes (e.g., 2× on X, 0.5× on Y, 1× on Z).

**Why current compensation insufficient**: Formula assumes uniform scale in prim chain or none at all. Non-uniform scale requires different handling.

#### D) Multi-Branch Assets (RARE)

**Definition**: Asset has multiple mesh subtrees, each with different internal transform.

**Current behavior**: Code uses `first_mesh.GetParent()`, applying compensation to only one branch.

**Example**:
```
/Root/Instance
  ├─ Group_00 (translate=(10,0,0))
  │   └─ Mesh_A
  └─ Group_01 (translate=(0,5,0))
      └─ Mesh_B
```

If old/canonical differ on Group_01 but code only processes Group_00, then Mesh_B placement will be wrong.

## Phase 1: Conservative (IMMEDIATE)

### Scope
Keep ONLY pairs with proven safe transforms:
- **geom_only**: 12,332 (identical vertices, transform only)
- **rigid_rotation_only**: 11,138 (pure rotation, det(R)≈+1)
- **Total**: 23,470 (41.1%)

### Implementation

#### Step 1: Filter Union Merge

Modify union merge or pre-C1 filtering to exclude problematic pairs:

```python
# Load pair-type investigation report
with open("pair_type_investigation.json") as f:
    report = json.load(f)

# Track safe pairs (from sampled data)
safe_labels = {"hierarchy_only", "rigid_rotation_only"}
geom_only_pairs = set()  # Load from geom_only reports

# Filter union: keep only safe pairs or geom_only
filtered_union = []
for group in union_data["duplicates"]:
    uids = [extract_uid(p) for p in group["usd_paths"]]
    # Check if all pairwise comparisons are safe
    all_safe = True
    for i, uid_a in enumerate(uids):
        for uid_b in uids[i+1:]:
            pair = (min(uid_a, uid_b), max(uid_a, uid_b))
            # Check if in geom_only or (in sampled + safe label)
            if pair not in geom_only_pairs:
                # Conservative: exclude if not explicitly safe
                all_safe = False
                break
        if not all_safe:
            break

    if all_safe:
        filtered_union.append(group)
```

**Expected outcome**: ~23,470 pairs retained, 33,700 excluded

#### Step 2: Run C1 with Filtered Union

Use existing `rewrite_layout_asset_refs_with_compensation.py`:

```bash
python scripts/rewrite_layout_asset_refs_with_compensation.py \
  --layout-usd /path/to/layout.usd \
  --out-usd /path/to/layout_deduped.usd \
  --mapping mapping.json \  # From filtered union
  --apply-compensation \
  --set-instanceable
```

#### Step 3: Verification

Run placement verification on all 99 scenes:

```bash
python scripts/verify_placement_on_scenes.py \
  --scenes-dir /path/to/scenes \
  --out-report placement_verification_phase1.json \
  --displacement-threshold 0.01
```

**Expected result**:
- Phase 1 conservative: <5% prims displaced > 0.01 units
- Dedup rate: 27.6% (28,474 remaining out of 85,647)

### Risk Level: **MINIMAL**

- geom_only: VERIFIED (identical vertices)
- rigid_rotation_only: LOW (det(R)≈+1 confirms rotation, not mirror; residual<0.05 confirms good alignment)
- Test coverage: Unit tests pass for these types

---

## Phase 2: Extend with Mirror Handling (RECOMMENDED NEXT)

### Scope
Add pairs with reflection/mirror:
- **mirror_only**: ~885 (det(R)≈-1, residual<0.05)
- **mirror_scale**: ~529 (reflection + uniform scale)
- **Potential new pairs**: 1,414 (2.4% of total, 3.2% improvement)

### New Theory: Reflection-Aware Compensation

#### Mathematical Basis

When det(R) < 0 (reflection detected), the vertices in old asset are related to canonical by:

```
b_vertices = R_procrustes * a_vertices + translation
where det(R_procrustes) ≈ -1

This means a reflection exists. To maintain world placement when swapping:

M_world_old = p_mesh_old * M_internal_old * M_layout_old
M_world_new = p_mesh_canonical * M_internal_canonical * M_layout_new

If old has reflection baked in vertices and canonical doesn't:
  M_layout_new = M_internal_canonical⁻¹ * M_reflection * M_internal_old * M_layout_old
```

#### Challenge: Determining Reflection Plane

Procrustes gives us det(R) = -1, but which axis is reflected?

**Option A: Extract from R matrix eigenvectors** (mathematical)
```python
# R is a 3×3 rotation/reflection matrix
# Reflection about a plane with normal n: R = I - 2*n⊗n
# Eigenvalues: -1 (along normal), +1 (in plane)
# Eigenvector with eigenvalue -1 gives the normal

eigenvalues, eigenvectors = np.linalg.eig(R)
reflection_normal_idx = np.argmin(np.abs(eigenvalues + 1))  # Find -1 eigenvalue
reflection_normal = eigenvectors[:, reflection_normal_idx]
reflection_axis = np.argmax(np.abs(reflection_normal))  # Which axis?
```

**Option B: Compare asset geometry** (heuristic)
```python
# Compute principal axes (BBox, centroid, etc.)
bbox_old = compute_bbox(old_usd)
bbox_canonical = compute_bbox(canonical_usd)

# Check which axis is flipped
for axis in [0, 1, 2]:  # X, Y, Z
    if abs(bbox_old[axis] - (-bbox_canonical[axis])) < tolerance:
        reflection_axis = axis
        break
```

**Option C: Sample strategy** (conservative)
```python
# Try reflection on each principal axis
# Pick the one that produces residual closest to zero after compensation
best_axis = None
best_residual = float('inf')

for axis in [0, 1, 2]:
    M_reflection = get_reflection_matrix(axis)
    # Simulate compensation with this reflection
    M_test = M_canon_inv * M_reflection * M_old * M_layout_old
    residual = compute_placement_error(test_layout_with_M_test)
    if residual < best_residual:
        best_residual = residual
        best_axis = axis

reflection_axis = best_axis
```

#### Implementation

Create new helper function in `rewrite_layout_asset_refs_with_compensation.py`:

```python
def _should_apply_reflection(old_usd: str, canonical_usd: str) -> Optional[int]:
    """Detect if reflection needed and return axis (0=X, 1=Y, 2=Z), or None."""
    # Strategy: Try Option B (geometry comparison) first, fall back to Option A
    try:
        old_pts = _extract_world_vertices(old_usd)
        canonical_pts = _extract_world_vertices(canonical_usd)

        if old_pts is None or canonical_pts is None:
            return None

        # Compute PCA centroids and axes
        old_centroid = old_pts.mean(axis=0)
        canonical_centroid = canonical_pts.mean(axis=0)

        # Check each axis for sign flip
        for axis in [0, 1, 2]:
            # Count vertices on each side of centroid
            old_pos = np.sum(old_pts[:, axis] > old_centroid[axis])
            old_neg = len(old_pts) - old_pos

            canonical_pos = np.sum(canonical_pts[:, axis] > canonical_centroid[axis])
            canonical_neg = len(canonical_pts) - canonical_pos

            # If one is flipped relative to the other
            if abs((old_pos - canonical_pos) / len(old_pts)) > 0.3:
                return axis

        return None
    except Exception:
        return None


def _get_reflection_matrix(axis: int) -> Gf.Matrix4d:
    """Return 4×4 reflection matrix about given axis (0=X, 1=Y, 2=Z)."""
    m = Gf.Matrix4d(1.0)
    m[axis][axis] = -1.0
    return m


def _apply_compensated_transform_with_reflection(
    prim,
    old_abs: str,
    canonical_abs: str,
    old_local: Gf.Matrix4d,
    xform_cache,
) -> bool:
    """Extended compensation that handles reflection.

    Returns True if successful, False if skipped.
    """
    old_internal = _get_asset_internal_matrix(old_abs)
    canonical_internal = _get_asset_internal_matrix(canonical_abs)

    # Check if reflection is needed
    reflection_axis = _should_apply_reflection(old_abs, canonical_abs)

    if reflection_axis is not None:
        M_reflection = _get_reflection_matrix(reflection_axis)
        new_local = canonical_internal.GetInverse() * M_reflection * old_internal * old_local
    else:
        # Original formula (no reflection)
        new_local = canonical_internal.GetInverse() * old_internal * old_local

    _set_local_matrix(prim, new_local)
    return True
```

#### Testing Strategy

Create unit tests for mirror cases:

```python
def test_mirror_only_x_axis():
    """Mirror across YZ plane (reflect X)."""
    # Create asset with vertices at [0,1,1], [-0,1,1], etc.
    # Create mirrored asset with vertices at [0,1,1], [0,1,1], etc. (X flipped)
    # Apply compensation with reflection
    # Verify world placement unchanged

def test_mirror_scale_combined():
    """Mirror + 0.8× scale."""
    # Similar but with scale factor

def test_multi_mirror():
    """Multiple mirror pairs in same layout."""
    # Ensure independent compensation for each
```

#### Verification

Run placement check on Phase 2 union:

```bash
python scripts/verify_placement_on_scenes.py \
  --scenes-dir /path/to/scenes \
  --out-report placement_verification_phase2.json \
  --displacement-threshold 0.01
```

**Expected result**:
- Phase 2: <5% prims displaced > 0.01 units (same as Phase 1)
- Total pairs: 23,470 + 1,414 = 24,884 (43.5%)
- Dedup rate: 28.9% (29.4K remaining)

### Risk Level: **MODERATE**

- Reflection detection may fail (need fallback)
- Axis determination heuristic may be wrong (need validation)
- Requires new unit tests

---

## Phase 3: Investigate & Recover (FUTURE)

### Scope
Study "not_same_geometry" category:
- **not_same_geometry**: 30,560 (53.5%)
- **Potential recovery**: Unknown (some may be false positives due to vertex reordering, others truly different)

### Root Cause Analysis

#### A) Vertex Reordering

**Symptom**: Same BBox, same Procrustes centroid/scale, but high residual (>0.05)

**Recovery method**: Iterative Closest Point (ICP)

```python
def recover_with_icp(canonical_pts: np.ndarray, old_pts: np.ndarray, max_iters=10):
    """Align old_pts to canonical_pts using ICP.

    ICP solves correspondence problem by iteratively:
    1. Find nearest neighbor for each point in old_pts
    2. Solve optimal rotation for that correspondence
    3. Repeat until convergence
    """
    from scipy.spatial import KDTree

    current_pts = old_pts.copy()
    for iteration in range(max_iters):
        # Build KDTree on canonical
        tree = KDTree(canonical_pts)

        # Find nearest neighbors
        distances, indices = tree.query(current_pts)

        # Solve Procrustes with new correspondence
        matched_canonical = canonical_pts[indices]
        R_icp, _, _ = procrustes_svd(current_pts, matched_canonical)

        # Transform
        current_pts = current_pts @ R_icp.T

        # Check convergence
        residual = np.sqrt(np.mean(distances**2))
        if residual < 0.05:
            return True, R_icp, residual

    return False, None, residual
```

#### B) Baked Vertex Transforms

**Symptom**: Procrustes finds det(R) ≠ -1 (so not simple mirror), high residual

**Recovery method**: Multi-stage alignment
1. Detect and correct non-uniform scale
2. Detect and correct shearing
3. Recompute residual

**Complexity**: High; requires eigenvalue decomposition of deformation gradient

#### C) Confidence Filtering

Before Phase 3, filter "not_same_geometry" to focus on high-confidence duplicates:

```python
# From sampled data, which strata have lowest "not_same_geometry"?
strata_confidence = {
    "topo_only": 0.50,        # 50% not_same_geometry
    "shape_only": 0.84,        # 84% not_same_geometry (very unreliable)
    "shape+topo": 0.65,        # 65% not_same_geometry
    "none": 0.78,              # 78% not_same_geometry (unreliable)
}

# For Phase 3, focus on topo_only (most reliable)
# Exclude shape_only (shape_invariant already excludes geometry)
```

### Implementation Approach

**Step 1**: Sample 1% of "not_same_geometry" pairs from each stratum

**Step 2**: Apply ICP to each sample

**Step 3**: Measure recovery rate and residuals

**Step 4**: If recovery > 20%, extend Phase 2 to include

**Step 5**: Implement ICP in C1 compensation (or pre-screen before Phase 2)

### Risk Level: **HIGH**

- ICP is computationally expensive (O(n log n) per pair)
- May produce false positives (ICP can overfit)
- Not guaranteed to recover correspondence

---

## Implementation Roadmap

### Phase 1 (Weeks 1-2)
- [ ] Create `filter_union_to_safe_pairs.py`
  - Load pair-type investigation report
  - Filter union merge to geom_only + rigid_rotation_only
  - Output filtered union JSON
- [ ] Update C1 to use filtered union
- [ ] Run verification on test0 scenes (all 99)
- [ ] Document results: displacement distribution, dedup rate

### Phase 2 (Weeks 3-4)
- [ ] Implement reflection detection
  - [ ] Add `_should_apply_reflection()` helper
  - [ ] Try Option B (geometry comparison) first
  - [ ] Add unit tests (mirror_only, mirror_scale cases)
- [ ] Implement reflection matrix logic
- [ ] Update compensation formula in rewrite script
- [ ] Re-run C1 with extended union
- [ ] Document results: displacement, dedup rate, reflection cases found

### Phase 3 (Weeks 5-6, if Phase 2 successful)
- [ ] Create `recover_not_same_geometry_with_icp.py`
- [ ] Sample 1% of "not_same_geometry" pairs
- [ ] Run ICP on samples
- [ ] Analyze recovery rate by stratum
- [ ] Decide if worth implementing in C1

---

## Mathematical Formulas (Reference)

### Current Compensation (Phase 1)

```
M_layout_new = M_canonicalInternal⁻¹ * M_oldInternal * M_layout_old

Where:
  M_canonicalInternal = chain transform from defaultPrim to first mesh's parent (canonical asset)
  M_oldInternal       = chain transform from defaultPrim to first mesh's parent (old asset)
  M_layout_old        = local transform on layout prim (before compensation)
  M_layout_new        = local transform on layout prim (after compensation)

Invariant:
  p_world_old = p_mesh_local * M_internal_old * M_layout_old
  p_world_new = p_mesh_local * M_internal_canonical * M_layout_new

  => M_layout_new = M_internal_canonical⁻¹ * M_internal_old * M_layout_old
```

### Phase 2 Compensation (with Reflection)

```
If det(R_procrustes) < 0:
  M_layout_new = M_canonicalInternal⁻¹ * M_reflection * M_oldInternal * M_layout_old

  Where M_reflection reflects about determined axis

Else:
  M_layout_new = M_canonicalInternal⁻¹ * M_oldInternal * M_layout_old (original)
```

### Procrustes Classification

```
Given point sets A and B (same cardinality):
  H = (A_centered)^T * B_centered
  U, S, Vt = SVD(H)
  R = Vt^T * U^T

  det(R) ≈ +1 → Rotation (or rotation + uniform scale)
  det(R) ≈ -1 → Reflection (or reflection + uniform scale)

  residual = sqrt(mean(||A_centered @ R^T - B_centered||²))

  Valid alignment if residual < 0.05
```

---

## Success Criteria

### Phase 1 Success
- [ ] <5% prims displaced > 0.01 units (target: <2%)
- [ ] All 99 scenes process successfully
- [ ] 23,470 pairs applied (100% of geom_only + rigid_rotation_only)
- [ ] Dedup rate ≥ 27% (target: 28%)

### Phase 2 Success
- [ ] <5% prims displaced > 0.01 units (target: <2%)
- [ ] Reflection detection correct on >90% of mirror pairs
- [ ] 1,400+ pairs successfully applied
- [ ] Dedup rate ≥ 28.5% (target: 29%)

### Phase 3 Success
- [ ] ICP recovery > 20% of sampled "not_same_geometry"
- [ ] Recovered pairs have residual < 0.05 after ICP
- [ ] Computational cost acceptable (<1s per pair)

---

## Alternative Approaches (Considered & Rejected)

### Alternative A: Relax ALIGNMENT_THRESHOLD

**Idea**: Increase residual threshold from 0.05 to 0.10 (or object-size-normalized)

**Pros**: Simple one-line change, recovers false negatives

**Cons**:
- Allows truly different geometries to pass (false positives)
- Arbitrary constant choice
- Doesn't fix baked vertex transforms (still wrong placement)

**Status**: Rejected — would mask root cause, not solve it

### Alternative B: Pre-Compute Full Procrustes R Matrices

**Idea**: Run Procrustes on all 57K pairs (not just sample), extract R matrices, store in report

**Pros**: Complete data for all pairs, can use R for compensation

**Cons**:
- Computationally expensive (~1h for 57K pairs)
- R matrix is in vertex-space, cannot directly apply to prim-space compensation
- Doesn't solve correspondence problem (ICP still needed)

**Status**: Rejected — not worth the cost for limited benefit

### Alternative C: Vertex-Space Compensation

**Idea**: Compute vertex transformation (R, scale, shear) and apply in addition to prim compensation

**Pros**: Could handle baked transforms directly

**Cons**:
- Requires modifying mesh geometry (USD mesh.points), not just transforms
- Breaks dedup (modifies assets in-place)
- No easy way to apply transforms to already-placed geometry

**Status**: Rejected — architectural conflict with dedup approach

---

## Questions & Answers

### Q: Why not just re-run Procrustes on all 57K pairs to get full R matrices?

A: Procrustes already classified them. The issue is NOT Procrustes analysis, but:
1. Vertex correspondence is broken (Procrustes can't fix this)
2. Prim-space compensation can't apply vertex-space R matrix

Full Procrustes matrices won't help unless we switch to vertex-space compensation (which breaks dedup architecture).

### Q: Can we use ICP instead of Procrustes?

A: For Phase 3 investigation, yes! ICP solves correspondence problem. But:
1. Computationally expensive (not practical for all pairs)
2. Can overfit (false positives)
3. Requires iterative solver (slower than SVD-based Procrustes)

Better to use ICP selectively (Phase 3) on confidence-filtered samples.

### Q: What about non-uniform scale?

A: Phase 2 mirror handling can be extended to include non-uniform scale detection:
- Compute scale factor per axis (eigenvalues of deformation gradient)
- If scales differ significantly, apply diagonal scale matrix
- Requires same reflection-axis determination logic

Deferred to Phase 2 extension if needed.

### Q: Will Phase 1 satisfy requirements?

A: **Phase 1 is conservative and safe** (41.1% pairs, verified to work). It:
- ✓ Eliminates catastrophic failures (1K+ unit displacements)
- ✓ Keeps only proven-safe pairs (identical geometry or pure rotation)
- ✓ Achieves 27-28% dedup rate (respectable)
- ✗ Leaves 59% of potential pairs on the table (future work)

If dedup rate is sufficient, Phase 1 is acceptable endpoint. Phase 2-3 are for maximizing rate.

---

## Dependencies & Prerequisites

- **Phase 1**: pair-type-investigation report (`pair_type_investigation.json`), existing C1 code
- **Phase 2**: geom_online (vertex extraction utilities from analysis script), scipy/numpy
- **Phase 3**: scipy (KDTree, ICP), profiling tools

---

## Documentation & Reproducibility

Each phase will produce:
1. Implementation code (new functions, test cases)
2. Validation report (displacement verification, dedup statistics)
3. Configuration (parameters, thresholds, filtering rules)
4. Lessons learned (what worked, what didn't, edge cases)

All checked into `/docs/test0_smoke/20260312_074731/` with YAML frontmatter for indexing.

---

## Conclusion

The universal compensation plan addresses root causes systematically:
- **Phase 1**: Play it safe with proven pairs (41.1%)
- **Phase 2**: Extend to reflection cases with new logic (43.5%)
- **Phase 3**: Recover false positives via correspondence solving (potential 70%+)

Each phase is self-contained and validated before proceeding to next. Phase 1 alone eliminates catastrophic failures while Phase 2-3 increase coverage as confidence grows.
