---
title: "Scale-Invariant Dedup Proposal: --mode shape_invariant"
code_reference: scripts/report_asset_mesh_dedup.py
created_at: 2026-03-11
updated_at: 2026-03-11
maintainer: zhuzihou
status: draft
---

# Scale-Invariant Dedup Proposal

## 1. Problem Statement

The current dedup algorithm in `report_asset_mesh_dedup.py` has three known blind spots that prevent it from catching visually identical assets:

| Blind Spot | Root Cause | Example |
|------------|-----------|---------|
| **Scale variants** | `geom_only` hashes raw vertex coordinates; a bottle at scale (1,1,1) and (2,2,2) have completely different vertex positions | Same mesh exported at different sizes |
| **Face winding/ordering** | Topology hash includes `face_vertex_indices` in their original order; reordered faces with identical geometry produce different hashes | Re-exported meshes, different DCC tools |
| **Strict topology match for tolerance merge** | `--merge-tolerance` requires identical `topo_sig_hex` before doing pairwise vertex comparison; assets with reordered faces never reach the vertex comparison stage | Same mesh with faces in different order |

The `--merge-tolerance` fix (commit `993d292`) solved the float-noise bucket-boundary problem but does not address these three issues. The 3 known bottle assets (f418b527, f3e81129, f525e096) happened to share identical topology, so tolerance merge caught them. But assets with the same shape at different scales, or with reordered faces, remain undetected.

## 2. Proposed Solution: `--mode shape_invariant`

Add a new dedup mode that produces **scale-invariant, topology-relaxed** signatures. This mode runs alongside (not replacing) the existing `geom_only` / `scale_only` / `full_matrix` reports.

### 2.1 Architecture Overview

```
Asset USD
  |
  v
[Per-mesh processing]
  |
  +---> Shape descriptor (cheap pre-filter)
  |       - vertex count, face count
  |       - normalized surface area ratio
  |       - normalized volume
  |       - bbox aspect ratios (sorted)
  |
  +---> Scale-invariant signature
  |       - Normalize vertices to unit bounding box
  |       - Sort vertex positions lexicographically
  |       - Hash sorted positions (ignoring face topology)
  |
  v
[Asset-level aggregation]
  |
  +---> Pre-filter: group by shape descriptor hash
  |       (vertex_count, face_count, quantized aspect ratios)
  |
  +---> Pairwise comparison within groups:
  |       Hausdorff distance on unit-normalized vertices
  |
  v
[Output: shape_invariant dedup report]
```

### 2.2 Step-by-Step Design

#### Step A: Vertex Normalization to Unit Bounding Box

For each mesh, compute the axis-aligned bounding box (AABB) and normalize all vertices to `[0, 1]^3`:

```
bbox_min = min(points, axis=0)
bbox_max = max(points, axis=0)
extent = bbox_max - bbox_min
# Avoid division by zero on degenerate axes
extent = max(extent, 1e-10) per axis
# Use uniform scale to preserve aspect ratio
scale = max(extent)
normalized_points = (points - bbox_min) / scale
```

Key decisions:
- **Uniform scale** (divide by `max(extent)`, not per-axis): preserves the aspect ratio of the mesh. A tall thin bottle and a short wide bottle should NOT match.
- **Translation to origin**: removes position dependence.
- **No rotation normalization** (PCA alignment): too fragile for meshes with near-symmetric principal axes. Instead, we rely on sorted vertex positions for rotation-independent comparison when needed. For the primary use case (same mesh at different scales), rotation is already identical.

#### Step B: Topology-Relaxed Signature

Instead of hashing `face_vertex_counts` + `face_vertex_indices` (which is order-sensitive), compute a signature from **sorted vertex positions**:

```python
def _shape_invariant_mesh_sig(normalized_points, eps):
    # Sort vertices lexicographically (x, then y, then z)
    sorted_pts = sorted(normalized_points, key=lambda p: (
        _quantize(p[0], eps),
        _quantize(p[1], eps),
        _quantize(p[2], eps),
    ))
    h = _sha256_init("mesh_shape_invariant_v1")
    h.update(struct.pack("<I", len(sorted_pts)))
    for p in sorted_pts:
        _hash_update_floats(h, (p[0], p[1], p[2]), eps=eps)
    return h.hexdigest()
```

This signature is:
- **Scale-invariant**: vertices are normalized to unit bbox first.
- **Translation-invariant**: bbox is shifted to origin.
- **Face-order-invariant**: faces are not included; only sorted vertex positions matter.
- **NOT rotation-invariant**: intentionally, to avoid false positives between rotationally-distinct objects.

Trade-off: ignoring face topology means two meshes with identical vertex positions but different face connectivity (e.g., one triangulated, one quad) would match. This is acceptable because the vertices define the shape, and face connectivity is a tessellation detail.

#### Step C: Shape Descriptor Pre-Filter

Before pairwise comparison, group assets by a cheap shape descriptor to reduce the O(n^2) comparison space:

```python
@dataclass(frozen=True)
class ShapeDescriptor:
    vertex_count: int
    face_count: int
    aspect_ratio_hash: str  # quantized sorted bbox aspect ratios

def _compute_shape_descriptor(points, face_count, eps=0.01):
    bbox_min = [min(p[i] for p in points) for i in range(3)]
    bbox_max = [max(p[i] for p in points) for i in range(3)]
    extents = [bbox_max[i] - bbox_min[i] for i in range(3)]
    max_extent = max(extents) or 1e-10

    # Aspect ratios normalized to longest axis, sorted for orientation invariance
    ratios = sorted([e / max_extent for e in extents])
    quantized = tuple(_quantize(r, eps) for r in ratios)

    h = hashlib.sha256()
    for r in quantized:
        h.update(struct.pack("<d", r))

    return ShapeDescriptor(
        vertex_count=len(points),
        face_count=face_count,
        aspect_ratio_hash=h.hexdigest()[:16],
    )
```

Assets must match on `(vertex_count, aspect_ratio_hash)` to be compared pairwise. `face_count` is included in the descriptor for informational purposes but NOT used as a grouping key (to allow triangulated vs. quad variants to match).

**Why not include face_count in the grouping key**: Two identical shapes can have different face counts if one was re-tessellated. However, vertex count must match because we compare vertex positions pairwise.

#### Step D: Pairwise Hausdorff Distance

Within each shape-descriptor group, for assets whose `shape_invariant_sig` does not match exactly (hash miss due to float noise), compute the **bidirectional Hausdorff distance** on unit-normalized vertices:

```python
def _hausdorff_distance(pts_a, pts_b):
    """Bidirectional Hausdorff distance between two point sets.

    For equal-sized sorted point sets, this reduces to max per-point distance.
    For unequal sizes, compute true Hausdorff (max of min distances).
    """
    if len(pts_a) == len(pts_b):
        # Fast path: paired comparison (like existing _max_vertex_distance
        # but on unit-normalized vertices)
        return max(
            math.sqrt(sum((a[i]-b[i])**2 for i in range(3)))
            for a, b in zip(pts_a, pts_b)
        )

    # Slow path: true Hausdorff for different vertex counts
    # (should be rare given pre-filter on vertex_count)
    def _directed(src, dst):
        max_min_d = 0.0
        for s in src:
            min_d = min(
                math.sqrt(sum((s[i]-d[i])**2 for i in range(3)))
                for d in dst
            )
            max_min_d = max(max_min_d, min_d)
        return max_min_d

    return max(_directed(pts_a, pts_b), _directed(pts_b, pts_a))
```

**Threshold**: Since vertices are unit-normalized, the tolerance is relative to mesh size. A threshold of `0.005` on unit-normalized vertices means shapes are considered identical if they differ by less than 0.5% of their bounding box extent. This is much more robust than an absolute tolerance on raw coordinates.

#### Step E: Asset-Level Aggregation

For multi-mesh assets, aggregate per-mesh shape signatures the same way as existing code (sorted concatenation). The shape descriptor is computed per-asset by combining all meshes' vertices into a single point cloud.

### 2.3 New CLI Interface

```bash
./scripts/isaac_python.sh scripts/report_asset_mesh_dedup.py \
    --assets-root GRScenes-test1-normalized/GRScenes_assets/bottle \
    --out-dir check_reports/bottle_shape \
    --dataset test1_bottle \
    --mode shape_invariant \
    --shape-tolerance 0.005 \
    --float-quantize-eps 1e-2
```

New arguments:
- `--mode shape_invariant`: enables the new mode. Produces a 4th report file `<dataset>_asset_mesh_dedup_shape_invariant.json` in addition to the 3 existing reports.
- `--shape-tolerance 0.005`: Hausdorff distance threshold on unit-normalized vertices for pairwise comparison (default: 0.005).

Backward compatibility: when `--mode` is not specified (or set to `all`), the existing 3 reports are generated as before. When `--mode shape_invariant` is specified, the shape_invariant report is also generated.

## 3. Code Changes Required

### 3.1 New Data Structures

```python
@dataclass(frozen=True)
class MeshSig:
    # ... existing fields ...
    shape_invariant_sig_hex: str   # NEW: scale-invariant sorted-vertex hash
    shape_descriptor_key: str      # NEW: pre-filter grouping key

@dataclass(frozen=True)
class AssetRecord:
    # ... existing fields ...
    asset_shape_invariant_sig_hex: str  # NEW
    asset_shape_descriptor_key: str     # NEW
```

### 3.2 Changes to `_compute_mesh_sigs()`

After computing `geom_sig_hex` and `topo_sig_hex`, add:

1. Compute AABB, normalize vertices to unit bbox (uniform scale).
2. Sort normalized vertices lexicographically.
3. Hash sorted positions to produce `shape_invariant_sig_hex`.
4. Compute shape descriptor (vertex count + quantized aspect ratios) to produce `shape_descriptor_key`.

### 3.3 New Function: `_shape_invariant_merge()`

Similar structure to `_tolerance_merge()` but:

1. Group by `asset_shape_descriptor_key` instead of `asset_topo_sig_hex`.
2. Within each group, check `asset_shape_invariant_sig_hex` for exact matches first.
3. For non-matching pairs, load meshes, normalize to unit bbox, sort vertices, compute Hausdorff distance.
4. Merge pairs within `--shape-tolerance`.

### 3.4 Changes to `_write_report()`

Add a new mode `"shape_invariant"` that uses `asset_shape_invariant_sig_hex` as the grouping key, then applies `_shape_invariant_merge()`.

### 3.5 Changes to `main()`

- Add `--mode` and `--shape-tolerance` arguments.
- When mode includes `shape_invariant`, generate the 4th report.

## 4. Expected Impact

### 4.1 Bottle Category

Current results with `--merge-tolerance 0.005`:
- 175 groups, 936 assets, 55.1% dedup rate

Expected with `--mode shape_invariant`:
- Should catch additional duplicates where the same bottle mesh exists at different scales (e.g., a small bottle and a large bottle that are geometrically identical after normalization).
- Conservative estimate: 5-15% additional dedup on top of the existing 55.1%.

### 4.2 Other Categories

Categories with many scale variants (e.g., `plate`, `cup`, `book`) should see significant improvement. Categories like `wall` and `ground` (which are already high-dedup from tolerance merge) may see modest gains.

### 4.3 Performance

- **Shape descriptor pre-filter** reduces pairwise comparisons by ~90-95% (most assets have different vertex counts).
- **Unit normalization + sort** is O(V log V) per mesh, negligible compared to USD stage loading.
- **Hausdorff distance** on sorted point sets is O(V) per pair (fast path) or O(V^2) per pair (slow path, rare).
- Overall runtime increase: estimated 10-20% over current `--merge-tolerance` mode.

## 5. Risks and Mitigations

| Risk | Likelihood | Mitigation |
|------|-----------|------------|
| **False positives from ignoring face topology** | Low | Shape descriptor pre-filter on vertex count; visual spot-check on first run |
| **False positives from quantization of aspect ratios** | Low | Use fine eps (0.01) for aspect ratio quantization; different shapes rarely have identical vertex counts AND aspect ratios |
| **Rotation variants treated as different** | Medium (intentional) | This is by design; rotation-invariant dedup would require PCA alignment which is fragile. Could be added as a future `--rotation-invariant` flag |
| **Multi-mesh assets with differently-ordered submeshes** | Low | Asset-level signature uses sorted submesh signatures (existing behavior) |
| **Degenerate meshes (flat planes, lines)** | Low | Guard against zero-extent axes in normalization (clamp to 1e-10) |

## 6. Future Extensions

1. **`--rotation-invariant` flag**: PCA-align vertices before normalization. Useful for detecting rotated copies of the same object. Requires careful handling of sign ambiguity in eigenvectors.

2. **Material-aware dedup**: Two geometrically identical meshes with different materials should arguably NOT be deduped. Add an optional `--include-materials` flag that includes material binding paths in the signature.

3. **Progressive dedup pipeline**: Run shape_invariant mode first to find candidates, then do a visual render comparison (Isaac Sim thumbnail) for final verification.

4. **KD-tree acceleration**: For the slow-path Hausdorff distance (unequal vertex counts), build a KD-tree on the target point set for O(V log V) instead of O(V^2).

## 7. Comparison with Existing Modes

| Mode | Scale | Translation | Rotation | Face Order | Float Noise | Use Case |
|------|-------|-------------|----------|------------|-------------|----------|
| `geom_only` | Sensitive | Sensitive | Sensitive | Sensitive | Sensitive | Exact duplicates |
| `geom_only` + `--merge-tolerance` | Sensitive | Sensitive | Sensitive | Sensitive | **Tolerant** | Near-exact duplicates with float noise |
| `scale_only` | Sensitive | Invariant | Invariant | Sensitive | Sensitive | Same mesh, different placement |
| `full_matrix` | Sensitive | Sensitive | Sensitive | Sensitive | Sensitive | Strictest; includes world transform |
| **`shape_invariant`** (proposed) | **Invariant** | **Invariant** | Sensitive | **Invariant** | **Tolerant** | Same shape at any scale/position |

## 8. Implementation Plan

1. **Phase 1**: Add `_normalize_to_unit_bbox()` and `_shape_invariant_mesh_sig()` functions. Add `shape_invariant_sig_hex` to `MeshSig`.
2. **Phase 2**: Add shape descriptor pre-filter (`ShapeDescriptor`, grouping logic).
3. **Phase 3**: Add `_shape_invariant_merge()` with Hausdorff distance comparison.
4. **Phase 4**: Wire up CLI (`--mode`, `--shape-tolerance`), add report output.
5. **Phase 5**: Test on bottle category, compare with existing results, validate no false positives.
