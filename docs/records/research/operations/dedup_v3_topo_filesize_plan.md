---
title: "Dedup V3: Topology-Invariant + File-Size Mode Implementation Plan"
code_reference: "scripts/report_asset_mesh_dedup.py, scripts/union_dedup_reports.py"
created_at: "2026-03-11"
updated_at: "2026-03-11"
maintainer: "zhuzihou"
status: "implemented"
---

# Dedup V3: Topology-Invariant + File-Size Mode

## 1. Problem Statement

### 1.1 Current Gap

The existing dedup pipeline has three modes:
- **geom_only** (hash-based): Exact vertex/face/index matching. Misses assets with reordered vertices/faces.
- **shape_invariant** (Hausdorff-based): Normalize to unit bbox, lexsort vertices, pairwise Hausdorff distance. Catches scale differences and minor float noise, but still requires vertices to be in similar lexicographic order after sorting.
- **Union merge**: Combines geom_only + shape_invariant via union-find.

**Remaining blind spot**: DCC exporters (Blender, 3ds Max, Maya) can reorder vertices AND faces independently per model instance. Two exports of the same model may have completely different `face_vertex_counts`, `face_vertex_indices`, and vertex ordering. This means:
- `geom_only` hash differs (different index arrays)
- `topo_sig_hex` differs (includes `face_vertex_counts` + `face_vertex_indices`)
- `shape_descriptor_key` may differ if vertex count changes due to vertex merging/splitting
- shape_invariant pre-filter groups them separately; even if they land in the same pre-filter group, Hausdorff lexsort-pairing assumes similar vertex ordering

The result: genuinely identical models with reordered topology are never compared against each other.

### 1.2 Proposed Approach

A new dedup mode using **topology invariants** (vertex/face counts per mesh, mesh count, material bindings) combined with **file-size approximate matching** (USD + GLB file sizes within tolerance) as a lightweight, high-precision duplicate detector.

**Key insight**: Assets that are genuinely the same model but with reordered vertices/faces will have:
- Identical vertex count and face count per mesh
- Identical mesh count
- Identical material binding structure
- Nearly identical file sizes (reordering doesn't change data volume significantly)

This approach avoids opening USD files for expensive geometry comparison in the pre-filter stage, using only filesystem metadata and annotation data.

### 1.3 Ground Truth: Bottle Triplet

Three known-duplicate bottle assets that current modes fail to group together:

| UID | GLB Size (bytes) | USD Size (bytes) | Vertex Count | Face Count |
|-----|-------------------|-------------------|--------------|------------|
| `7861bdaa89323558eb8046679f567498` | 310,084 | 103,555 | 779 | 1,444 |
| `79088d12b87f6758da805eb64c8a3582` | 310,080 | 103,432 | 779 | 1,444 |
| `79090fe893611281c78d1c237c2f5b64` | 310,080 | 103,819 | 779 | 1,444 |

- GLB size spread: 4 bytes (0.001%) -- excellent discriminator
- USD size spread: 387 bytes (0.37%) -- wider but still tight
- All three have identical topology: 779 vertices, 1,444 faces, same material bindings
- DCC exporter reordered 98.2% of face indices across instances
- Asset A (7861bd) is ~18-22% non-uniformly scaled vs B/C; B and C are near-identical (Hausdorff 0.005)
- Current dedup (geom_only + shape_invariant) both fail due to vertex/face reordering

## 2. Design

### 2.1 New Mode Name and CLI

**Mode name**: `topo_filesize`

**CLI integration**: Add to existing `--mode` choices in `report_asset_mesh_dedup.py`:

```python
parser.add_argument(
    "--mode",
    choices=["all", "shape_invariant", "topo_filesize"],
    default="all",
    ...
)
```

**New arguments**:
```python
parser.add_argument(
    "--filesize-tolerance",
    type=float,
    default=0.02,
    help="File size tolerance for topo_filesize mode. "
         "Assets are considered matching if GLB and USD sizes are within "
         "this fraction of each other (default 0.02 = 2%%)",
)
```

When `--mode topo_filesize` is specified, the script will:
1. Generate the standard 3 reports (geom_only, scale_only, full_matrix) as always
2. Additionally generate a `<dataset>_asset_mesh_dedup_topo_filesize.json` report

### 2.2 Signature Computation

#### 2.2.1 Topology Invariant Signature

The topology invariant signature captures mesh structure without depending on vertex/face ordering:

```python
@dataclass(frozen=True)
class TopoInvariantDescriptor:
    """Per-mesh topology invariant (order-independent)."""
    vertex_count: int
    face_count: int
    material_binding: Optional[str]  # bound material path relative to asset root

@dataclass(frozen=True)
class AssetTopoInvariant:
    """Asset-level topology invariant signature."""
    mesh_count: int
    mesh_descriptors: Tuple[TopoInvariantDescriptor, ...]  # sorted by (vertex_count, face_count)
    sig_hex: str  # SHA256 of the above
```

**Computation** (added to `_compute_mesh_sigs()`):

For each mesh prim:
1. Get `vertex_count = len(points)`
2. Get `face_count = len(face_vertex_counts)`
3. Get material binding path via `UsdShade.MaterialBindingAPI(prim).GetDirectBinding().GetMaterialPath()`, relativized to asset root

For the asset:
1. Collect all per-mesh `TopoInvariantDescriptor`
2. Sort by `(vertex_count, face_count, material_binding)` -- order-independent
3. Hash the sorted tuple into `sig_hex`

**Why material bindings?** Two meshes with the same vertex/face count but different materials are likely different objects (e.g., a red bottle vs a green bottle of the same shape). Including material bindings reduces false positives without adding ordering dependency, since material paths are stable across re-exports.

**Note on material binding**: We only use the material *path* (not material content), and only the leaf name or relative path. Material path is typically stable across DCC re-exports since it's defined by the artist, not auto-generated. If the material binding is absent, we use a sentinel value.

**Implementation note**: `UsdShade` is not currently imported in `report_asset_mesh_dedup.py` -- this will be a new import (`from pxr import UsdShade`). No existing code reads material/shader data. An alternative lightweight approach is to use the `material` text field from the annotation JSON (`<uid>_annotation.json`), which avoids USD parsing overhead but is less precise (free-text descriptions vs structured binding paths). The recommended approach is `UsdShade.MaterialBindingAPI` for precision, since the USD stage is already open during scanning.

**Data availability** (confirmed by sampling 50 assets per category across bottle, plate, wall, other):
- GLB files: 100% present across all sampled categories
- USD files: Always binary crate format (PXR-USDC header)
- File size ranges vary by category: wall has tiny USD (5-13KB) but large GLB (up to 27MB); other has huge variance (5KB to 35.7MB USD); bottle is moderate (28KB to 8.9MB USD)

#### 2.2.2 File Size Matching

File sizes are obtained from the filesystem at signature computation time:

```python
def _get_asset_file_sizes(usd_path: str) -> Tuple[Optional[int], Optional[int]]:
    """Get USD and GLB file sizes for an asset.

    Args:
        usd_path: Path to the asset USD file (e.g., .../usd/<uid>.usd)

    Returns:
        (usd_size_bytes, glb_size_bytes) -- None if file doesn't exist
    """
    usd_size = os.path.getsize(usd_path) if os.path.isfile(usd_path) else None

    # GLB path: ../glb/<uid>.glb (sibling directory of usd/)
    uid = os.path.splitext(os.path.basename(usd_path))[0]
    glb_dir = os.path.join(os.path.dirname(os.path.dirname(usd_path)), "glb")
    glb_path = os.path.join(glb_dir, f"{uid}.glb")
    glb_size = os.path.getsize(glb_path) if os.path.isfile(glb_path) else None

    return usd_size, glb_size
```

**Stored in AssetRecord** as new fields:
```python
@dataclass(frozen=True)
class AssetRecord:
    ...
    usd_file_size: Optional[int] = None
    glb_file_size: Optional[int] = None
    asset_topo_invariant_sig_hex: str = ""
```

### 2.3 Group Formation Algorithm

The `topo_filesize` mode uses a two-stage approach:

#### Stage 1: Pre-filter by Topology Invariant Signature

Group all assets by `asset_topo_invariant_sig_hex`. Drop singletons (groups with only 1 member). This is an O(N) operation.

Expected behavior: Assets with identical (vertex_count, face_count, mesh_count, material_bindings) -- regardless of vertex/face ordering -- will land in the same group.

#### Stage 2: File-Size Pairwise Filter within Groups

Within each topology-invariant group, apply file-size matching:

```python
def _filesize_match(size_a: Optional[int], size_b: Optional[int], tolerance: float) -> bool:
    """Check if two file sizes are within tolerance of each other.

    tolerance is a fraction (e.g., 0.02 = 2%).
    Returns True if |size_a - size_b| / max(size_a, size_b) <= tolerance.
    Returns False if either size is None.
    """
    if size_a is None or size_b is None:
        return False
    if size_a == 0 and size_b == 0:
        return True
    max_size = max(size_a, size_b)
    return abs(size_a - size_b) / max_size <= tolerance
```

**Matching logic**:
- **GLB match**: Required. GLB files contain raw geometry data in a binary format that's highly sensitive to data volume. GLB size is the primary discriminator.
- **USD match**: Optional secondary check (if GLB is missing or for additional confidence). USD size can vary more due to text formatting, attribute ordering, metadata differences.
- **Both missing GLB**: Fall back to USD-only matching with tighter tolerance (half the specified tolerance).

**Why GLB is primary**: GLB (binary glTF) stores vertex data in compact binary buffers. The buffer size is directly proportional to `vertex_count * sizeof(float) * 3 + face_count * sizeof(int) * avg_vertices_per_face`. Two meshes with identical topology but reordered vertices/faces will have nearly identical GLB sizes (within bytes, due to different compression or padding).

Evidence from the bottle triplet:
- GLB spread: 4 bytes / 310,080 = 0.001%
- USD spread: 387 bytes / 103,555 = 0.37%

#### Stage 3: Union-Find Merge

Use union-find within each topology-invariant group to merge assets that pass the file-size filter:

```python
for i in range(n):
    for j in range(i + 1, n):
        if _find(i) == _find(j):
            continue
        glb_ok = _filesize_match(group[i].glb_file_size, group[j].glb_file_size, tolerance)
        usd_ok = _filesize_match(group[i].usd_file_size, group[j].usd_file_size, tolerance)

        if glb_ok:
            # Primary match on GLB
            _union(i, j)
        elif group[i].glb_file_size is None and group[j].glb_file_size is None and usd_ok:
            # Fallback: both missing GLB, match on USD with tighter tolerance
            if _filesize_match(group[i].usd_file_size, group[j].usd_file_size, tolerance / 2):
                _union(i, j)
```

**Performance**: O(N) pre-filter + O(G^2) pairwise within groups where G is group size. Since topology-invariant groups are expected to be small (most assets have unique vertex/face count combinations), this should be efficient even for 52,904 assets.

### 2.4 Report Output

The new report follows the same JSON format as existing modes:

```json
{
  "meta": {
    "dataset": "bottle",
    "mode": "topo_filesize",
    "filesize_tolerance": 0.02,
    "asset_usd_count": 1698,
    "duplicate_group_count": N,
    ...
  },
  "duplicates": [
    {"sig": "topo_filesize_merge_0", "count": 3, "usd_paths": [...]}
  ],
  "assets": [...],
  "errors": [...]
}
```

## 3. Union Merge Transitive Closure Fix

### 3.1 Bug Description

**File**: `scripts/union_dedup_reports.py`, function `union_merge()`, lines 95-129.

**Current behavior**: Only `shape_only_adds` (assets in shape_invariant removable but NOT in geom_only removable) are linked into the union-find with their shape group representative. Members of shape_invariant groups that are NOT in `shape_only_adds` (because they're the shape canonical, or because they're already in geom_rem) are not linked.

**Consequence**: When a shape_invariant group contains both:
- An asset that is canonical in geom_only (so it's in `geom_rem`'s complement)
- An asset that is canonical in shape_invariant

...the union-find doesn't connect them, creating fragmented components. This causes the C1 soft-delete pipeline to find "transitive canonical conflicts" where an asset is canonical for one group but marked as old (removable) in another.

### 3.2 Fix

Replace the partial linking (lines 98-112) with full linking of ALL mode groups:

```python
# Link ALL geom_only group members
for rep, members in geom_groups:
    for m in members:
        uf.union(rep, m)

# Link ALL shape_invariant group members (not just shape_only_adds)
for rep, members in shape_groups:
    for m in members:
        uf.union(rep, m)

# When topo_filesize mode exists, also link its groups:
# for rep, members in topo_filesize_groups:
#     for m in members:
#         uf.union(rep, m)
```

And update `all_grouped` collection similarly:

```python
all_grouped = set()
for _, members in geom_groups:
    all_grouped.update(members)
for _, members in shape_groups:
    all_grouped.update(members)
```

### 3.3 Generalized N-Way Union Merge

With V3 adding a third dedup mode, the union merge script should be generalized to accept N reports instead of exactly 2:

```python
def union_merge_n(reports: List[dict]) -> dict:
    """Merge N dedup reports by union of all groups via union-find."""
    uf = UnionFind()
    all_grouped = set()

    for report in reports:
        for g in report.get("duplicates", []):
            paths = [normalize_path(p) for p in g["usd_paths"]]
            rep = paths[0]
            for m in paths:
                uf.union(rep, m)
                all_grouped.add(m)

    # Collect connected components
    components = {}
    for asset in all_grouped:
        root = uf.find(asset)
        components.setdefault(root, []).append(asset)

    # Pick representative per component, build output
    ...
```

**CLI changes**: Accept `--reports` (list of JSON paths) instead of `--geom-only` and `--shape-invariant` separately. Keep backward compatibility with the existing flags.

### 3.4 Impact on C1 Pipeline

After the fix:
- Union merge will produce fully connected components
- No asset will be canonical in one component and removable in another
- The C1 post-promote scan will pass without transitive canonical conflicts
- The ABORTED full-run (blocked at `book`, 7/79 categories) can be retried

## 4. Implementation Plan

### 4.1 Phase 1: Core Implementation (report_asset_mesh_dedup.py)

**Files to modify**: `scripts/report_asset_mesh_dedup.py`

1. Add `TopoInvariantDescriptor` and `AssetTopoInvariant` dataclasses
2. Add `usd_file_size`, `glb_file_size`, `asset_topo_invariant_sig_hex` fields to `AssetRecord`
3. Add `_get_asset_file_sizes()` helper
4. Compute topology invariant sig in `_compute_mesh_sigs()` -- add material binding extraction
5. Compute file sizes in the main scan loop
6. Add `_topo_filesize_merge()` function (analogous to `_shape_invariant_merge()`)
7. Add `--mode topo_filesize` and `--filesize-tolerance` CLI args
8. Wire up report generation in `_write_report()`

**Key design decisions**:
- Material binding extraction uses `UsdShade.MaterialBindingAPI` -- requires adding `from pxr import UsdShade` import
- File sizes read from filesystem (not annotation JSON) for accuracy and to avoid stale annotation data
- Topology invariant sig computed alongside existing signatures in the same scan pass (no second file read)

### 4.2 Phase 2: Union Merge Fix (union_dedup_reports.py)

**Files to modify**: `scripts/union_dedup_reports.py`

1. Fix transitive closure bug (link ALL group members, not just shape_only_adds)
2. Generalize to N-way merge (accept list of report paths)
3. Update batch mode to auto-discover all mode reports per category
4. Add `--topo-filesize` argument for backward-compatible single-mode invocation
5. Update meta output to include all source modes

### 4.3 Phase 3: DLC Integration

**Files to modify**: `scripts/dlc/dedup_by_category.py`, `scripts/dlc/run_task.sh`

1. Add `--mode topo_filesize` passthrough in `dedup_by_category.py`
2. Add `--filesize-tolerance` passthrough
3. No new DLC execution mode needed -- reuses existing `dedup` mode with different `--mode` arg

**DLC submission command**:
```bash
python scripts/dlc/submit_batch.py --total 79 --name dedup_v3_topo_filesize \
  --command_args "dedup --mode topo_filesize --filesize-tolerance 0.02 \
  --assets-root GRScenes-test1-normalized/GRScenes_assets \
  --out-dir check_reports/topo_filesize/"
```

Or via shell loop for per-category jobs:
```bash
for i in $(seq 0 78); do
  bash scripts/dlc/launch_job.sh dedup_v3_topo_fs $i 79 "" \
    "dedup --mode topo_filesize --filesize-tolerance 0.02 \
     --assets-root GRScenes-test1-normalized/GRScenes_assets \
     --out-dir check_reports/topo_filesize/"
done
```

### 4.4 Phase 4: Testing

**Unit tests** (add to `tests/test_shape_invariant.py` or new `tests/test_topo_filesize.py`):

1. `test_topo_invariant_sig_same_with_reordered_verts`: Two meshes with same vertex/face counts but different ordering should produce same topology invariant sig
2. `test_topo_invariant_sig_differs_with_different_counts`: Different vertex/face counts should produce different sig
3. `test_filesize_match_within_tolerance`: Test tolerance comparison logic
4. `test_filesize_match_none_handling`: None file sizes should not match
5. `test_material_binding_extraction`: Material binding path correctly relativized
6. `test_topo_filesize_merge_basic`: End-to-end merge with known duplicates
7. `test_topo_filesize_merge_no_glb_fallback`: Verify USD-only fallback when GLB missing

**Integration test** (bottle triplet):
```bash
./scripts/isaac_python.sh scripts/report_asset_mesh_dedup.py \
  --assets-root GRScenes-test1-normalized/GRScenes_assets/bottle \
  --out-dir /tmp/test_v3 \
  --dataset bottle_v3_test \
  --mode topo_filesize \
  --filesize-tolerance 0.02
```

Verify: The three bottle triplet assets (7861bd, 79088d, 79090f) must appear in the same duplicate group.

**Union merge test**:
```bash
python scripts/union_dedup_reports.py \
  --reports /tmp/test_v3/bottle_v3_test_asset_mesh_dedup_geom_only.json \
           /tmp/test_v3/bottle_v3_test_asset_mesh_dedup_topo_filesize.json \
  --output /tmp/test_v3/bottle_union_v3.json
```

### 4.5 Phase 5: Full Dataset Run

1. Run `topo_filesize` mode on all 79 categories via DLC (79 jobs)
2. Run updated union merge (3-way: geom_only + shape_invariant + topo_filesize)
3. Compare results against V2 union baseline (29,313 removable, 55.4%)
4. Validate no transitive canonical conflicts with the fixed union merge
5. Retry C1 soft-delete full-run

## 5. Performance Considerations

### 5.1 Scan Phase

File size lookup is O(1) per asset (two `os.path.getsize()` calls). Material binding extraction via `UsdShade.MaterialBindingAPI` is O(meshes_per_asset). Both are negligible compared to existing mesh signature computation.

**Total overhead per asset**: ~1ms (file size) + ~0.5ms per mesh (material binding) on top of existing ~10ms per asset for geometry hashing.

### 5.2 Pre-filter Groups

Topology invariant groups are expected to be larger than shape_descriptor_key groups because they ignore vertex coordinates entirely. However, the pairwise comparison within groups is extremely cheap (just two integer comparisons for file sizes), so even groups of 100+ assets complete in microseconds.

### 5.3 Memory

Two additional `Optional[int]` fields per `AssetRecord` = ~16 bytes per asset. For 52,904 assets = ~0.8 MB. Negligible.

### 5.4 DLC Compatibility

No special considerations. The mode uses the same per-category chunking as shape_invariant. Each DLC job processes one category independently. No cross-category dependencies.

## 6. False Positive Risk Analysis

### 6.1 Topology Invariant Alone

Topology invariants (vertex_count, face_count, mesh_count, material_bindings) alone have moderate collision risk. Many simple objects (cubes, planes, standardized furniture pieces) may share the same vertex/face counts. However, within a single category (e.g., all bottles), the combination of:
- Exact vertex count per mesh
- Exact face count per mesh
- Exact mesh count
- Material binding structure

...significantly narrows the candidate set.

### 6.2 File Size as Discriminator

File size adds a powerful second filter:
- GLB files encode geometry in binary buffers where size is tightly coupled to actual data content
- Two genuinely different models with identical vertex/face counts will almost certainly have different GLB sizes (different vertex positions = different buffer content = different compression results)
- The bottle triplet demonstrates: GLB sizes within 4 bytes for true duplicates

### 6.3 Combined False Positive Rate

The combination of topology invariants + file size matching is expected to have very low false positive rate:
- Same (vertex_count, face_count, mesh_count, materials): narrows to likely-similar models
- Same GLB file size (within 2%): further confirms same data content

**Risk mitigation**: The 2% default tolerance is conservative. For the bottle triplet, even 0.01% would suffice. Users can tighten the tolerance to reduce false positives at the cost of potentially missing some edge cases.

### 6.4 Known Limitations

1. **Mesh splitting/merging**: If a DCC exporter splits one mesh into two (or merges two into one), the mesh count and per-mesh vertex counts will differ. This mode will not catch such cases.
2. **Material rebinding**: If the same geometry is bound to different materials, they will have different topology invariant sigs. This is arguably correct behavior (different materials = different visual appearance).
3. **Missing GLB files**: Assets without GLB files fall back to USD-only matching with tighter tolerance, which has higher false positive risk due to USD file size variability.

## 7. Rollout Plan

### 7.1 Step 1: Implement and Test Locally

1. Implement core `topo_filesize` mode in `report_asset_mesh_dedup.py`
2. Run bottle category test locally
3. Verify bottle triplet grouped correctly
4. Run unit tests

### 7.2 Step 2: Fix Union Merge

1. Apply transitive closure fix in `union_dedup_reports.py`
2. Re-run union merge on existing V2 reports to verify fix
3. Confirm no canonical conflicts

### 7.3 Step 3: DLC Full Run

1. Submit 79-job DLC batch for `topo_filesize` mode
2. Collect results
3. Run 3-way union merge (geom_only + shape_invariant + topo_filesize)
4. Report total dedup improvement

### 7.4 Step 4: C1 Soft-Delete Retry

1. Generate new union-merged mappings per category
2. Run C1 autorun on all 79 categories
3. Monitor for transitive canonical conflicts (should be zero with fix)

### 7.5 Success Criteria

- Bottle triplet (7861bd, 79088d, 79090f) grouped as duplicates
- No transitive canonical conflicts in union merge
- Full C1 soft-delete completes for all 79 categories
- Total dedup rate improvement over V2 baseline (>55.4%)
- Zero false positives in spot-check validation (sample 10 newly-detected groups)

## 8. Appendix: Existing Mode Comparison

| Mode | Pre-filter | Comparison | Catches | Misses |
|------|-----------|------------|---------|--------|
| `geom_only` | geom hash | Exact hash | Bit-identical geometry | Float noise, reordered verts/faces, scale |
| `geom_only` + `--merge-tolerance` | topo hash | Pairwise vertex distance | Float noise within tolerance | Reordered faces, scale |
| `shape_invariant` | shape descriptor (vertex_count + aspect_ratio) | Hausdorff on unit-normalized sorted verts | Scale differences, minor noise | Reordered faces (different topo hash pre-filter) |
| **`topo_filesize`** (new) | Topology invariant (counts + materials) | File size comparison | **Reordered verts/faces** | Different mesh count, different materials, vastly different file sizes |

The four modes are complementary. Union merge combines all four for maximum coverage.
