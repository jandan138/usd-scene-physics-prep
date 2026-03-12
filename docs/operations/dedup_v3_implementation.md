---
title: "Dedup V3: topo_filesize Mode Implementation"
code_reference: "scripts/report_asset_mesh_dedup.py, scripts/union_dedup_reports.py, scripts/dlc/dedup_by_category.py"
created_at: "2026-03-11"
updated_at: "2026-03-11"
maintainer: "zhuzihou"
status: "implemented"
---

# Dedup V3: topo_filesize Mode Implementation

## 1. Summary

Implemented a new `topo_filesize` dedup mode that detects duplicate assets even when DCC exporters have reordered vertices and faces independently per model instance. This addresses the remaining blind spot in the existing `geom_only` and `shape_invariant` modes, which both fail when vertex/face ordering differs between exports of the same model.

Additionally fixed a transitive closure bug in `union_dedup_reports.py` and generalized it to support N-way merge (arbitrary number of dedup reports), enabling 3-way union merge of `geom_only + shape_invariant + topo_filesize`.

## 2. Files Modified

### 2.1 `scripts/report_asset_mesh_dedup.py`

- **New dataclasses**: `TopoInvariantDescriptor` (per-mesh: vertex_count, face_count, material_binding) and `AssetTopoInvariant` (asset-level: mesh_count, sorted mesh descriptors, SHA256 sig_hex).
- **New fields on `AssetRecord`**: `usd_file_size`, `glb_file_size`, `asset_topo_invariant_sig_hex`.
- **`_get_asset_file_sizes()`**: Helper that reads USD and GLB file sizes from the filesystem. GLB path is derived from the USD path (`../glb/<uid>.glb`).
- **Material binding extraction**: Added `UsdShade.MaterialBindingAPI` usage in `_compute_mesh_sigs()` to extract per-mesh material binding paths, relativized to the asset root prim. New import: `from pxr import UsdShade`.
- **Topology invariant signature**: Computed in `_compute_mesh_sigs()` alongside existing signatures. Per-mesh descriptors are sorted by (vertex_count, face_count, material_binding) to be order-independent, then hashed into `sig_hex`.
- **`_filesize_match()`**: Compares two file sizes within a fractional tolerance. Returns False if either size is None.
- **`_topo_filesize_merge()`**: Two-stage algorithm: (1) pre-filter by topology invariant sig, (2) pairwise file-size matching within groups using union-find. GLB is the primary discriminator; falls back to USD-only with half tolerance when GLB is missing.
- **CLI args**: `--mode topo_filesize` added to existing choices; `--filesize-tolerance` (default 0.02 = 2%) added.
- **Report generation**: New `topo_filesize` report written alongside standard reports when mode is selected.

### 2.2 `scripts/union_dedup_reports.py`

- **Transitive closure fix**: Changed group linking to connect ALL members of ALL mode groups via union-find, not just `shape_only_adds`. This prevents the scenario where an asset is canonical in one group but removable in another (transitive canonical conflict).
- **N-way merge**: New `union_merge_n()` function accepts a list of N report dicts and merges all groups via union-find. The existing `union_merge()` 2-way function is preserved for backward compatibility.
- **`--reports` CLI**: New argument accepting arbitrary number of JSON report paths for N-way merge. Coexists with the existing `--geom-only` / `--shape-invariant` flags.
- **`--topo-dir` in batch mode**: New argument for batch mode to auto-discover `topo_filesize` reports per category alongside `geom-dir` and `shape-dir`.

### 2.3 `scripts/dlc/dedup_by_category.py`

- **`--mode topo_filesize` passthrough**: Added `topo_filesize` to the mode choices and passes `--filesize-tolerance` to the underlying `report_asset_mesh_dedup.py` invocation.
- No new DLC execution mode needed; reuses existing `dedup` mode with different `--mode` arg.

### 2.4 `tests/test_topo_filesize.py` (new)

Unit tests for the topo_filesize mode covering:
- Topology invariant signature: same sig for reordered vertices, different sig for different counts
- `_filesize_match()`: tolerance logic, None handling, edge cases (zero sizes, exact boundary)
- `_topo_filesize_merge()`: end-to-end merge, no-GLB USD fallback, singleton filtering
- Material binding extraction correctness

### 2.5 `tests/test_union_merge.py` (new)

Unit tests for the union merge fix and N-way generalization:
- Transitive closure correctness (all group members linked)
- N-way merge with 3+ reports
- Degenerate cases (empty reports, single report, overlapping groups)

## 3. Key Design Decisions

1. **Material binding via `UsdShade.MaterialBindingAPI`**: Chosen over annotation JSON `material` text field for precision. The USD stage is already open during scanning, so there is no additional I/O cost. Material path is relativized to asset root for stability across different mount points.

2. **GLB as primary file-size discriminator**: GLB (binary glTF) stores vertex data in compact binary buffers where size is tightly coupled to actual data content. Evidence from the bottle triplet: GLB spread is 4 bytes (0.001%) vs USD spread of 387 bytes (0.37%).

3. **USD fallback with half tolerance**: When GLB files are missing for both assets being compared, the script falls back to USD-only matching but uses `tolerance / 2` (i.e., 1% default) to compensate for USD's higher size variability.

4. **Union-find for group merging**: Within each topology-invariant pre-filter group, pairwise file-size comparisons are merged via union-find to produce connected components. This handles transitive matches correctly (if A matches B and B matches C, all three end up in one group).

5. **Order-independent topology sig**: Per-mesh descriptors are sorted by `(vertex_count, face_count, material_binding)` before hashing, ensuring the signature is invariant to mesh traversal order in the USD stage.

## 4. CLI Changes

### `scripts/report_asset_mesh_dedup.py`

```
--mode {all,shape_invariant,topo_filesize}   # "topo_filesize" is new
--filesize-tolerance FLOAT                    # default 0.02 (2%), new arg
```

### `scripts/union_dedup_reports.py`

```
--reports FILE [FILE ...]   # N-way merge (new, alternative to --geom-only/--shape-invariant)
--topo-dir DIR              # batch mode: directory with topo_filesize reports (new)
```

### `scripts/dlc/dedup_by_category.py`

```
--mode {all,shape_invariant,topo_filesize}   # "topo_filesize" is new
--filesize-tolerance FLOAT                    # passthrough to report script (new)
```

## 5. Test Results

All tests passing:
- `tests/test_topo_filesize.py` -- topo_filesize mode unit tests
- `tests/test_union_merge.py` -- union merge fix and N-way merge tests
- `tests/test_shape_invariant.py` -- existing tests (no regressions)

## 6. Next Steps

1. Run `topo_filesize` mode on all 79 categories via DLC batch submission
2. Run 3-way union merge (`geom_only + shape_invariant + topo_filesize`) using `--reports`
3. Verify the bottle triplet (7861bd, 79088d, 79090f) is grouped correctly
4. Retry C1 soft-delete full-run with the fixed union merge
5. Validate no transitive canonical conflicts in the merged results
6. Compare total dedup rate against V2 baseline (29,313 removable, 55.4%)
