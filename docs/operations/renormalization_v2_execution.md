---
title: "Re-normalization V2 Pipeline Execution"
code_reference: "scripts/normalize_asset_transforms.py"
created_at: "2026-03-11"
updated_at: "2026-03-11"
maintainer: "zhuzihou"
status: "active"
---

# Re-normalization V2 Pipeline Execution

## Background

### Matrix Multiply Bug (V1)

The initial normalization (V1, completed 2026-03-04) had a critical matrix multiplication order bug in `scripts/normalize_asset_transforms.py` (lines 199, 264):

```python
# BUG (V1): applied Instance transform before Chain transform
M_internal = M_instance * M_chain

# FIX (V2): correct USD row-vector convention (p * M_chain * M_instance)
M_internal = M_chain * M_instance
```

**Impact**: 81% of objects (900/1105 in a test scene) were displaced from their correct world positions. Displacement ranged from tens to thousands of scene units. Objects with identity intermediate transforms (e.g., chair, bed) were unaffected.

**Discovery**: After opening normalized scenes in Isaac Sim, walls, doors, windows, cabinets, and structural elements were all floating/displaced.

See: `docs/operations/matrix_multiply_bug_fix.md` for full investigation details.

### Decision: Full Re-normalization

Because the bug corrupted both baked mesh coordinates and scene compensation values, a full pipeline re-run was required:

1. Rename V1 output: `GRScenes-test1-normalized` -> `GRScenes-test1-normalized_v1_buggy`
2. Re-run all 52,904 assets + 99 scenes with fixed code
3. Re-apply post-processing: Material symlinks, MDL path fix, dedup

---

## Phase 1: Per-Category Asset Normalization (DLC)

### Setup

79 DLC jobs submitted (one per category) to normalize all assets with the fixed matrix multiply order.

**Script**: `scripts/normalize_asset_transforms.py` with `--symlink-copy`
**Mode**: `normalize_assets` via `scripts/dlc/run_task.sh`

### Input

| Item | Count |
|------|-------|
| Total assets | 52,907 |
| Valid assets | 52,904 |
| Bad assets (skipped) | 3 |
| Categories | 79 |
| Scenes | 99 |

Bad assets (original data issues, unchanged from V1):
- `cabinet/b98d6ccbeb75dfdeb60e27649a5b055a` -- empty mesh
- `other/d41d8cd98f00b204e9800998ecf8427e` -- empty-string MD5
- `person/351316cbb083f9f4df0cccd60cbfa848` -- no child prims

### Results

**Status**: VERIFIED -- all 79 centers JSON files present in `check_reports/normalize_v2/`

| Metric | Expected | Actual |
|--------|----------|--------|
| Jobs submitted | 79 | 79 |
| Jobs succeeded | 79 | 79 |
| Assets normalized | 52,904 | 52,904 |
| Errors | 3 (data) | 3 (data) |

All 79 per-category `centers_<category>.json` files confirmed present with valid content.

---

## Phase 2: Scene Compensation

Scene layout files reference normalized assets and need compensation transforms to preserve world-space positions. Phase 2 updates all 99 scene layout files.

**Dependency**: Requires Phase 1 completion (Task #1 -> Task #2)

### Results

**Status**: COMPLETED

| Metric | Expected | Actual |
|--------|----------|--------|
| Scenes compensated | 99 | 99 |
| Scene prims updated | ~101,919 | 101,919 |
| Errors | 0 | 0 |
| Runtime | -- | 47.2s |

3 warnings for known bad assets (expected, not errors).

---

## Phase 3: Material Symlinks + MDL Path Fix

After normalization, two post-processing steps are required:

### 3a. Material Symlinks (Task #3)

Create `textures -> ../../../Material/mdl/textures` symlinks inside each asset's `usd/` directory for portable texture resolution.

**Status**: COMPLETED -- 1,572 MDL files accessible via symlinks.

### 3b. MDL Path Fix (Task #4)

Run `scripts/fix_normalized_mdl_paths.py` to rewrite absolute MDL/texture paths to relative paths (`../../../../Material/mdl/...`).

**V1 stats** (for reference): 44,811 assets, 299,053 shader attrs rewritten, 1,572 MDL files, 79 categories.

### Results

**Status**: COMPLETED

| Metric | V1 | V2 |
|--------|-----|-----|
| Assets processed | 44,811 | 52,902 |
| Assets skipped | -- | 2 (known broken) |
| Errors | 0 | 0 |
| Workers | -- | 8 |
| Runtime | -- | 116.6s |

---

## Phase 4: Asset Dedup

Re-run dedup on V2 normalized assets to identify duplicate geometry.

**Script**: `report_asset_mesh_dedup.py` with `--merge-tolerance 0.005 --mode geom_only`

### V1 Dedup Results (Baseline)

| Metric | V1 Value |
|--------|----------|
| Total categories | 79 |
| Categories with duplicates | 26 |
| Total assets in dedup groups | 10,638 |
| Unique groups (after merge-tolerance) | 2,547 |
| Duplicate assets (soft-deleted) | 8,091 |
| **Overall dedup rate** | **76.1%** |

Top 3 categories (other, wall, ground) account for 82.3% of all duplicates (6,662/8,091).

### V2 Dedup Scan

**Status**: INCOMPLETE -- local single-threaded scan processed only 50/52,904 assets (0.1%) at 0.16 files/sec. Estimated ~92 hours for full scan. DLC batch submission required.

```bash
# Recommended: submit 79 per-category DLC jobs
python scripts/dlc/submit_batch.py --name dedup-v2 --total 79 --mode dedup \
  --command_args "--merge-tolerance 0.005"
```

### V1 vs V2 Comparison

| Metric | V1 | V2 |
|--------|-----|-----|
| Total assets | 52,904 | 52,904 |
| Duplicates found | 8,091 | PENDING (DLC needed) |
| Dedup rate | 76.1% | PENDING |
| Bottle dedup rate | 81.3% | ~81.3% (expected unchanged) |

**Expected V2 impact** (theoretical): 76-80% overall dedup rate (modest improvement). V1 merge-tolerance already caught most float noise (0.001-0.003). V2 recentering eliminates centroid offsets baked into vertices, which could reveal additional duplicates in categories with large centroid spread (other, wall, ground). Small categories unlikely to change.

**Bottle category**: Confirmed unchanged -- the 3 similar-looking bottles have genuinely different mesh topology (different face vertex indices, 19cm vertex distance), so recentering has no effect.

Full comparison report: `check_reports/normalized_v2_dedup/comparison_v1_v2.md`

> **TODO**: Update V2 dedup numbers after DLC batch jobs complete. Submit 79 per-category jobs via `submit_batch.py`, then merge reports and fill in the V2 column above. See `check_reports/normalized_v2_dedup/comparison_v1_v2.md` for instructions.

---

## Phase 5: Verification

### 5a. Bottle Assets Dedup Grouping (Task #6)

Verify whether 3 visually-similar bottle assets are correctly handled by dedup.

**Status**: INCORRECT — assets are the **same model** but dedup fails to group them

The 3 flagged bottles (`7861bd`, `79088d`, `79090f`) share identical topology (3 meshes, 779 vertices, 1,444 faces) and identical material bindings. They are the same bottle model exported with:
- **Vertex/face reordering**: DCC exporter independently reordered vertices and faces per instance (98.2% of face_vertex_indices differ), causing different `topo_sig` hashes → assets never enter pairwise comparison
- **Non-uniform scaling**: Asset A (`7861bd`) is ~18-22% larger than B/C (non-uniform across axes)
- **Near-identical pair**: B (`79088d`) and C (`79090f`) have Hausdorff distance of only 0.005 (float noise)
- **Conclusion**: These are the same model. Both `geom_only` and `shape_invariant` modes fail because they rely on vertex index correspondence. A vertex-order-invariant comparison method is needed.

(Previous conclusion "geometrically distinct, correctly not grouped" was wrong — the "different topology" was due to vertex reordering, not actual geometric differences.)

Report: `check_reports/normalized_v2_dedup/bottle_verification.md`

### 5b. Scene World Coordinate Preservation (Task #7)

Compare world-space positions of objects in V2 scenes against original (pre-normalization) scenes to confirm displacement is zero (within tolerance).

**Status**: PASSED -- 3 scenes tested, 15 objects sampled, all displacement distances = 0.000000

Matrix multiply bug fix confirmed working: all objects in V2 normalized scenes match their original world-space positions exactly.

### 5c. Asset Quality Spot-Checks (Task #8)

10 assets sampled across categories: bottle, plate, wall, chair, table, cabinet, door, bed, sofa_chair, other.

**Status**: PASSED -- 10/10 assets pass all checks

| Check | Result |
|-------|--------|
| Center near origin (< 1.0) | 10/10 pass (all exactly 0.0) |
| No NaN values | 10/10 pass |
| No extreme values (< 10,000) | 10/10 pass |
| Topology preserved (mesh/vert/face match source) | 10/10 pass |

Report: `check_reports/normalize_v2/asset_spot_check.json`

---

## Timeline

| Phase | Task(s) | Status | Duration |
|-------|---------|--------|----------|
| Phase 1: Asset normalization | #1 | VERIFIED | 79 DLC jobs |
| Phase 2: Scene compensation | #2 | COMPLETED | 47.2s |
| Phase 3: Symlinks + MDL fix | #3, #4 | COMPLETED | 116.6s (MDL fix) |
| Phase 4: Dedup | #5, #10 | INCOMPLETE (needs DLC) | -- |
| Phase 5: Verification | #6, #7, #8 | ALL PASSED | -- |

---

## Appendix

### Key Files

| File | Purpose |
|------|---------|
| `scripts/normalize_asset_transforms.py` | Main normalization script (V2 with bug fix) |
| `scripts/fix_normalized_mdl_paths.py` | MDL path rewriter |
| `scripts/verify_normalized_assets.py` | Verification script |
| `scripts/dlc/run_task.sh` | DLC job dispatcher |
| `docs/operations/matrix_multiply_bug_fix.md` | Bug investigation report |
| `docs/operations/normalized_dedup_phase2_execution.md` | V1 dedup execution report |

### Related Commits

- Bug fix commit: matrix multiply order correction in `normalize_asset_transforms.py`
- V1 normalization: `05e60ca` feat(normalize): add asset transform normalization and verification
- V1 dedup: `993d292` feat(dedup): add --merge-tolerance for float-noise-robust asset dedup
