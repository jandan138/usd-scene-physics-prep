---
title: "Design: MDL Path Fix (absolute → relative) for Normalized Dataset"
code_reference: specs_normalizer/utils/mdl_rewrite.py, scripts/fix_normalized_mdl_paths.py
created_at: 2026-03-10
updated_at: 2026-03-10
maintainer: designer
status: approved
---

# MDL Path Fix Design

## Problem

All 44,816 assets in `GRScenes-test1-normalized/GRScenes_assets/` have **absolute MDL/texture paths** pointing to the original dataset:

```
/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test1/Material/mdl/Num5ac9e5d43cf8ab2e12006b69.mdl
/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test1/Material/mdl/textures/white.png
```

These paths are machine-specific and break portability. Scene layout USDs (`GRScenes100/`) already have correct relative paths (`../../../Material/mdl/...`).

## Root Cause

The `normalize_asset_transforms.py` script (used during asset normalization) opens each asset USD, applies transforms, and re-exports — but does not rewrite MDL paths. Since the original assets in `GRScenes-test1/` already had absolute paths (or they were resolved to absolute during stage traversal), the normalized copies inherited them.

## Solution

### Step 1: Create Material/ symlink in normalized dataset

```bash
ln -s /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test1/Material \
      /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test1-normalized/Material
```

**Rationale**: Symlink over copy — saves 29GB disk, the original dataset is on the same filesystem, and the scene layout USDs already reference `../../../Material/mdl/` relative to their location (which resolves to `GRScenes-test1-normalized/Material/mdl/`). The symlink makes these scene references work immediately.

If full portability is needed later (e.g., distributing the dataset), replace the symlink with a real copy.

### Step 2: Rewrite absolute paths → relative in all asset USDs

Use `rewrite_usd_mdl_paths_inplace()` from `specs_normalizer/utils/mdl_rewrite.py`.

For each asset at:
```
GRScenes-test1-normalized/GRScenes_assets/<category>/<hash>/usd/<hash>.usd
```

Call:
```python
rewrite_usd_mdl_paths_inplace(
    usd_path=asset_usd_path,
    materials_dir="/abs/path/to/GRScenes-test1-normalized/Material/mdl",
    use_relative=True,
    # relative_base defaults to dirname(usd_path), which is the usd/ dir
    backup=False,  # no .bak files for 44k assets
)
```

This will compute `os.path.relpath(materials_dir, usd_dir)` = `../../../../Material/mdl` and rewrite all `info:mdl:sourceAsset` and `inputs:*_Tex` attributes accordingly.

**Expected output paths**:
- MDL: `../../../../Material/mdl/Num5ac9e5d43cf8ab2e12006b69.mdl`
- Textures: `../../../../Material/mdl/textures/white.png`

### Step 3: Verify

Spot-check a handful of assets to confirm paths are relative and resolve correctly.

## Script Design: `scripts/fix_normalized_mdl_paths.py`

### Interface

```bash
python scripts/fix_normalized_mdl_paths.py \
  --dataset-root /path/to/GRScenes-test1-normalized \
  --materials-dir /path/to/GRScenes-test1-normalized/Material/mdl \
  [--category bottle]        # optional: process single category
  [--dry-run]                # scan and report without modifying
  [--workers 8]              # parallel workers (default: 8)
  [--no-backup]              # skip .bak creation (default: no backup)
```

### Processing Logic

```
1. Enumerate all asset USDs:
   glob: {dataset_root}/GRScenes_assets/*/*/usd/*.usd

2. For each USD file:
   a. Open stage
   b. For each prim attribute of type Asset or AssetArray:
      - If path contains "Material/" or "mdl/" or "/textures/":
        Rewrite to relative path using relpath(materials_dir, usd_dir)
   c. Export in-place (no backup for scale)

3. Report: total assets processed, total attributes rewritten, any errors
```

### Reuse of `mdl_rewrite.py`

**Decision: Reuse `rewrite_usd_mdl_paths_inplace()` directly.**

The existing function handles all the path patterns we need:
- `info:mdl:sourceAsset` (AssetPath to `.mdl` files)
- `inputs:*_Tex` (AssetPath to texture files)
- `AssetArray` attributes
- Texture path normalization (`/Textures/` → `/textures/`)
- Module string refs (`::.::Materials::`)

The function does full prim traversal which is slightly more work than needed (we know exactly which attributes to check), but the overhead is negligible per-asset and the robustness is worth it.

### Parallelism Strategy

**Per-asset parallelism with `multiprocessing.Pool`**, not per-category.

- 44,816 assets across 79 categories (uneven distribution: `other` has ~15k, some categories have <10)
- Per-category chunking would cause severe load imbalance
- Per-asset with Pool(workers=8) gives good utilization
- Each asset is independent (separate USD file, no cross-references during rewrite)

Estimated time: ~1-3 seconds per asset (open + traverse + export) × 44,816 assets / 8 workers = ~90-280 minutes. For faster execution, increase workers or use DLC batch.

### DLC Mode

For running on the cluster, the script should also work as a DLC task:

```bash
# In run_task.sh, add mode "fix_mdl_paths":
./scripts/isaac_python.sh scripts/fix_normalized_mdl_paths.py \
  --dataset-root $DATA_ROOT/GRScenes-test1-normalized \
  --materials-dir $DATA_ROOT/GRScenes-test1-normalized/Material/mdl \
  --category-chunk $CHUNK_ID/$CHUNK_TOTAL
```

But given the simplicity (no Isaac Sim APIs needed, just pxr), this can run locally with standard Python.

## Edge Cases

| Case | Count | Handling |
|------|-------|----------|
| No-shader assets (pillow) | 2 | Script finds 0 attributes to rewrite, logs "0 updated", moves on |
| Twinbru S3 URL MDL | 1 | Path won't match `Material/` pattern, left unchanged (correct behavior) |
| Bad assets (empty/broken) | 3 | `Usd.Stage.Open()` may fail or return empty stage; catch exception, log, continue |
| Already-relative paths | 0 expected | `rewrite_usd_mdl_paths` is idempotent — if path is already relative, `newp == path` and nothing changes |
| Soft-deleted (dedup) assets | 8,091 | Their USD dirs were moved to `_dedup_assets/`; glob won't find them. No action needed. |

## File Changes Summary

| File | Action |
|------|--------|
| `scripts/fix_normalized_mdl_paths.py` | **New** — main fix script |
| `GRScenes-test1-normalized/Material` | **New symlink** → `../GRScenes-test1/Material` |
| 44,816 asset `.usd` files | **Modified in-place** — absolute→relative paths |

## Validation Criteria

1. Sample 10 random assets across different categories
2. Confirm all `info:mdl:sourceAsset` paths are relative (`../../../../Material/mdl/*.mdl`)
3. Confirm all `inputs:*_Tex` paths are relative (`../../../../Material/mdl/textures/*.png`)
4. Confirm relative paths resolve to existing files (via the Material symlink)
5. No `.bak` files created (to save disk)
6. Scene layout USDs unchanged (they already have correct relative paths)
