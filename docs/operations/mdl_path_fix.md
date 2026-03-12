---
title: "MDL Path Fix: Absolute to Relative Path Rewriting for Normalized Dataset"
code_reference: scripts/fix_normalized_mdl_paths.py, specs_normalizer/utils/mdl_rewrite.py
created_at: 2026-03-10
updated_at: 2026-03-10
maintainer: zhuzihou
status: active
---

# MDL Path Fix

## Problem

After running `normalize_asset_transforms.py` to produce the `GRScenes-test1-normalized` dataset, all 44,811 asset USD files contained **absolute MDL and texture paths** pointing to the original dataset location:

```
/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test1/Material/mdl/Num5ac9e5d43cf8ab2e12006b69.mdl
/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test1/Material/mdl/textures/white.png
```

These machine-specific absolute paths break dataset portability. Scene layout USDs (`GRScenes100/`) were not affected -- they already had correct relative paths (`../../../Material/mdl/...`).

Isaac Sim logs showed `"Empty identifier"` errors for MDL shaders. Initial investigation suggested `info:id` being `None` might be the cause. However, this turned out to be a **red herring**: `info:id` being `None` is correct behavior when `info:implementationSource="sourceAsset"`. In that configuration, the MDL module is loaded from the file referenced by `info:mdl:sourceAsset`, not from `info:id`. The actual problem was that `info:mdl:sourceAsset` pointed to absolute paths that did not exist in the target environment.

## Root Cause

The `normalize_asset_transforms.py` script opens each asset USD, applies transform normalization (recenter + Y-up to Z-up rotation), and re-exports the stage. During this export, the `pxr` library resolves relative MDL/texture paths to absolute paths based on the machine where the export ran. The normalized copies therefore inherited absolute paths to the original `GRScenes-test1/Material/mdl/` directory.

## Key Insight: `info:id` Is Not the Issue

When investigating the Isaac Sim `"Empty identifier"` errors, research confirmed:

- When `info:implementationSource = "sourceAsset"`, the MDL module is loaded from the file at `info:mdl:sourceAsset`
- In this mode, `info:id` is expected to be empty/None -- it is not used
- The `"Empty identifier"` log messages are misleading; they appear even when the shader works correctly
- The real fix is ensuring `info:mdl:sourceAsset` points to a valid, resolvable path

## Fix Approach

### Step 1: Create Material symlink

Link the normalized dataset to the original Material directory to avoid duplicating 29GB of MDL/texture data:

```bash
ln -s /path/to/GRScenes-test1/Material \
      /path/to/GRScenes-test1-normalized/Material
```

### Step 2: Rewrite absolute paths to relative

Run the batch fix script:

```bash
python scripts/fix_normalized_mdl_paths.py \
    --dataset-root /path/to/GRScenes-test1-normalized \
    [--materials-dir /path/to/GRScenes-test1-normalized/Material/mdl] \
    [--category bottle] \
    [--dry-run] \
    [--workers 8]
```

The script:
1. Finds all asset USDs under `GRScenes_assets/*/*/usd/*.usd`
2. For each asset, computes `os.path.relpath(materials_dir, usd_dir)` to get the relative prefix (typically `../../../../Material/mdl`)
3. Rewrites all `Sdf.AssetPath` attributes matching `Material/`, `Materials/`, or `/textures/` patterns
4. Handles `AssetArray` attributes and `::.::Materials::` module string references
5. Exports in-place (no backup files to save disk on 44k+ assets)
6. Produces a per-category JSON report

### Step 3: Verify

Spot-check assets to confirm paths are relative and resolve correctly via the Material symlink.

## Before / After

**Before** (absolute path in asset USD):
```
asset info:mdl:sourceAsset = @/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test1/Material/mdl/Num5ac9e5d43cf8ab2e12006b69.mdl@
asset inputs:diffuse_Tex = @/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test1/Material/mdl/textures/T_Wood_Pine_D.png@
```

**After** (relative path):
```
asset info:mdl:sourceAsset = @../../../../Material/mdl/Num5ac9e5d43cf8ab2e12006b69.mdl@
asset inputs:diffuse_Tex = @../../../../Material/mdl/textures/T_Wood_Pine_D.png@
```

## Statistics

| Metric | Value |
|--------|-------|
| Total assets processed | 44,811 |
| Total shader attributes rewritten | 299,053 |
| Unique MDL files referenced | 1,572 |
| Categories affected | 79 |
| Assets with 0 rewrites (no shaders) | ~5 (pillow, bad assets) |
| Scene layout USDs | Unchanged (already relative) |

## Scripts

### `scripts/fix_normalized_mdl_paths.py`

Batch driver script with multiprocessing support. Finds all asset USDs, dispatches path rewriting in parallel, and produces a JSON report with per-category statistics.

**Options**:
- `--dataset-root` (required): Root of the normalized dataset
- `--materials-dir`: Absolute path to Material/mdl (defaults to `{dataset-root}/Material/mdl`)
- `--category`: Process a single category only
- `--dry-run`: Scan and report without modifying files
- `--workers`: Parallel workers (default: 8)
- `--report-dir`: Directory for JSON report output (default: `check_reports`)

### `specs_normalizer/utils/mdl_rewrite.py`

Reusable library module providing:
- `rewrite_usd_mdl_paths(src, dst, materials_dir, ...)` -- rewrite and export to a new file
- `rewrite_usd_mdl_paths_inplace(usd_path, materials_dir, ...)` -- rewrite in-place with optional backup

Handles all MDL path patterns: `AssetPath`, `AssetArray`, texture paths (`/textures/` and `/Textures/`), module string references (`::.::Materials::`), and `Material/mdl/` prefix deduplication.

## Related Documents

- [MDL Path Fix Design](mdl_path_fix_design.md) -- detailed design doc with edge cases and DLC mode
- [Isaac Sim MDL Workflow](isaacsim_mdl_workflow.md) -- general MDL reference
- [Asset Transform Normalization](../../docs/operations/) -- the normalization step that introduced the absolute paths
