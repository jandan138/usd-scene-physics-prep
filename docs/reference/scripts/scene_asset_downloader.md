---
title: Scene UID Downloader (scene + all referenced assets)
code_reference: scripts/build_scene_uid_subset_package.py
created_at: '2026-01-19'
updated_at: '2026-01-19'
maintainer: Codex
status: Active
---

# Scene UID Downloader (scene + all referenced assets)

> Last Updated: 2026-01-19
>
> Script: `scripts/build_scene_uid_subset_package.py`

## What this script does

Given a **scene UID** (under `GRScenes100/**/<scene_uid>_usd/`), this script builds a downloadable subset folder that contains:

- the full scene folder: `GRScenes100/<category>/<scene_uid>_usd/**`
- **all referenced assets** (transitively): `GRScenes_assets/<asset_category>/<asset_uid>/**`
- the minimal material library needed for rendering:
  - `Material/mdl/*.mdl`
  - `Material/mdl/textures/**`

The output directory keeps the **same directory structure** as the original package root.

## Inputs and outputs

- `--src`: full GRScenes package root (e.g. `GRScenes-test1/`)
- `--dst`: output subset root
- `--scene-uid`: scene uid, e.g. `MV7J6NIKTKJZ2AABAAAAADA8`
- `--scene-category` (optional): `home` or `commercial` (auto-detected if omitted)

## How referenced assets are detected

The script opens the scene USD(s) via `pxr.Usd` and collects referenced asset paths from:

- prim `references`
- prim `payloads`
- asset-valued attributes pointing to `.usd/.usda/.usdc`
- used layers (safety net)

Any path that matches `GRScenes_assets/<category>/<uid>/...` is interpreted as an asset UID.

It then performs a transitive closure: asset USDs are scanned too, so nested references are included.

## Quickstart

```bash
./scripts/isaac_python.sh scripts/build_scene_uid_subset_package.py \
  --src /abs/path/to/GRScenes-test1 \
  --dst /abs/path/to/subset_scene \
  --scene-uid MV7J6NIKTKJZ2AABAAAAADA8 \
  --write-manifest
```

Optional verification (checks missing MDL/texture refs in both scenes and assets):

```bash
./scripts/isaac_python.sh scripts/build_scene_uid_subset_package.py \
  --src /abs/path/to/GRScenes-test1 \
  --dst /abs/path/to/subset_scene \
  --scene-uid MV7J6NIKTKJZ2AABAAAAADA8 \
  --verify
```

## Notes

- This script requires `pxr` (USD Python bindings). If system Python doesn’t have it, use `./scripts/isaac_python.sh`.
- `--verify` may report missing files that are already missing in the source package (data quality issue).
