---
title: Rebuilt GRScenes Temp Outputs Investigation
code_reference:
  - /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp
  - /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp
created_at: 2026-03-14T07:20:28Z
updated_at: 2026-03-14T07:20:28Z
maintainer: Codex
status: completed
---

# Summary

This investigation checked actual rebuilt outputs under:

- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp`
- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp`

The main failure is confirmed in the scene layers, not the rebuilt asset tree:

- `layout.usd` files exist in both temp outputs.
- Scene prims carry authored USD `references`, not payloads.
- Those references still point to legacy `models/.../instance.usd` paths.
- The legacy `models/.../instance.usd` files are absent in the rebuilt temp outputs.
- The rebuilt `GRScenes_assets/<category>/<uid>/usd/<uid>.usd` files do exist and open successfully.

Result: opening `layout.usd` appears empty or half-empty because scene composition still targets nonexistent legacy asset paths.

# Key Findings

## 1. Layout files exist and are not empty

Representative scene files:

- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAAEI8_usd/layout.usd`
- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/GRScenes100/commercial/MV5M25QKTKJZ2AABAAAAAAQ8_usd/layout.usd`

Quick counts:

- home layouts: `69`
- commercial layouts: `30`

The files are binary USD crate layers (`PXR-USDC`), not missing placeholder files.

## 2. Scene references still point to legacy paths

Representative authored references found in scene layers:

- `/Root/Meshes/BaseAnimation/cabinet/model_f337b19942ec6e5ef73698bff19e9a24_0 -> models/layout/articulated/cabinet/f337b19942ec6e5ef73698bff19e9a24/instance.usd`
- `/Root/Meshes/Animation/oven/model_ea458b8e2d89766d334b0af89c058df8_0 -> models/object/articulated/oven/ea458b8e2d89766d334b0af89c058df8/instance.usd`
- `/Root/Meshes/BaseAnimation/door/model_1795c86fd1d93d5be331ca29f2563cf1_0 -> models/layout/articulated/door/1795c86fd1d93d5be331ca29f2563cf1/instance.usd`
- `/Root/Meshes/BaseAnimation/window/model_bc4df386981b494b7c811478fa20b43f_0 -> models/layout/articulated/window/bc4df386981b494b7c811478fa20b43f/instance.usd`

Representative counts from `Usd.Stage.Open()`:

- home sample `layout.usd`: `567` references, `0` payloads
- commercial sample `layout.usd`: `710` references, `0` payloads

## 3. The referenced legacy files are absent, while rebuilt assets exist

Checked pairs:

- Missing legacy path:
  - `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp/models/layout/articulated/cabinet/f337b19942ec6e5ef73698bff19e9a24/instance.usd`
- Present rebuilt asset:
  - `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp/GRScenes_assets/cabinet/f337b19942ec6e5ef73698bff19e9a24/usd/f337b19942ec6e5ef73698bff19e9a24.usd`

- Missing legacy path:
  - `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp/models/object/articulated/oven/ea458b8e2d89766d334b0af89c058df8/instance.usd`
- Present rebuilt asset:
  - `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp/GRScenes_assets/oven/ea458b8e2d89766d334b0af89c058df8/usd/ea458b8e2d89766d334b0af89c058df8.usd`

- Missing legacy path:
  - `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/models/layout/articulated/window/bc4df386981b494b7c811478fa20b43f/instance.usd`
- Present rebuilt asset:
  - `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/GRScenes_assets/window/bc4df386981b494b7c811478fa20b43f/usd/bc4df386981b494b7c811478fa20b43f.usd`

- Missing legacy path:
  - `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/models/layout/articulated/door/1795c86fd1d93d5be331ca29f2563cf1/instance.usd`
- Present rebuilt asset:
  - `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/GRScenes_assets/door/1795c86fd1d93d5be331ca29f2563cf1/usd/1795c86fd1d93d5be331ca29f2563cf1.usd`

## 4. USD emits direct composition warnings on scene open

Representative warning text while opening rebuilt scene layers:

- `Could not open asset @models/object/articulated/oven/ea458b8e2d89766d334b0af89c058df8/instance.usd@`
- `Could not open asset @models/layout/articulated/door/1795c86fd1d93d5be331ca29f2563cf1/instance.usd@`
- `Could not open asset @models/layout/others/wall/424fa43fcbc41ca0ab50d511fa515aab/instance.usd@`

These failures cover both furniture/object assets and structural layout assets such as wall, ground, ceiling, window, and door. That explains why some scenes look almost entirely empty.

## 5. Additional empty-scene causes were sampled and not reproduced

Representative rebuilt asset files open cleanly:

- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp/GRScenes_assets/cabinet/f337b19942ec6e5ef73698bff19e9a24/usd/f337b19942ec6e5ef73698bff19e9a24.usd`
- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp/GRScenes_assets/oven/ea458b8e2d89766d334b0af89c058df8/usd/ea458b8e2d89766d334b0af89c058df8.usd`
- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/GRScenes_assets/window/bc4df386981b494b7c811478fa20b43f/usd/bc4df386981b494b7c811478fa20b43f.usd`
- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/GRScenes_assets/door/1795c86fd1d93d5be331ca29f2563cf1/usd/1795c86fd1d93d5be331ca29f2563cf1.usd`

Sampled asset health:

- default prim present: `/Root`
- meshes present in all four sampled assets
- no authored payloads or references inside those sample assets
- no inactive prims
- no authored `visibility = "invisible"` on sampled prims
- no missing sampled asset-valued dependencies under `Material/mdl`

Representative material and texture paths that resolve:

- `../../../../Material/mdl/WorldGridMaterial.mdl`
- `../../../../Material/mdl/MI_6578264e4c99360001dbf401.mdl`
- `../../../../Material/mdl/textures/white.png`

## 6. Scene layers are not hidden or deactivated at the root

Sampled scene-layer health:

- default prim present: `/Root`
- `/Root` is active
- `/Root` visibility is `inherited`
- no sampled inactive prims
- no sampled authored invisible prims

This does not rule out every possible per-scene issue, but it rules out the main class of “scene exists but hidden/deactivated” failures in the sampled rebuilt outputs.

## 7. Why some scenes look empty and others half-empty

Representative stage counts:

- home sample:
  - `16738` prims
  - `5094` meshes
  - `567` broken scene references
- commercial sample:
  - `788` prims
  - `65` meshes
  - `710` broken scene references

Interpretation:

- Some home scenes still show partial geometry because they also contain substantial authored geometry directly in the scene layer.
- Some commercial scenes depend much more heavily on external referenced assets, so they look almost empty when those references fail.

# Decisions

- No code changes were made during this investigation.
- The current evidence is sufficient to treat scene reference rewrite failure as the primary data issue in the rebuilt temp outputs.
- No second equally severe asset-layer blocker was reproduced in sampled rebuilt assets.

# Merge And Validate Impact

- `merge`: not mechanically blocked by filesystem structure. Both temp outputs contain `GRScenes100`, `GRScenes_assets`, and `Material/mdl`.
- semantic release decision: blocked. Merging now would preserve broken scene references into the merged dataset.
- `validate`: should be treated as blocked at a content level, because scene composition currently targets nonexistent legacy files.
- caution: a purely structural validation pass could still report success if it only checks file presence and not scene reference resolution.

# Test Commands And Results

Representative commands run:

```bash
find /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp/GRScenes100/home -name layout.usd | sed -n '1,5p'
find /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/GRScenes100/commercial -name layout.usd | sed -n '1,5p'
```

Result: layout files exist in both temp outputs.

```bash
python3 - <<'PY' 2>/tmp/rebuilt_layout_warnings.log
from pxr import Usd
samples = [
    '/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAAEI8_usd/layout.usd',
    '/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/GRScenes100/commercial/MV5M25QKTKJZ2AABAAAAAAQ8_usd/layout.usd',
]
for p in samples:
    stage = Usd.Stage.Open(p)
    refs = []
    payloads = []
    for prim in stage.Traverse():
        if prim.HasAuthoredReferences():
            md = prim.GetMetadata('references')
            if md:
                for item in md.GetAddedOrExplicitItems():
                    refs.append((str(prim.GetPath()), getattr(item, 'assetPath', '')))
        if prim.HasAuthoredPayloads():
            md = prim.GetMetadata('payloads')
            if md:
                for item in md.GetAddedOrExplicitItems():
                    payloads.append((str(prim.GetPath()), getattr(item, 'assetPath', '')))
    print(p, len(refs), len(payloads), refs[:10])
PY
```

Result: scene layers carry many `models/.../instance.usd` references and zero payloads. USD emitted direct “Could not open asset” warnings.

```bash
python3 - <<'PY'
from pathlib import Path
checks = [
    ('home', 'models/layout/articulated/cabinet/f337b19942ec6e5ef73698bff19e9a24/instance.usd'),
    ('home', 'models/object/articulated/oven/ea458b8e2d89766d334b0af89c058df8/instance.usd'),
    ('commercial', 'models/layout/articulated/window/bc4df386981b494b7c811478fa20b43f/instance.usd'),
    ('commercial', 'models/layout/articulated/door/1795c86fd1d93d5be331ca29f2563cf1/instance.usd'),
]
for split, old_ref in checks:
    root = Path('/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp' if split == 'home' else '/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp')
    parts = old_ref.split('/')
    leaf = parts[3]
    uid = parts[4]
    print((root / old_ref).exists(), root / old_ref)
    print((root / 'GRScenes_assets' / leaf / uid / 'usd' / f'{uid}.usd').exists(), root / 'GRScenes_assets' / leaf / uid / 'usd' / f'{uid}.usd')
PY
```

Result: legacy referenced files are absent; rebuilt asset USDs exist.

```bash
python3 - <<'PY' 2>/tmp/asset_probe_warn.log
from pxr import Usd, UsdGeom
samples = [
    '/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp/GRScenes_assets/cabinet/f337b19942ec6e5ef73698bff19e9a24/usd/f337b19942ec6e5ef73698bff19e9a24.usd',
    '/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp/GRScenes_assets/oven/ea458b8e2d89766d334b0af89c058df8/usd/ea458b8e2d89766d334b0af89c058df8.usd',
]
for p in samples:
    stage = Usd.Stage.Open(p)
    print(stage.GetDefaultPrim().GetPath())
PY
```

Result: sampled asset USDs open without warnings and expose valid default prims.

# Errors And Resolutions

- `Sdf.Layer.FindOrOpen()` failed in one shell environment with USD plugin loading errors.
- Resolution: switched back to `Usd.Stage.Open()` for all dataset checks, which worked reliably in the current environment.

# Code Changes

- None.
