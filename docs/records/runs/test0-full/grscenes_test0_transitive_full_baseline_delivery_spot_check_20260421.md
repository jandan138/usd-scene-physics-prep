---
title: "GRScenes Test0 Transitive Full Baseline Delivery Spot Check"
created_at: "2026-04-21"
updated_at: "2026-04-21"
maintainer: "OpenCode"
status: "completed"
code_reference:
  - "/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418/README.md"
  - "/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418/MANIFEST.json"
  - "/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418/dangling_references.json"
  - "/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418/GRScenes100/home/MWBGLKQKTKJZ2AABAAAAACA8_usd/layout.usd"
  - "/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418/GRScenes100/home/MVUHLWYKTKJ5EAABAAAAAEI8_usd/layout.usd"
  - "/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418/GRScenes100/commercial/MWF4WLIKTIFZIAABAAAAAEI8_usd/layout.usd"
---

# Scope

User-facing read-only spot check of delivery root:

- `/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418`

This spot check validates packaging and immediate readability only. It does not
establish that the delivery root is a promote-applied duplicate-removed clone;
later investigation showed the source suffixed layouts came from non-cumulative
dry-run apply outputs.

Requested checks:

- top-level structure and manifest/readme coherence
- three representative scenes with `layout.usd` present
- reference style in a representative `layout.usd`
- existence of a couple of retained assets
- immediate downstream usability issues

# Top-Level Structure

Observed top-level entries:

- `GRScenes100/`
- `GRScenes_assets/`
- `Material/`
- `README.md`
- `MANIFEST.json`
- `asset_pruning_summary.json`
- `dangling_references.json`

`README.md` and `MANIFEST.json` are broadly coherent on the key summary values:

- exported scenes: `99`
- retained assets: `85549`
- dangling references: `35`

`README.md` explicitly states dangling references are recorded rather than
rewritten away. `MANIFEST.json` records provenance roots in the repo workspace
as source-of-truth metadata. This packaging coherence does not by itself prove
promote semantics for the source rerun layouts.

# Scene Checks

Checked scene layouts:

1. `GRScenes100/home/MWBGLKQKTKJZ2AABAAAAACA8_usd/layout.usd`
   - file exists
   - stage opens successfully under `./scripts/isaac_python.sh`
   - `616` references found
   - `0` absolute references
   - `0` missing referenced asset files
2. `GRScenes100/home/MVUHLWYKTKJ5EAABAAAAAEI8_usd/layout.usd`
   - file exists
   - stage opens, but emits a USD warning for one missing referenced asset
   - `1307` references found
   - `0` absolute references
   - `1` missing referenced asset file:
     `../../../GRScenes_assets/door_38D2F4C3_69C3_4029_8C6F_7345AF68D71C/d41d8cd98f00b204e9800998ecf8427e/usd/d41d8cd98f00b204e9800998ecf8427e.usd`
3. `GRScenes100/commercial/MWF4WLIKTIFZIAABAAAAAEI8_usd/layout.usd`
   - file exists
   - stage opens successfully under `./scripts/isaac_python.sh`
   - `348` references found
   - `0` absolute references
   - `0` missing referenced asset files

# Reference Style

Representative scene inspected:

- `GRScenes100/home/MWBGLKQKTKJZ2AABAAAAACA8_usd/layout.usd`

Sample resolved references are relative bundle-local paths such as:

- `../../../GRScenes_assets/window/0b1ddc45c59b13e5075fea2d20fb4d6a/usd/0b1ddc45c59b13e5075fea2d20fb4d6a.usd`
- `../../../GRScenes_assets/door/da377b21d5a12150ed1e3011731dbc8c/usd/da377b21d5a12150ed1e3011731dbc8c.usd`

No absolute repo paths were found in the sampled scene references.

# Retained Asset Existence

Verified these retained asset files exist at the expected bundled locations:

- `GRScenes_assets/window/0b1ddc45c59b13e5075fea2d20fb4d6a/usd/0b1ddc45c59b13e5075fea2d20fb4d6a.usd`
- `GRScenes_assets/door/8b5f6b28132346ac09edb44100ae0024/usd/8b5f6b28132346ac09edb44100ae0024.usd`

# Immediate Usability Concern

The immediate downstream issues are retained dangling references and the broader
semantic caveat that this bundle is a dry-run-derived delivery snapshot rather
than a proven promote-applied clone:

- `dangling_references.json` reports `35` known missing assets
- one sampled home layout reproduces this immediately as a USD warning on open
- later investigation showed the source rerun suffix layouts were overwritten
  across categories during dry-run execution, so this bundle should not be used
  as evidence that duplicate-removal promotion already completed in place

This means a downstream user can open the dataset, but some scenes may log missing-asset warnings and load with incomplete content.

# Verification Commands

```bash
./scripts/isaac_python.sh - <<'PY'
from pxr import Usd
from pathlib import Path
scenes = [
    '/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418/GRScenes100/home/MWBGLKQKTKJZ2AABAAAAACA8_usd/layout.usd',
    '/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418/GRScenes100/home/MVUHLWYKTKJ5EAABAAAAAEI8_usd/layout.usd',
    '/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418/GRScenes100/commercial/MWF4WLIKTIFZIAABAAAAAEI8_usd/layout.usd',
]
for scene in scenes:
    stage = Usd.Stage.Open(scene)
    refs = []
    abs_refs = []
    missing = []
    for prim in stage.Traverse():
        if prim.HasMetadata('references'):
            for ref in prim.GetMetadata('references').GetAddedOrExplicitItems():
                p = ref.assetPath
                refs.append(p)
                if p.startswith('/'):
                    abs_refs.append(p)
                resolved = (Path(scene).parent / p).resolve()
                if not resolved.exists():
                    missing.append((p, str(resolved)))
    print(scene, bool(stage), len(refs), len(abs_refs), len(missing))
PY
```
