---
title: specs_normalizer Scene Ref Rewrite Investigation
code_reference:
  - /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/specs_normalizer/exporters/scenes.py
  - /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/specs_normalizer/utils/scene_rewrite.py
  - /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/specs_normalizer/normalize.py
  - /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/dlc/run_task.sh
  - /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/rebuild_test0_from_legacy.py
created_at: 2026-03-14T08:06:00Z
updated_at: 2026-03-14T08:06:00Z
maintainer: Codex
status: completed
---

# Summary

This investigation traced the legacy scene-reference rewrite path used by the
rebuild flow:

- `scripts/dlc/run_task.sh`
- `python -m specs_normalizer`
- `specs_normalizer/normalize.py`
- `specs_normalizer/exporters/scenes.py`
- `specs_normalizer/utils/scene_rewrite.py`

The current code path is capable of correctly rewriting commercial legacy scene
references from:

- `models/<scope>/<subcat>/<category>/<uid>/instance.usd`

to:

- `../../../GRScenes_assets/<category>/<uid>/usd/<uid>.usd`

for the exact same legacy commercial sample that appeared broken in the DLC temp
output.

The most likely root cause is not a path-mapping bug. The strongest evidence
points instead to exporter-side silent failure behavior:

1. `export_scenes()` copies the source layer first and only then attempts
   in-place rewrite.
2. The rewrite call is wrapped in a broad `except Exception` block.
3. The fallback rewrite path is also wrapped in a broad `except Exception` block.
4. If both paths fail, the original copied scene layer remains in place with
   legacy `models/.../instance.usd` references, and the overall normalize job
   can still print `done`.

The secondary issue is that the fallback path is fragile for binary USD:

- it depends on `pxr.Sdf.Layer.FindOrOpen()` / `Export()`
- in the current environment that lower-level path can fail with file-format or
  plugin errors even when `Usd.Stage.Open()` succeeds

So if the primary `rewrite_scene_refs_inplace()` path ever throws, the fallback
is not reliable enough to be trusted as the only recovery path.

# Code Path Traced

## DLC entrypoint

`scripts/dlc/run_task.sh` runs:

```bash
"$ISAAC_PYTHON" -m specs_normalizer "$@"
```

under:

- `cd "$CODE_ROOT"`
- `PYTHONPATH="$CODE_ROOT:${PYTHONPATH:-}"`

That means the intended package import path is the workspace checkout, not some
arbitrary unrelated directory.

## specs_normalizer main flow

`specs_normalizer/normalize.py` runs:

1. `export_materials(...)`
2. `export_assets(..., rewrite_mdl_paths=True, ...)`
3. `export_scenes(...)`

for the rebuild command:

```bash
normalize --src-target <legacy_split> --dst-root <tmp_root> \
  --asset-name GRScenes_assets --scene-name GRScenes100 \
  --scene-category <home|commercial> --with-annotations
```

## Scene export path

`specs_normalizer/exporters/scenes.py`:

1. chooses `start_result_raw.usd` first
2. copies it to `layout.usd`
3. copies sibling USDs from the source scene directory
4. selects `models_abs = <dst_root>/GRScenes_assets` when `asset_name` exists
5. runs `rewrite_scene_refs_inplace(usdf, mats_abs, models_abs, relative_base=out_dir)`
   for every exported USD in the scene directory

Critical behavior:

- `rewrite_scene_refs_inplace(...)` is wrapped by `except Exception` at
  `specs_normalizer/exporters/scenes.py:85-87`
- binary/text fallback is also wrapped by `except Exception` at
  `specs_normalizer/exporters/scenes.py:188-190`

This is the main silent-failure channel.

## Rewrite helper

`specs_normalizer/utils/scene_rewrite.py`:

- rewrites material paths containing `Materials/`
- rewrites model paths containing `models/`
- maps legacy asset paths via `_remap_model_path()`
- saves the root layer with `root.Save()`

The model remap logic is:

```python
<category>/<uid>/usd/<uid>.usd
```

from the legacy path tail around `instance.usd`.

# Evidence

## 1. Home temp output is actually rewritten

Sample:

- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_home_tmp/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd`

Observed composed metadata:

- `old refs = 0`
- `new refs = 1105`

`Usd.Stage.GetUsedLayers()` for the same layout includes many layers under
`GRScenes_assets/.../usd/...usd`.

This corrected an earlier false impression based on `PrimStack` inspection.

## 2. Commercial temp output from DLC is broken before merge

Sample:

- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/GRScenes100/commercial/MV4AFHQKTKJZ2AABAAAAAEI8_usd/layout.usd`

Observed composed metadata:

- `old refs = 3680`
- `new refs = 0`

Sample authored references still present in composed metadata:

- `models/object/articulated/shoppingtrolley/.../instance.usd`
- `models/layout/others/wall/.../instance.usd`
- `models/layout/articulated/table/.../instance.usd`

The same pattern reproduces across sampled commercial scenes:

- `MV4AFHQKTKJZ2AABAAAAADQ8_usd -> 987 old / 0 new`
- `MV4AFHQKTKJZ2AABAAAAADY8_usd -> 8243 old / 0 new`
- `MV4AFHQKTKJZ2AABAAAAAEA8_usd -> 1963 old / 0 new`
- `MV4AFHQKTKJZ2AABAAAAAEI8_usd -> 3680 old / 0 new`
- `MV5M25QKTKJZ2AABAAAAAAA8_usd -> 794 old / 0 new`

So the breakage is not a single-scene anomaly.

## 3. Manual helper invocation rewrites the exact broken commercial scene successfully

On a copy of the already-broken exported commercial `layout.usd`:

- `changed = 62128`
- `old refs = 0`
- `new refs = 3680`

Representative rewritten reference:

- `../../cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/GRScenes_assets/shoppingtrolley/.../usd/...usd`

This proves:

- `_remap_model_path()` is not the blocker
- `rewrite_scene_refs_inplace()` can handle this commercial scene
- the asset layout assumption `GRScenes_assets/<category>/<uid>/usd/<uid>.usd`
  is valid for this data

## 4. Re-running current `export_scenes()` on one commercial sample works

Using the current workspace code, a one-scene probe succeeded both under:

- `python3`
- `./scripts/isaac_python.sh`

Results for the probe output `layout.usd`:

- `old refs = 0`
- `new refs = 3680`

This substantially weakens two hypotheses:

- wrong legacy source path assumption
- wrong target asset layout assumption

It also weakens the claim that the current DLC entrypoint necessarily imports
the wrong package version.

## 5. Fallback binary rewrite path is fragile

In the current environment, low-level `Sdf.Layer.FindOrOpen()` / `Export()`
operations can fail on these `.usd` files with errors such as:

- `Cannot determine file format for @.../start_result_raw.usd@`
- plugin load failures around `libusd_kind...`

At the same time, `Usd.Stage.Open()` does succeed on the same files.

This matters because the fallback branch in
`specs_normalizer/exporters/scenes.py:95-179` depends on `Sdf.Layer.FindOrOpen()`
for binary USD conversion. If the primary helper throws and the code lands in
fallback, fallback may also fail. The outer broad `except` then reduces the
failure to a log line and continues.

## 6. Rebuild merge step already contains a second-pass repair

`scripts/rebuild_test0_from_legacy.py:496-505` explicitly runs
`_repair_scene_refs_under_root()` on both temp roots before merge.

That means the rebuild orchestration already compensates for a possible bad temp
export by re-running `rewrite_scene_refs_inplace()` before the final merged
dataset is assembled.

So the current rebuild flow is designed with the expectation that temp outputs
may still need scene-ref repair.

# Root-Cause Hypotheses

## Highest likelihood

Silent rewrite failure in `export_scenes()` is the main design bug.

Why:

- broken temp outputs are exact copies of the failure mode expected when the
  source scene is copied but not successfully rewritten
- `export_scenes()` swallows helper exceptions and then swallows fallback
  exceptions
- the job can still terminate normally and print `done`
- current code can rewrite the same commercial scene successfully when run
  directly, so the mapping logic itself is not the leading suspect

## Medium likelihood

The DLC run encountered a runtime-specific exception in `rewrite_scene_refs_inplace()`
for commercial scenes, but the exception was hidden by the broad `except`.

Why:

- commercial temp output is systematically unrepaired
- local reruns now succeed, including under Isaac python
- the code does not record which scene failed or why

Without stronger logging, the exact thrown exception cannot be recovered from the
finished DLC run.

## Medium likelihood

The fallback binary path is not trustworthy enough for production recovery.

Why:

- it uses a lower-level Sdf open/export path that is demonstrably less robust
  than `Usd.Stage.Open()` in this environment
- even when fallback fails, the job does not fail fast

## Low likelihood

Wrong source scene topology or wrong destination asset structure.

Why it is low:

- current `export_scenes()` rewrites the same commercial legacy scene correctly
- `_remap_model_path()` maps the observed source path shapes correctly
- `asset_name='GRScenes_assets'` resolves to the expected destination root

## Low likelihood

Wrong package import due to DLC entrypoint environment.

Why it is low:

- `run_task.sh` prepends the workspace root to `PYTHONPATH`
- exact current code works under `./scripts/isaac_python.sh`
- there is no direct evidence from the recorded command or logs that a different
  package revision was imported

# Most Likely Fix

An upstream patch is warranted in `specs_normalizer/exporters/scenes.py`.

The most important behavior changes should be:

1. Do not silently continue after `rewrite_scene_refs_inplace()` failure.
2. Log the scene file path and exception details.
3. Verify the rewritten file after save by reopening it and checking for any
   remaining `models/...` authored refs or payloads.
4. Only keep the output file if rewrite plus verification succeeds.
5. Treat fallback as explicit recovery, not a silent best-effort branch.
6. Prefer temp-file write plus atomic replace over copy-then-mutate in place.

Recommended follow-up patch areas:

- `specs_normalizer/exporters/scenes.py`
- optionally `specs_normalizer/utils/scene_rewrite.py` to expose stronger
  verification/reporting hooks

# Test Commands And Results

Representative commands used in this investigation:

```bash
python3 - <<'PY' 2>/dev/null
from pathlib import Path
from pxr import Usd
root = Path('/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/GRScenes100/commercial')
for scene_dir in sorted([p for p in root.iterdir() if p.is_dir()])[:5]:
    layout = scene_dir/'layout.usd'
    stage = Usd.Stage.Open(str(layout))
    old = new = 0
    for prim in stage.Traverse():
        refs = prim.GetMetadata('references')
        if not refs:
            continue
        items = refs.GetAddedOrExplicitItems() or refs.GetPrependedItems() or []
        for r in items:
            ap = getattr(r,'assetPath','') or ''
            old += ('models/' in ap)
            new += ('GRScenes_assets/' in ap)
    print(scene_dir.name, old, new)
PY
```

Result: all sampled commercial scenes still had only legacy refs in the DLC temp
output.

```bash
python3 - <<'PY' 2>/tmp/commercial_manual_rewrite.stderr
from pathlib import Path
from shutil import copy2
from specs_normalizer.utils.scene_rewrite import rewrite_scene_refs_inplace
from pxr import Usd
src = Path('/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/GRScenes100/commercial/MV4AFHQKTKJZ2AABAAAAAEI8_usd/layout.usd')
out_dir = Path('/tmp/commercial_manual_rewrite')
out_dir.mkdir(parents=True, exist_ok=True)
probe = out_dir / 'layout.usd'
copy2(src, probe)
changed = rewrite_scene_refs_inplace(
    str(probe),
    '/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/Material/mdl',
    '/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt_commercial_tmp/GRScenes_assets',
    relative_base=str(out_dir),
)
print(changed)
stage = Usd.Stage.Open(str(probe))
old = new = 0
for prim in stage.Traverse():
    refs = prim.GetMetadata('references')
    if not refs:
        continue
    items = refs.GetAddedOrExplicitItems() or refs.GetPrependedItems() or []
    for r in items:
        ap = getattr(r, 'assetPath', '') or ''
        old += ('models/' in ap)
        new += ('GRScenes_assets/' in ap)
print(old, new)
PY
```

Result: `changed = 62128`, `old = 0`, `new = 3680`.

```bash
./scripts/isaac_python.sh - <<'PY' > /tmp/export_scene_probe_isaac.stdout 2> /tmp/export_scene_probe_isaac.stderr
from specs_normalizer.exporters.scenes import export_scenes
export_scenes(
    '/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/commercial_scenes',
    '/tmp/export_scene_probe_isaac',
    'GRScenes100',
    'commercial',
    with_annotations=False,
    scene_ids={'MV4AFHQKTKJZ2AABAAAAAEI8_usd'},
    asset_name='GRScenes_assets',
)
print('done')
PY
```

Result: probe completed successfully. The resulting probe `layout.usd` had
`old = 0`, `new = 3680`.

# Errors And Resolutions

- `Sdf.Layer.FindOrOpen()` / `Export()` failed in the current environment for
  some `.usd` files with file-format or plugin errors.
- Resolution: composition and rewrite validation used `Usd.Stage.Open()` instead,
  which worked reliably for the targeted scene checks.

# Code Changes

- None.
