---
title: test0 smoke descendant override remap plan
code_reference:
  - scripts/normalize_asset_transforms.py
  - scripts/audit_normalize_phase2.py
  - scripts/check_normalize_gate_from_reports.py
created_at: 2026-03-12
updated_at: 2026-03-12
maintainer: codex
status: draft
---

# Summary

This note records the repair plan after the `S1` normalize-only investigation
for `GRScenes-test0` scene `home/MV7J6NIKTKJZ2AABAAAAADA8_usd`.

The immediate problem is now understood:

- The refrigerator outlier is not caused by dedup.
- The normalized asset written in `Phase 1` matches the current normalization
  math.
- The failure comes from a scene-authored xform override beneath the referenced
  asset root that stops composing correctly after normalization changes the
  asset's internal xform structure.

The repair target is therefore **not** "adjust the top-level root prim
compensation formula again". The repair target is to make normalize preserve
the semantics of scene-authored descendant xform overrides such as
`/Instance/xformOp:scale`.

# Root Cause

The failing prim is:

- Scene prim:
  `/Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0`
- Descendant with scene-local override:
  `/Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0/Instance`

In the original scene, that descendant authors:

- `xformOp:scale = (0.098, 0.1012024, 0.1012024)`

That override composes correctly against the original asset because the
referenced asset still exposes the expected descendant xform structure under
`/Root/Instance`.

After normalization:

- `Phase 1` rewrites the referenced asset's internal xform structure.
- `Phase 2` compensates only the referenced root prim.
- The original scene-local descendant override is either inert or no longer
  semantically equivalent in the normalized composition.

Observed consequences:

- top-level `phase2` matrix audit remains clean
- the refrigerator still shows real world-space geometry drift
- the drift is concentrated in the normalized scene where the descendant
  override stopped behaving like the source scene

This makes the current normalize model incomplete: it handles referenced root
prims, but not scene-authored xform edits beneath those roots.

# Formal Repair Target

The implementation should preserve this invariant:

> If a scene authors descendant xform opinions beneath a referenced asset root,
> then the normalized scene must produce the same composed world-space geometry
> as the source scene, to within floating-point tolerance.

For the first repair pass, the scope is:

- support scene-authored descendant xform overrides beneath referenced asset
  roots during `Phase 2`
- focus first on xform overrides that are already known to break `S1`,
  especially `/Instance/xformOp:scale`
- keep the existing root-prim compensation behavior unless the descendant
  remap requires a targeted adjustment

Out of scope for this pass:

- dedup
- full generalization to every possible scene-layer authored property under
  referenced descendants
- changing the gate back to permissive behavior

# Proposed Implementation Approach

## 1. Detect descendant scene-layer xform overrides during `Phase 2`

Extend scene compensation so that, for each referenced root prim being
normalized, it also scans the source scene for descendant prims beneath that
root which:

- are backed by referenced asset specs
- have scene-layer authored xform properties
- are therefore at risk of changing meaning after normalization

The existing audit helper logic is already a good model for how to detect these
prims.

## 2. Remap the source descendant override into normalized local space

For each affected descendant prim:

- load the source asset stage
- load the normalized asset stage
- resolve the matching descendant path in both assets
- compute:
  - source asset local xform at the descendant
  - source scene composed local xform at the descendant
  - normalized asset local xform at the descendant

Use these to derive the scene-authored delta relative to the source asset, then
re-express that delta in normalized local space before writing it back into the
normalized scene layer.

The first implementation should prefer a single explicit
`xformOp:transform` override on the descendant in the normalized scene layer,
rather than trying to preserve the old op-by-op shape. The goal of the first
pass is semantic correctness, not minimal authored diff.

## 3. Write the remapped descendant override into the normalized layout

After writing the existing root-prim compensation:

- locate the descendant prim in the destination layout
- author the remapped descendant local transform
- record the action in the `Phase 2` report

Suggested `Phase 2` report additions:

- `descendant_overrides_detected`
- `descendant_overrides_remapped`
- `descendant_overrides_failed`
- top examples with:
  - scene root prim
  - descendant path
  - source scene local matrix
  - normalized remapped matrix

## 4. Keep the new gate strict

Do not weaken the new descendant-override gate.

The expected progression is:

- before the fix: gate fails because inert descendant overrides are detected
- after the fix: gate should pass because the remapped descendant overrides
  keep world-space geometry aligned and the inert count returns to zero or is
  otherwise explained by a compensated replacement

# Validation Plan

## Primary repro

Use the existing `S1` refrigerator repro only:

- Scene:
  `home/MV7J6NIKTKJZ2AABAAAAADA8_usd`
- Prim:
  `/Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0`

Required checks after the fix:

- `debug_scene_prim_geometry_delta.py` shows refrigerator aggregate centroid
  delta near zero
- refrigerator pointwise max delta returns to floating-point noise
- `check_normalize_gate_from_reports.py` no longer fails on descendant override
  counts for this repro

## Control check

Re-run one clean control asset in the same scene, such as the sampled
`nightstand`, to ensure the fix does not regress assets that already pass.

Expected result:

- control asset remains at the current `~1e-4` or better noise level

## Gate-level check

Re-run the normalize-only gate on the repaired `S1` output and require:

- pairwise displaced `> 0.01 = 0`
- `matrix_mismatch = 0`
- descendant override failure counts are zero or otherwise resolved by the new
  compensation path

## Stop condition

Do not proceed to dedup until the `S1` normalize-only gate is clean under the
new verdict logic.

# Deliverables For The Repair Pass

The repair pass should leave behind:

- code changes in `scripts/normalize_asset_transforms.py`
- updated `Phase 2` reporting for remapped descendant overrides
- test outputs for refrigerator and one clean control asset
- a short execution note and results note under `docs/test0_smoke/20260312_074731/`

