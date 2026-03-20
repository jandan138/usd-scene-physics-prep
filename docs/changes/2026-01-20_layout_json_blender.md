---
title: 2026-01-20 — Generate layout.json for Blender (scene subset)
code_reference: scripts/generate_layout_json_from_usd.py
created_at: '2026-01-20'
updated_at: '2026-01-20'
maintainer: Codex
status: completed
---

# 2026-01-20 — Generate layout.json for Blender (scene subset)

## Summary

Adds a workflow to export a Blender-friendly `layout.json` from a scene `layout.usd`, so Blender can reconstruct the scene layout by instancing per-asset GLBs.

## Changes

- Added script: [scripts/generate_layout_json_from_usd.py](../../scripts/generate_layout_json_from_usd.py)
  - Reads `GRScenes100/**/layout.usd`
  - Extracts referenced `GRScenes_assets/<category>/<uid>` placements
  - Computes world transforms and outputs `layout.json` that points to `glb/<uid>.glb`
- Added usage doc: [docs/usage/layout_json_for_blender.md](../usage/layout_json_for_blender.md)
- Added deep-dive doc: [docs/usage/layout_usd_to_layout_json_deep_dive.md](../usage/layout_usd_to_layout_json_deep_dive.md)
- Updated documentation index to include the new usage doc
