---
title: "Anomalous Asset Mesh Investigation: test0 vs normalized"
code_reference: "scripts/normalize_asset_transforms.py"
created_at: "2026-03-12"
updated_at: "2026-03-12"
maintainer: "codex"
status: "completed"
---

# Anomalous Asset Mesh Investigation

## Summary

This note investigates four anomalous assets that showed inconsistent states
between `GRScenes-test0` and `GRScenes-test1-normalized`:

- `blanket/cea9f547f8db55c5778d0959079a5927`
- `cabinet/b98d6ccbeb75dfdeb60e27649a5b055a`
- `other/d41d8cd98f00b204e9800998ecf8427e`
- `person/351316cbb083f9f4df0cccd60cbfa848`

The conclusion is that these assets do not share the same failure mode:

- `blanket/cea9...` normalized successfully and was later soft-deleted by an
  earlier dedup run (`c1_union`), so its absence from the live normalized tree
  is expected.
- `cabinet/b98...`, `other/d41...`, and `person/3513...` were already bad
  inputs for normalization. Their normalized directories are partial shells
  because the source USDs could not produce valid normalized asset USD files.

## Investigated Paths

### Source dataset (`GRScenes-test0`)

- `GRScenes-test0/GRScenes_assets/blanket/cea9f547f8db55c5778d0959079a5927/cea9f547f8db55c5778d0959079a5927.usd`
- `GRScenes-test0/GRScenes_assets/cabinet/b98d6ccbeb75dfdeb60e27649a5b055a/b98d6ccbeb75dfdeb60e27649a5b055a.usd`
- `GRScenes-test0/GRScenes_assets/other/d41d8cd98f00b204e9800998ecf8427e/d41d8cd98f00b204e9800998ecf8427e.usd`
- `GRScenes-test0/GRScenes_assets/person/351316cbb083f9f4df0cccd60cbfa848/351316cbb083f9f4df0cccd60cbfa848.usd`

### Normalized dataset (`GRScenes-test1-normalized`)

- `GRScenes-test1-normalized/GRScenes_assets/blanket/cea9f547f8db55c5778d0959079a5927`
- `GRScenes-test1-normalized/GRScenes_assets/cabinet/b98d6ccbeb75dfdeb60e27649a5b055a`
- `GRScenes-test1-normalized/GRScenes_assets/other/d41d8cd98f00b204e9800998ecf8427e`
- `GRScenes-test1-normalized/GRScenes_assets/person/351316cbb083f9f4df0cccd60cbfa848`

## Raw USD Mesh Findings

The original `GRScenes-test0` USDs were opened with `pxr.Usd.Stage.Open()` and
traversed for `UsdGeom.Mesh` prims.

| Asset | Mesh prims in source USD | Finding |
|---|---:|---|
| `blanket/cea9...` | 2 | Valid mesh data exists under `/Root/Instance/.../component{0,1}` |
| `cabinet/b98...` | 1 | Mesh prim exists at `/Root/Instance/Group_Static/SM_00` but has no points, no face counts, and no extent |
| `other/d41...` | 0 | No mesh prims anywhere in the source USD |
| `person/3513...` | 0 | No mesh prims anywhere in the source USD |

### Detailed prim structure

#### `blanket/cea9f547f8db55c5778d0959079a5927`

- Default prim: `/Root`
- `/Root/Instance` exists and contains `SM_ZXUTNXJVAIICWPTULI888888`
- Mesh prims:
  - `/Root/Instance/SM_ZXUTNXJVAIICWPTULI888888/component1` with 104 points and 208 faces
  - `/Root/Instance/SM_ZXUTNXJVAIICWPTULI888888/component0` with 80 points and 156 faces

This is a normal, valid source asset.

#### `cabinet/b98d6ccbeb75dfdeb60e27649a5b055a`

- Default prim: `/Root`
- `/Root/Instance` exists and contains `Group_Static`
- Mesh prim:
  - `/Root/Instance/Group_Static/SM_00`
- But all core geometry attributes are empty:
  - `points = None`
  - `faceVertexCounts = None`
  - `faceVertexIndices = None`
  - `extent = None`

So the source stage contains a `Mesh` prim by type, but not usable mesh data.

#### `other/d41d8cd98f00b204e9800998ecf8427e`

- Default prim: `/Root`
- `/Root/Instance` exists and contains `Group_default_00`
- No `Mesh` prims were found under the stage at all

#### `person/351316cbb083f9f4df0cccd60cbfa848`

- Default prim: `/Root`
- `/Root/Instance` exists but has no children
- No `Mesh` prims were found under the stage at all

## Normalize-Stage Evidence

The normalization docs already recorded exactly three bad assets:

- `cabinet/b98d6ccbeb75dfdeb60e27649a5b055a`
- `other/d41d8cd98f00b204e9800998ecf8427e`
- `person/351316cbb083f9f4df0cccd60cbfa848`

Source:

- `docs/usage/normalize_asset_transforms.md`
- `docs/operations/renormalization_v2_execution.md`

The normalize report stores explicit errors for those three assets:

- `cabinet/...` -- `No meshes found under /Root/Instance ...`
- `other/...` -- `No meshes found under /Root/Instance ...`
- `person/...` -- `No meshes found under /Root/Instance ...`

This wording slightly compresses the distinctions found in the raw-stage
inspection:

- `cabinet/...` has a typed mesh prim, but it is empty
- `other/...` has no mesh prim at all
- `person/...` has no child prims under `/Root/Instance`

Operationally, all three collapse to the same outcome: normalization cannot
emit a valid normalized asset USD.

## Current Normalized-Tree State

| Asset | Current live status |
|---|---|
| `blanket/cea9...` | Missing from live normalized tree |
| `cabinet/b98...` | Directory exists, but normalized `usd/<uid>.usd` is missing |
| `other/d41...` | Directory exists, but normalized `usd/<uid>.usd` is missing |
| `person/3513...` | Directory exists, but normalized `usd/<uid>.usd` is missing |

For the three bad normalize inputs, the current live state matches the
normalize-phase documentation: partial output directories exist, but no valid
normalized asset USD was produced.

## Dedup / Soft-Delete Trace

### `blanket/cea9...`

This asset was normalized successfully and verified as valid:

- `check_reports/normalize/normalize_report.json`
- `check_reports/normalize/verification_report.json`

Later, an earlier 2-way dedup run with group label `c1_union` soft-deleted it:

- `check_reports/c1_bulk/blanket_bulk_step6_v1/soft_delete_old_assets_report.json`

The backed-up asset still exists here:

- `GRScenes-test1-normalized_bak/_dedup_assets/c1_union_20260311_095415/GRScenes_assets/blanket/cea9f547f8db55c5778d0959079a5927`

When the later 3-way dedup run (`c1_union_3way`) processed `blanket`, it marked
this UID as `missing` because it had already been moved out of the live tree by
the earlier run:

- `check_reports/c1_bulk/blanket_bulk_step6_v2/soft_delete_old_assets_report.json`

### `cabinet/b98...`, `other/d41...`, `person/3513...`

These assets were already broken before dedup started. Early C1 logs from
2026-03-09 already contain `Could not open asset` warnings for their normalized
USD paths:

- `check_reports/c1_bulk/_autorun/c1_autorun_20260309_120150/tea_table/01_build_mapping.log`

This shows the problem predates later V3 dedup execution.

## Final Classification

| Asset | Final classification |
|---|---|
| `blanket/cea9...` | Healthy source asset; normalized correctly; later soft-deleted by earlier dedup run |
| `cabinet/b98...` | Bad source asset with empty mesh prim; normalization could not generate usable output |
| `other/d41...` | Bad source asset with no mesh prims; normalization could not generate usable output |
| `person/3513...` | Bad source asset with no child prims / no mesh prims; normalization could not generate usable output |

## Recommended Interpretation

For future counting and validation:

- Do **not** treat `blanket/cea9...` as a lost asset. It is a normal dedup
  backup case.
- Do treat `cabinet/b98...`, `other/d41...`, and `person/3513...` as known
  normalization failures inherited from source data quality.
- When computing effective asset counts in `GRScenes-test1-normalized`, count
  assets by the presence of a valid normalized `usd/<uid>.usd` file, not by
  directory count alone.
