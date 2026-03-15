---
title: "GRScenes test0-rebuilt Phase 1 Allowlist Verdict"
code_reference: scripts/check_test0_phase1_allowlist.py
created_at: 2026-03-15
updated_at: 2026-03-15
maintainer: claude
status: active
---

# GRScenes test0-rebuilt Phase 1 Allowlist Verdict

## Run Context

- **Run ID**: 20260314_rebuilt_full_dlc_v1
- **Run root**: `check_reports/test0_rebuilt_full/20260314_rebuilt_full_dlc_v1/`
- **Source dataset**: GRScenes-test0-rebuilt
- **Phase 1 output**: GRScenes-test0-rebuilt-normalized/GRScenes_assets/
- **Categories**: 114 total
- **DLC result**: 79 Succeeded / 35 Failed

## Phase 1 Failure Breakdown

### Group 1: Previously Known Bad Assets (3 categories)

| Category | Asset Hash | Error |
|----------|-----------|-------|
| cabinet | `b98d6ccbeb75dfdeb60e27649a5b055a` | No meshes found under /Root/Instance |
| other | `d41d8cd98f00b204e9800998ecf8427e` | No meshes found under /Root/Instance |
| person | `351316cbb083f9f4df0cccd60cbfa848` | No meshes found under /Root/Instance |

All 3 are documented in prior runs.

### Group 2: door_UUID Categories (31 categories)

All 31 `door_<UUID>` categories contain a single asset: `d41d8cd98f00b204e9800998ecf8427e` (the empty-string MD5 hash). This is the **same known-bad asset** as in the "other" category.

Each `door_<UUID>` represents a distinct door instance from the source data. These are placeholder/broken door assets with no mesh geometry. Centers files: `{}` (empty JSON object).

### Group 3: book Category (1 category)

- **Total assets**: 13,249
- **Normalized**: 13,213 (99.7% success)
- **Failed**: 36 assets — non-empty USD files (10KB–303KB) that lack mesh geometry under `/Root/Instance`
- **Centers**: `centers_book.json` (1.9MB, valid)

Source data defects, not normalization regressions.

## Allowlist Script Changes

### 1. `_submit_logs` filtering
Added `_submit_logs` to `IGNORED_DIR_NAMES` to prevent the DLC submit log directory from being treated as a category.

### 2. `door_*` pattern allowlist
- `_allowed_exit_codes_for()`: categories starting with `door_` get `{0, 1}`
- `_allowed_asset_errors_for()`: categories starting with `door_` get the `d41d8cd98f00b204e9800998ecf8427e` asset allowlisted with dynamic category prefix

### 3. `book` category allowlist
Added `book` to `DEFAULT_ALLOWED_NONZERO_EXITS` with `{1}`.

## Verdict

**ALLOWLIST: ACCEPTABLE** — All 35 failures are source data defects, not normalization regressions. Phase 2 may proceed.

## Impact on Phase 2

- 79 succeeded categories have valid centers files (verified)
- 31 door_* categories with `{}` centers: no prims reference these empty assets in layouts
- book's 36 missing assets: scene compensation will skip these (no centers entry) — acceptable for source-broken assets
- cabinet/other/person: same as prior runs, no impact
