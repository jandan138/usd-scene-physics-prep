---
title: Commercial Scene Export Fixes (2026-01-09)
code_reference:
- specs_normalizer/exporters/scenes.py
- scripts/fix_usd_paths_api.py
created_at: '2026-01-09'
updated_at: '2026-01-09'
maintainer: Codex
status: completed
---

# Commercial Scene Export Fixes (2026-01-09)

## Overview
This document details the issues encountered during the export of **Commercial Scenes** (GRScenes100/commercial) and the comprehensive solutions implemented to fix them. The primary issues involved broken asset references, missing textures due to case sensitivity, and missing asset files.

## Issues Identified

### 1. Broken Asset References (Absolute Paths)
*   **Symptom**: Isaac Sim reported `Could not open asset @/cpfs/...@` errors when loading scenes.
*   **Cause**: The source USD files were in **Binary (Crate)** format. The `specs_normalizer` export script's fallback logic for path rewriting (regex-based) was only checking for text-based USD files and skipping binary ones. This caused absolute paths from the source environment to persist in the exported files.
*   **Diagnosis**: Verified by inspecting `layout.usd` header (`PXR-USDC`) and confirming the script's `is_text` check failure.

### 2. Missing Textures (Case Sensitivity)
*   **Symptom**: Materials failed to load with errors like `References an asset that can not be found: .../Material/mdl/Textures/Day.png`.
*   **Cause**:
    *   **Case Mismatch**: Source MDL files referenced the `Textures` directory (capital T), but the Linux filesystem contained `textures` (lowercase t).
    *   **Incomplete Fix**: An initial fix attempted to replace `/Textures/` with `/textures/` via regex, but it failed because:
        *   It only targeted paths containing `Materials/` prefix (source paths were sometimes `Material/`).
        *   Binary files were not being processed correctly (see Issue #1).
        *   Some shader parameters in binary files might not match the expected `@path@` regex pattern after conversion.

### 3. Missing Assets
*   **Symptom**: Specific asset UIDs (e.g., `chair/e6468d75...`) were reported as missing by Isaac Sim.
*   **Cause**: The asset files were genuinely missing from the `GRScenes_assets` directory. This was a data integrity issue in the source or a filtering result during the initial asset export phase.

## Solutions Implemented

### 1. Binary USD Support in `specs_normalizer`
*   **Fix**: Modified `specs_normalizer/exporters/scenes.py` to handle binary USD files.
*   **Mechanism**:
    *   Detects binary files (`PXR-USDC` header).
    *   Uses `pxr.Sdf.Layer` API to export the binary layer to a temporary text file (`.tmp.usda`).
    *   Applies the regex-based path rewriting to the text file.
    *   Converts the text file back to binary using `Sdf.Layer.Export`.
*   **Benefit**: Ensures path normalization (absolute -> relative) works for ALL scene files, regardless of format.

### 2. Robust Texture Path Normalization
*   **Fix**: Updated the path rewriting logic in `specs_normalizer/exporters/scenes.py`.
*   **Mechanism**:
    *   **Global Replacement**: Implemented a check that unconditionally replaces `/Textures/` with `/textures/` in any path string, regardless of its prefix or context.
    *   **Deduplication**: Added logic to fix `Material/mdl/mdl/` duplication errors.
*   **Benefit**: Guarantees that all texture references match the lowercase directory structure required on Linux.

### 3. API-Based Deep Fix Script (`fix_usd_paths_api.py`)
*   **Tool**: Created `scripts/fix_usd_paths_api.py`.
*   **Mechanism**:
    *   Uses the `pxr.Usd` API to traverse the entire stage (Prims and Attributes).
    *   Identifies `Sdf.AssetPath` and string-based attributes.
    *   Directly modifies the values to enforce `Textures -> textures` and fix path anomalies.
*   **Usage**: Run as a post-processing step to catch any edge cases that regex replacement might miss (e.g., complex shader inputs).

### 4. Asset Recovery
*   **Action**: Re-ran the asset export process using `specs_normalizer` to recover the missing asset files.
*   **Result**: Verified that the missing chair asset is now present in `GRScenes_assets`.

## Verification
*   **Texture Loading**: Confirmed that `Textures` references are now lowercase `textures` and load correctly.
*   **Asset Loading**: Confirmed that `layout.usd` uses correct relative paths (`../../../GRScenes_assets/...`) and successfully resolves the now-present asset files.
*   **Diagnostics**: Used `ls` and custom diagnostic scripts to verify file existence and path correctness.

## Changed Files
*   `specs_normalizer/exporters/scenes.py`: Added binary support and improved path rewriting logic.
*   `set_physics/tools/export_some_scenes.py`: Fixed case sensitivity in regex and comments.
*   `scripts/fix_usd_paths_api.py`: New utility script for API-based path fixing.
*   `scripts/inspect_usd_refs.py`: New utility script for inspecting USD references.
