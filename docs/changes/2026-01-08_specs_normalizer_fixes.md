---
title: Specs Normalizer Enhancements and Fixes (2026-01-08)
code_reference:
- specs_normalizer/normalize.py
- scripts/fix_usd_textures_case.py
created_at: '2026-01-08'
updated_at: '2026-01-08'
maintainer: Codex
status: completed
---

# Specs Normalizer Enhancements and Fixes (2026-01-08)

## 1. Features
- **Resume Capability (Skip Existing)**: 
  - `specs_normalizer` now checks if the destination file exists before exporting. If it does, the export is skipped, allowing for faster recovery from interrupted runs.
  - This is particularly useful for large-scale exports on networked file systems (e.g., DLC).
- **Centralized Asset Annotation**:
  - Modified `specs_normalizer` to generate a single, top-level `Asset_annotation.json` at the asset root (e.g., `GRScenes_assets/Asset_annotation.json`) instead of redundant files in each category subdirectory. This aligns with the dataset structure specification.

## 2. Bug Fixes
- **MDL Path Normalization**:
  - Fixed an issue where texture paths in MDL files were case-sensitive (`Textures` vs `textures`), causing missing textures on Linux. The normalizer now enforces lowercase `textures`.
  - Fixed a path duplication bug where `Material/mdl/` was being appended redundantly (e.g., `.../Material/mdl/mdl/...`). The rewriter now correctly strips the prefix from the source path before joining with the base.

## 3. Scripts
- **New Utility Scripts**:
  - `scripts/inspect_single_usd.py`: Diagnostic tool to inspect USD prim structure and references.
  - `scripts/fix_usd_textures_case.py`: Batch repair tool to apply MDL path fixes (case sensitivity and deduplication) to already exported USD files in place.
