---
title: test0 smoke s1 lead run note
code_reference:
  - scripts/normalize_asset_transforms.py
created_at: 2026-03-12
updated_at: 2026-03-12
maintainer: codex
status: draft
---

# Summary

This run executed the `S1` smoke validation for `GRScenes-test0` scene `MV7J6NIKTKJZ2AABAAAAADA8`.

The run stopped after `normalize-only` because the agreed gate was not clean:

- `placement_pairwise_compare.py`: `displaced > 0.01 = 1`
- `audit_normalize_phase2.py`: `center_found = 1105`, `matrix_mismatch = 0`

The single outlier was a refrigerator prim with centroid displacement `0.061537`.

# Code change used in this run

The lead updated `scripts/normalize_asset_transforms.py` so `discover_assets()` can consume both asset layouts used in this repo:

- normalized-style: `<category>/<uid>/usd/<uid>.usd`
- flat legacy style: `<category>/<uid>/<uid>.usd`

The purpose of this change was only to remove an input-structure assumption that prevented `GRScenes-test0` smoke data from entering the normalize pipeline. No normalize math or dedup logic was changed.

# Execution notes

- The final successful `Phase 1` strategy was category-based execution, with centers merged into `check_reports/test0_smoke/20260312_074731/normalize/phase1/centers_merged/`.
- `Phase 2` then completed cleanly and a synthetic `pre_c1` backup was created so `audit_normalize_phase2.py` could run against the pre-dedup normalized state.
- Dedup was intentionally skipped because the normalize gate did not pass.
