---
title: Test0 Smoke Baseline And S1 Subset Status
code_reference:
  - check_reports/test0_smoke/20260312_074731/baseline/baseline_brief.json
  - check_reports/test0_smoke/20260312_074731/summary/subset_s1_status.json
created_at: 2026-03-12
updated_at: 2026-03-12
maintainer: Codex
status: active
---

## Summary

This run is validating whether `GRScenes-test0` can be used as a healthy starting
point for a `normalize + dedup` smoke pass. The current run uses `S1` with the
single scene `home/MV7J6NIKTKJZ2AABAAAAADA8_usd`.

## Baseline

- `test1` is explicitly excluded as a baseline because the existing evidence
  already shows placement drift before normalize starts.
- The evidence set is anchored in
  `check_reports/placement_evidence_closure`.
- The known problematic sample scene remains
  `home/MV7J6NIKTKJZ2AABAAAAADA8_usd`.

## S1 Subset

- The `S1` subset was built from `GRScenes-test0` using `layout-only` scene
  scanning.
- `subset_manifest.json` reports `1105` assets in the closure and no missing
  references in the built subset.
- The subset build exposed an interface mismatch: asset USDs are stored as
  `<uid>/<uid>.usd`, while `normalize_asset_transforms.py` expects
  `<uid>/usd/<uid>.usd`.
- A compatibility preparation step was added after subset generation so the
  smoke run can continue without changing normalize logic.
