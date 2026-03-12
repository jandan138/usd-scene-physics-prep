---
title: Worker 2 S1 Subset Build
code_reference:
  - scripts/build_scene_uid_subset_package.py
created_at: 2026-03-12
updated_at: 2026-03-12
maintainer: Codex Worker-2
status: completed
---

# Summary

Built the `S1` smoke subset for scene UID `MV7J6NIKTKJZ2AABAAAAADA8` from `GRScenes-test0` into `GRScenes-test0-smoke-S1`.

# Commands

```bash
./scripts/isaac_python.sh scripts/build_scene_uid_subset_package.py \
  --src GRScenes-test0 \
  --dst GRScenes-test0-smoke-S1 \
  --scene-uid MV7J6NIKTKJZ2AABAAAAADA8 \
  --scene-category home \
  --write-manifest \
  --verify
```

# Results

- Wrote `GRScenes-test0-smoke-S1/subset_manifest.json`.
- Smoke root contains `GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd`, `GRScenes_assets`, and `Material/mdl`.
- Manifest counts:
  - `assets=1105`
  - `usd_files_scanned=1106`
  - `scene_files_skipped=4`
  - `asset_files_skipped=2210`
- Verify block in manifest reports `missing_count=0`.
- Upgraded the smoke subset asset layout to normalize-compatible form:
  - from `GRScenes_assets/<category>/<uid>/<uid>.usd`
  - to `GRScenes_assets/<category>/<uid>/usd/<uid>.usd`
- Rewrote `layout.usd` so all `1105` authored references now point to the `usd/` subpath form.

# Notes

- The build emitted source-side unresolved reference warnings from `start_result_navigation.usd` for `models/layout/articulated/door/.../instance.usd`.
- Those warnings did not prevent manifest generation and did not produce verify misses in the subset manifest.
- The subset is ready for the `normalize-only` stage.
- The path upgrade used:

```bash
python - <<'PY'
# moved asset USDs into usd/<uid>.usd and wrote mapping JSON
PY

./scripts/isaac_python.sh scripts/rewrite_layout_asset_refs_with_compensation.py \
  --layout-usd GRScenes-test0-smoke-S1/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd \
  --subset-root GRScenes-test0-smoke-S1 \
  --mapping-json check_reports/test0_smoke/20260312_074731/summary/subset_s1_path_upgrade_mapping.json \
  --no-compensation \
  --report-out check_reports/test0_smoke/20260312_074731/summary/subset_s1_layout_path_upgrade_report.json \
  --preview 5
```
