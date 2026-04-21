---
title: "Transitive Full Baseline Export"
code_reference:
  - scripts/export_transitive_full_baseline_dataset.py
  - tests/test_export_transitive_full_baseline_dataset.py
created_at: 2026-04-20
updated_at: 2026-04-20
maintainer: OpenCode
status: active
---

# Transitive Full Baseline Export

## Summary

Added `scripts/export_transitive_full_baseline_dataset.py` to derive a fresh slim delivery dataset from the authoritative transitive full rerun dataset root.

The exporter:
- reads final rerun layouts from `GRScenes100/**/layout.c1_v8_tier2_rollout_transitive_full_dlc_bbox_primary_rmse_observe_v1.usd`
- writes them to the delivery tree as `layout.usd`
- copies `Material/` wholesale
- retains only referenced `GRScenes_assets`
- emits `MANIFEST.json`, `asset_pruning_summary.json`, `dangling_references.json`, and `README.md`

## Code Changes

- Added a single entrypoint function, `export_transitive_full_baseline_dataset(...)`.
- Added layout reference collection with two runtime paths:
  - direct `pxr` stage traversal when USD Python bindings are available
  - ASCII reference parsing for fixture-driven tests when `pxr` is unavailable
- Added CLI auto-reexec through `scripts/isaac_python.sh` so `python scripts/export_transitive_full_baseline_dataset.py ...` still works in environments where plain `python` lacks `pxr`.
- Normalized absolute layout asset refs back to delivery-relative `GRScenes_assets/...` paths before pruning.
- Tightened asset retention so dangling refs are recorded when the referenced USD file is missing even if the surrounding asset directory exists.
- Aggregated rerun provenance metrics from `full_rerun_root/**/*.stats.json` into `MANIFEST.json`, `asset_pruning_summary.json`, and `dangling_references.json`.

## Tests

Added fixture-driven coverage in `tests/test_export_transitive_full_baseline_dataset.py` for:
- exporting final rerun layouts as delivery `layout.usd`
- retaining only referenced assets
- manifest, summary, dangling-ref, and README generation
- refusing to write into an existing output root
- ASCII fallback reference parsing when `pxr` is unavailable
- CLI auto-reexec into Isaac Sim Python when `pxr` is unavailable
- absolute asset ref normalization
- dangling-ref recording when a referenced USD file is missing inside an existing asset directory

## TDD And Debugging Notes

Initial red phase:

```bash
python -m pytest tests/test_export_transitive_full_baseline_dataset.py -q
```

Observed red results:
- first failed with `ModuleNotFoundError: No module named 'export_transitive_full_baseline_dataset'`
- after adding the stub, failed with `NotImplementedError`

Runtime debugging repro:

```bash
python scripts/export_transitive_full_baseline_dataset.py \
  --dataset-root "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup-transitive-rerun-20260413" \
  --full-rerun-root "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout_transitive_full_dlc_20260415" \
  --output-root "/tmp/grscenes-test0-transitive-full-baseline-smoke" \
  --full-rerun-job-id "dlcve680agoitv7g"
```

Observed root causes and fixes:
- plain `python` lacked `pxr` in this environment, so the CLI now re-execs via `scripts/isaac_python.sh`
- real layout refs arrived as absolute dataset paths, so refs are now normalized back to `GRScenes_assets/...`
- some refs pointed at missing USD files inside existing asset directories, so exact-file existence is now checked before counting an asset as retained

## Verification

Unit verification:

```bash
python -m pytest tests/test_export_transitive_full_baseline_dataset.py -q
```

Result:
- `10 passed in 0.11s`

Smoke export verification:

```bash
python scripts/export_transitive_full_baseline_dataset.py \
  --dataset-root "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup-transitive-rerun-20260413" \
  --full-rerun-root "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout_transitive_full_dlc_20260415" \
  --output-root "/tmp/grscenes-test0-transitive-full-baseline-smoke" \
  --full-rerun-job-id "dlcve680agoitv7g"
python -c "import json; from pathlib import Path; root = Path('/tmp/grscenes-test0-transitive-full-baseline-smoke'); manifest = json.loads((root / 'MANIFEST.json').read_text()); summary = json.loads((root / 'asset_pruning_summary.json').read_text()); print((root / 'MANIFEST.json').exists()); print((root / 'asset_pruning_summary.json').exists()); print((root / 'dangling_references.json').exists()); print(manifest['full_rerun_job_id']); print(manifest['exported_scene_count']); print(manifest['retained_asset_count']); print(manifest['dangling_reference_count']); print(summary['omitted_asset_count'])"
```

Smoke result:
- manifest files exist: `True`, `True`, `True`
- `full_rerun_job_id`: `dlcve680agoitv7g`
- `exported_scene_count`: `99`
- `retained_asset_count`: `85549`
- `dangling_reference_count`: `35`
- `omitted_asset_count`: `98`

The smoke run emitted USD warnings for unresolved assets while scanning layouts; those now correspond to recorded dangling references rather than being silently ignored.

## Actual Delivery Export

The exporter was also run to the real delivery location:

```bash
python scripts/export_transitive_full_baseline_dataset.py \
  --dataset-root "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup-transitive-rerun-20260413" \
  --full-rerun-root "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout_transitive_full_dlc_20260415" \
  --output-root "/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418" \
  --full-rerun-job-id "dlcve680agoitv7g"
```

Observed delivery result:

- output root: `/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418`
- `exported_scene_count`: `99`
- `retained_asset_count`: `85549`
- `dangling_reference_count`: `35`
- `omitted_asset_count`: `98`
- required top-level outputs present:
  - `GRScenes100/`
  - `GRScenes_assets/`
  - `Material/`
  - `MANIFEST.json`
  - `asset_pruning_summary.json`
  - `dangling_references.json`
  - `README.md`

## Concerns

- The smoke output is intentionally left under `/tmp/grscenes-test0-transitive-full-baseline-smoke` for inspection.
- The exporter currently counts rerun metrics by summing numeric top-level keys from `*.stats.json`; if future stats files add unrelated numeric fields, the aggregation may need tightening.
