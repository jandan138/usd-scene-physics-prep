---
title: "Worker-4 Normalize Command Plan for S1 Smoke"
code_reference:
  - "scripts/normalize_asset_transforms.py"
  - "scripts/build_scene_uid_subset_package.py"
created_at: "2026-03-12T07:49:55Z"
updated_at: "2026-03-12T07:49:55Z"
maintainer: "Codex Worker-4"
status: "done"
---

# Summary

Prepared the ready-to-run `normalize-only` command sequence for the future subset root `GRScenes-test0-smoke-S1`.

# Findings

- `scripts/normalize_asset_transforms.py` supports an explicit split workflow:
  - `--phase 1` writes normalized assets and `centers_*.json`
  - `--phase 2` requires `--centers-dir` and writes compensated scene layouts
- The script writes `normalize_report.json` to `--report-dir`.
- With no category filter, the centers filename is `centers_all.json`.
- To avoid report overwrite, Phase 1 and Phase 2 should use separate report directories.

# Ready Commands

```bash
mkdir -p \
  /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/phase1 \
  /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/phase2

./scripts/isaac_python.sh scripts/normalize_asset_transforms.py \
  --assets-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1/GRScenes_assets \
  --scenes-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1/GRScenes100 \
  --output-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1-normalized \
  --phase 1 \
  --report-dir /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/phase1

./scripts/isaac_python.sh scripts/normalize_asset_transforms.py \
  --assets-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1/GRScenes_assets \
  --scenes-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1/GRScenes100 \
  --output-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1-normalized \
  --phase 2 \
  --centers-dir /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/phase1 \
  --report-dir /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/phase2
```

# Expected Outputs

- Phase 1:
  - `check_reports/test0_smoke/20260312_074731/normalize/phase1/centers_all.json`
  - `check_reports/test0_smoke/20260312_074731/normalize/phase1/normalize_report.json`
  - `GRScenes-test0-smoke-S1-normalized/GRScenes_assets`
- Phase 2:
  - `check_reports/test0_smoke/20260312_074731/normalize/phase2/normalize_report.json`
  - `GRScenes-test0-smoke-S1-normalized/GRScenes100`

# Constraints

- I did not run normalize because the subset root is a prerequisite and the task explicitly asked for handoff readiness only.
- I stayed within the assigned write scope and did not edit repository code.
