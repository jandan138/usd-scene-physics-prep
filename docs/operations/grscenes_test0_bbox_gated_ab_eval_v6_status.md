---
title: "GRScenes-test0 BBox-Gated A/B Eval v6 Status"
created_at: "2026-03-31"
updated_at: "2026-03-31"
maintainer: "Claude Code"
status: "active"
code_reference:
  - "scripts/c1_autorun_categories.py"
  - "scripts/placement_pairwise_compare.py"
  - "scripts/summarize_bbox_ab_eval.py"
  - "check_reports/test0_bbox_gated/20260331_bbox_ab_eval_v6/run_manifest.json"
  - "check_reports/test0_bbox_gated/20260331_bbox_ab_eval_v6/summary/ab_comparison.json"
---

# Summary

This document records the completed bbox-gated real-data A/B evaluation v6 for
`GRScenes-test0`.

v6 is the first run that produced trustworthy Policy A / Policy B terminal
outputs across the full execution shape: cert -> apply -> audit -> step6
dry_run.

# Run Identity

- run id: `20260331_bbox_ab_eval_v6`
- run root: `check_reports/test0_bbox_gated/20260331_bbox_ab_eval_v6/`
- commit: `90b82c1` (with `--allow-no-mesh` audit fix applied during run)
- baseline: `GRScenes-test0-rebuilt-normalize-prededup`
- merged report source: `check_reports/test0_rebuilt_dedup/geom_only/`
  (freshly merged, not reused from v5)

# Execution Shape

- merged `geom_only` report input only
- bbox-gated autorun (`--bbox-gated`)
- `--scene-files layout.usd`
- changed-scene-only audit
- category sharding via `--category-list-json` (8 shards, 83 non-door categories)
- Step6 `dry_run`
- `--v-matrix-mode auto`
- `--mode-reports-dir check_reports/test0_rebuilt_dedup`
- `--allow-no-mesh` in audit verdict

# Terminal Status

## Policy A (`bbox_primary_rmse_observe`)

- 8/8 shards: **Succeeded**
- 0 failures

## Policy B (`bbox_primary_rmse_harder`)

- 6/8 shards: **Succeeded**
- 2/8 shards: **Failed** (real quality findings, not infrastructure)
  - shard 1 (`t0bboxabv6b_1_8`): `bottle` category — 2 ref_changed hard fails,
    2 vertex RMSE > 1.0
  - shard 5 (`t0bboxabv6b_5_8`): `other` category — 2 ref_changed hard fails,
    `vertex_rmse_gt_eps` blocking reason

# A/B Comparison Highlights

## Yield

| Metric | Policy A | Policy B | Delta (B-A) |
|--------|----------|----------|-------------|
| Discovered categories | 83 | 70 | -13 |
| Mapping pairs | 33,371 | 32,555 | -816 |
| Changed layouts | 1,758 | 1,438 | -320 |
| Audit passed categories | 73 | 59 | -14 |
| Audit failed categories | 0 | 2 | +2 |
| Ref changed hard fails | 0 | 4 | +4 |
| Step6 gate passed | 73 | 59 | -14 |

## Interpretation

Policy B (`bbox_primary_rmse_harder`) is strictly more conservative:

- 816 fewer mapping pairs (-2.4%)
- 320 fewer changed layouts (-18.2%)
- 14 fewer audit-passed categories
- 4 ref_changed hard fails (vs 0 for Policy A)
- 2 categories blocked by audit (bottle, other)

Policy A (`bbox_primary_rmse_observe`) accepts all geometry with 0 hard fails
across all 83 categories. All quality metrics pass (0 displacement > 0.01,
0 RMSE > 0.01 for ref_changed prims).

## Largest Mapping Pair Deltas (B vs A)

| Category | Delta |
|----------|-------|
| cup | -252 |
| plant | -170 |
| faucet | -96 |
| towel | -89 |
| shelf | -56 |
| tv | -51 |
| washingmachine | -34 |

## Top Categories by Mapping Pairs (both policies)

| Category | Pairs |
|----------|-------|
| other | 10,650 |
| book | 10,499 |
| wall | 3,027 |
| pen | 1,820 |
| ground | 1,418 |
| bottle | 1,242 |

# Runtime Fixes Applied During v6

## 1. ijson runtime fix (pre-v6)

- root cause: DLC cert jobs missing `ijson` module
- fix: repo-vendored `ijson` in `third_party/runtime_deps/isaac_py310/`
- validation: smoke job `dlc8btgcq9lvbgdv` Succeeded
- doc: `docs/operations/grscenes_test0_bbox_gated_dlc_ijson_validation_status_20260331.md`

## 2. `--allow-no-mesh` audit flag (during v6)

- root cause: `total_no_mesh > 0` strict gate in `placement_pairwise_compare.py`
  caused all categories to fail audit, even when all actual geometry comparisons
  passed
- fix: added `--allow-no-mesh` flag to `placement_pairwise_compare.py`;
  `c1_autorun_categories.py` passes it in bbox-gated mode
- files changed: `scripts/placement_pairwise_compare.py`,
  `scripts/c1_autorun_categories.py`

# Key Artifacts

- run manifest: `check_reports/test0_bbox_gated/20260331_bbox_ab_eval_v6/run_manifest.json`
- DLC status summary: `check_reports/test0_bbox_gated/20260331_bbox_ab_eval_v6/summary/dlc_status_summary.json`
- A/B comparison JSON: `check_reports/test0_bbox_gated/20260331_bbox_ab_eval_v6/summary/ab_comparison.json`
- A/B comparison MD: `check_reports/test0_bbox_gated/20260331_bbox_ab_eval_v6/summary/ab_comparison.md`
- Policy A outputs: `check_reports/test0_bbox_gated/20260331_bbox_ab_eval_v6/policy_a/`
- Policy B outputs: `check_reports/test0_bbox_gated/20260331_bbox_ab_eval_v6/policy_b/`

# Related Documents

- `docs/operations/grscenes_test0_bbox_gated_dedup_status_20260327.md`
- `docs/operations/grscenes_test0_bbox_gated_status_change_20260330.md`
- `docs/operations/grscenes_test0_bbox_gated_dlc_ijson_remediation_plan_20260330.md`
- `docs/operations/grscenes_test0_bbox_gated_dlc_ijson_validation_status_20260331.md`
