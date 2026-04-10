---
title: "v8 Shuffle Fix Probe — 状态记录"
code_reference:
  - scripts/compute_vertex_transform.py
  - scripts/c1_autorun_categories.py
created_at: "2026-04-03"
updated_at: "2026-04-03"
maintainer: "zhuzihou"
status: "probe-done-bottle"
---

# v8 Shuffle Fix Probe — 状态记录 (2026-04-03)

## 背景

topo_filesize 去重配对存在顶点 shuffle 问题（详见 `docs/changes/2026-04-03_topo_nn_shuffle_gated_fallback.md`）。
修复已合入主分支 `scripts/compute_vertex_transform.py`，本次跑 Step 2 探针验证修复效果。

## 探针命令

```bash
python3 scripts/c1_autorun_categories.py \
  --dataset-root GRScenes-test0-rebuilt-normalize-prededup \
  --bak-root GRScenes-test0-rebuilt-normalized_bak \
  --report check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/all_categories_union_merged.json \
  --c1-bulk-dir check_reports/test0_rebuilt_dedup/c1_bulk_v8_shuffle_fix \
  --include-regex "^(bottle|other)$" \
  --group-label c1_v8_shuffle_fix_probe \
  --step6-mode dry_run \
  --bbox-gated \
  --bbox-policy bbox_primary_rmse_observe \
  --v-matrix-mode auto \
  --mode-reports-dir check_reports/test0_rebuilt_dedup/v8_prededup
```

## Bottle 探针结果

### Cert 认证对比

| 指标 | 修复前 (c1_bulk_v8_multimode) | 修复后 (c1_bulk_v8_shuffle_fix) | 变化 |
|------|-----|-----|------|
| eligible_count | 996 | **1294** | **+298** |
| bbox_precheck_failed_topo_filesize | 595 | **297** | **-298** |
| bbox_precheck_failed_shape_invariant | 14 | 14 | 不变 |
| transitive_not_supported | 256 | 256 | 不变 |

### Audit 审计

| 指标 | 修复前 | 修复后 |
|------|--------|--------|
| scenes | 52 | 52 |
| total compared prims | — | 70,102 |
| displaced > 0.01 | 0 | **0** |
| vertex_rmse > 0.01 | — | 16 (单场景 MVSYCXYKTKJ66AABAAAAAAA8) |
| ref_changed_hard_fails | 489 | **390** (-99) |
| audit passed | false | **false** (已知 observe 策略行为) |

### 结论

- **修复有效**：298 对 topo_filesize 配对从 bbox_precheck_failed 变为 eligible
- **位置安全**：displaced > 0.01 = 0，无任何位置回归
- **audit false 是已知行为**：ref_changed 的 bbox delta 在 observe 策略下触发 hard fail，修复前后一致

## 未完成

- `other` 类目探针：因 bottle audit exit(1) 未跑
- Step 3 (apply + pairwise compare)
- Step 5 (全量 83 类目 rollout)

## 输出路径

| 产物 | 路径 |
|------|------|
| 新 c1_bulk_dir | `check_reports/test0_rebuilt_dedup/c1_bulk_v8_shuffle_fix/` |
| Bottle cert | `.../bottle_bbox_primary_rmse_observe_v1/01_cert/` |
| Bottle audit | `.../bottle_bbox_primary_rmse_observe_v1/03_audit/` |
| Ledger | `.../c1_bulk_v8_shuffle_fix/_autorun/c1_v8_shuffle_fix_probe_*_20260403_051510/ledger.jsonl` |
