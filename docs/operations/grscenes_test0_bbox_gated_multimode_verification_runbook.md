---
title: "BBox-Gated Multi-Mode Verification Runbook"
created_at: "2026-04-01"
updated_at: "2026-04-01"
maintainer: "Claude Code (reviewed)"
status: "planned"
code_reference:
  - "scripts/report_asset_mesh_dedup.py"
  - "scripts/compute_vertex_transform.py"
  - "scripts/rewrite_layout_asset_refs_with_compensation.py"
  - "scripts/c1_autorun_categories.py"
  - "scripts/c1_build_bulk_mapping_from_dedup_report.py"
  - "scripts/placement_pairwise_compare.py"
---

# 目标

验证 6-change multi-mode 实现（commit `0c51924`）的正确性，确保：
1. geom_only 报告不再被 tolerance_merge 污染
2. 三种模式（geom_only、topo_filesize、shape_invariant）均可通过 bbox-gated 管线
3. 场景中物体位置无回归（pairwise compare 通过）
4. 去重覆盖率相比 v5/v6 有提升

# 前置条件

| 项目 | 路径 |
|------|------|
| 数据集 | `GRScenes-test0-rebuilt-normalized/` |
| 备份 | `GRScenes-test0-rebuilt-normalized_bak/` |
| 已有 dedup 报告 | `check_reports/test0_rebuilt_dedup/{geom_only,topo_filesize,shape_invariant}/` (114 categories each) |
| v5 基线 | `check_reports/test0_rebuilt_dedup/c1_bulk_v5_translation_fix/` (ground only) |
| v5 验证 | `check_reports/test0_rebuilt_dedup/v5_verification/pairwise_compare.json` (99 scenes, 0 displaced) |
| 联合报告 | `check_reports/test0_rebuilt_dedup/union_3way/all_categories_union_merged.json` |

# Step 1: 重新生成 bottle 和 other 的 geom_only 报告

## 为什么

Change 1 移除了 `_tolerance_merge` 对 geom_only 的污染。需要验证新生成的报告中 `tolerance_merged_groups` 为 0，且组数比旧报告少（减去的就是之前被混入的 tolerance_merge 组）。

## 命令

```bash
# bottle（--assets-root 指向类目子目录，而非整个 GRScenes_assets）
./scripts/isaac_python.sh scripts/report_asset_mesh_dedup.py \
  --assets-root GRScenes-test0-rebuilt-normalized/GRScenes_assets/bottle \
  --out-dir check_reports/test0_rebuilt_dedup/geom_only/bottle \
  --dataset bottle \
  --mode all \
  --merge-tolerance 0.005 \
  --progress-every 50

# other
./scripts/isaac_python.sh scripts/report_asset_mesh_dedup.py \
  --assets-root GRScenes-test0-rebuilt-normalized/GRScenes_assets/other \
  --out-dir check_reports/test0_rebuilt_dedup/geom_only/other \
  --dataset other \
  --mode all \
  --merge-tolerance 0.005 \
  --progress-every 50
```

> **注意**：`--merge-tolerance 0.005` 保留传参但不再对 geom_only 生效（Change 1 已移除该逻辑）。传参是为了和之前的报告元数据保持一致可比性。

## 验证标准

```bash
# 检查 tolerance_merged_groups 必须为 0
jq '.meta.tolerance_merged_groups' \
  check_reports/test0_rebuilt_dedup/geom_only/bottle/bottle_asset_mesh_dedup_geom_only.json
# 期望: 0

jq '.meta.tolerance_merged_groups' \
  check_reports/test0_rebuilt_dedup/geom_only/other/other_asset_mesh_dedup_geom_only.json
# 期望: 0

# 对比旧报告的组数（旧报告应该更多，因为包含了 tolerance_merge 组）
jq '.meta.duplicate_group_count' \
  check_reports/test0_rebuilt_dedup/geom_only/bottle/bottle_asset_mesh_dedup_geom_only.json
```

# Step 2: 探针测试 — 对 bottle 和 other 跑 bbox-gated 管线

## 为什么

验证 Changes 2-6 协同工作：新模式能通过证书阶段、白名单阶段，且使用正确的补偿公式。

## 方案 A：用 c1_autorun_categories 自动化（推荐）

```bash
python3 scripts/c1_autorun_categories.py \
  --dataset-root GRScenes-test0-rebuilt-normalized \
  --bak-root GRScenes-test0-rebuilt-normalized_bak \
  --report check_reports/test0_rebuilt_dedup/union_3way/all_categories_union_merged.json \
  --c1-bulk-dir check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode \
  --include-regex "^(bottle|other)$" \
  --group-label c1_v7_multimode_probe \
  --step6-mode dry_run \
  --bbox-gated \
  --bbox-policy bbox_primary_rmse_observe \
  --v-matrix-mode auto \
  --mode-reports-dir check_reports/test0_rebuilt_dedup
```

## 方案 B：手动分步执行（调试用）

### Step 2a: 构建证书和映射

```bash
# bottle
./scripts/isaac_python.sh scripts/c1_build_bulk_mapping_from_dedup_report.py \
  --report check_reports/test0_rebuilt_dedup/geom_only/bottle/bottle_asset_mesh_dedup_geom_only.json \
  --dataset-root GRScenes-test0-rebuilt-normalized \
  --out-mapping-json check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode/bottle_bulk_batch_v1/filtered_mapping.json \
  --out-stats-json check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode/bottle_bulk_batch_v1/stats.json \
  --bbox-gated \
  --bbox-policy bbox_primary_rmse_observe \
  --out-certificate-jsonl check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode/bottle_bulk_batch_v1/pair_certificates.jsonl \
  --out-certificate-summary-json check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode/bottle_bulk_batch_v1/pair_certificate_summary.json \
  --out-certified-graph-json check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode/bottle_bulk_batch_v1/certified_graph.json
```

### Step 2b: 干跑重写（不实际修改 USD）

```bash
# 对一个场景干跑，检查 reject 记录
./scripts/isaac_python.sh scripts/rewrite_layout_asset_refs_with_compensation.py \
  --layout-usd GRScenes-test0-rebuilt-normalized/GRScenes100/home/SCENE_ID/layout.usd \
  --mapping-json check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode/bottle_bulk_batch_v1/filtered_mapping.json \
  --dry-run \
  --report-out /tmp/bottle_dryrun_report.json \
  --v-matrix-mode auto \
  --mode-reports-dir check_reports/test0_rebuilt_dedup \
  --bbox-gated \
  --bbox-policy bbox_primary_rmse_observe \
  --preview 50
```

## 验证标准

```bash
# 从 ledger 检查无 category_fail 事件
grep '"event": "category_fail"' \
  check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode/_autorun/*/ledger.jsonl
# 期望: 无输出

# 从证书统计检查 mode_not_enabled 拒绝数大幅下降
jq '.reject_reason_counts.mode_not_enabled_topo_filesize // 0' \
  check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode/bottle_bulk_batch_v1/pair_certificate_summary.json
# 期望: 0（之前是非零）

# 检查 eligible 对数 > v5 同类别
jq '.eligible_count' \
  check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode/bottle_bulk_batch_v1/pair_certificate_summary.json
```

# Step 3: Pairwise Compare 审计

## 为什么

确认去重替换后场景中每个物体的世界空间位置不变（bbox、质心、顶点 RMSE 均在阈值内）。

## 命令

```bash
./scripts/isaac_python.sh scripts/placement_pairwise_compare.py \
  --left-root GRScenes-test0-rebuilt-normalized_bak \
  --right-root GRScenes-test0-rebuilt-normalized \
  --label "v7_multimode_probe_bottle_other" \
  --out check_reports/test0_rebuilt_dedup/v7_verification/pairwise_compare.json \
  --verdict-out check_reports/test0_rebuilt_dedup/v7_verification/verdict.json \
  --workers 8 \
  --bbox-policy bbox_primary_rmse_observe \
  --eps-bbox 0.01 \
  --eps-pos 0.01 \
  --eps-angle 1.0 \
  --eps-geom 0.01 \
  --allow-no-mesh
```

## 验证标准

```bash
# 审计判定必须通过
jq '.verdict.passed' \
  check_reports/test0_rebuilt_dedup/v7_verification/verdict.json
# 期望: true

# 无位移超阈值的 prim
jq '.aggregate.displaced_breakdown' \
  check_reports/test0_rebuilt_dedup/v7_verification/pairwise_compare.json
# 期望: 全部为 0

# 无顶点 RMSE 超阈值
jq '.aggregate.vertex_rmse_breakdown' \
  check_reports/test0_rebuilt_dedup/v7_verification/pairwise_compare.json
# 期望: 全部为 0
```

# Step 4: 覆盖率对比（v5 → v7）

## 为什么

量化多模式启用后的去重覆盖率提升。

## 命令

```bash
# v5 ground-only 基线的映射对数
jq 'length' check_reports/test0_rebuilt_dedup/c1_bulk_v5_translation_fix/ground_geom_only_mapping.json

# v7 探针的映射对数
jq 'length' check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode/bottle_bulk_batch_v1/filtered_mapping.json
jq 'length' check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode/other_bulk_batch_v1/filtered_mapping.json

# 证书中各模式的分布
jq -r '.mode' check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode/bottle_bulk_batch_v1/pair_certificates.jsonl \
  | sort | uniq -c | sort -rn

# 各拒绝原因分布
jq '.reject_reason_counts' \
  check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode/bottle_bulk_batch_v1/pair_certificate_summary.json
```

## 期望结果

- eligible 对数 > v5 同类别（因为新增了 topo_filesize 和 shape_invariant 模式的对）
- `mode_not_enabled_*` 拒绝原因应该消失（对三种模式而言）
- 仍可能存在 `aspect_ratio_rejected`、`mesh_probe_failed` 等正常拒绝

# Step 5（可选）: 全量 83 类目滚动发布

探针通过后，对全部类目执行：

```bash
# --skip-done 默认为 True，无需显式传参；不存在 --ledger 标志（ledger 路径由 --c1-bulk-dir 自动派生）
python3 scripts/c1_autorun_categories.py \
  --dataset-root GRScenes-test0-rebuilt-normalized \
  --bak-root GRScenes-test0-rebuilt-normalized_bak \
  --report check_reports/test0_rebuilt_dedup/union_3way/all_categories_union_merged.json \
  --c1-bulk-dir check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode \
  --step6-mode apply \
  --bbox-gated \
  --bbox-policy bbox_primary_rmse_observe \
  --v-matrix-mode auto \
  --mode-reports-dir check_reports/test0_rebuilt_dedup
```

随后全量 pairwise compare 作为最终签核。

# 风险与回退

| 风险 | 应对 |
|------|------|
| topo_filesize/shape_invariant V 矩阵补偿不准 | bbox gate 会拦截；pairwise compare 兜底 |
| 报告重新生成与旧缓存冲突 | 每个 step 输出到独立版本目录（v7_multimode） |
| 全量发布出现个别失败 | `--skip-done`（默认开启）+ ledger 支持断点续跑 |
| 需要回退 | `_bak` 备份完整，可从备份恢复原始 layout |

# 相关文档

- `docs/operations/grscenes_test0_bbox_gated_multimode_plan.md` — 6-change 实现计划（status: implemented）
- `docs/operations/grscenes_test0_bbox_gated_v6_rmse_bbox_investigation.md` — v6 RMSE/bbox 调查
- `docs/operations/v_compensation_translation_scaling_bug.md` — V 补偿缩放 bug 记录
- `docs/changes/2026-04-01_*.md` — 本次代码变更文档
