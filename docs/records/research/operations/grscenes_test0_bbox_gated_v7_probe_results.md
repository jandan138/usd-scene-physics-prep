---
title: "BBox-Gated Multi-Mode v7 Probe Results"
created_at: "2026-04-01"
updated_at: "2026-04-01"
maintainer: "Claude Code"
status: "probe-passed"
code_reference:
  - "scripts/report_asset_mesh_dedup.py"
  - "scripts/compute_vertex_transform.py"
  - "scripts/rewrite_layout_asset_refs_with_compensation.py"
  - "scripts/c1_build_bulk_mapping_from_dedup_report.py"
---

# 概要

6-change multi-mode 实现（commit `0c51924`）的探针验证通过。在 bottle 类目上验证了所有 6 个 Change 的正确性。

# 数据集说明

| 数据集 | 用途 | bottle 资产数 |
|--------|------|-------------|
| `GRScenes-test0-rebuilt-normalize-prededup` | 去重前快照，用于验证 | 2,253 |
| `GRScenes-test0-rebuilt-normalized` | 当前工作集，已被 v3 全量去重 | 374 |

> 注意：`normalized` 已被 v3 去重（83 类目），重复资产已物理删除。所有验证必须在 `prededup` 上执行。

# Step 1: geom_only 报告去污验证

## 结果

| 类目 | 资产数 | 重复组数 | tolerance_merged | 状态 |
|------|--------|---------|-----------------|------|
| bottle | 2,253 | 74 | **0** | PASS |
| other | 22,357 | 1,891 | **0** | PASS |

旧基线对比（含 tolerance_merge 污染）：
- bottle: 239 → 74 groups（-165），tolerance_merge: 28 → 0
- other: 2,283 → 1,891 groups（-392），tolerance_merge: 264 → 0

**结论**：Change 1 正确移除了 `_tolerance_merge` 对 geom_only 的污染。

## 报告路径

- `check_reports/test0_rebuilt_dedup/v7_geom_only/bottle/bottle_asset_mesh_dedup_geom_only.json`
- `check_reports/test0_rebuilt_dedup/v7_geom_only/other/other_asset_mesh_dedup_geom_only.json`

# Step 2: bbox-gated 管线探针

## Step 2a: 证书构建

| 指标 | 值 |
|------|-----|
| groups_seen | 74 |
| groups_included | 74 |
| candidate_pairs | 190 |
| eligible_pairs | **190** |
| rejected_pairs | **0** |
| mode_not_enabled rejections | **0** |
| bbox_policy | bbox_primary_rmse_observe |

**结论**：Changes 2-3 正确扩展了允许模式，移除了硬编码 gate。190/190 对全部通过证书。

## Step 2b: 干跑重写

场景：`GRScenes-test0-rebuilt-normalize-prededup/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd`

### 第一次（错误的 mode-reports-dir）

`--mode-reports-dir check_reports/test0_rebuilt_dedup/v7_geom_only`（只有 geom_only 报告）

| 指标 | 值 |
|------|-----|
| refs_changed | 0 |
| xform_compensated | 0 |
| reject_records | 1 (`mode_not_enabled_transitive`) |

V 矩阵查找 fallback 到 transitive 模式（因缺少 topo/shape 报告），被正确拒绝。

### 第二次（正确的 mode-reports-dir）

`--mode-reports-dir check_reports/test0_rebuilt_dedup`（包含 geom_only + topo_filesize + shape_invariant）

| 指标 | 值 |
|------|-----|
| refs_changed | **1** |
| xform_compensated | **1** |
| reject_records | **0** |
| dedup_mode used | **topo_filesize** |
| aspect_ratio_rejected (pre-filter) | 3 |

**结论**：
- Changes 4-5 正确工作：topo_filesize 通过白名单，使用 `V * old_local` 公式补偿
- Change 6（全对索引）确保 V 矩阵查找命中 topo_filesize 而非 fallback 到 transitive

## 报告路径

- 证书：`check_reports/test0_rebuilt_dedup/c1_bulk_v7_multimode/bottle_bulk_batch_v1/`
- 干跑 v2：`/tmp/bottle_v7_dryrun_report_v2.json`

# 每个 Change 的验证状态

| Change | 描述 | 验证方式 | 状态 |
|--------|------|---------|------|
| 1 | 移除 tolerance_merge 污染 | Step 1: tolerance_merged=0 | **PASS** |
| 2 | 扩展 BBOX_GATED_ALLOWED_MODES | Step 2a: 0 mode_not_enabled reject | **PASS** |
| 3 | 移除硬编码 cert gate | Step 2a: 190/190 eligible | **PASS** |
| 4 | 扩展 apply 白名单 | Step 2b: topo_filesize 对未被白名单拒绝 | **PASS** |
| 5 | mode-aware 补偿分支 | Step 2b: dedup_mode=topo_filesize, V*old_local | **PASS** |
| 6 | 全对索引 | Step 2b: 直接命中 topo_filesize 不 fallback transitive | **PASS** |

# 待完成

1. ~~Step 1 other 类目报告生成~~ — 完成：1,891 groups, tolerance_merged=0
2. Step 3 pairwise compare：需在 prededup 上实际 apply 后对比位移
3. Step 4 全量覆盖率统计
4. 全量 83 类目滚动发布

# 相关文档

- `docs/operations/grscenes_test0_bbox_gated_multimode_plan.md` — 6-change 计划（status: implemented）
- `docs/operations/grscenes_test0_bbox_gated_multimode_verification_runbook.md` — 验证手册
