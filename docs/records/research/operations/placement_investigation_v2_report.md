---
title: "Placement Investigation V2: Dedup Compensation Bug (test0 Baseline)"
code_reference: scripts/rewrite_layout_asset_refs_with_compensation.py
created_at: 2026-03-12
updated_at: 2026-03-12
maintainer: claude-agent
status: confirmed
---

# Placement Investigation V2 Report

## Executive Summary

**V1 报告结论（已过时）**: "Normalization 后物品位置完全正确，位移为零。"
— 错误原因：V1 用 test1 作为基准比较 normalized，但 test1 和 normalized 来自同一个导出，没有经过 dedup，所以比较的是 "normalization 是否正确"（答案是 yes），而非 "最终 layout 是否正确"（答案是 no）。

**V2 结论（本报告）**: Normalization 数学正确（误差 < 1e-6），但 **C1 dedup 换引用后补偿代码是 no-op**，导致 18.2% 物品世界坐标位移。

## 调查方法

### 基准选择

| 基准 | 含义 | V1 用了？ | V2 用了？ |
|------|------|-----------|-----------|
| test0 (`GRScenes-test0/`) | 原始拆分导出，未经 normalization/dedup | 否 | **是** |
| test1 (`GRScenes-test1/`) | 重新拆分导出（不同 hash 算法），未经 normalization/dedup | 是 | 否 |
| normalized (`GRScenes-test1-normalized/`) | test1 经过 normalize + C1 dedup 后的结果 | 是（对比目标） | **是（对比目标）** |

V2 使用 test0 作为真实基准（ground truth），因为 test0 保留了原始场景拆分后的布局，可以检测 normalization + dedup 全流程是否引入位移。

### 比较方法

使用 `compare_test0_vs_normalized.py`，对每个 layout prim：
1. 找到其引用的资产下所有 Mesh 子节点
2. 读取顶点并乘以 `LocalToWorldTransform` 转换到世界坐标
3. 计算聚合质心（按顶点数加权）的位移距离
4. 同时计算每个子 mesh 的最大位移

这是**几何真实性**比较，不受坐标系表示差异影响。

## 根因分析

### 问题流程定位

```
test0 (原始拆分)
  → test1 (重新拆分)           ✅ 正确（不同 hash 但几何相同）
    → normalized (变换归一化)   ✅ 正确（数学验证误差 < 1e-6）
      → C1 dedup (去重换引用)  ❌ 补偿代码 no-op → 位移
```

### Bug 详情

**文件**: `scripts/rewrite_layout_asset_refs_with_compensation.py`
**函数**: `_get_asset_internal_matrix()` (L165-188)
**问题**: 读取 `stage.GetDefaultPrim()`（即 `/Root`），其 xform 是 identity。应该读取 `/Root/Instance`，那里存储着 normalization 烤入的实际变换（recentering + Y-up→Z-up rotation + scale）。

**补偿公式**（L539）:
```python
new_world = old_world * old_internal * canonical_internal.GetInverse()
```

当 `old_internal = I` 且 `canonical_internal = I` 时：
```
new_world = old_world * I * I^{-1} = old_world  # no-op
```

### 为什么会导致位移

Normalization 将每个资产的 `/Root/Instance` 变换（orient、scale、translate）烤入顶点坐标。当两个"重复"资产 A 和 B 有不同的 Instance 变换时：

| | 资产 A (old) | 资产 B (canonical) |
|---|---|---|
| Instance orient | Rz(90°) | Rz(180°) |
| Instance scale | (0.075, 0.096, 0.1) | (0.057, 0.057, 0.055) |
| 烤入后顶点 | 按 A 的变换变换过 | 按 B 的变换变换过 |

Dedup 将 A 的引用替换为 B，但 layout prim 的 transform 没有补偿 A→B 的 Instance 变换差异 → 物品位置/大小/朝向错误。

### 为什么部分物品没有位移

在 1,105 个 prim 中，904 个（81.8%）没有位移。原因：
- 它们的 canonical 资产恰好与原始资产有**相同的 Instance 变换**
- 补偿公式虽然是 no-op，但 `M_old_instance == M_canonical_instance` 时不需要补偿

## 影响范围（场景 MV7J6）

### 总体统计

| 指标 | 值 |
|------|-----|
| 总 prim | 1,105 |
| 位移 > 0.01 | 201 (18.2%) |
| 位移 > 0.1 | 160 (14.5%) |
| 位移 > 1.0 | 92 (8.3%) |
| 位移 > 10.0 | 17 (1.5%) |
| 最大质心位移 | 53.9 单位 |
| 最大 per-mesh 位移 | 345.2 单位 |
| 平均位移 | 0.578 |
| 中位数位移 | ~0 (4e-9) |

### 品类位移统计

#### 全部/大部分位移的品类

| 品类 | 总数 | 位移>0.01 | 位移比例 | 最大位移 |
|------|------|-----------|----------|----------|
| curtain | 21 | 19 | 90.5% | 14.68 |
| towel | 9 | 9 | 100% | 5.93 |
| book | 7 | 7 | 100% | 7.71 |
| desk | 3 | 2 | 66.7% | 36.26 |
| other | 149 | 57 | 38.3% | 53.90 |
| cup | 22 | 11 | 50.0% | 0.84 |
| chair | 13 | 7 | 53.8% | 6.78 |
| ground | 254 | 20 | 7.9% | 7.43 |
| wall | 452 | 23 | 5.1% | 1.25 |
| clothes | 7 | 4 | 57.1% | 2.10 |

#### 零位移品类

bed, bowl, cabinet, ceiling, couch, door, hearth, laptop, light, monitor, mouse, pan, person, plate, pot, sofachair, stool, table, toilet, window — 共 20 个品类完全无位移。

### Top 5 最严重位移

| 排名 | Prim 路径 | 质心位移 | per-mesh 最大 | 品类 |
|------|-----------|----------|---------------|------|
| 1 | other/model_a61393c2... | 53.90 | 345.20 | other |
| 2 | desk/model_98628704... | 36.26 | 101.44 | desk |
| 3 | shoecabinet/model_50a76d68... | 25.69 | — | shoecabinet |
| 4 | decoration/model_adc5a6dc... | 21.48 | — | decoration |
| 5 | sideboardcabinet/model_46af4d9f... | 20.01 | — | sideboardcabinet |

所有 top 5 位移 prim 的 `ref_changed=true`（引用被 dedup 换过）。

## V1 报告 vs V2 报告对比

| 维度 | V1 | V2 |
|------|----|----|
| 基准 | test1 | test0 |
| 比较范围 | normalization 正确性 | normalization + dedup 全流程 |
| 结论 | "位移为零，一切正确" | "18.2% 位移，dedup 补偿 bug" |
| V1 是否错？ | 否 — normalization 确实正确 | V1 结论正确但不完整，遗漏了 dedup 的影响 |
| 对 dedup 的评估 | "补偿是 no-op 但 harmless" | **补偿是 no-op 且有害** |

## 修复方案

### 推荐：方案 C — 原地修补（不回滚）

> **WARNING**: 下面的公式 `M_layout_new = M_layout_old * M_old_instance * M_canon_instance^{-1}` 是旧版右乘公式，**已确认错误**。正确公式见 `placement_fix_plan_c_execution.md:33`: `M_new_local = M_canon_inst^{-1} * M_old_inst * M_old_local`（左乘）。

1. 写修补脚本遍历所有 99 个 scene 的 layout.usd
2. 对每个被 dedup 换过引用的 prim：
   - 从 `_dedup_assets/` 备份读 old asset 的 `/Root/Instance` transform (`M_old_instance`)
   - 从 canonical asset 读 `/Root/Instance` transform (`M_canon_instance`)
   - 计算补偿：`M_layout_new = M_layout_old * M_old_instance * M_canon_instance^{-1}`
   - 写入 layout prim
3. 修复 `_get_asset_internal_matrix()` 读 `/Root/Instance` 而非 `/Root`

**优势**: 不需要回滚 dedup，不需要重跑 C1 全量，最快
**依赖**: `_dedup_assets/` 备份完整（位于 `GRScenes-test1-normalized_bak/_dedup_assets/`）

### 验证方法

修复后重跑 `compare_test0_vs_normalized.py`：
- 期望所有 1,105 个 prim 位移 < 0.01（浮点精度范围）
- 重点验证 curtain (19/21)、towel (9/9)、book (7/7) 品类

## 数据位置

| 数据 | 路径 |
|------|------|
| V2 位移数据 | `check_reports/placement_investigation/displacement_test0_vs_normalized.json` |
| V2 比较脚本 | `check_reports/placement_investigation/compare_test0_vs_normalized.py` |
| V1 报告（已过时） | `docs/operations/placement_investigation_report.md` |
| 补偿 bug 代码 | `scripts/rewrite_layout_asset_refs_with_compensation.py:165-188` |
| test0 基准 | `GRScenes-test0/GRScenes100/home/` |
| normalized 数据 | `GRScenes-test1-normalized/GRScenes100/home/` |
| dedup 备份 | `GRScenes-test1-normalized_bak/_dedup_assets/` |
| C1 合并 dedup 报告 | `check_reports/union_merged_3way/all_categories_union_merged.json` |
