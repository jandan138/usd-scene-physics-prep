---
title: "GRScenes-test1 资产去重评估与行动方案"
code_reference: scripts/report_asset_mesh_dedup.py, scripts/normalize_asset_transforms.py
created_at: 2026-03-08
updated_at: 2026-03-08
maintainer: zhuzihou
status: active
---

# GRScenes-test1 资产去重评估与行动方案

## 1. 背景

同事反馈三个 bottle 资产（`f418b527`, `f3e81129`, `f525e096`）渲染图完全一致，但未被去重。经调研发现根因是**世界坐标烘焙**问题，进而对整个 GRScenes-test1 数据集进行了系统评估。

## 2. 数据集概况

| 指标 | 数值 |
|------|------|
| 总类别数 | 80 |
| 总资产数 | 52,907 |
| 总磁盘占用 | 222 GB |
| 场景数 | 99 |
| 前三大类别 | wall (15,961), other (12,210), ground (10,107) |

## 3. 核心问题：世界坐标烘焙

### 3.1 问题描述

场景拆分（`clean_data.py`）时，部分资产的世界空间坐标被烘焙进了 mesh 顶点数据中。同一个 3D 模型在不同场景位置有不同的顶点坐标，导致 mesh 级去重失败。

### 3.2 影响范围

对 5 个类别的抽样检查显示，世界坐标烘焙是**系统性问题**，非个别现象：

| 类别 | 抽样数 | centroid 偏移 >1 单位 | 平均偏移 | 最大偏移 |
|------|--------|----------------------|---------|---------|
| bottle | 5 | 4/5 (80%) | 2.40 | 4.99 |
| cup | 5 | 3/5 (60%) | 1.16 | 1.98 |
| bowl | 5 | 1/5 (20%) | 2.07 | 8.11 |
| plate | 5 | 0/5 (0%) | 0.13 | 0.23 |
| box | 5 | 3/5 (60%) | 2.16 | 5.75 |

偏移主要沿 Z 轴（原始场景的高度方向），反映了场景中地面/桌面的位置。

### 3.3 触发 bottle 调查的案例

三个 bottle 资产 `f418b527`, `f3e81129`, `f525e096`：

- **视觉**：12 张渲染图（4 角度 × 3 资产）完全无法区分
- **材质**：相同的 2 个材质，相同的纹理文件
- **来源**：全部来自同一场景 `commercial/MWF4WLIKTIFZIAABAAAAACY8_usd`
- **几何**：相同拓扑（190 点、336 面），相同 BBox 尺寸（67.3 × 67.3 × 261.4），但顶点坐标因世界位置不同而不同
- **去重结果（原始数据）**：即使 epsilon 放到 0.1 也未被判为重复（世界坐标偏移导致）
- **normalize 后实测**：Hausdorff 距离仅 **0.001-0.003 单位**（亚毫米级），确认是同一模型。残余差异来自不同世界坐标 bake/unbake 过程中的浮点精度累积。但 `report_asset_mesh_dedup.py` 的量化 hash 算法在 eps=1e-2 下仍未匹配（量化桶边界效应），说明脚本存在对微小差异的灵敏度问题

## 4. Normalize 前后去重效果对比

`normalize_asset_transforms.py` 已在 DLC 上完成全量处理（52,904/52,907 资产），输出在 `GRScenes-test1-normalized/`。

在 normalized 资产上重新运行 `report_asset_mesh_dedup.py`（eps=1e-2），结果如下：

### 4.1 去重发现量对比

| 类别 | 总资产 | 原始 geom_only 组数 | 原始涉及资产 | **Normalized 组数** | **Normalized 涉及资产** | **重复率** |
|------|--------|-------------------|------------|-------------------|----------------------|----------|
| bottle | 1,698 | 20 | 40 | **108** | **248** | **14.6%** |
| cup | 549 | 0 | 0 | **3** | **12** | **2.2%** |
| plate | 426 | 0 | 0 | **36** | **157** | **36.9%** |
| bowl | 82 | 0 | 0 | 0 | 0 | 0% |
| box | 97 | 0 | 0 | 0 | 0 | 0% |

### 4.2 关键发现

1. **Normalize 使去重发现量爆发式增长**：
   - bottle: 20 → 108 组（5.4×），40 → 248 资产（6.2×）
   - cup: 0 → 3 组，12 资产（从无到有）
   - plate: 0 → 36 组，157 资产（37% 是重复的！）

2. **三种去重模式结果完全一致**（geom_only = scale_only = full_matrix），说明 normalize 已完全消除了 transform 维度的差异，所有重复都是纯几何重复。

3. **plate 类别重复率最高**（36.9%），是优先处理目标。

4. **这只是 5 个类别的结果**。剩余 75 个类别（含 wall/other/ground 共 38,278 个资产）尚未检测，预计整体可节省资产数可观。

## 5. 现有工具链

| 工具 | 作用 | 输入 | 输出 |
|------|------|------|------|
| `normalize_asset_transforms.py` | Recenter + Y→Z-up + bake transform | 原始 USD | `GRScenes-test1-normalized/` |
| `report_asset_mesh_dedup.py` | Mesh 级去重分析报告 | 资产目录 | JSON 报告 |
| C1 去重工作流 | 引用替换 + 软删除 | 报告 + mapping | 去重后场景 |
| `specs_normalizer` | 结构化导出 | target/ | export_specs/ |

详见 `docs/operations/asset_dedup_c1_scaling_workflow.md`。

## 6. 行动方案

### Phase 1：全量去重扫描（预计 1-2 天）

**目标**：获取 GRScenes-test1-normalized 全部 80 个类别的去重报告。

```bash
# 方案 A: 本地串行（适合小规模）
for cat in $(ls GRScenes-test1-normalized/GRScenes_assets/); do
  python3 scripts/report_asset_mesh_dedup.py \
    --assets-root GRScenes-test1-normalized/GRScenes_assets/$cat \
    --epsilon 1e-2 \
    --output-dir check_reports/normalized_dedup/$cat
done

# 方案 B: DLC 并行（推荐，适合全量）
python scripts/dlc/submit_batch.py --name dedup-scan --total 10 \
  --mode custom --command_args "scripts/report_asset_mesh_dedup.py ..."
```

**产出**：每个类别的 `{geom_only,scale_only,full_matrix}_duplicates.json`。

### Phase 2：去重执行（按优先级）

按重复率排序，优先处理高收益类别：

| 优先级 | 类别 | 原因 |
|--------|------|------|
| P0 | plate | 37% 重复率，收益最高 |
| P0 | bottle | 15% 重复率，已有案例调查 |
| P1 | cup | 2% 重复率 |
| P1 | wall, other, ground | 资产基数大，即使低重复率也有大量绝对数 |
| P2 | 其他 75 个类别 | 根据 Phase 1 报告排序 |

每个类别按 C1 工作流执行：
1. 确定 canonical 资产
2. 生成 mapping JSON
3. Dry-run 改写 layout 引用
4. 人工抽查 3-8 个场景
5. Promote + 软删除

### Phase 3：Pipeline 改进

1. **`clean_data.py` 导出时增加 recenter 步骤**：在 `create_instance` 函数中，将 mesh 顶点 recenter 到原点后再写入 `instance.usd`，从源头消除世界坐标烘焙问题。

2. **`unique_id` 函数增加几何感知**：当前 fingerprint 仅基于引用链 + transform，不看几何。可以增加一个 optional 的几何 hash 通道，在 reference 相同时进一步用顶点 hash 判断是否真正相同。

3. **Annotation 一致性检查**：同一模型的不同实例 annotation（dimensions, mass）不一致（如三个 bottle 的 mass 分别是 0.2, 0.5, 0.2 kg），需要在去重合并时统一。

### Phase 4：验证与度量

- **去重前后资产数对比**
- **去重前后磁盘占用对比**
- **场景加载性能对比**（如适用）
- **视觉回归测试**：抽样场景渲染对比，确认去重未引入视觉差异

## 6b. 量化桶边界修复（2026-03-08）

### 问题

`report_asset_mesh_dedup.py` 的 `_quantize(v, eps)` 函数使用 `round(v/eps)*eps` 对顶点坐标量化后 hash。当两个值差异小于 eps 但落在不同桶边界时（如 0.945→0.95 vs 0.944→0.94），会产生不同的 hash。实测发现 normalize 后三个 bottle 有 21-80/190 个顶点跨越桶边界。

Hash 量化方案对连续浮点噪声根本无解——即使用 `round(1)`（0.1 精度），仍有约 2.5% 的坐标碰撞率。

### 修复

在 `report_asset_mesh_dedup.py` 中新增 `--merge-tolerance` 参数（推荐值 0.005）：

1. 第一阶段：hash 量化分组（保留原有逻辑）
2. 第二阶段：按拓扑签名（topology hash，不含顶点坐标）预分组
3. 同拓扑组内做 pairwise 逐顶点最大距离比较
4. 最大距离 ≤ tolerance 的资产合并为重复组

### 验证结果（bottle 类别）

| 指标 | 修复前 | 修复后 |
|------|--------|--------|
| 去重组数 | 29 | **175** (+146 tolerance) |
| 涉及资产 | 76 | **936** |
| 重复率 | 4.5% | **55.1%** |

三个目标 bottle（f418b527, f3e81129, f525e096）被正确合并到一个 34 成员的去重组中。

详细分析见 `docs/operations/dedup_quantize_boundary_analysis.md`。

## 7. 风险与注意事项

1. **epsilon 选择**：过大可能误合并不同资产，过小会遗漏重复。建议 1e-2（厘米级）为基准，辅以人工抽查。
2. **目标三个 bottle 的特殊情况**：normalize 后 Hausdorff 距离仅 0.001-0.003 单位（亚毫米级浮点噪声），确认是同一模型。但 `report_asset_mesh_dedup.py` 在 eps=1e-2 下仍未匹配——原因是脚本的签名算法对顶点坐标做 epsilon 量化后 hash，微小浮点差异可能落在不同量化桶边界。**需调查脚本量化逻辑，或在 normalize 流程中增加浮点 round 步骤**（如 round 到 1e-3），确保同一模型的不同实例 normalize 后顶点坐标完全一致。这类"量化边界遗漏"可能导致实际重复率被低估。
3. **Annotation 不一致**：合并资产时需决定以哪个实例的 annotation 为准。
4. **场景引用完整性**：C1 工作流的 promote 步骤会改写 layout.usd 中的引用路径，必须确保所有场景都被正确扫描和更新。

## 8. 附录

### 8.1 调查过程文档

- Bottle 视觉分析报告：`memory/bottle_visual_analysis_report.md`
- Bottle 去重数值报告：`memory/bottle_dedup_investigation.md`
- 去重数值报告文件：`check_reports/bottle_dedup/` 和 `/tmp/dedup_normalized/`

### 8.2 相关文档

- C1 去重工作流：`docs/operations/asset_dedup_c1_scaling_workflow.md`
- Normalize 任务进展：`memory/normalize_asset_task.md`
- DLC 提交指南：`docs/dlc/README.md`
