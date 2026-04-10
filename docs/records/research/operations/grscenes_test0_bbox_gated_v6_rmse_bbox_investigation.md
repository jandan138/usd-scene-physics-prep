---
title: "v6 A/B Eval RMSE vs BBox Investigation: Why BBox Passed but RMSE Failed"
created_at: "2026-03-31"
updated_at: "2026-03-31"
maintainer: "Claude Code"
status: "active"
code_reference:
  - "scripts/placement_pairwise_compare.py"
  - "scripts/report_asset_mesh_dedup.py"
  - "check_reports/test0_bbox_gated/20260331_bbox_ab_eval_v6/summary/ab_comparison.json"
---

# 问题

v6 A/B 评估中，Policy B (`bbox_primary_rmse_harder`) 在 bottle 和 other 两个类别发现了 RMSE 硬性失败：

| 类别 | 最大 vertex_rmse (cm) | 最大 displacement (cm) | bbox 结果 |
|------|----------------------|----------------------|----------|
| bottle | 5.83 | 0.00014 | PASS |
| other | 9.02 | 0.000115 | PASS |

核心疑问：**如果 RMSE 这么大（5~9 厘米），为什么 bbox（包围盒）检查通过了？**

> 注：场景 `metersPerUnit=0.01`，所有数值单位为厘米。

# 三个度量的计算方式

`placement_pairwise_compare.py` 对每个场景 prim（场景中放置的一个物体实例）做三个检查。
关键区别在于**每个度量用了哪些 mesh 数据**。

## 1. BBox（包围盒检查）— 用全部 mesh

```python
# placement_pairwise_compare.py:514-519
pts_left_all  = np.vstack([pts for _, pts in meshes_left])   # 所有 mesh 全部堆在一起
pts_right_all = np.vstack([pts for _, pts in meshes_right])
bbox_left  = _bbox_from_points(pts_left_all)   # 算一个包围盒
bbox_right = _bbox_from_points(pts_right_all)
```

做法：把这个物体里**所有子 mesh 的所有顶点**堆在一起，算一个最小/最大包围盒，比较替换前后的差异。

**不关心内部分几块 mesh、每块叫什么名字。只看整体轮廓。**

## 2. Displacement（质心偏移）— 用全部 mesh

```python
# placement_pairwise_compare.py:481-487
centroid_left, verts_left, nmesh_left = _aggregate_centroid(meshes_left)  # 所有 mesh 的所有顶点取平均
displacement = float(np.linalg.norm(centroid_left - centroid_right))
```

同样不关心内部拆分，只看整体重心偏移了多少。

## 3. Vertex RMSE（逐顶点均方根误差）— 只用路径名完全匹配的 mesh

```python
# placement_pairwise_compare.py:489-511
left_mesh_map  = {relative_path: points for relative_path, points in meshes_left}
right_mesh_map = {relative_path: points for relative_path, points in meshes_right}
common_meshes  = set(left_mesh_map.keys()) & set(right_mesh_map.keys())  # 取交集！

for mesh_path in common_meshes:
    pts_l = left_mesh_map[mesh_path]
    pts_r = right_mesh_map[mesh_path]
    rmse = sqrt(mean(sum((pts_l - pts_r)^2, axis=1)))
```

做法：先找出左右两边**路径名完全一样**的 mesh（取交集），然后只在这些共同 mesh 上逐顶点计算 RMSE。

**如果没有任何路径名匹配 → `common_meshes` 为空 → `vertex_rmse = None`（跳过不算）。**

## 对比总结

| 度量 | 用了哪些 mesh | 对内部拆分敏感吗 |
|------|-------------|--------------|
| BBox | **全部** mesh 的全部顶点 | 不敏感 |
| Displacement | **全部** mesh 的全部顶点 | 不敏感 |
| Vertex RMSE | **只用路径名完全匹配**的 mesh | 极其敏感 |

# 具体案例分析

## 案例 1：Bottle（RMSE=5.83 cm）

旧资产 `e84892f9...` 和新 canonical `c0c1c1a1...` 各有 6 个 mesh，但路径完全不同：

```
旧资产 (e84892f9) 的 6 个 mesh:              新 canonical (c0c1c1a1) 的 6 个 mesh:
────────────────────────────────             ────────────────────────────────
Group_default_00/                            Group_bottle_cap_00/
  SM_00_component9_0   (200 verts)             SM_00_component15_0  (200 verts)
  SM_01_component16_0  (105 verts)           Group_bottle_body_00/
  SM_02_component18_0  (392 verts)             SM_01_component12_0  (498 verts)
  SM_03_component5_0   (498 verts)             SM_02_component11_0  (548 verts)
  SM_04_component17_0  (18 verts)              SM_03_component10_0  (392 verts)
  SM_05_component4_0   (548 verts)             SM_04_component14_0  (105 verts)
                                               SM_05_component13_0  (18 verts)
```

**差异**：

1. **分组方式不同**：旧资产把所有零件放在同一个 `Group_default_00` 下；新资产按语义拆分为 `Group_bottle_cap_00`（瓶盖）和 `Group_bottle_body_00`（瓶身）
2. **组件编号不同**：旧的是 component9/16/18/5/17/4，新的是 component15/12/11/10/14/13
3. **顺序不同**：同样 498 顶点的 mesh，旧资产排在第 4 位 (SM_03)，新资产排在第 2 位 (SM_01)

结果：

```
common_meshes = {旧路径} ∩ {新路径} = ∅（空集）

→ vertex_rmse = None  （7349 个 prim 中大部分是这种情况）
```

但两边聚合包围盒完全一致：

```
旧资产 AGGREGATE bbox: min=[-34.317, -91.015, -34.221], max=[34.317, 91.015, 34.221]
新资产 AGGREGATE bbox: min=[-34.317, -91.015, -34.221], max=[34.317, 91.015, 34.221]

→ bbox_delta = 0  → BBox PASS ✓
```

**打个比方**：同一栋楼，旧图纸按楼层分（1F/2F/3F），新图纸按功能分（卧室/厨房/卫生间）。外墙轮廓完全一样（bbox 过），但你没法把"1F的第3面墙"和"卧室的第3面墙"逐点对比——它们根本不是同一面墙。

那 RMSE=5.83 是怎么来的？场景里一定有少量 prim，它们的新旧资产**碰巧有一两个 mesh 路径名相同**，但这些同名 mesh 实际装的是完全不同的几何内容（比如旧版的 SM_00 是瓶盖，新版的 SM_00 碰巧也叫这个名但其实是另一个零件）。

## 案例 2：Other（RMSE=9.02 cm）

旧资产 `52f0befd...` 和新 canonical `00b05d86...` 各只有 1 个 mesh：

```
旧资产 (52f0befd):  Group_default_00/SM_00_obj5_153  (122 verts)
新 canonical (00b05d86):  Group_default_00/SM_00_obj3_80   (122 verts)
```

**差异**：同一个父节点 `Group_default_00`，但 mesh 名不同（`obj5_153` vs `obj3_80`）。

```
common_meshes = {"SM_00_obj5_153"} ∩ {"SM_00_obj3_80"} = ∅

→ vertex_rmse = None  （大部分 prim 是这种情况）
```

包围盒几乎一致：

```
旧: min=[-20.635, -30.942, -24.369], max=[20.635, 30.942, 24.369]
新: min=[-20.634, -30.942, -24.369], max=[20.634, 30.942, 24.369]

→ bbox_delta = 0.001  → BBox PASS ✓
```

**打个比方**：两个大小一样的箱子（bbox 一致），里面的物品编号不同（obj5_153 vs obj3_80），所以没法逐件比对。RMSE=9.02 来自场景中极少数碰巧同名但内容不同的 mesh。

# 为什么 top_20_worst 里全是 rmse=None？

`top_20_worst` 按 **displacement**（质心偏移）降序排列（`placement_pairwise_compare.py:581`），取前 20。

但 RMSE 高的 prim 的 displacement 都很小（< 0.0001 cm），排不进 top 20。而 displacement 最大的 prim（0.00014 cm）反而全部 `rmse=None`——因为它们的新旧 mesh 路径完全不同，根本算不出 RMSE。

这两个维度是"正交"的：displacement 衡量整体位置偏移，RMSE 衡量内部几何一致性。一个物体可以位置完全正确（displacement≈0）但内部内容完全错误（RMSE 很大）。

# 根因：tolerance_merge 混入 geom_only 报告

这些新旧资产并非真正的 geom_only（几何哈希完全一致）对，而是 **tolerance_merge**（容差合并）对——外形相似但不完全一致，被 `report_asset_mesh_dedup.py` 错误地混入了 geom_only 报告。

证据：

| | bottle 对 | other 对 |
|--|----------|---------|
| 旧资产 geom_sig | `44073e50...` | `0db01ec6...` |
| 新资产 geom_sig | `55ec6bcf...` | `1f1b6899...` |
| geom_sig 相同？ | **不同** | **不同** |
| topo_sig | `24bbea81...`（相同） | `f44a76af...`（相同） |
| 报告中的分组标记 | `tolerance_merge_155`（2 成员） | `tolerance_merge_10`（68 成员） |

geom_sig 不同证明它们不是几何一致的。它们被归在一起只是因为拓扑结构相同（顶点数/面数一致）且顶点距离在容差范围内。

**geom_only 报告中 tolerance_merge 组的混入比例**：

| 类别 | 总组数 | 真 geom_only | tolerance_merge | 混入比例 |
|------|--------|-------------|-----------------|---------|
| bottle | 239 | 80 | 159 | **66%** |
| other | 2283 | 1915 | 368 | **16%** |

# 结论

BBox 通过但 RMSE 失败，不是 BBox 检查的缺陷，而是因为：

1. **度量维度不同**：BBox 看整体轮廓，RMSE 看内部子 mesh 逐顶点对齐。两个外形相似但内部拆分方式不同的资产，bbox 天然会通过。
2. **tolerance_merge 混入**：geom_only 报告里混入了并非真正几何一致的 tolerance_merge 组，这些资产对有相同的拓扑结构但不同的几何内容和 mesh 路径名。
3. **RMSE 在路径不匹配时无法计算**：大部分 prim 的 `vertex_rmse=None`（mesh 路径完全不同），少数碰巧路径匹配的 prim 产生了极大的 RMSE 值。

Policy B (`bbox_primary_rmse_harder`) 正确地拦住了这些问题。

# 修复方向

1. **Bug 1**：从 geom_only 报告中排除 tolerance_merge 组（或标记为独立模式），确保 geom_only 只包含 `asset_geom_sig` 完全一致的对
2. **Bug 2**：`build_mode_index()` 应索引组内所有两两组合而非只有 `paths[0]` 和其他，避免 cert 阶段的 canonical 选择导致索引 miss → fallback 到非 Identity V 矩阵

# 相关文档

- `docs/operations/grscenes_test0_bbox_gated_ab_eval_v6_status.md` — v6 A/B 评估总状态
- `docs/operations/grscenes_test0_bbox_gated_dedup_status_20260327.md` — bbox-gated 去重总状态
- `docs/operations/v_compensation_translation_scaling_bug.md` — V 矩阵补偿 bug 历史
