---
title: "去重量化桶边界问题分析与修复"
code_reference: scripts/report_asset_mesh_dedup.py
created_at: 2026-03-08
updated_at: 2026-03-08
maintainer: zhuzihou
status: active
---

# 去重量化桶边界问题分析与修复

## 1. 问题背景

`report_asset_mesh_dedup.py` 使用量化 hash 进行资产去重。三个已知相同的 bottle 资产（f418b527, f3e81129, f525e096）经过 normalize 后 Hausdorff 距离仅 0.001-0.003，但在 `eps=1e-2` 下仍未被匹配为重复。

## 2. 根因分析

### 2.1 量化函数

```python
def _quantize(v, eps):
    return round(v / eps) * eps  # eps=0.01
```

`round(v / 0.01) * 0.01` 将浮点值对齐到 0.01 网格。当两个值差异 < eps 但落在不同桶边界时，会得到不同的量化值。

### 2.2 实测数据

对三个 normalized bottle 的 190 个顶点进行逐坐标对比：

| 对比对 | 不同顶点数 | 最大差异 | 桶边界碰撞 |
|--------|-----------|---------|-----------|
| f418 vs f3e8 | 190/190 (100%) | 0.00253 | 73/190 |
| f418 vs f525 | 186/190 (97.9%) | 0.00104 | 21/190 |
| f3e8 vs f525 | 190/190 (100%) | 0.00253 | 80/190 |

### 2.3 桶边界碰撞示例

```
v34.Z:  0.94546610 → q=0.9500  vs  0.94448954 → q=0.9400  (diff=0.00098)
v121.X: -16.74502563 → q=-16.7500  vs  -16.74499512 → q=-16.7400  (diff=0.00003)
```

v121.X 仅差 0.00003 就被分到不同桶。

### 2.4 为什么 rounding 无法解决

| 精度 | 桶宽 | 噪声/桶宽比 | 预期碰撞率 | 实测 |
|------|------|------------|----------|------|
| round(4) = 0.0001 | 0.0001 | 25× | ~50% | 不匹配 |
| round(3) = 0.001 | 0.001 | 2.5× | ~25% | 不匹配 |
| round(2) = 0.01 | 0.01 | 0.25× | ~6% | 不匹配 |
| round(1) = 0.1 | 0.1 | 0.025× | ~2.5% | 仍有碰撞 |

**结论**：hash 量化方案对连续浮点噪声根本无解。只要噪声 > 0，总有一定概率的坐标落在桶边界上导致 hash 不同。

### 2.5 其他发现

- **拓扑完全一致**：face_vertex_counts, face_vertex_indices, UV 值、UV indices、normals 值完全相同
- **仅顶点坐标有浮点噪声**：来自 normalize 流程中 transform bake/unbake 的浮点精度累积
- **26% 的坐标值接近桶边界**（距边界 < 10% 桶宽），说明碰撞不是偶然

## 3. 修复方案

### 3.1 实施方案：Tolerance Merge（已实现）

在 `report_asset_mesh_dedup.py` 中新增 `--merge-tolerance` 参数：

**原理**：
1. 第一阶段（不变）：hash 量化分组，捕获精确重复
2. 第二阶段（新增）：按**拓扑签名**（topology hash，不含顶点坐标）预分组
3. 同拓扑组内做 pairwise **逐顶点最大距离**比较
4. 最大距离 ≤ tolerance 的资产合并为重复组

**拓扑签名**包含：face_vertex_counts, face_vertex_indices, vertex count, subdivision scheme, doubleSided, normals interp/count, UV values/indices。不包含顶点坐标和法线坐标值。

**用法**：
```bash
python scripts/report_asset_mesh_dedup.py \
    --assets-root GRScenes-test1-normalized/GRScenes_assets/bottle \
    --float-quantize-eps 1e-2 \
    --merge-tolerance 0.005 \
    --out-dir check_reports/bottle
```

### 3.2 修复效果（bottle 类别）

| 指标 | 修复前 (hash-only) | 修复后 (+tolerance 0.005) |
|------|-------------------|--------------------------|
| 去重组数 | 29 (hash) | **175** (29 hash + 146 tolerance) |
| 涉及资产 | 76 | **936** |
| 重复率 | 4.5% | **55.1%** |

三个目标 bottle（f418b527, f3e81129, f525e096）被正确合并到一个 **34 成员**的去重组中。

### 3.3 推荐 tolerance 值

- **0.005**：推荐值。覆盖 normalize 引入的 0.001-0.003 浮点噪声，同时避免误合并
- **0.01**：宽松值。可能增加误合并风险，适合保守估计上限
- **0.001**：严格值。可能遗漏部分噪声较大的重复

## 4. 代码变更

- `scripts/report_asset_mesh_dedup.py`：
  - `MeshSig` / `AssetRecord`：新增 `topo_sig_hex` / `asset_topo_sig_hex` 字段
  - `_compute_mesh_sigs()`：新增拓扑签名计算
  - `_read_mesh_points()` / `_max_vertex_distance()` / `_tolerance_merge()`：新增容差合并逻辑
  - `_write_report()`：接受 `merge_tolerance` 参数，在 geom_only 模式下执行容差合并
  - `main()`：新增 `--merge-tolerance` CLI 参数

## 5. 未采用的替代方案

| 方案 | 原因 |
|------|------|
| 在 normalize 中 round 顶点 | 无法完全消除桶边界碰撞（见 2.4 节） |
| 多桶 LSH | 3D × N 顶点需 2^(3N) 探测，计算量不可接受 |
| 排序 pairwise 距离矩阵 | O(n²) 签名，大量顶点时太慢 |
| 更大 epsilon | 会增加误合并风险 |
