---
title: "Bug Fix: Gf.Matrix4d[i][j] 赋值静默失败导致 V 矩阵补偿无效"
code_reference:
  - scripts/compute_vertex_transform.py
  - scripts/placement_pairwise_compare.py
  - scripts/rewrite_layout_asset_refs_with_compensation.py
created_at: 2026-03-20
updated_at: 2026-03-20
maintainer: claude-team
status: active
---

# Bug Fix: Gf.Matrix4d[i][j] 赋值静默失败导致 V 矩阵补偿无效

## 问题发现

用户在 Isaac Sim 中打开场景 `MV7J6NIKTKJZ2AABAAAAADA8_usd` 后发现一个 desk 物体有明显的旋转偏移。

**异常 prim**: `/Root/Meshes/Animation/desk/model_64a72e14d9b7e061a46e6fc802452525_0/Instance/Group_00/Component_9`

该 prim 对应的资产在 C1 v2_vmatrix 去重中被替换：
- 原始资产: `desk/64a72e14d9b7e061a46e6fc802452525`
- 替换为: `desk/08331d5e4c3ca970076b9c0f0a02d799`（canonical）
- 去重模式: **topo_filesize**（Procrustes SVD 对齐）

两个资产几何拓扑相同（7771 顶点，顶点顺序一致），但 canonical 相比 old 存在 **90° Y 轴旋转**（X 和 Z 维度互换）。

## Root Cause

### Bug 1: `Gf.Matrix4d[i][j]` 赋值静默失败

`scripts/compute_vertex_transform.py:689-703` 的 `numpy_to_gf_matrix4d()` 函数使用 `gf_m[i][j] = float(m[i][j])` 赋值矩阵元素。

**在 pxr Python bindings 中，`Gf.Matrix4d[i]` 返回行的临时拷贝，不是引用。** 对拷贝的写入被丢弃，函数始终返回 Identity 矩阵。

```python
# BUG: 赋值被丢弃
m = Gf.Matrix4d(1.0)
m[0][2] = 99.0
print(m[0][2])  # 输出 0.0，不是 99.0

# FIX: 使用 SetRow
m.SetRow(0, Gf.Vec4d(0, 0, -1, 0))
print(m[0][2])  # 输出 -1.0
```

### 影响

所有非 Identity 的 V 矩阵都被丢失：

| 去重模式 | Pairs 数 | 预期 V | 实际 V | 影响 |
|---------|---------|--------|--------|------|
| geom_only | 12,332 | Identity | Identity | **不受影响** |
| topo_filesize | 17,433 | Procrustes 旋转/缩放 | Identity | **丢失旋转/缩放补偿** |
| shape_invariant | 10,572 | bbox 反归一化 + Procrustes | Identity | **丢失全部补偿** |
| transitive | 15,538 | BFS 路径累积 | Identity | **丢失全部补偿** |

约 **43,543 pairs** 的补偿无效。

### Bug 2: `placement_pairwise_compare.py` 只比较质心

验证脚本 `placement_pairwise_compare.py:421` 使用 `np.linalg.norm(centroid_left - centroid_right)` 作为位移指标。**纯旋转不改变质心位置**，因此旋转错误被完全漏检。

这解释了为什么 99 scenes / 30,965 prims 的验证显示 "0 displaced"，但视觉上存在明显旋转。

## 修复

### Fix 1: `numpy_to_gf_matrix4d` (已完成)

**文件**: `scripts/compute_vertex_transform.py:689`

```python
# Before (BROKEN):
gf_m = Gf.Matrix4d()
for i in range(4):
    for j in range(4):
        gf_m[i][j] = float(m[i][j])

# After (FIXED):
gf_m = Gf.Matrix4d()
for i in range(4):
    gf_m.SetRow(i, Gf.Vec4d(*m[i].tolist()))
```

测试: 12/12 通过 (`python -m pytest tests/test_compute_vertex_transform.py -v`)

### Fix 2: `placement_pairwise_compare.py` 增加旋转检测 (待实施)

需要增强位移指标，方案选项：
- **per-vertex RMSE**: 比较同名 mesh 的所有顶点世界坐标，计算 RMSE
- **bbox corners**: 比较 8 个 bounding box 角点而非质心
- **transform matrix diff**: 直接比较 prim 的 world transform 矩阵

推荐: per-vertex RMSE，与 Procrustes 指标一致。

## 恢复与重跑计划

### 第一步: 恢复 pre-dedup 状态

```bash
python scripts/restore_pre_dedup_working_tree.py \
  --dataset-root GRScenes-test0-rebuilt-normalized \
  --bak-root GRScenes-test0-rebuilt-normalized_bak/_dedup_assets_merged/GRScenes_assets
```

验证: 资产数 >= 85,617，layout 恢复到 pre-C1 快照。

### 第二步: 重跑 C1 autorun

```bash
python3 scripts/c1_autorun_categories.py \
  --dataset-root GRScenes-test0-rebuilt-normalized \
  --bak-root GRScenes-test0-rebuilt-normalized_bak \
  --report check_reports/test0_rebuilt_dedup/union_3way/all_categories_union_merged.json \
  --c1-bulk-dir check_reports/test0_rebuilt_dedup/c1_bulk_v3 \
  --group-label c1_v3_vmatrix_fix \
  --out-version v1 \
  --v-matrix-mode auto \
  --mode-reports-dir check_reports/test0_rebuilt_dedup \
  --skip-done --skip-door-variants
```

注意: 使用 `c1_bulk_v3` 新目录，与 v2 结果区分。

预计耗时: ~23 小时（基于 v2 运行经验）。

### 第三步: 增强验证脚本

修改 `placement_pairwise_compare.py`，增加 per-vertex 或 bbox corner 比较，确保旋转错误可被检出。

### 第四步: 重新验证

```bash
./scripts/isaac_python.sh scripts/placement_pairwise_compare.py \
  --left-root GRScenes-test0-rebuilt-normalized \
  --right-root GRScenes-test0-rebuilt-normalized \
  --left-mode current --right-mode earliest_pre_c1 \
  --label v3_vmatrix_fix_final \
  --out check_reports/test0_rebuilt_dedup/v3_verification/pairwise_compare.json
```

Gate 标准（增强版）:
- 质心位移 > 0.01 的 prim 数量 == 0
- **per-vertex RMSE > 0.01 的 prim 数量 == 0**（新增）

### 第五步: 可视化抽查

对之前发现问题的 desk prim 进行 Isaac Sim 可视化对比，确认旋转问题已修复。

## 验证用的诊断数据

### desk pair 64a72e14 vs 08331d5e

| 指标 | 值 |
|------|-----|
| 顶点数 | 7771（两者相同） |
| Procrustes RMSE | 0.0000012 |
| Old bbox (instance space) | X=[-36,36] Y=[-35,35] Z=[-69,69] |
| Canon bbox (instance space) | X=[-69,69] Y=[-35,35] Z=[-36,36] |
| 旋转 | ~90° Y 轴（X/Z 互换） |
| 质心位移 | 0（两者均居中于原点） |
| 最大顶点位移 | 10.48（修复前） |
| M_internal (两者) | 0.1 * I（均匀缩放） |

## 诊断脚本

调查过程中创建的脚本（均为一次性诊断用途）：
- `scripts/debug_desk_dedup_pair.py` — 顶点/变换完整对比
- `scripts/debug_desk_dedup_pair_v2.py` — pre/post layout 对比
- `scripts/debug_desk_dedup_pair_v3.py` — Gf.Matrix4d 赋值和 V 矩阵验证
- `scripts/debug_gf_matrix_assign.py` — Gf.Matrix4d 赋值 bug 最小复现
