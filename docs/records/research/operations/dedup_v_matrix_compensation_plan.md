---
title: 去重补偿 V 矩阵方案 — 完整执行 Plan
code_reference:
  - scripts/rewrite_layout_asset_refs_with_compensation.py
  - scripts/compute_vertex_transform.py
  - scripts/c1_autorun_categories.py
created_at: 2026-03-19
updated_at: 2026-03-19
maintainer: claude-team
status: approved
---

# 去重补偿 V 矩阵方案 — 完整执行 Plan

## Context

`GRScenes-test0-rebuilt-normalized/` 已执行过一次 C1 去重（57K pair, 66.8% 去重率），但补偿公式只处理了顶点完全相同的情况，导致 24.9% 的 prim 产生严重位移（最大 248 单位）。需要恢复 pre-dedup 状态，实现按去重模式分发补偿策略的 V 矩阵逻辑，重新执行 C1，产出干净的去重资产集。

**核心思路**：每种去重模式的签名定义了 pair 间确定的几何关系，从签名本身推导正确的补偿方法：
- geom_only → V = Identity（顶点完全相同）
- topo_filesize → V = Procrustes SVD（顶点顺序一致，可精确对齐）
- shape_invariant → V = bbox反归一化 + Procrustes/ICP（归一化空间形状匹配）
- transitive → BFS 找中转路径，沿路径累积 V

## 补偿公式

### 当前公式（仅适用于 geom_only）

```
M_new_local = M_canon_internal⁻¹ × M_old_internal × M_old_local
```

### 扩展公式（适用于所有模式）

```
M_new_local = M_canon_internal⁻¹ × V × M_old_internal × M_old_local
```

其中 V 是**顶点空间变换矩阵**（从 canonical 顶点空间到 old 顶点空间的仿射变换），当 V = I 时退化为当前公式。

### 推导

```
原始:     p_world = p_mesh_old × M_old_internal × M_old_local
替换后:   p_world = p_mesh_canon × M_canon_internal × M_new_local

因为: p_mesh_old ≈ p_mesh_canon × V
代入: p_mesh_canon × V × M_old_internal × M_old_local = p_mesh_canon × M_canon_internal × M_new_local
∴ M_new_local = M_canon_internal⁻¹ × V × M_old_internal × M_old_local
```

## 三种去重模式的签名 → 几何关系 → V 矩阵计算

### geom_only

**签名 hash**：mesh-local 顶点坐标 + faceVertexCounts + faceVertexIndices + 法线 + UV
（`report_asset_mesh_dedup.py:350-409`，不应用任何 prim transform）

**几何关系**：两个资产的 mesh 在 local space 中**顶点完全相同**。差异只在 prim transform chain。

**V 矩阵**：`V = Identity`

### topo_filesize

**签名 hash**：faceVertexCounts + faceVertexIndices + vertex_count + UV values（不含顶点坐标），然后 GLB 文件大小 ±2%
（`report_asset_mesh_dedup.py:416-446`）

**几何关系**：faceVertexIndices 相同 → **顶点顺序一致**。但顶点 xyz 坐标可以不同（烘焙了旋转/缩放/镜像）。

**V 矩阵**：Procrustes SVD（顶点对应关系已知，精确解）

```python
V = procrustes_full(pts_canon, pts_old)  # 含旋转/反射/缩放/平移
```

### shape_invariant

**签名**：预过滤 `vertex_count + aspect_ratio_hash`，精确匹配 `归一化到 unit bbox 后 Hausdorff < 0.05`
（`report_asset_mesh_dedup.py:632-810`）

归一化：`(pts - bbox_min) / max_extent`（消除平移+缩放，保持纵横比）

**几何关系**：归一化后形状几乎相同。原始坐标差异 = 平移 + 均匀缩放 + 可能的旋转。Hausdorff 不依赖顶点顺序。

**V 矩阵**：bbox 反归一化 + 归一化空间 Procrustes（如果顶点顺序不一致则 ICP fallback）

```python
V = compute_V_shape_invariant(pts_canon, pts_old)
```

### transitive（union merge 传递闭包）

**来源**：old 和 canonical 不在任何单一模式中直接匹配，但通过 union-find 传递闭包关联。

**处理**：组内 BFS 找中转路径，沿路径按各步所属模式计算 V 并累积。

```python
V = find_transitive_V(old, canonical, group_members, geom_idx, topo_idx, shape_idx)
```

## Agent Team 结构

```
Team Lead（协调器）
  ├── code-dev         — 实现 V 矩阵模块 + 修改补偿管线
  ├── test-dev         — 编写单元测试
  ├── restore-agent    — 恢复 pre-dedup 状态
  ├── c1-runner        — 执行 C1 去重
  └── verify-agent     — 最终验证
```

## 任务分解 & 执行顺序

### 阶段 1: 并行启动（无依赖）

#### Task 1: 实现 V 矩阵计算模块 [code-dev]

**新建** `scripts/compute_vertex_transform.py`：

```python
# 公共 API
def procrustes_full(pts_canon, pts_old) -> np.ndarray(4x4)
    # 增强版 Procrustes：自动尝试 旋转/反射 × 带缩放/不带 = 4种组合
    # 取 RMSE 最小的，输出 4×4 V 矩阵（行向量约定）

def compute_V_shape_invariant(pts_canon, pts_old) -> np.ndarray(4x4)
    # bbox 归一化 → 归一化空间 Procrustes → 还原到原始空间
    # 如果顶点顺序不一致 fallback 到 ICP

def icp_in_normalized_space(c_norm, o_norm, max_iter=30) -> (R_3x3, rmse)
    # scipy KDTree ICP，在 [0,1] 归一化空间内运行

def build_mode_index(geom_report, topo_report, shape_report) -> (dict, dict, dict)
    # 构建 asset_path → group_id 查找索引

def determine_compensation_mode(old, canonical, geom_idx, topo_idx, shape_idx) -> str
    # 按优先级 geom_only > topo_filesize > shape_invariant > transitive

def find_transitive_V(old, canonical, group_members, ...) -> np.ndarray(4x4)
    # BFS 找中转路径，沿路径累积 V

def compute_V_for_pair(old_usd, canonical_usd, mode) -> Gf.Matrix4d
    # 顶层分发：根据 mode 调用对应函数
```

**可复用的现有代码**：
- `analyze_dedup_pair_types.py:_extract_world_vertices` (L90-128) → 顶点提取
- `analyze_dedup_pair_types.py:_try_procrustes` (L252-275) → SVD
- `usd_xform_utils.py:get_chain_transform` → chain walk
- `report_asset_mesh_dedup.py:_normalize_to_unit_bbox` (L179-201) → bbox 归一化

#### Task 2: 恢复 pre-dedup 状态 [restore-agent]

1. **Live run**（用户已确认可以执行）：
```bash
python scripts/restore_pre_dedup_working_tree.py \
  --dataset-root GRScenes-test0-rebuilt-normalized \
  --bak-root GRScenes-test0-rebuilt-normalized_bak/_dedup_assets_merged/GRScenes_assets
```

2. 恢复 ~57K 资产目录 + 99 个 scene layout

3. **验证**：资产数 >= 85,617，layout.usd 恢复到 pre-C1 快照

### 阶段 2: 依赖 Task 1

#### Task 3: 编写单元测试 [test-dev]

**新建** `tests/test_compute_vertex_transform.py`：

| 测试用例 | 输入 | 期望 |
|---------|------|------|
| 顶点完全相同 | 相同 pts | V = Identity |
| 90° Y-rotation | rotated pts | V 含 R_y(90°) |
| X-axis mirror | mirrored pts | V 含 diag(-1,1,1) |
| 均匀缩放 2x | pts * 2 | V 含 diag(2,2,2) |
| 旋转+缩放+平移 | R*s*pts + t | 完整仿射 V |
| round-trip | 用 V 变换后对齐 | 残差 < 1e-6 |
| shape_invariant bbox | 不同 bbox | 正确缩放矩阵 |
| transitive 2步 | A→B→C | V = V_AB × V_BC |

运行：`python -m pytest tests/test_compute_vertex_transform.py -v`

#### Task 4: 修改补偿管线 [code-dev]

**修改** `scripts/rewrite_layout_asset_refs_with_compensation.py`：

核心改动（L573 附近）：
```python
# 当前：
new_local = canonical_internal.GetInverse() * old_internal * old_local
# 改为：
V = _get_V_cached(old_abs, new_abs)
new_local = canonical_internal.GetInverse() * V * old_internal * old_local
```

具体修改：
1. 添加参数 `--mode-reports-dir`（三份模式报告路径）和 `--v-matrix-mode auto|none`
2. 在 `rewrite_layout()` 中构建模式索引（`build_mode_index`）
3. 添加 V 矩阵缓存 `v_matrix_cache: Dict[Tuple[str, str], Gf.Matrix4d]`
4. `--v-matrix-mode=none` 保持向后兼容

**修改** `scripts/c1_bulk_apply_layout_dedup.py`：
- 传递 `--mode-reports-dir` 和 `--v-matrix-mode` 给 rewriter

**修改** `scripts/c1_autorun_categories.py`：
- 添加 `--mode-reports-dir` 和 `--v-matrix-mode` 参数
- 传递给 `c1_bulk_apply_layout_dedup.py`

### 阶段 3: 依赖 Task 1-4 + Task 2 全部完成

#### Task 5: Smoke 测试（5 个 scene）[c1-runner]

从 99 个 scene 中选 5 个，手动跑补偿：
1. 构建 mapping
2. 用 V 矩阵模式重写 layout
3. `placement_pairwise_compare.py` 对比位移

**Gate**：> 99% 的 prim 位移 < 0.01

#### Task 6: 全量执行 C1 [c1-runner]

```bash
python3 scripts/c1_autorun_categories.py \
  --dataset-root GRScenes-test0-rebuilt-normalized \
  --bak-root GRScenes-test0-rebuilt-normalized_bak \
  --report check_reports/test0_rebuilt_dedup/union_3way/all_categories_union_merged.json \
  --c1-bulk-dir check_reports/test0_rebuilt_dedup/c1_bulk_v2 \
  --group-label c1_v2_vmatrix \
  --out-version v1 \
  --mode-reports-dir check_reports/test0_rebuilt_dedup \
  --v-matrix-mode auto \
  --skip-done --skip-door-variants
```

**每个 category 的 3 步**：
1. `c1_build_bulk_mapping` → 构建 mapping
2. `c1_bulk_apply_layout_dedup` → 重写 layout + V 矩阵补偿
3. `c1_bulk_step6` → promote + scan gate + soft-delete

**Gate per category**：
- post_promote scan: `hit_files == 0`
- post_soft_delete scan: `hit_layouts == 0`

**Gate overall**：autorun ledger 0 failures

### 阶段 4: 最终验证

#### Task 7: 全量验证 [verify-agent]

```bash
./scripts/isaac_python.sh scripts/placement_pairwise_compare.py \
  --left-root GRScenes-test0-rebuilt-normalized \
  --right-root GRScenes-test0-rebuilt-normalized \
  --left-mode current \
  --right-mode earliest_pre_c1 \
  --out check_reports/test0_rebuilt_dedup/v2_verification/pairwise_compare.json
```

**最终 Gate**：
- 位移 > 0.01 的 prim 数量 == 0（或极接近 0）
- 所有 scene status == "ok"

## 执行时间线

```
T+0h   ─── Task 1 (V矩阵代码) ──────────┐
       ─── Task 2 (恢复pre-dedup) ─────┐  │
T+1h                                    │  ├── Task 3 (测试)
                                        │  ├── Task 4 (管线修改)
T+2h                                    │  │
       ◄─── Task 2 完成 ───────────────┘  │
T+2.5h                                    ◄── Tasks 3,4 完成
       ─── Task 5 (smoke测试 5个scene) ──┐
T+3h                                      │
       ◄─── Task 5 通过 ─────────────────┘
       ─── Task 6 (全量C1) ──────────────┐
T+5h                                      │
       ◄─── Task 6 完成 ─────────────────┘
       ─── Task 7 (验证) ────────────────┐
T+5.5h                                    │
       ◄─── Task 7 通过 ─────────────────┘
```

**预计总耗时**：5-6 小时

## 文件变更清单

| 文件 | 操作 | 关键改动 |
|------|------|---------|
| `scripts/compute_vertex_transform.py` | **新建** | V 矩阵计算：procrustes_full, shape_invariant, ICP, 模式分发 |
| `tests/test_compute_vertex_transform.py` | **新建** | 8+ 个测试用例 |
| `scripts/rewrite_layout_asset_refs_with_compensation.py` | **修改** | L573: 加 V 矩阵；加 `--v-matrix-mode`/`--mode-reports-dir` 参数 |
| `scripts/c1_bulk_apply_layout_dedup.py` | **修改** | 传递 V 矩阵参数给 rewriter |
| `scripts/c1_autorun_categories.py` | **修改** | 添加 `--mode-reports-dir`/`--v-matrix-mode` 参数 |

## 关键路径

| 项目 | 路径 |
|------|------|
| Union 报告 | `check_reports/test0_rebuilt_dedup/union_3way/all_categories_union_merged.json` |
| Geom-only 报告 | `check_reports/test0_rebuilt_dedup/geom_only/` |
| Shape-invariant 报告 | `check_reports/test0_rebuilt_dedup/shape_invariant/` |
| Topo-filesize 报告 | `check_reports/test0_rebuilt_dedup/topo_filesize/` |
| C1 bulk dir (v2) | `check_reports/test0_rebuilt_dedup/c1_bulk_v2/` |
| 备份 | `GRScenes-test0-rebuilt-normalized_bak/` |
| 数据集 | `GRScenes-test0-rebuilt-normalized/` |

## 回滚策略

1. **代码问题**：修复代码，无数据影响
2. **恢复失败**：备份不可变，重新跑恢复
3. **C1 部分失败**：`--skip-done` 跳过已完成的 category；或用 pre_c1 快照完全回滚
4. **验证失败**：分析失败 pair，修补补偿逻辑后从 pre_c1 快照重跑
5. **核武器选项**：`GRScenes-test0-rebuilt-normalized_bak/_dedup_assets/`（76 batch）从未被修改，永远可恢复

## 最终产出

```
GRScenes-test0-rebuilt-normalized/
├── GRScenes_assets/   # ~28K 资产（从 85K 去重，去重率 ~67%）
├── GRScenes100/       # 99 个 scene layout（补偿正确，位移 < 0.01）
└── Material/          # 材质不变
```
