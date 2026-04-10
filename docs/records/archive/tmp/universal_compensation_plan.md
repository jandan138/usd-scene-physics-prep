---
title: 去重补偿方案 v2 — 按去重模式分发补偿策略
code_reference: scripts/rewrite_layout_asset_refs_with_compensation.py
created_at: 2026-03-19
updated_at: 2026-03-19
maintainer: claude-team
status: final-v2
---

# 去重补偿方案 v2 — 按去重模式分发补偿策略

## 0. 核心思路

每种去重模式的签名定义了 pair 之间**确定的几何关系**，从签名本身就能推导出**正确且唯一的补偿方法**。不需要通用猜测算法，不需要验证后跳过。

```
补偿策略 = f(该 pair 属于哪种去重模式)
```

## 1. 三种去重模式的签名 → 几何关系 → 补偿策略

### 1.1 geom_only

**签名 hash 了什么**（`report_asset_mesh_dedup.py:350-409`）：
```
hash(mesh-local 顶点坐标 + faceVertexCounts + faceVertexIndices
     + 法线 + UV + subdivisionScheme + doubleSided)
```
全部在 mesh-local space 读取（`mesh.GetPointsAttr().Get()`），**不应用任何 prim transform**。

**几何关系**：两个资产的每个 mesh 在各自 local space 中 **顶点完全相同**。所有差异都在 prim transform chain 里。

**补偿**：
```
V = Identity
M_new_local = M_canon_internal⁻¹ × M_old_internal × M_old_local
```
就是当前已有的公式（chain walk 修复后）。

---

### 1.2 topo_filesize

**签名 hash 了什么**（`report_asset_mesh_dedup.py:416-446`）：
```
hash(faceVertexCounts + faceVertexIndices + vertex_count
     + subdivisionScheme + doubleSided + normals_count + UV values)
```
**不包含顶点坐标**。然后 merge 时额外要求 GLB 文件大小 ±2%。

**几何关系**：
- faceVertexIndices 相同 → **顶点顺序一致**（vertex i 在两个 mesh 中对应同一拓扑位置）
- 但顶点 xyz 坐标**可以完全不同**（烘焙了旋转/缩放/镜像到顶点中）
- 文件大小 ±2% → 坐标值不会差太远

**补偿**：顶点顺序一致 → Procrustes correspondence 成立 → 直接 SVD 对齐：
```python
# 提取 Instance 空间顶点
pts_canon = extract_instance_space_vertices(canonical_usd)
pts_old = extract_instance_space_vertices(old_usd)

# Procrustes：找最优 旋转/反射/缩放 使得 pts_old ≈ pts_canon × V
V = procrustes_with_reflection_and_scale(pts_canon, pts_old)

M_new_local = M_canon_internal⁻¹ × V × M_old_internal × M_old_local
```

Procrustes 在顶点对应关系已知时是**精确解**（一次 SVD，0.3ms），不需要 ICP。

---

### 1.3 shape_invariant

**签名 hash 了什么**：
- 预过滤：`vertex_count + face_count + aspect_ratio_hash`（`_compute_shape_descriptor`）
- 精确匹配：归一化到 unit bbox 后做 **Hausdorff distance < 0.05**（`_shape_invariant_merge`）

归一化过程（`_normalize_to_unit_bbox`, L179-201）：
```python
bbox_min = pts.min(axis=0)
scale = max(extent)              # 均匀缩放，保持纵横比
normalized = (pts - bbox_min) / scale
```

然后在归一化空间中 Hausdorff 匹配。

**几何关系**：
- 两个资产在各自 unit-bbox 归一化空间中形状几乎相同（Hausdorff < 0.05）
- 原始坐标之间的差异 = **平移 (bbox_min) + 均匀缩放 (max_extent) + 可能的旋转**
- Hausdorff distance 不依赖顶点顺序 → **顶点对应关系可能不成立**
- 但归一化后 mesh 对齐，说明**顶点在空间中是对应的**（只是 index 可能不同）

**补偿**：需要恢复从 canonical 归一化空间到 old 原始空间的完整变换。

```python
# 分别计算两个资产的 bbox 参数
pts_canon = extract_instance_space_vertices(canonical_usd)
pts_old = extract_instance_space_vertices(old_usd)

canon_min = pts_canon.min(axis=0)
canon_scale = max(pts_canon.max(axis=0) - canon_min)
old_min = pts_old.min(axis=0)
old_scale = max(pts_old.max(axis=0) - old_min)

# 归一化后的点
canon_norm = (pts_canon - canon_min) / canon_scale
old_norm = (pts_old - old_min) / old_scale

# 在归一化空间中 Procrustes 对齐（处理旋转/反射）
R_norm = procrustes_in_normalized_space(canon_norm, old_norm)

# 还原到原始空间：
# p_old = ((p_canon - canon_min) / canon_scale × R_norm) × old_scale + old_min
# 整理为 V 矩阵：p_old = p_canon × V
#   V = (1/canon_scale) × R_norm × old_scale
#   + translation component
s = old_scale / canon_scale
V[:3,:3] = R_norm * s
V[3,:3] = old_min - canon_min @ (R_norm * s)
```

**注意**：归一化空间中的 Procrustes 可能面临顶点顺序不一致的问题（因为 Hausdorff 不依赖顺序）。两种处理方式：
- **方式 A**：如果两个 mesh 的 faceVertexIndices 碰巧也相同 → 顶点顺序一致，直接 Procrustes
- **方式 B**：如果顶点顺序不同 → 用 ICP 在归一化空间对齐（归一化后范围 [0,1]，ICP 收敛快）

---

## 2. Union Merge 传递闭包的处理

### 问题

Union group 选一个 canonical 后，某个 `old → canonical` pair 可能不在任何单一模式中直接匹配（"none" stratum）。

### 解决方案：按优先级查找直接匹配

对每个 `(old_asset, canonical_asset)` pair，在三种模式报告中查找：

```python
def determine_compensation_mode(old_path, canonical_path,
                                 geom_groups, topo_groups, shape_groups):
    """判断 pair 属于哪种模式，决定补偿策略。"""

    # 优先级 1: geom_only（最简单、最可靠）
    if in_same_group(old_path, canonical_path, geom_groups):
        return "geom_only"

    # 优先级 2: topo_filesize（顶点顺序一致，Procrustes 精确）
    if in_same_group(old_path, canonical_path, topo_groups):
        return "topo_filesize"

    # 优先级 3: shape_invariant（需要 bbox 归一化 + 可能 ICP）
    if in_same_group(old_path, canonical_path, shape_groups):
        return "shape_invariant"

    # 优先级 4: 无直接匹配 → 找中转路径
    return "transitive"
```

### "transitive" 的处理

对于只通过传递闭包关联的 pair（old 和 canonical 不在任何单一模式中直接匹配）：

**方法：找中转资产**

```
union group: [canonical=A, removable=B, removable=C, removable=D]

如果 D→A 无直接匹配，但：
  D→B 在 topo_filesize 中匹配
  B→A 在 geom_only 中匹配

那么 D 的补偿可以分两步：
  V_D→B = Procrustes(pts_B, pts_D)     # topo_filesize 补偿
  V_B→A = Identity                       # geom_only 补偿
  V_D→A = V_B→A × V_D→B = V_D→B        # 组合
```

实现：在 union group 内构建**模式连接图**，BFS 找 old→canonical 的最短路径，沿路径累积 V 矩阵。

```python
def find_transitive_V(old_path, canonical_path, group_members,
                       geom_groups, topo_groups, shape_groups):
    """BFS 找中转路径，累积 V 矩阵。"""
    # 建图：group_members 之间的模式连接
    graph = {}
    for a in group_members:
        for b in group_members:
            if a == b:
                continue
            mode = None
            if in_same_group(a, b, geom_groups):
                mode = "geom_only"
            elif in_same_group(a, b, topo_groups):
                mode = "topo_filesize"
            elif in_same_group(a, b, shape_groups):
                mode = "shape_invariant"
            if mode:
                graph.setdefault(a, []).append((b, mode))

    # BFS: old_path → canonical_path
    path = bfs(graph, old_path, canonical_path)
    # path = [(old, mode1, mid1), (mid1, mode2, mid2), ..., (midN, modeN, canonical)]

    # 沿路径累积 V
    V_total = Identity
    for (src, mode, dst) in path:
        V_step = compute_V_for_mode(src, dst, mode)
        V_total = V_total × V_step

    return V_total
```

## 3. 扩展补偿公式

统一公式（所有模式通用）：

```
M_new_local = M_canon_internal⁻¹ × V × M_old_internal × M_old_local
```

| 模式 | V 的计算 | 复杂度 |
|------|---------|--------|
| geom_only | `V = I` | O(1) |
| topo_filesize | `V = Procrustes(pts_canon, pts_old)` | O(N) 一次 SVD |
| shape_invariant | `V = bbox_unnorm × R_norm × bbox_norm` | O(N) SVD + bbox |
| transitive | `V = V_step1 × V_step2 × ...` | 路径长度 × O(N) |

## 4. Procrustes 增强（支持反射+缩放）

当前 `_try_procrustes`（L252-275）只输出分类标签，不输出可用的 V 矩阵。需要增强为输出完整 4×4 V。

```python
def procrustes_full(pts_canon: np.ndarray, pts_old: np.ndarray) -> np.ndarray:
    """计算 V: pts_old ≈ pts_canon × V (行向量约定)

    自动选择最优：旋转 vs 反射，带缩放 vs 不带缩放。
    Returns: 4×4 numpy array (V matrix)
    """
    centroid_c = pts_canon.mean(axis=0)
    centroid_o = pts_old.mean(axis=0)
    a = pts_canon - centroid_c
    b = pts_old - centroid_o

    # 子采样加速
    if len(a) > 5000:
        idx = np.random.RandomState(42).choice(len(a), 5000, replace=False)
        a_sub, b_sub = a[idx], b[idx]
    else:
        a_sub, b_sub = a, b

    # SVD
    H = a_sub.T @ b_sub
    U, S, Vt = np.linalg.svd(H)

    # 4 种候选：(旋转 vs 反射) × (带缩放 vs 不带)
    d = np.linalg.det(Vt.T @ U.T)
    R_rot = Vt.T @ np.diag([1, 1, np.sign(d)]) @ U.T   # 强制 det=+1
    R_ref = Vt.T @ U.T                                    # 自然结果

    norm_a = np.linalg.norm(a_sub, 'fro')
    norm_b = np.linalg.norm(b_sub, 'fro')
    scale = norm_b / max(norm_a, 1e-10)

    best_V = np.eye(4)
    best_rmse = float('inf')

    for R in [R_rot, R_ref]:
        for s in [1.0, scale]:
            aligned = a @ (R * s).T
            rmse = float(np.sqrt(np.mean(np.sum((aligned - b) ** 2, axis=1))))
            if rmse < best_rmse:
                best_rmse = rmse
                V = np.eye(4)
                V[:3, :3] = R * s
                V[3, :3] = centroid_o - centroid_c @ (R * s)
                best_V = V

    return best_V
```

## 5. shape_invariant 专用 V 计算

```python
def compute_V_shape_invariant(pts_canon: np.ndarray, pts_old: np.ndarray) -> np.ndarray:
    """shape_invariant 模式的 V 矩阵计算。

    已知：两个资产归一化到 unit-bbox 后形状匹配 (Hausdorff < 0.05)。
    需要：还原 平移 + 缩放 + 旋转/反射。
    """
    # bbox 参数
    c_min, c_max = pts_canon.min(0), pts_canon.max(0)
    o_min, o_max = pts_old.min(0), pts_old.max(0)
    c_extent = c_max - c_min
    o_extent = o_max - o_min
    c_scale = max(float(c_extent.max()), 1e-10)
    o_scale = max(float(o_extent.max()), 1e-10)

    # 归一化
    c_norm = (pts_canon - c_min) / c_scale
    o_norm = (pts_old - o_min) / o_scale

    # 在归一化空间中对齐
    # 检查顶点数和拓扑是否一致
    if len(c_norm) == len(o_norm):
        # 尝试 Procrustes（假设顶点顺序一致）
        R_norm_V = procrustes_full(c_norm, o_norm)
        R_norm = R_norm_V[:3, :3]  # 归一化空间中的旋转/反射（scale≈1）

        # 验证
        aligned = c_norm @ R_norm.T + R_norm_V[3, :3]
        rmse = float(np.sqrt(np.mean(np.sum((aligned - o_norm) ** 2, axis=1))))

        if rmse > 0.1:
            # 顶点顺序不一致 → ICP
            R_norm, rmse = icp_in_normalized_space(c_norm, o_norm)
    else:
        # 顶点数不同 → ICP
        R_norm, rmse = icp_in_normalized_space(c_norm, o_norm)

    # 还原到原始空间
    # p_old = ((p_canon - c_min) / c_scale) @ R_norm * o_scale + o_min
    # p_old = p_canon @ (R_norm * o_scale / c_scale) + (o_min - c_min @ R_norm * o_scale / c_scale)
    s = o_scale / c_scale
    V = np.eye(4)
    V[:3, :3] = R_norm * s
    V[3, :3] = o_min - c_min @ (R_norm * s)

    return V
```

ICP 只在 shape_invariant 模式中使用，且在归一化空间 [0,1] 内运行（收敛快）：

```python
def icp_in_normalized_space(c_norm, o_norm, max_iter=30, tol=1e-5):
    """归一化空间内的 ICP。范围 [0,~1]，收敛快。"""
    from scipy.spatial import KDTree

    src = c_norm.copy()
    R_accum = np.eye(3)

    for _ in range(max_iter):
        tree = KDTree(o_norm)
        dists, indices = tree.query(src)
        matched = o_norm[indices]

        # Procrustes on matched pairs
        H = (src - src.mean(0)).T @ (matched - matched.mean(0))
        U, S, Vt = np.linalg.svd(H)
        d = np.linalg.det(Vt.T @ U.T)
        R = Vt.T @ np.diag([1, 1, np.sign(d)]) @ U.T

        src = (src - src.mean(0)) @ R.T + matched.mean(0)
        R_accum = R @ R_accum

        rmse = float(np.sqrt(np.mean(dists ** 2)))
        if rmse < tol:
            break

    return R_accum, rmse
```

## 6. 完整执行流程

```
输入:
  - union_3way 去重报告 (57K pairs)
  - geom_only / topo_filesize / shape_invariant 三份单模式报告
  - GRScenes-test0-rebuilt-normalized_bak/ (pre-dedup 备份)

步骤:
1. 从备份恢复 pre-dedup 状态
2. 读取 union 报告 → 构建 (old → canonical) mapping
3. 读取三份单模式报告 → 构建模式查找索引
4. 对每个 (old, canonical) pair:
   a. mode = determine_compensation_mode(old, canonical, geom/topo/shape indices)
   b. if mode == "geom_only":     V = Identity
      elif mode == "topo_filesize":  V = procrustes_full(pts_canon, pts_old)
      elif mode == "shape_invariant": V = compute_V_shape_invariant(pts_canon, pts_old)
      elif mode == "transitive":     V = find_transitive_V(...)
   c. M_new = M_canon_internal⁻¹ × V × M_old_internal × M_old_local
   d. 应用 M_new 到 scene layout prim
5. 统计报告: 各模式使用量、V 矩阵分布
6. 最终验证: placement_pairwise_compare.py 全量对比
```

## 7. 模式查找索引的构建

```python
def build_mode_index(geom_report, topo_report, shape_report):
    """从三份报告构建 asset→group 的查找索引。

    Returns:
        geom_index:  {asset_path: group_id}  — geom_only groups
        topo_index:  {asset_path: group_id}  — topo_filesize groups
        shape_index: {asset_path: group_id}  — shape_invariant groups
    """
    def _build(report):
        index = {}
        for i, group in enumerate(report["duplicates"]):
            for path in group["usd_paths"]:
                index[path] = i
        return index

    return _build(geom_report), _build(topo_report), _build(shape_report)


def in_same_group(path_a, path_b, index):
    """检查两个资产是否在同一个 group 中。"""
    ga = index.get(path_a)
    gb = index.get(path_b)
    return ga is not None and ga == gb
```

## 8. 文件变更清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `scripts/compute_vertex_transform.py` | **新建** | V 矩阵计算：procrustes_full, compute_V_shape_invariant, icp_in_normalized_space |
| `scripts/rewrite_layout_asset_refs_with_compensation.py` | **修改** | L567-573: 加入 mode 判定 + V 矩阵 + 扩展公式 |
| `scripts/usd_xform_utils.py` | **修改** | 添加 `extract_instance_space_vertices()` |
| `scripts/c1_build_bulk_mapping_from_dedup_report.py` | **修改** | 输出 mapping 时附带 mode 信息 |
| `tests/test_compute_vertex_transform.py` | **新建** | 各模式 V 矩阵计算的单元测试 |
| `tests/test_dedup_compensation_chain.py` | **修改** | 添加 topo_filesize (旋转/镜像) 和 shape_invariant 的端到端测试 |

## 9. 预期效果

| pair 来源 | 数量 | 补偿方式 | 预期精度 |
|----------|------|---------|---------|
| geom_only | 12,332 | V=I | 精确（顶点完全相同） |
| topo_filesize | ~17,000 | Procrustes SVD | 精确（顶点顺序一致，SVD 是精确解） |
| shape_invariant | ~10,000 | bbox归一化 + Procrustes/ICP | 高（Hausdorff<0.05 保证形状匹配） |
| transitive | ~15,000 | 路径累积 | 取决于路径中各步精度 |
| **总计** | ~57,000 | | 理论上全部可补偿 |

## 10. 性能预估

| 步骤 | 耗时 |
|------|------|
| 模式索引构建 | ~5s（读 3 份 JSON） |
| geom_only V=I | 忽略（12K × 0） |
| topo_filesize Procrustes | ~5s（17K × 0.3ms） |
| shape_invariant V 计算 | ~30s（10K × 3ms，含 ICP fallback） |
| transitive BFS + 累积 | ~15s（15K × 1ms） |
| USD stage 打开（LRU 缓存） | ~20min（瓶颈，57K unique assets × 50ms，缓存后减少） |
| **总计** | **~20-25 min** |

## 11. 测试计划

### 单元测试 (`tests/test_compute_vertex_transform.py`)

| 测试 | 输入 | 模式 | 期望 V |
|------|------|------|--------|
| 顶点完全相同 | 相同 pts | geom_only | Identity |
| 90° Y-旋转 | pts rotated | topo_filesize | R_y(90°) |
| X-axis 镜像 | pts mirrored | topo_filesize | diag(-1,1,1) |
| 均匀缩放 2x | pts * 2 | topo_filesize | diag(2,2,2) |
| 旋转+缩放+平移 | R*s*pts + t | topo_filesize | 完整仿射 |
| bbox 缩放不同 | 不同 bbox | shape_invariant | 缩放矩阵 |
| 归一化后旋转 | 归一化后旋转 | shape_invariant | 缩放 × 旋转 |
| 传递闭包 2 步 | A→B→C | transitive | V_AB × V_BC |

### 集成测试（真实 pair）

从 pair_type_investigation 中取真实 pair：
- desk (90° 旋转) → topo_filesize 模式 → Procrustes 补偿
- curtain (镜像) → topo_filesize 模式 → 反射 Procrustes 补偿
- wall (大尺寸) → 验证归一化后精度
- bottle (scale ≈ 1) → geom_only 或 topo_filesize

### Smoke 测试

5 个 scene 完整跑补偿 + placement_pairwise_compare.py 验证。

## 12. 风险

| 风险 | 缓解 |
|------|------|
| shape_invariant 顶点顺序不一致时 ICP 收敛问题 | 归一化空间 [0,1] 内 ICP 收敛快；可加 PCA 预对齐 |
| transitive 路径过长（V 累积误差） | 限制最大路径长度（3-4 步）；超过则降级为直接 Procrustes |
| USD stage 打开慢 | LRU 缓存 + 并行加载 |
| 极少数 pair 在三种模式中都不直接匹配且组内无中转 | 降级为直接 Procrustes（如果顶点数一致）或 bbox 对齐 |
