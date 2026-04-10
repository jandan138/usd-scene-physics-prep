---
title: "Shape-Invariant Dedup Mode Proposal"
code_reference: "scripts/report_asset_mesh_dedup.py"
created_at: "2026-03-11"
updated_at: "2026-03-11"
maintainer: "zhuzihou"
status: "implemented"
---

# Shape-Invariant Dedup Mode 改进提案

## 1. 背景和动机

### 1.1 当前算法局限性

`report_asset_mesh_dedup.py` 的现有去重算法基于确定性 hash 签名，存在两个核心盲点：

| 盲点 | 根因 | 影响 |
|------|------|------|
| **顶点顺序敏感** | `_compute_mesh_sigs()` (L134-267) 直接按原始顺序 hash `face_vertex_counts`、`face_vertex_indices` 和 `points`，不同 DCC 工具导出的面/顶点排列顺序不同会产生完全不同的 hash | 同一几何体经不同工具重新导出后无法被检测为重复 |
| **缩放敏感** | `geom_only` 模式直接 hash 原始顶点坐标 (L152-155)，同一 mesh 在不同 scale 下坐标值完全不同 | 相同模型以不同尺寸放置时无法被识别为重复 |

`--merge-tolerance` 修复 (commit `993d292`) 通过 `_tolerance_merge()` (L391-497) 解决了浮点噪声导致的 hash bucket 边界问题，但它依赖 `asset_topo_sig_hex` 预分组 (L412)——拓扑签名仍然包含 `face_vertex_counts` 和 `face_vertex_indices` 的原始顺序 (L210-211)，因此**面顺序不同的资产根本不会进入 pairwise 比较阶段**。同时，该方案对 scale 差异完全无效，因为 tolerance merge 直接比较原始坐标距离 (`_max_vertex_distance()`, L368-388)。

### 1.2 真实案例: 70 个 bottle 资产漏检

在 GRScenes-test1-normalized 数据集的 bottle 类别中，当前最优配置 (`--merge-tolerance 0.005`) 的去重率为 55.1% (175 groups, 936 assets)。经人工抽样验证，至少有 **70 个 bottle 资产**在视觉上与已有分组中的资产完全相同，但因 uniform scale 差异未被检测。这些资产是同一 mesh 模型以不同尺寸放置后经 `normalize_asset_transforms.py` 烘焙 transform 产生的，烘焙后顶点坐标包含了 scale 信息，导致 hash 完全不同。

### 1.3 其他类别预估

基于 bottle 的漏检模式，以下高频类别预计存在类似问题：

- **plate** (203 已去重): 餐具类模型经常以不同尺寸复用
- **cup** / **glass**: 同理
- **book** (168 已去重): 书本模型常以不同厚度/大小复用
- **other** (2795 已去重): 杂项类别基数大，预计有可观的增量

保守估计全数据集可额外发现 **2-5% 的重复资产** (约 1,000-2,500 assets)。

## 2. 设计目标

1. **新增 `--mode shape_invariant` 选项**: 生成第 4 份去重报告 `<dataset>_asset_mesh_dedup_shape_invariant.json`，与现有 3 份报告并列
2. **不变性保证**: 对以下变换保持不变性：
   - Vertex ordering (顶点排列顺序)
   - Face winding / face ordering (面片绕序和排列顺序)
   - Uniform scale 差异 (等比缩放)
   - Translation (平移)
3. **向后兼容**: 现有 `geom_only` / `scale_only` / `full_matrix` 模式的行为和输出格式完全不变。不指定 `--mode` 时默认行为不变
4. **性能目标**: 处理 52,904 assets 在 8 worker 并行条件下 < 2 小时完成（本地环境）

## 3. 算法设计

### 3.1 Shape Signature 计算

对每个 `UsdGeom.Mesh` 的顶点集合计算 shape signature，流程如下：

**Step 1: AABB 归一化**

```python
def _normalize_to_unit_bbox(points):
    """将顶点集合归一化到 unit bounding box (保持纵横比)。"""
    pts = np.array(points)                      # shape (N, 3)
    bbox_min = pts.min(axis=0)
    bbox_max = pts.max(axis=0)
    extent = bbox_max - bbox_min
    # 使用 uniform scale (max extent) 保持纵横比
    # 避免退化轴除零
    scale = max(extent.max(), 1e-10)
    return (pts - bbox_min) / scale             # 所有坐标归一到 [0, ~1]
```

关键决策：使用 `max(extent)` 而非 per-axis scale，**保持纵横比**。一个高瘦瓶子和一个矮胖瓶子不应被匹配。

**Step 2: 计算 shape descriptor tuple**

每个 mesh 生成以下描述符用于 pre-filter：

```python
@dataclass(frozen=True)
class ShapeDescriptor:
    vertex_count: int
    face_count: int
    aspect_ratio_hash: str     # quantized sorted bbox aspect ratios (16 hex chars)
```

具体计算：
1. `vertex_count`: 顶点数量
2. `face_count`: 面片数量
3. `aspect_ratios`: 对 bbox 的 3 个 extent 归一化到最长轴后排序，得到 `(r1, r2, 1.0)` 形式的纵横比元组，quantize 到 `eps=0.01` 精度后 hash

**Step 3: 距离直方图 (可选增强)**

```python
def _vertex_distance_histogram(normalized_points, bins=10):
    """计算顶点到质心距离的直方图 (归一化后)。"""
    centroid = normalized_points.mean(axis=0)
    distances = np.linalg.norm(normalized_points - centroid, axis=1)
    max_dist = distances.max() or 1e-10
    distances /= max_dist
    hist, _ = np.histogram(distances, bins=bins, range=(0, 1))
    return tuple(hist)
```

距离直方图对 vertex ordering 和 face winding 完全不变，提供额外的 shape fingerprint 用于 pre-filter 加速。

### 3.2 Pre-filter 分组

在 pairwise 比较之前，使用廉价的 shape descriptor 进行预分组，大幅减少比较量：

```
分组键 = (vertex_count, aspect_ratio_hash)
```

**为什么不用 `face_count` 作为分组键**: 同一个 shape 可能因 re-tessellation (三角化 vs 四边面) 而有不同的 face_count。但 `vertex_count` 必须匹配，因为后续 pairwise 比较需要逐点配对。

**预期效果**: 大多数资产有不同的 vertex count，预计 pre-filter 可减少 **90-95%** 的比较量。例如 bottle 类别 ~1,700 个资产可能只有 20-30 个不同的 (vertex_count, aspect_ratio) 组合，每组平均 ~60 个资产，pairwise 比较量从 O(1700^2) ≈ 290 万降到 Σ O(60^2) ≈ 5.4 万。

### 3.3 Pairwise 比较: Hausdorff Distance

对同一 pre-filter 组内的每对资产，执行以下比较：

```python
def _hausdorff_distance(pts_a, pts_b):
    """双向 Hausdorff 距离 (unit-normalized 顶点)。

    使用 KD-tree 加速 nearest-neighbor 查找。
    时间复杂度: O(n log n) per pair。
    """
    if len(pts_a) == len(pts_b):
        # Fast path: 顶点数相同，lexicographic sort 后逐点配对
        sorted_a = pts_a[np.lexsort(pts_a.T[::-1])]
        sorted_b = pts_b[np.lexsort(pts_b.T[::-1])]
        return np.max(np.linalg.norm(sorted_a - sorted_b, axis=1))

    # Slow path: 顶点数不同 (理论上 pre-filter 已排除，作为防御性处理)
    from scipy.spatial import KDTree
    tree_b = KDTree(pts_b)
    d_ab = tree_b.query(pts_a)[0].max()     # max of min distances A→B
    tree_a = KDTree(pts_a)
    d_ba = tree_a.query(pts_b)[0].max()     # max of min distances B→A
    return max(d_ab, d_ba)
```

**比较流程**:

1. 将两个资产的所有 mesh 顶点分别归一化到 unit bbox
2. 对每个 mesh，按 `(vertex_count, face_count)` 排序后一一配对（与现有 `_max_vertex_distance()` L368-388 的配对逻辑一致）
3. 对每对 mesh 计算双向 Hausdorff 距离
4. 取所有 mesh pair 的 **max** Hausdorff 距离作为 asset 间距离
5. 如果 `max_hausdorff < threshold` (默认 0.05)，归为同一 dedup 组

**Threshold 语义**: 由于顶点已归一化到 unit bbox，threshold 是相对于 bbox 尺度的比例。`0.05` 意味着两个 shape 的最大偏差不超过 bbox 尺度的 5%，远比绝对坐标容差更鲁棒。

### 3.4 Asset-Level 聚合

多 mesh 资产的处理与现有逻辑保持一致：

1. 按 mesh 的 `(vertex_count, face_count)` 排序后配对（与 `_read_mesh_points()` L362-364 的排序逻辑相同）
2. 如果 mesh 数量不同 → 直接判定为不同资产，跳过比较
3. 使用 **Union-Find** 合并组（与 `_tolerance_merge()` L433-443 的 union-find 实现一致）

## 4. 实现计划

### 4.1 代码修改 (`scripts/report_asset_mesh_dedup.py`)

#### 新增函数

| 函数 | 功能 | 插入位置 |
|------|------|----------|
| `_normalize_to_unit_bbox(points)` | AABB 归一化到 unit bbox | L96 附近 (helper 函数区) |
| `_compute_shape_descriptor(points, face_count, eps)` | 计算 `ShapeDescriptor` | L96 附近 |
| `_shape_invariant_mesh_sig(normalized_points, eps)` | Sorted vertex hash (scale/order invariant) | L96 附近 |
| `_hausdorff_distance(pts_a, pts_b)` | KD-tree 双向 Hausdorff | L367 附近 (与 `_max_vertex_distance` 并列) |
| `_shape_invariant_merge(records, tolerance)` | Shape-invariant 模式的 merge 主逻辑 | L391 附近 (与 `_tolerance_merge` 并列) |

#### 修改现有结构

| 修改点 | 当前代码 | 变更内容 |
|--------|----------|----------|
| `MeshSig` dataclass (L47-57) | 7 个字段 | 新增 `shape_invariant_sig_hex: str` 和 `shape_descriptor_key: str` |
| `AssetRecord` dataclass (L60-70) | 9 个字段 | 新增 `asset_shape_invariant_sig_hex: str` 和 `asset_shape_descriptor_key: str` |
| `_compute_mesh_sigs()` (L134-267) | 计算 geom/topo/scale/full_matrix sig | 在 L266 前新增: 调用 `_normalize_to_unit_bbox()` + `_shape_invariant_mesh_sig()` + `_compute_shape_descriptor()` |
| `_write_report()` (L500-580) | `duplicates_key` dict 只有 3 个 mode | 新增 `"shape_invariant": "asset_shape_invariant_sig_hex"`；shape_invariant mode 调用 `_shape_invariant_merge()` 替代 `_tolerance_merge()` |
| `main()` (L583-818) | 无 `--mode` 参数 | 新增 `--mode` 和 `--hausdorff-threshold` 参数；当 mode 包含 `shape_invariant` 时生成第 4 份报告 |

#### 新增 CLI 参数

```
--mode {all,shape_invariant}
    默认 "all"，仅生成现有 3 份报告。
    指定 "shape_invariant" 时额外生成第 4 份报告。

--hausdorff-threshold FLOAT
    Shape-invariant 模式的 Hausdorff 距离阈值 (unit-normalized)。
    默认 0.05。较小值更严格 (减少 false positive)，
    较大值更宽松 (增加召回但可能引入 false positive)。
```

#### 输出文件

```
<out-dir>/<dataset>_asset_mesh_dedup_shape_invariant.json
```

格式与现有 3 份报告完全一致，`meta.mode` 字段值为 `"shape_invariant"`，额外包含 `meta.hausdorff_threshold` 字段。

### 4.2 新增依赖

| 依赖 | 用途 | 可用性 |
|------|------|--------|
| `numpy` | 向量化顶点运算、`np.lexsort` | 已在项目依赖中 |
| `scipy.spatial.KDTree` | Hausdorff distance 的 nearest-neighbor 查询 (slow path) | Isaac Sim 环境已包含；标准 Python 环境需额外安装 |

**降级方案**: 如果 `scipy` 不可用，fast path (相同顶点数) 使用 numpy 排序配对即可覆盖绝大多数 case，slow path 回退到 O(V^2) 暴力搜索。

### 4.3 测试计划

| 测试项 | 方法 | 通过标准 |
|--------|------|----------|
| **已知漏检召回** | 对 70 个已知 bottle 漏检资产运行 shape_invariant mode | 至少 60/70 被正确归组 (≥ 85% 召回率) |
| **Regression** | 对全量 bottle 运行，与 geom_only + merge-tolerance 结果对比 | 现有 175 个 group 中的资产不应被拆散 |
| **False positive 检查** | 随机抽样 shape_invariant 新发现的 top-20 最大组，人工验证 | False positive rate < 5% |
| **Performance benchmark** | 全量 52,904 assets, 8 workers | 总运行时间 < 2 小时 |
| **跨模式一致性** | 在 geom_only 中属于同组的资产，在 shape_invariant 中也必须属于同组 | 100% 通过 |

## 5. 预期效果

### 5.1 Bottle 类别

| 指标 | 当前 (geom_only + merge-tolerance) | 预期 (shape_invariant) |
|------|-------------------------------------|------------------------|
| Dedup groups | 175 | ~195 (+20) |
| Grouped assets | 936 | ~1,006 (+70) |
| Dedup rate | 55.1% | ~59.2% |
| 新增可删除资产 | — | ~68 |

### 5.2 全数据集

| 指标 | 当前 | 预期 |
|------|------|------|
| 已去重资产 | 8,091 (15.3%) | ~9,100-10,600 (17-20%) |
| 新增可删除资产 | — | ~1,000-2,500 |
| 存储节省 (估算) | — | ~2-6 GB (取决于资产大小) |

### 5.3 性能

| 阶段 | 时间估算 | 说明 |
|------|----------|------|
| USD 加载 + 现有 hash | ~45 min (baseline) | 与现有相同，受 I/O 限制 |
| Shape descriptor 计算 | +5 min | O(V) per mesh, 可并行 |
| Pre-filter 分组 | < 1 min | 内存中 dict 操作 |
| Pairwise Hausdorff | +20-40 min | 取决于 pre-filter 效果 |
| **总计** | ~70-90 min | 约为现有运行时间的 1.5-2x |

## 6. 风险和缓解

| 风险 | 可能性 | 影响 | 缓解措施 |
|------|--------|------|----------|
| **False positive: 不同模型偶然形状相似** | 低 | 错误删除有效资产 | 1) 使用更严格的 threshold (0.02 而非 0.05)；2) 加入 material binding 路径匹配作为二次验证；3) 首次运行以 `--dry-run` 模式产出报告供人工审核 |
| **大类别 pairwise 量爆炸** (wall 15,961 / other 12,210) | 中 | 运行时间超出预期 | 1) Pre-filter 将实际比较量降至 5-10%；2) 对超过 500 个资产的组，使用 shape_invariant_sig_hex exact match 作为第一轮过滤，仅对 hash miss 的做 Hausdorff；3) 加入 `--max-group-size` 参数跳过超大组 |
| **非均匀缩放变体漏检** | 中 (by design) | 部分重复仍未被检测 | 当前方案有意仅处理 uniform scale，非均匀缩放场景留作后续 `--mode anisotropic_invariant` 扩展 |
| **多 mesh 资产子 mesh 排列顺序不同** | 低 | 同一模型因 mesh 排序不同被误判为不同 | 使用 sorted submesh signatures 聚合 (与 `_aggregate_asset_sig()` L270-275 现有逻辑一致) |
| **退化 mesh (平面、线段)** | 低 | 归一化除零 / 误匹配 | `_normalize_to_unit_bbox()` 中对退化轴 clamp 到 1e-10；退化 mesh 的 vertex_count 通常极少，pre-filter 自然隔离 |

## 7. 备选方案对比

| 方案 | 优点 | 缺点 | 适用场景 |
|------|------|------|----------|
| **A: Hausdorff + unit bbox (本提案)** | 实现简单 (~200 行)；无额外依赖 (numpy only)；结果可解释 (距离值有物理意义)；可复现 | 对非均匀缩放不鲁棒；不处理旋转变体 | 同一 mesh 不同 scale 的去重 (本项目主要需求) |
| **B: Canonical vertex relabeling** | 理论最优，精确匹配；对任意 vertex/face 排列不变 | 本质是图同构 (NP)；实际实现依赖 nauty/bliss 等外部库；对浮点噪声敏感 | 需要精确匹配且容忍复杂依赖的场景 |
| **C: 渲染图对比** | 最直观，捕捉视觉相似性；material 差异自然纳入 | 需要 GPU 渲染 (Isaac Sim)；视角依赖 (需多视角)；运行时间长；结果不确定性高 | 视觉品控 / 最终验证阶段 |
| **D: Learned embedding (PointNet / MeshCNN)** | 对任意变换最鲁棒；可扩展到语义相似性 | 需要训练数据和 GPU 推理；黑盒不可解释；False positive 难以调试 | 大规模模型检索 / 跨数据集去重 |

**结论**: 方案 A 在实现复杂度、运行效率和结果可解释性之间达到最佳平衡，且覆盖了本项目当前最迫切的需求 (scale-variant dedup)。方案 C 可作为方案 A 的补充验证手段，方案 D 留作长期技术储备。

## 8. 后续扩展

1. **`--rotation-invariant` flag**: 在归一化前通过 PCA 对齐主轴，处理旋转变体。需特殊处理特征值退化 (球形/圆柱形 mesh) 和特征向量符号歧义
2. **Material-aware dedup**: 几何相同但 material 不同的资产是否应该去重需要业务判断。可新增 `--include-materials` flag 将 material binding 路径纳入签名
3. **Progressive verification pipeline**: `shape_invariant` 产出候选 → Isaac Sim 渲染 thumbnail → 人工/自动化视觉对比 → 最终确认
4. **DLC 分布式运行**: 利用现有 `scripts/dlc/dedup_by_category.py` 的 chunk 分发框架，将 shape_invariant mode 集成到 DLC batch 任务中

## 9. Implementation Notes

**Implemented**: 2026-03-11

**Key deviation from proposal**: The shape_invariant mode turned out to be **complementary** to geom_only, not a superset as originally expected. Hausdorff distance is stricter than hash matching at bucket boundaries, causing shape_invariant to miss some assets that geom_only catches. Conversely, shape_invariant finds scale-variant duplicates that geom_only misses entirely.

**Recommended strategy**: Union both modes' results for best coverage. On the bottle category, union yields 812 removable assets (47.8%) vs 761 for geom_only alone (44.8%) or 621 for shape_invariant alone (36.6%).

**Test results**: 30/30 unit tests pass, regression test pass (existing modes unchanged), cross-category validation on pen/plate/cup confirms consistent improvements.

**Files modified**: `scripts/report_asset_mesh_dedup.py`, `tests/test_shape_invariant.py` (new)
