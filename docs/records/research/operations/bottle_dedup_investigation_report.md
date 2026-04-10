---
title: "Bottle 去重失败深度调查报告"
code_reference: "scripts/report_asset_mesh_dedup.py"
created_at: "2026-03-11"
updated_at: "2026-03-11"
maintainer: "zhuzihou"
status: "active"
---

# Bottle 去重失败深度调查报告

## 1. 问题描述

### 1.1 问题来源

用户同事在 GRScenes-test1-normalized 数据集中发现 3 个 bottle 类别的资产渲染预览图看起来**完全一样**（绿标玻璃瓶，白色瓶盖），但去重系统未能将它们归为同一组。

### 1.2 涉及资产

| 资产 Hash | Mesh 路径组件名 | 状态 |
|-----------|-----------------|------|
| `7861bdaa89323558eb8046679f567498` | `component8_21`, `component7_94`, `component6_52` | 未归组 |
| `79088d12b87f6758da805eb64c8a3582` | `component8_72`, `component7_21`, `component6_43` | 未归组 |
| `79090fe893611281c78d1c237c2f5b64` | `component8_90`, `component7_73`, `component6_99` | 未归组 |

三个资产均位于 `GRScenes-test1-normalized/GRScenes_assets/bottle/` 目录下。

### 1.3 严重性

该问题不仅涉及这 3 个资产，经扩展搜索发现**总共 70 个 bottle 资产**是同一模型的变体，且**全部 0/70 被去重系统识别**。这意味着去重系统存在系统性盲区。

---

## 2. 调查方法

本次调查采用 **8 个 agent 并行调查**的方式，从不同角度对问题进行全方位分析：

| Agent | 调查方向 | 主要产出 |
|-------|----------|----------|
| Agent 1 | 几何深度分析 | 顶点坐标对比、bbox extent 计算、缩放比推导 |
| Agent 2 | 来源追溯 | scene splitting 阶段的 `unique_id()` 函数分析 |
| Agent 3 | 算法分析 | `report_asset_mesh_dedup.py` 的 `topo_sig` / `geom_sig` 失败路径分析 |
| Agent 4 | 视觉对比 | GLB 渲染预览对比、文件字节差异分析 |
| Agent 5 | 缩放归一化 | unit bbox 归一化后的 Hausdorff 距离计算 |
| Agent 6 | 源码追溯 | `data_clean.py` 的 hash 生成链路逆向分析 |
| Agent 7 | 相似资产搜索 | 全量 bottle 资产扫描，发现 70 个变体 |
| Agent 8 | 改进方案设计 | `shape_invariant` 模式设计方案 |

---

## 3. 核心发现

### 3.1 确认三个资产是同一模型

#### 3.1.1 渲染外观一致

三个资产的渲染预览图完全一致：绿色标签玻璃瓶身，白色瓶盖。GLB 导出文件之间仅差 **4 字节**（浮点精度差异）。

#### 3.1.2 Mesh 结构完全相同

三个资产具有完全相同的 mesh 拓扑结构（3 个子 mesh）：

| Mesh | 用途 | 顶点数 | 面数 | Connected Components |
|------|------|--------|------|---------------------|
| Mesh 0 | 瓶身 (bottle_body) | 561 | 1,072 | 2 |
| Mesh 1 | 瓶盖底座 (bottle_body) | 24 | 20 | 2 |
| Mesh 2 | 瓶盖 (bottle_cap) | 194 | 352 | 2 |
| **合计** | | **779** | **1,444** | |

#### 3.1.3 材质完全一致

三个资产绑定了**完全相同的 3 个 MDL 材质 ID**：

| Mesh | 材质路径 |
|------|----------|
| 瓶身 | `/Root/Looks/_e97b2d9b1d8ad000102e80b` |
| 瓶盖底座 | `/Root/Looks/_e97b2d9d849e100012e7976` |
| 瓶盖 | `/Root/Looks/_e97b2d95974ed00012a6b00` |

#### 3.1.4 归一化后几何距离极小

将顶点归一化到 unit bounding box 后的比较结果：

| 比较对 | Hausdorff 距离 | 判定 |
|--------|----------------|------|
| B vs C | 0.005 | 几乎完全相同（float noise） |
| A vs B | 0.017 | 高度相似（< 2%） |
| A vs C | 0.017 | 高度相似（< 2%） |

径向分布差异 (Radial Distribution Difference)：

| 比较对 | 径向分布差异 |
|--------|-------------|
| B vs C | 0.0000018 |
| A vs B | 0.0106 |
| A vs C | 0.0106 |

B 和 C 之间的差异低至 $10^{-6}$ 级别，仅为浮点噪声；A 与 B/C 之间差异约 1%，源于非均匀缩放导致的微小形变。

### 3.2 影响范围：70 个同类资产

#### 3.2.1 扩展搜索结果

通过全量 bottle 资产扫描，发现不只是这 3 个，**总共 70 个 bottle 资产**是同一模型的不同缩放变体。

#### 3.2.2 两个尺寸簇

这 70 个资产分为两个尺寸簇：

| 簇 | 资产数 | 瓶身高度 (bbox Y extent) | 代表资产 |
|----|--------|--------------------------|----------|
| 大瓶 | 35 | 215.7 | `7861bdaa...` (资产 A) |
| 小瓶 | 35 | 190.0 | `79088d12...` (资产 B), `79090fe8...` (资产 C) |

两簇之间的缩放比为：

```
Scale ratio (小/大): X=0.820, Y=0.881, Z=0.820
```

注意缩放比在 Y 轴（高度方向）与 X/Z 轴不同，说明这是**非均匀缩放** (non-uniform scaling)。

#### 3.2.3 去重系统识别率

**当前去重系统对这 70 个资产的识别率为 0%** — 全部被视为不同资产。

### 3.3 根因链：两层失败机制

该问题的根因是一个**两层级联失败**：第一层在 scene splitting 阶段产生了不必要的重复资产，第二层在 dedup 阶段未能识别出这些重复。

#### 3.3.1 第一层失败：Scene Splitting (`clean_data.py`)

**位置**: `set_physics/pxr_utils/data_clean.py` 第 467-502 行，`unique_id()` 函数。

**机制**:

```python
# data_clean.py:488-497
if prim.HasAuthoredReferences():
    tmp_references_list = []
    for prim_spec in prim.GetPrimStack():
        tmp_references_list.extend(prim_spec.referenceList.prependedItems)

    for r in tmp_references_list:
        ap = str(r.assetPath).split('/')[-1]  # 取 basename
        pp = str(r.primPath)
        f = ap + "@" + pp
        res.append(f)
```

`unique_id()` 使用 reference 文件路径的 **basename** 作为模型指纹的一部分。同一模型在源场景 USD 中以不同的组件编号（component name）存在时，会产生不同的 basename。

**本案例**：三个瓶子均来自**同一场景** `MV4AFHQKTKJZ2AABAAAAAEA8`，但在场景中以不同的组件名出现：

| 资产 | 组件名 |
|------|--------|
| A | `component8_21`, `component7_94`, `component6_52` |
| B | `component8_72`, `component7_21`, `component6_43` |
| C | `component8_90`, `component7_73`, `component6_99` |

**结果**：不同文件名 → 不同 basename → `unique_id()` 返回不同字符串 → MD5 hash 不同 → 被拆分为 3 个独立资产目录。

实际 hash 计算逻辑（`data_clean.py:612-615`）：

```python
model_list = unique_id(inst, False)  # consider_transform=False
model_name_joint = '_'.join(model_list)
temp_name_hash = hashlib.new("md5", model_name_joint.encode('utf-8'))
model_name = temp_name_hash.hexdigest()
```

#### 3.3.2 第二层失败：Dedup Algorithm (`report_asset_mesh_dedup.py`)

Dedup 算法存在**两个子层面的失败**：

##### 子层面 A：Topology Hash 不匹配

**位置**: `scripts/report_asset_mesh_dedup.py` 第 204-239 行，`topo_sig` 计算。

```python
# report_asset_mesh_dedup.py:209-211
h_topo = _sha256_init("mesh_topo_v1")
_hash_update_ints(h_topo, face_vertex_counts)
_hash_update_ints(h_topo, face_vertex_indices)   # ← 问题所在
```

`topo_sig` 哈希包含 `face_vertex_indices`（面顶点索引数组），且按**原始数组顺序**哈希。DCC 导出器（源场景中的 3D 建模工具）在导出每个实例时，会**独立重排顶点和面的顺序**。

**量化数据**：3 个瓶身 mesh（各 1,072 面 × 3 索引 = 3,216 个面索引值）中，**3,157 个索引值不同**（98.2%）。这导致 `topo_sig` 完全不同。

`topo_sig` 还包含 UV 值（第 224-228 行），进一步放大了差异。

**后果**：不同的 `topo_sig_hex` → 资产不会进入 `--merge-tolerance` 的 pairwise 比较阶段 → 直接被判定为不同资产。

##### 子层面 B：即使强制比较也会失败

即使绕过 `topo_sig` 检查，强制进入 pairwise 顶点比较，去重仍会失败：

1. **顶点顺序不一致**：当前 pairwise 比较假设两个 mesh 的顶点数组中第 i 个顶点对应第 i 个顶点。由于顶点被重排，索引 0 的顶点在 A 中可能是瓶底，在 B 中可能是瓶口。
2. **缩放差异**：A 比 B/C 大约 13.5%（高度方向）和 22%（宽度方向），原始坐标中的顶点距离约 180 个单位，远超 `--merge-tolerance 0.005` 的阈值。

### 3.4 算法局限性分析

通过本次调查，共识别出 **6 个已知算法局限性**：

| # | 问题 | 严重性 | 描述 |
|---|------|--------|------|
| 1 | 顶点顺序敏感 | **HIGH** | `geom_sig` 按数组顺序哈希顶点坐标（第 152-155 行），假设相同索引位置的顶点对应。DCC 导出器重排顶点顺序后，完全相同的几何体产生不同的 hash |
| 2 | 面索引/三角化敏感 | **HIGH** | `topo_sig` 包含 `face_vertex_indices`（第 211 行），不同的面绕行顺序 (winding order) 或三角化方式导致拓扑哈希完全不同 |
| 3 | 缩放不变性缺失 | **MEDIUM** | 所有模式（`geom_only`, `scale_only`, `full_matrix`）都直接哈希原始顶点坐标值，没有任何模式将顶点归一化到 unit bounding box |
| 4 | UV 值包含在 `topo_sig` 中 | **MEDIUM** | `topo_sig` 包含 UV 坐标值（第 227-228 行），UV 的浮点噪声会阻止拓扑匹配，从而阻止 tolerance merge |
| 5 | Tolerance merge 仅限 `geom_only` 模式 | **LOW** | `--merge-tolerance` 只在 `topo_sig_hex` 匹配的前提下工作，无法跨拓扑哈希组做比较 |
| 6 | Pairwise 比较依赖顶点对应关系 | **MEDIUM** | `_max_vertex_distance()` 按索引配对顶点，假设两个 mesh 的顶点排列完全一致，无法处理重排场景 |

### 3.5 几何对比详细数据

#### 3.5.1 Bounding Box 对比

| 属性 | 资产 A (`7861bd...`) | 资产 B (`79088d...`) | 资产 C (`79090f...`) |
|------|---------------------|---------------------|---------------------|
| 瓶身 X extent | 6.298 | 5.164 | 5.163 |
| 瓶身 Y extent | 20.475 | 18.036 | 18.036 |
| 瓶身 Z extent | 6.298 | 5.164 | 5.163 |
| 整体高度 (全3 mesh) | ~215.7 | ~190.0 | ~190.0 |
| 表面积 (瓶身) | 391.42 | 279.15 | 279.15 |
| 体积 (瓶身) | 456.75 | 267.93 | 267.93 |

#### 3.5.2 缩放比分析

```
B/A 缩放比: X=0.820, Y=0.881, Z=0.820
C/A 缩放比: X=0.820, Y=0.881, Z=0.820
B/C 缩放比: X≈1.000, Y=1.000, Z≈1.000
```

B 与 C 之间的缩放比几乎完全为 1.0（差异 < 0.001），确认 B 和 C 是同一尺寸。A 是一个较大的变体，Y 轴（高度）缩放与 X/Z 轴（宽度）缩放不同，说明是**非均匀缩放**。

#### 3.5.3 B vs C 精细对比

瓶盖 mesh (194 顶点) 的逐顶点距离分析：

```
最大顶点距离: 0.35 (归一化前)
平均顶点距离: ~0.001
```

差异完全在浮点精度噪声范围内（< 0.5 个单位），确认 B 和 C 为**同一几何体的不同导出**。

#### 3.5.4 Vertex Hash 对比

虽然几何相同，但由于顶点顺序不同，所有 vertex hash 均不匹配：

| Mesh | 资产 A | 资产 B | 资产 C |
|------|--------|--------|--------|
| 瓶身 | `6810f054...` | `509ad81e...` | `5843f35e...` |
| 瓶盖底座 | `84ede6e5...` | `c5fc754a...` | `78aba507...` |
| 瓶盖 | `ee335746...` | `4da19764...` | `51c1fb06...` |
| Combined | `4c352c3b...` | `68536832...` | `e96d13b4...` |

---

## 4. 失败路径详解

以下是一个资产对（例如 B 和 C）通过去重管线时的完整失败路径：

```
输入: 资产 B (79088d...) 和 资产 C (79090f...)
│
├─ Step 1: 计算 geom_sig_hex
│  ├─ face_vertex_counts: 相同 ✓
│  ├─ face_vertex_indices: 不同 ✗ (3157/3216 索引不同)
│  ├─ 顶点坐标 (按原始顺序): 不同 ✗ (顺序被重排)
│  ├─ normals: 不同 ✗ (顺序被重排)
│  └─ UV values: 不同 ✗ (顺序被重排)
│  → geom_sig_hex 完全不同
│
├─ Step 2: 计算 topo_sig_hex
│  ├─ face_vertex_indices: 不同 ✗
│  ├─ UV values: 不同 ✗
│  └─ topo_sig_hex 完全不同
│  → 不满足 --merge-tolerance 的前置条件
│
├─ Step 3: geom_only 分组
│  └─ geom_sig_hex 不同 → 分入不同组 → 不会被识别为重复
│
├─ Step 4: --merge-tolerance pairwise 比较
│  └─ topo_sig_hex 不同 → 跳过 → 不会进入 pairwise 比较
│
└─ 最终结果: B 和 C 被判定为不同资产 ✗
```

---

## 5. 相关输出文件

### 5.1 调查产出

| 文件 | 说明 |
|------|------|
| `check_reports/bottle_compare/comparison_report.json` | 3 个资产的详细几何对比数据（bbox、面积、体积、材质、顶点统计） |
| `check_reports/bottle_compare/7861bdaa...obj` | 资产 A 的 OBJ 导出（用于第三方工具检查） |
| `check_reports/bottle_compare/79088d12...obj` | 资产 B 的 OBJ 导出 |
| `check_reports/bottle_compare/79090fe8...obj` | 资产 C 的 OBJ 导出 |

### 5.2 分析脚本

| 脚本 | 用途 |
|------|------|
| `compare_bottles.py` | 三瓶对比：几何统计、OBJ 导出、材质提取 |
| `analyze_bottle_scale_dedup.py` | 缩放归一化分析：unit bbox 归一化后的距离计算 |
| `scripts/find_similar_bottles.py` | 全量 bottle 扫描：发现 70 个同类变体 |

### 5.3 设计文档

| 文档 | 说明 |
|------|------|
| `docs/operations/dedup_scale_invariant_proposal.md` | `--mode shape_invariant` 改进方案设计 |
| `docs/operations/dedup_quantize_boundary_analysis.md` | `_quantize()` bucket boundary 问题分析（前期调查） |

---

## 6. 结论

### 6.1 总结

本次调查确认了去重系统的一个**系统性盲区**：当 DCC 导出器对同一几何模型的不同实例独立重排顶点/面顺序，并施加不同缩放变换后，当前算法的所有模式（`geom_only`, `scale_only`, `full_matrix`, 以及 `--merge-tolerance`）均无法识别这些资产为同一模型的变体。

根因是一个两层级联失败：

1. **Scene splitting 层** (`data_clean.py:unique_id()`)：使用 reference 文件的 basename 做 MD5 指纹，同一模型因不同组件编号被拆为独立资产。
2. **Dedup 算法层** (`report_asset_mesh_dedup.py`)：topology hash 包含按原始顺序排列的 `face_vertex_indices`，重排的顶点/面导致拓扑哈希完全不同，阻止了所有后续比较。

### 6.2 影响范围

- **bottle 类别**：已确认 70 个资产受影响（分两个尺寸簇，各 35 个），0% 被识别。
- **其他类别**：类似的 DCC 导出器重排行为可能广泛存在于其他类别中，影响范围有待全量评估。预计 `plate`, `cup`, `book` 等存在大量实例复制的类别会有类似问题。

### 6.3 建议

#### 短期（推荐）

在 `report_asset_mesh_dedup.py` 中新增 `--mode shape_invariant` 模式（详见 `docs/operations/dedup_scale_invariant_proposal.md`），核心改进：

1. **顶点归一化到 unit bounding box**：消除缩放差异。
2. **按字典序排序顶点后哈希**：消除顶点/面顺序依赖。
3. **Shape descriptor 预过滤**：通过 vertex count + quantized aspect ratio 分组，减少 O(n^2) 比较空间。
4. **Hausdorff 距离 pairwise 比较**：对归一化顶点做真正的几何距离比较，而非依赖数组索引对应。

#### 中期

考虑改进 `clean_data.py` 的 `unique_id()` 函数，在 scene splitting 阶段就基于几何内容（而非文件名）生成模型指纹，从根源上减少不必要的重复资产产生。

#### 长期

建立 progressive dedup pipeline：`shape_invariant` 模式筛选候选 → Isaac Sim thumbnail 渲染 → 视觉对比验证，实现全自动的高精度去重。

---

## 7. 关键代码引用

| 位置 | 说明 |
|------|------|
| `set_physics/pxr_utils/data_clean.py:467-502` | `unique_id()` 函数，使用 reference basename 做指纹 |
| `set_physics/pxr_utils/data_clean.py:612-616` | MD5 hash 计算，将 `unique_id()` 结果拼接后取 MD5 |
| `scripts/report_asset_mesh_dedup.py:96-99` | `_quantize()` 函数，bucket boundary 问题根源 |
| `scripts/report_asset_mesh_dedup.py:134-258` | `_compute_mesh_sigs()` 函数，计算 `geom_sig` 和 `topo_sig` |
| `scripts/report_asset_mesh_dedup.py:149-150` | `face_vertex_counts/indices` 直接哈希，导致顺序敏感 |
| `scripts/report_asset_mesh_dedup.py:209-211` | `topo_sig` 包含 `face_vertex_indices`，原始顺序 |
| `scripts/report_asset_mesh_dedup.py:224-228` | `topo_sig` 包含 UV 值，浮点噪声可阻止匹配 |
