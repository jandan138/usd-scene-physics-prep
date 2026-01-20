# layout.usd → layout.json：流程原理 + 代码解析（非常详细）

> 本文是“原理/实现细节”文档：解释我们如何从 `layout.usd` 抽取物体实例、计算世界变换、并生成 Blender 可消费的 `layout.json`。
>
> 快速上手与 Blender 导入示例请看：
> - [docs/usage/layout_json_for_blender.md](layout_json_for_blender.md)

---

## 1. 背景与目标

### 1.1 为什么需要 layout.json？

- `layout.usd` 是场景“摆放权威”：它记录了场景里每个物体实例的层级关系与位姿（平移/旋转/缩放等）。
- 但 Blender 侧通常更容易批量导入 `glb`（glTF），而不是完整解析 USD 的依赖与材质系统。
- 因此我们把问题拆成两件事：
  1) **物体几何/材质**：由每个物体资产的 `glb/<uid>.glb` 提供；
  2) **物体摆放/位姿**：由场景的 `layout.json` 提供。

### 1.2 本方案的严格边界（重要）

- 本方案只关心 **物体资产实例**（`GRScenes_assets/<category>/<uid>`）。
- 本方案 **不补齐 scene-only 内容**（例如场景自身作者化的墙地面、灯光、相机等）。
- 不做坐标轴转换、不展开 PointInstancer。

这使得它在“scene subset + 物体已有 GLB”的前提下非常稳定、可控。

---

## 2. 输入/输出与目录约定

### 2.1 输入

- `layout.usd`：
  - 典型路径：`<subset_root>/GRScenes100/<scene_category>/<scene_uid>_usd/layout.usd`
- `subset_root`：包含以下两棵树：
  - `<subset_root>/GRScenes_assets/**`
  - `<subset_root>/GRScenes100/**`

### 2.2 输出

- `layout.json`：默认写到 `layout.usd` 同目录：
  - `<subset_root>/GRScenes100/<scene_category>/<scene_uid>_usd/layout.json`

### 2.3 物体 GLB 路径约定（核心）

对任意一个物体资产：

- USD：`GRScenes_assets/<category>/<uid>/usd/<uid>.usd`
- GLB：`GRScenes_assets/<category>/<uid>/glb/<uid>.glb`

`layout.json` 只做“索引”，不会生成 GLB。

---

## 3. 生成流程（原理版）

把 `layout.usd → layout.json` 看作 4 步：

1) **打开 USD stage**：读取 `layout.usd`。
2) **遍历 prim**：找出哪些 prim “代表一个物体实例”。
3) **对每个实例 prim**：
   - 找到它引用的 `GRScenes_assets/<category>/<uid>` 资产（提取 uid/category）
   - 计算它的世界变换矩阵（world matrix）
   - 拼出对应 GLB 相对路径
4) **写 JSON**：按 schema v1 输出实例列表与统计。

---

## 4. 代码入口与整体结构

实现脚本：
- `scripts/generate_layout_json_from_usd.py`

主要入口：

- `main()`：解析命令行参数，决定输出位置，调用生成函数。
- `generate_layout_json(...)`：核心逻辑（打开 stage、遍历 prim、收集实例、写 payload）。

---

## 5. 关键实现细节（代码解析）

下面按执行顺序拆解脚本做了什么。

### 5.1 环境依赖：为什么要用 Isaac Python？

脚本依赖 `pxr`（USD Python bindings）：

- `from pxr import Gf, Sdf, Usd, UsdGeom`

因此推荐用本仓库的 wrapper：

```bash
./scripts/isaac_python.sh scripts/generate_layout_json_from_usd.py ...
```

如果 `pxr` 不可用，脚本会在 `generate_layout_json()` 一开始抛错（明确失败）。

### 5.2 识别资产 UID：核心正则

脚本只接受路径中出现如下模式的引用：

```
GRScenes_assets/<category>/<uid>/...
```

对应实现：

- `_GRSCENES_ASSETS_RE = re.compile(r"(?:^|/)GRScenes_assets/([^/]+)/([^/]+)/")`
- `_parse_asset_ref_from_path(path_str)`：用正则抓出 `(category, uid)`。

为什么要做得这么“死板”？
- 因为我们要输出的是**每个资产对应的 GLB**，而 GLB 的目录结构就是按 `<category>/<uid>` 分发的。
- 路径强约束可以减少误匹配，把“非物体资产引用”（比如材质、纹理、scene 本身）排除掉。

### 5.3 从 prim 上“收集可能的资产路径”（找引用）

对每个 prim，我们需要找到它引用了哪个资产 USD。脚本使用三种信息源（从强到弱）：

1. **References**（最常见、最准确）
2. **Payloads**（场景也可能用 payload 机制）
3. **Asset-valued attributes**（兜底：属性值是 `Sdf.AssetPath` 且指向 USD 文件）

实现：`_iter_asset_paths_from_prim(prim)`。

为什么不只看 references？
- 数据并不总是完全一致：有的场景会把 USD 路径写在属性里；有的会用 payload。
- 做这三层扫描更鲁棒。

### 5.4 相对路径处理：为什么要 resolve？

USD 的引用路径可能是相对 `layout.usd` 文件所在目录的相对路径。脚本会对每个 path 做一次“按 layout.usd 所在目录 resolve”：

- `_resolve_maybe_relative(base_dir, path_str)`

随后把 `raw path` 和 `resolved path` 都拿去尝试解析 UID。

### 5.5 “一个 prim 可能有多个路径”：如何选最像资产 USD 的那条？

现实情况：
- 一个 prim 可能有多层引用、多个 layer 的路径，甚至一些无关的资产属性。

因此脚本实现了一个简单但有效的打分策略：

- `_score_candidate_path(path_str, ref)`：
  - 路径里出现 `/usd/` 加分
  - 以 `/usd/<uid>.usd` 结尾再加分
  - basename 是 `<uid>.usd` 也加分

最终用 `_choose_best_asset_ref(base_dir, raw_paths)` 挑出 `最佳 (AssetRef, resolved_asset_usd)`。

这个策略的核心假设（与规范结构一致）：
- 规范资产 USD 目录就是 `.../<category>/<uid>/usd/<uid>.usd`

### 5.6 只把“真的有位姿意义”的 prim 当实例：UsdGeom.Xformable

遍历到的 prim 很多（materials、scopes、looks 等），并不是每个 prim 都代表“一个物体摆放”。

因此脚本要求 prim 必须是 `UsdGeom.Xformable` 才会被当作实例：

- `xformable = UsdGeom.Xformable(prim)`

### 5.7 计算世界变换：UsdGeom.XformCache（关键）

我们要把 USD 场景层级的变换合成成**世界坐标下的 4×4 矩阵**。

脚本使用：

- `xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())`
- `world = xform_cache.GetLocalToWorldTransform(prim)`

为什么要用 XformCache？
- 它是官方推荐的方式：
  - 能正确处理层级变换累乘
  - 对大量 prim 计算更高效（缓存）

### 5.8 输出矩阵格式：row-major 的 16 长度数组

脚本将 `Gf.Matrix4d` 转换成：

- `transform.matrix_row_major = [m00, m01, ..., m33]`（共 16 个 float）

实现：`_matrix4d_to_row_major_list(m)`，按 `m[row][col]` 顺序展开。

为什么用矩阵而不是 TRS？
- TRS（平移/旋转/缩放）会涉及分解，有时会引入数值误差或丢失剪切信息。
- 4×4 矩阵对 Blender/引擎侧更通用。

### 5.9 GLB 路径构造 + 校验

脚本根据约定直接构造 GLB 路径：

- 绝对路径：
  - `<subset_root>/GRScenes_assets/<category>/<uid>/glb/<uid>.glb`
- JSON 中写入相对 `subset_root` 的相对路径：
  - `GRScenes_assets/<category>/<uid>/glb/<uid>.glb`

并提供两种缺失处理策略：

- `--strict`：只要缺一个就失败（强一致、适合生产）。
- `--skip-missing-glb`：缺的直接跳过（适合调试）。

### 5.10 写出 payload：除了 instances 还会写什么

输出 JSON 还包含：

- `schema_version`：当前为 1
- `generated_at`：UTC 时间戳
- `scene_uid`、`scene_category`：从路径推断（若路径不含 `GRScenes100` 则可能为空）
- `source_layout_usd`：相对路径（若在 subset_root 内）
- `units.meters_per_unit`：从 USD stage metadata 读取
- `up_axis`：从 USD stage metadata 读取
- `counts`：实例统计

这些字段主要用于：
- 追溯（知道 JSON 从哪里来）
- 导入侧做单位/坐标轴转换（未来扩展）

---

## 6. 一个实例记录代表什么？

每条 `instances[]` 记录代表“在 layout.usd 中摆放的一个物体实例”，其核心含义是：

- 读取 `glb` 指向的 GLB 文件
- 把其导入到 Blender
- 用 `transform.matrix_row_major` 设置这批导入对象的世界矩阵

实践上，Blender 侧一般会创建一个 `Empty` 作为根，然后把这次导入的对象 parent 到它下面（见快速上手文档里的示例脚本）。

---

## 7. 常见误解与坑

### 7.1 “layout.json 能不能在 Blender 里直接打开？”

不能。它需要 importer（脚本/插件）去解释。

### 7.2 “生成的场景是否一定和 Isaac Sim 一模一样？”

- **摆放布局**（位置/旋转/缩放）通常一致。
- **材质观感/灯光/scene-only 几何**不保证一致（本方案刻意不做 scene 补齐）。

### 7.3 单位问题（metersPerUnit）

本项目的某些数据 `meters_per_unit` 可能是 `0.01`，这会影响位移/尺寸。

- JSON 会把它写出来
- Blender importer 侧可以按需把平移（以及必要时缩放）乘上该系数

---

## 8. 扩展点（未来如果要做）

- 坐标轴转换：在 importer 侧统一左乘一个轴转换矩阵即可。
- PointInstancer：需要读取点云/朝向/缩放数组并展开实例。
- scene-only 补齐：可加一个 `scene_extras.glb` 方案（把无法映射到资产 UID 的几何单独导出）。

---

## 9. 对应命令与建议工作流

推荐工作流：

1) 先确保物体 GLB 完整（见 `usd_to_glb_in_subset.md`）
2) 再从 layout.usd 生成 layout.json
3) Blender importer 读 JSON → 导入 GLB → 应用矩阵

命令示例：

```bash
./scripts/isaac_python.sh scripts/generate_layout_json_from_usd.py \
  --layout-usd <subset_root>/GRScenes100/<scene_category>/<scene_uid>_usd/layout.usd \
  --subset-root <subset_root> \
  --strict
```
