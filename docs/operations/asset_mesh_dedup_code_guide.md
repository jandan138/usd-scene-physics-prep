# `report_asset_mesh_dedup.py` 代码导读（中文）

> 最后更新：2026-01-26

这篇文档用于“读懂代码逻辑”，配合脚本：
- `scripts/report_asset_mesh_dedup.py`

如果你更偏好逐行中文注释版，请看：
- `scripts/report_asset_mesh_dedup_zh_annotated.py`

---

## 1. 脚本做什么、不做什么

做什么：
- 扫描 `GRScenes_assets` 下的资产 USD（每个 UID 1 个主 USD）
- 遍历 USD 里的 `UsdGeom.Mesh` prim，提取几何数据与 transform
- 为每个 mesh / asset 计算 hash 签名
- 按签名聚合出“重复组”，并输出 3 份报告（geom_only / scale_only / full_matrix）

不做什么：
- 不移动文件、不修改 USD、不删除资产

---

## 2. 三种口径怎么实现

脚本里每个 mesh 都会生成 3 个签名（`MeshSig`）：

1) `geom_sig_hex`
- 仅由 mesh 几何决定：points + 拓扑 + （可选）normals/UV(st) + 一些 mesh token

2) `scale_sig_hex`
- `hash(geom_sig_hex + world_scale_xyz)`
- `world_scale_xyz` 取自 world matrix 的 3 个 basis 向量长度（等价于“看缩放”，忽略平移旋转）

3) `full_matrix_sig_hex`
- `hash(geom_sig_hex + world_matrix_16)`
- 直接把 world 4×4 矩阵 16 个元素加入 hash

asset 级别签名（`AssetRecord`）则是“把该资产里所有 mesh 的签名排序后再 hash 一次”。

---

## 3. 主要数据结构（读代码先看它们）

- `MeshSig`：一个 mesh prim 的签名与统计
  - `prim_path`：mesh prim 路径
  - `geom_sig_hex / scale_sig_hex / full_matrix_sig_hex`
  - `vertex_count / face_count`

- `AssetRecord`：一个资产 USD 的聚合信息
  - `usd_path`：资产 USD 路径
  - `category / uid`：从路径解析出来的类别与 UID
  - `asset_geom_sig_hex / asset_scale_sig_hex / asset_full_matrix_sig_hex`
  - `meshes[]`：该资产内所有 mesh 的 `MeshSig`

---

## 4. 关键函数导读（按执行顺序）

### 4.1 `_iter_asset_usd_files(assets_root)`：枚举资产 USD

- 目的：快速扫描 `GRScenes_assets/<category>/<uid>/usd/<uid>.usd`
- 设计：优先按 GRScenes 约定路径扫描，扫描不到再 fallback 到全量 walk（兼容不同布局）

### 4.2 `_compute_mesh_sigs(mesh, xform_cache, float_eps=...)`：计算 mesh 签名

几何部分（`geom_sig_hex`）：
- 读取 mesh points、faceVertexCounts、faceVertexIndices
- 可选读取 normals（含 interpolation）
- 可选读取 UV primvar `st`（含 interpolation / elementSize / indices）
- 还会把 `subdivisionScheme`、`doubleSided` 等 token 加入签名

transform 部分：
- `world_m = xform_cache.GetLocalToWorldTransform(prim)`
- `scale_only`：提取 3 个列向量长度作为 scale
- `full_matrix`：直接把 4×4 matrix 展平成 16 个 float

> 注意：如果你的资产 USD 里几何在 payload 里，脚本使用 `Usd.Stage.LoadNone` 可能导致 mesh 不会被加载，这会影响结果。

### 4.3 `_aggregate_asset_sig(mesh_sigs_hex, tag=...)`：聚合 asset 签名

- 把一个 asset 内所有 mesh 的签名排序后加入 hash
- 排序的意义：使结果与 stage Traverse 顺序无关（同一组 mesh 不管先后都得到同签名）

### 4.4 `_make_duplicates_map(records, key)`：生成重复组

- 根据 `AssetRecord` 上的某个签名字段，把 `usd_path` 按 sig 分组
- 只保留组内数量 > 1 的项

### 4.5 `_write_report(...)`：落盘报告

- 写入 `meta / duplicates / assets / errors`
- 其中 `assets` 很大（全量明细），是 300MB 的主要来源

### 4.6 `main()`：把上述步骤串起来

执行流程：
1) 解析参数
2) 初始化进度输出（json / jsonl）
3) discover：枚举 USD 文件列表（会间歇写 discover 进度）
4) scan：逐 USD 打开 stage、遍历 mesh、计算签名、聚合
5) finalize：输出 3 份报告

---

## 5. 如何用报告指导后续动作（思路）

- 先看 `scale_only`：符合“scale 不同视为不同资产”的口径
- 再对 top N 重复组做人工抽样：
  - 是否真的是同几何？
  - 是否只是 `other/` 类别下的 placeholder/重复导出？
- 确认策略后再决定：是否做自动合并/软链接/索引层 dedup（这些属于下一步工作，不在本脚本范围）
