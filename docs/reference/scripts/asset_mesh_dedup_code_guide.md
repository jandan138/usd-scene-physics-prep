---
title: '`report_asset_mesh_dedup.py` 代码导读（中文）'
code_reference:
- scripts/report_asset_mesh_dedup.py
- scripts/report_asset_mesh_dedup_zh_annotated.py
created_at: '2026-01-26'
updated_at: '2026-01-26'
maintainer: Codex
status: Active
---

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

这一节的目标是：你不用逐行读完整脚本，也能“脑子里跑一遍程序”，知道每个函数在流水线里扮演什么角色。

阅读小建议：把脚本想象成一条三段式流水线：
- **discover**：先把“要处理哪些 USD 文件”列出来（不打开 Stage）
- **scan**：逐个打开 USD，抽取 Mesh 信息，算出签名并汇总
- **write**：把汇总结果按三种口径写成报告

### 4.1 `_iter_asset_usd_files(assets_root)`：枚举资产 USD（discover 阶段）

你可以把它理解为：**“先把待处理文件清单找出来”**。

**它的输入/输出**
- 输入：`assets_root`（比如 `GRScenes-test1/GRScenes_assets`）
- 输出：一个生成器（`yield`），不断产出 `*.usd` 的文件路径字符串

**它做了哪些事（按逻辑顺序）**
1) 先 `os.scandir(assets_root)` 扫第一层目录，拿到所有 category（只要目录）。
2) 对每个 category，再 `os.scandir(cat.path)` 扫第二层目录，拿到所有 uid（只要目录）。
3) 对每个 uid，拼出 `uid/usd/` 目录：
  - 若没有 `usd` 目录，跳过。
  - 若有 `usd` 目录：优先找 `usd/<uid>.usd`（同名主 USD）。
  - 如果同名主 USD 不存在：就在 `usd/` 目录里随便找一个 `.usd` 当兜底。
4) 如果发现整个数据集都不符合上述布局（`saw_any=False`），才退回 `os.walk()` 全量遍历：只在名为 `usd` 的目录里找 `.usd`。

**为什么这么写（核心动机）**
- 全量 `os.walk()` 在大资产库上非常慢，而且用户会长时间看不到进度；分层 `scandir` 能把 IO 降到最低。
- “优先 `<uid>.usd`”：因为 GRScenes 常见约定是一个 uid 对应一个主 USD；其它 USD 可能是变体/临时产物。

**常见坑/你会看到什么现象**
- 如果你的目录结构不是 `category/uid/usd/`，脚本会走 fallback walk，速度会明显慢。
- 如果 `usd/` 里有多个 `.usd` 而且没有 `<uid>.usd`，脚本会选“目录里遇到的第一个”，这可能不是你期望的主文件（但至少不会漏扫）。

**排查建议**
- 想确认枚举是否正确：先用 `--limit 50` 跑一小段，或者直接在 log/progress 里看 `current_usd`。

### 4.2 `_compute_mesh_sigs(mesh, xform_cache, float_eps=...)`：计算一个 Mesh prim 的“三件套签名”

你可以把它理解为：**“给一个 mesh 做指纹采集”**。

**它的输入/输出**
- 输入：
  - `mesh`：一个 `UsdGeom.Mesh`
  - `xform_cache`：用于快速求 world transform
  - `float_eps`：浮点量化参数（抑制浮点噪声）
- 输出：`MeshSig`（包含：prim_path + 三种签名 + 一些统计/标记）

#### 4.2.1 几何签名 `geom_sig_hex`（忽略 transform）

目标是：只要两个 mesh 的**真实几何**一致（在脚本定义的维度上），就得到相同 `geom_sig_hex`。

具体写入 hash 的内容（按代码顺序）：
1) **拓扑**
  - `faceVertexCounts`：每个面几个顶点
  - `faceVertexIndices`：拓扑索引
  这两者决定“怎么把点连成面”，非常关键。
2) **顶点坐标**
  - `points`：对每个点写入 `(x,y,z)`（可按 `float_eps` 量化）
3) **一些会影响几何解释/渲染语义的 token**
  - `subdivisionScheme`：细分曲面方案不同，会导致表现不同
  - `doubleSided`：双面属性不同，有时会影响碰撞/法线语义（至少渲染语义不同）
4) **可选：Normals**
  - 若存在 normals：写入 interpolation + normals 数组
  - 若不存在：写入占位符 `<no_normals>`（确保“缺失”也是签名的一部分）
5) **可选：UV (st primvar)**
  - 若存在 st：写入 interpolation + elementSize + UV 值数组
  - 若 st 是 indexed primvar：再写入 indices
  - 若不存在：写入占位符 `<no_st>`

为什么要做“占位符”？
- 不做占位符的话，“没有 normals/UV” 与 “有 normals/UV 但恰好是空数组”的边界可能变得不清晰。

#### 4.2.2 transform 相关签名（在 geom_sig 基础上加一层约束）

这一步把“同一几何但摆放/缩放不同”的情况区分开：

1) 先求 mesh prim 的 world matrix：
- `world_m = xform_cache.GetLocalToWorldTransform(prim)`

2) `scale_only`（几何 + 缩放）
- 通过 `_matrix_scale_xyz(world_m)`，取 3×3 部分三列向量的长度得到 `(sx,sy,sz)`
- 然后 `hash(geom_sig_hex + (sx,sy,sz))`

3) `full_matrix`（几何 + 完整 4×4 矩阵）
- 通过 `_matrix_to_row_major16(world_m)` 把矩阵展平成 16 个数
- 然后 `hash(geom_sig_hex + m16)`

**一个很重要的“通俗提醒”**
- `scale_only` 的本质是“只关心大小，不关心摆放位置和朝向”。
- `full_matrix` 是“大小 + 位置 + 朝向都要一样”。

**常见坑/你会遇到的误差来源**
- 浮点误差：同一资产导出两次，矩阵元素可能有微小抖动；这就是 `float_eps` 的意义。
- shear（剪切）/非正交矩阵：`_matrix_scale_xyz` 不是严格分解，只是取列向量长度；一般 TRS 资产足够用。

> 注意（你文档里提到的点很关键）：如果几何藏在 payload/引用里，而打开 Stage 使用 `Usd.Stage.LoadNone`，那 mesh 可能根本没加载出来，结果会偏“少”。遇到这种情况，就需要改为允许加载或做显式加载策略（属于下一步优化）。

### 4.3 `_aggregate_asset_sig(mesh_sigs_hex, tag=...)`：把很多 mesh 指纹揉成一个 asset 指纹

你可以把它理解为：**“把一堆零件的条形码拼成整台机器的条形码”**。

**输入/输出**
- 输入：`mesh_sigs_hex`（比如一个资产里所有 mesh 的 `geom_sig_hex` 列表）
- 输入：`tag`（例如 `asset_geom_only_v1`，用于区分不同聚合口径）
- 输出：一个 hex 字符串（asset 级签名）

**关键设计点：排序**
- `stage.Traverse()` 的遍历顺序可能受 USD 文件内部 authoring 顺序影响；如果不排序，同一资产“只是顺序不同”也会变成不同 asset 签名。
- 排序不会丢失重复：如果同一个 mesh 签名出现两次，排序后仍会出现两次（multiset），因此能区分“有两个相同零件”与“只有一个”。

### 4.4 `_make_duplicates_map(records, key)`：把 asset 按“指纹”分堆（找重复组）

你可以把它理解为：**“把所有资产按签名分桶，同桶就是重复候选”**。

**输入/输出**
- 输入：`records`：所有 `AssetRecord`
- 输入：`key`：用哪个字段当分组键（例如 `asset_scale_sig_hex`）
- 输出：`{sig: [usd_path, ...]}`，只保留 `len(list) > 1` 的桶

**为什么只保留 count>1**
- 单独出现一次的签名没有“重复”意义，保留它只会让报告更大、读起来更困难。

### 4.5 `_write_report(...)`：把内存里的结果写成 JSON（write 阶段）

你可以把它理解为：**“把扫描结果整理成一个大 JSON，方便后续分析/抽样”**。

它写出的 JSON 有四块：
- `meta`：可复现信息 + 统计（多少资产、多少错误、多少重复组、耗时等）
- `duplicates`：最核心的“重复组列表”（每组包含 sig、count、usd_paths）
- `assets`：全量明细（每个资产的签名 + 每个 mesh 的签名）
- `errors`：扫描时遇到的异常

**为什么文件会很大（300MB 级别）**
- 大头在 `assets`：它把每个资产里每个 mesh 的信息都展开写出来。
  这非常适合做追溯/二次分析，但不适合人肉打开阅读，所以后面才配套了 summary 脚本。

### 4.6 `main()`：把 discover → scan → write 串起来（并提供进度可观测性）

`main()` 本质就是 orchestrator：按顺序调用前面的函数，并把结果汇总成三份报告。

它额外做了两件“工程上很重要”的事：

#### 4.6.1 `_emit_progress(...)`：为什么要写 progress.json/jsonl

你可以把它理解为：**“给长跑任务装一个仪表盘”**。

- `progress.jsonl`：每次更新都 append，一行一个 JSON，适合事后统计/画图。
- `progress.json`：每次覆盖写最新快照，适合 `watch cat ...` 实时查看。

它会输出：`phase / processed / total / rate / eta / current_usd / errors`。
其中 `current_usd` 是你在后台跑时最需要的：它能告诉你卡在哪个文件。

#### 4.6.2 执行流程（更细一层）

1) **解析参数**：把 assets_root、输出目录、dataset、float_eps、progress 参数都拿到。
2) **discover**：调用 `_iter_asset_usd_files()` 生成 usd_files 列表；中途会按较低频率写 discover 进度（避免刷屏/刷爆 jsonl）。
3) **scan**：对每个 usd_path：
  - `Usd.Stage.Open(..., LoadNone)` 打开 stage
  - `stage.Traverse()` 找到所有 Mesh prim
  - 对每个 mesh 调 `_compute_mesh_sigs()`
  - 聚合成 asset 级三种签名（`_aggregate_asset_sig`）
  - 收集成 `records[]`
  - 任意异常写进 `errors[]`，不中断全局
4) **write**：分别以三种 `mode` 调 `_write_report()` 写出三份报告。
5) **finalize**：写一条最终进度并打印输出路径。

---

## 5. 如何用报告指导后续动作（思路）

- 先看 `scale_only`：符合“scale 不同视为不同资产”的口径
- 再对 top N 重复组做人工抽样：
  - 是否真的是同几何？
  - 是否只是 `other/` 类别下的 placeholder/重复导出？
- 确认策略后再决定：是否做自动合并/软链接/索引层 dedup（这些属于下一步工作，不在本脚本范围）
