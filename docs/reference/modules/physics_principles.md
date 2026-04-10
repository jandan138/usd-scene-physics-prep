---
title: 物理原理与实现解析（PhysX / Isaac Sim / USD）
code_reference:
- set_physics/preprocess_for_interaction.py
- set_physics/preprocess_for_navigation.py
- set_physics/pxr_utils/usd_physics.py
created_at: '2025-12-28'
updated_at: '2025-12-28'
maintainer: Codex
status: Active
---

# 物理原理与实现解析（PhysX / Isaac Sim / USD）

本篇文档面向“把一个 USD 场景变成 Isaac Sim 可直接仿真的 sim-ready 场景”这个目标，解释：
- USD/PhysX 物理语义在文件层是如何表达的；
- 项目里选择了哪些碰撞近似方式，为什么这么选；
- interaction（交互）与 navigation（导航）两条预处理链路在“物理策略”上的差异；
- 常见坑点与可调参数入口。

> 说明：本文以仓库当前实现为准，关键脚本是：
> - `set_physics/preprocess_for_interaction.py`
> - `set_physics/preprocess_for_navigation.py`
> - 以及一键 CLI：`set_physics/simready.py`

---

## 1. “Sim-ready” 在这个项目里具体意味着什么

在 Isaac Sim / PhysX 里，一个“能仿真”的场景通常需要：
- **可碰撞（Collision）**：物体要有 collider（碰撞体），并且启用。
- **可动力学（RigidBody / Articulation）**：需要动态行为的物体要挂刚体（RigidBodyAPI），关节结构要启用（JointEnabled）。
- **合理的几何近似（Approximation）**：碰撞形状不能直接拿渲染网格硬算（太慢或不稳定），通常要用 SDF/凸包/凸分解等近似。
- **干净的变换（Transform）**：非规范的 xformOp 或非均匀缩放会导致碰撞生成、惯性估计、关节运动不稳定。
- （导航任务）**语义标签（Semantics）**：用于分割/语义感知或下游策略。

因此项目把“sim-ready”分成两类输出：
- **interaction（动态交互）**：给 pickable 物体上刚体 + 合适近似；给静态物体上静态碰撞；对有 joints 的模型启用 articulation。
- **navigation（静态导航）**：几乎所有物体只要静态碰撞即可（通常 triangle mesh），并写入语义；door 可能被禁用或做特殊处理。

---

## 2. USD 物理的文件级表达：Schema（API）是核心

USD 里“物理”不是一个黑盒，而是一组 Schema（API Schema）挂在 prim 上：

### 2.1 碰撞：CollisionAPI + MeshCollisionAPI
- `UsdPhysics.CollisionAPI`：表达“该 prim 参与碰撞”。
- `UsdPhysics.MeshCollisionAPI`：表达碰撞形状来源于 mesh，以及采用什么近似。
  - 常见字段：`physics:approximation`（或通过 API 的 ApproximationAttr 设置）。

### 2.2 刚体：RigidBodyAPI
- `UsdPhysics.RigidBodyAPI`：表达该 prim 是刚体（动力学参与者）。
- 是否启用：`physics:rigidBodyEnabled`。

> 经验法则：
> - **动态物体**必须是 rigid body，并且 collider 一般应该是凸的（或由凸分解组成），这样稳定且速度快。
> - **静态物体**可以用 triangle mesh collider（`approx = none`），因为静态三角网格碰撞更准确且 PhysX 支持更好。

### 2.3 关节：UsdPhysics.Joint（及其派生）
- USD 里 joint prim 通常会有 `physics:body0` / `physics:body1` relationship 指向被连接的刚体 prim。
- joint 是否启用：`physics:jointEnabled`。

项目在处理 articulated（带关节）对象时，一般会：
- 先给 joint 相关的 body prim 绑刚体/碰撞，
- 再把 joint enable 打开。

---

## 3. PhysX 扩展：PhysxSchema.* 负责更具体的“碰撞生成策略”

除了 USD 的通用物理 Schema，Isaac Sim/PhysX 还提供 `PhysxSchema` 来设置更具体的参数。

项目里常用的有：

### 3.1 SDF 碰撞（适合复杂几何的稳定近似）
- `PhysxSchema.PhysxSDFMeshCollisionAPI`
- 典型参数：`sdfResolution`（本项目默认设为 256）

特点：
- 表达能力强，能比较好地贴合复杂几何；
- 生成/计算开销相对更高；
- 对网格质量、尺度很敏感。

### 3.2 Convex Hull（单凸包）
- `PhysxSchema.PhysxConvexHullCollisionAPI`
- 典型参数：`hullVertexLimit`（项目默认 64）

特点：
- 非常快，非常稳定；
- 但只能表达凸形，凹陷结构会被“填平”。

### 3.3 Convex Decomposition（凸分解：用多个凸包逼近凹形）
- `PhysxSchema.PhysxConvexDecompositionCollisionAPI`
- 典型参数：`hullVertexLimit`、`maxConvexHulls`

特点：
- 动态物体常用的折中：比 SDF 更快、比单凸包更贴合；
- 生成凸分解可能很慢；
- hull 数量过多会显著影响仿真性能。

### 3.4 Triangle Mesh / Mesh Simplification
- Triangle mesh：`approx = none`（项目里用作“静态网格碰撞”）
- Simplification：`PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI`

经验法则：
- **动态物体尽量不要用 triangle mesh collider**（PhysX 对动态三角网格支持限制多、容易不稳定）。
- 静态环境/不可移动结构可以用 triangle mesh collider，准确且简洁。

---

## 4. 为什么项目会做 Transform 规范化（xformOp）

USD 中 prim 变换可以用多种组合表达：
- `xformOp:transform`（矩阵）
- `xformOp:translate` / `xformOp:orient` / `xformOp:scale`（TRS 分量）
- `xformOpOrder` 规定这些 op 的应用顺序

项目里经常将 `xformOp:transform` 转换为 TRS（translate/orient/scale），原因主要是：
- 物理 API（尤其是 collider 与刚体）在处理变换时，对“非标准 op 组合、非均匀缩放、矩阵里混入 scale”更敏感；
- TRS 的表达更容易被工具链正确解析，也更利于后续调参与 debug。

对应实现：
- interaction 预处理里有 `transform_to_rt`（将 transform 拆为 translate/orient/scale）。
- 一键脚本也保留了同类逻辑。

---

## 5. Mesh Merge Collision：为什么要“合并多个 leaf mesh”

真实资产常见结构是：
- 一个逻辑物体（Xform）下面挂了多个 Mesh（leaf meshes）。

如果逐个 Mesh 绑 collider：
- collider 数量变多；
- 稳定性更差（尤其是动态物体和关节体）；
- 性能更差。

项目使用 `PhysxSchema.PhysxMeshMergeCollisionAPI` + Collection 来指定：
- **includes**：应该参与合并碰撞的 leaf mesh 列表
- **excludes**：排除列表

然后把 collider 绑定在更高层的 prim 上（Xform），让 PhysX 在物理层把多个 mesh 视为一个“合并的碰撞几何”。

---

## 6. interaction（交互）策略：动态 vs 静态、pickable 的意义

这一节用“新手视角”把 interaction 场景里最关键的设计讲清楚：为什么要区分动态/静态？pickable 到底是什么？它和碰撞近似、性能、稳定性有什么关系？

先给一个直觉：
- **动态（Dynamic）** = 会动、会被推动、会掉落、会被抓取的物体。它必须参与物理求解（质量、惯性、速度、碰撞响应）。
- **静态（Static）** = 场景里的“地形/建筑/固定结构”。它只需要提供“碰撞边界”，不需要被物理引擎推进。

交互场景的核心目标是：
- 让机器人/人能“拿起来/推得动”的物体表现像现实一样（动态刚体）；
- 让环境保持稳定、碰撞准确且性能可控（静态碰撞）。

为什么不能把所有东西都设成动态？
- **性能会爆炸**：每个动态物体都需要持续求解（碰撞检测 + 接触约束 + 关节约束）。动态数量一多，帧率会急剧下降。
- **稳定性会下降**：动态物体越多、形状越复杂（尤其是三角网格），越容易出现抖动、穿透、弹飞等问题。
- **任务语义不匹配**：比如墙/地面本来就是“不可移动”的，把它设成动态反而会让场景变得不真实。

因此本项目把“应该变成动态的那一小部分”抽出来，用一个经验规则集合来表达：**pickable 列表**。

项目里典型策略（见 `preprocess_for_interaction.py` 与 `simready.py`）：

### 6.1 scope 的分流
本项目的数据里，场景通常按层级组织为：
- `/Root/Meshes/<scope>/<category>/<instance>`

你可以把它理解成：
- **scope**：大类分区（例如基础结构、可动物体、带关节/动画的物体等）。
- **category**：语义类别（例如 `cup`、`basket`、`door`、`oven`）。
- **instance**：某个具体模型实例（名字通常包含 hash/编号）。

interaction 的处理会先按 **scope** 做第一层分流，因为 scope 往往暗示了“这批物体应该怎么动”：

- `Base`
  - 直觉：这是“场景地基/固定结构”。
  - 策略：**静态碰撞**即可。
  - 典型做法：绑定 triangle mesh collider（`approx = none`）。因为它是静态的，三角网格最贴合、也最省心。

- `BaseAnimation` / `Animation`
  - 直觉：这是“带关节/可动部件”的对象集合（比如抽屉、柜门、可转动部件）。
  - 策略：走 articulation/关节流程：给相关部件绑定刚体/碰撞，然后启用 joint。
  - 关键点：如果不启用 joint，关节物体就会变成“一坨刚体”或完全不动，交互会失败。

- 其他 scope（常见：`Furniture` 等）
  - 直觉：既可能是桌椅等固定大件，也可能是杯子等小物件。
  - 策略：进一步看 **category 是否 pickable**：
    - 是 pickable → 动态刚体 + 动态友好碰撞近似
    - 非 pickable → 静态 triangle mesh

### 6.2 pickable 列表
pickable 的意思很朴素：**在交互任务里，我们“希望它能被拿起/推动/掉落”的类别**。

它不是物理引擎的概念，而是任务需求的概念：
- 机器人要学习/执行“抓取杯子”→ `cup` 必须能动。
- 机器人要“推开篮子”→ `basket` 必须能动。
- 机器人一般不会去“搬走墙体/地板/大柜子”→ 它们就不应该是动态物体。

为什么用列表而不是自动推断？
- 从几何上很难可靠推断“是否可拿起”。体积小不一定可抓，体积大也可能可推。
- 从语义上（category）推断更稳定：数据集的 category 是相对稳定且可控的。

当某个 category 被判定为 pickable 时，典型绑定组合是：
- **RigidBodyAPI**：让它参与动力学（会掉落、会被推、会被抓取）。
- **Collider（动态友好）**：通常选 `convexDecomposition`（凸分解）。
  - 为什么：动态物体尽量避免 triangle mesh；凸分解是“准确度/性能/稳定性”的折中。

当某个 category 不是 pickable 时，典型绑定是：
- **静态 triangle mesh collider**（`approx = none`）
  - 为什么：静态三角网格碰撞更准确，而且静态场景稳定性更好。

维护建议（给新手的落地做法）：
- 如果你发现某类物体“应该能抓却抓不起来”，优先把它加进 pickable 列表。
- 如果你发现场景太卡，优先把“其实不需要动”的类别从 pickable 列表移出去。
- 如果某类物体动态后容易抖动/穿透，优先尝试把该类的碰撞近似从 `convexDecomposition` 降级成 `convexHull`（更稳更快但更粗）。

### 6.3 articulated（带关节）对象
这类对象内部常按命名约定分组：
- `group_static`：关节链里固定部分
- `group_00`：主可动部分（是否可动可能取决于 pickable）
- 其他 group：通常视为刚体

流程上一般是：
- 对相关 group 绑刚体/碰撞；
- 最后启用 joints（`physics:jointEnabled = True`）。

新手常见误区：
- **只启用 joint 但没给 body 上刚体/碰撞**：关节没有“被连接的动力学体”，结果要么不动要么行为怪异。
- **给关节体用 triangle mesh collider**：动态/关节系统对复杂三角网格更敏感，优先用凸分解或凸包。

一个可操作的检查清单（debug 时非常有用）：
1) 这个对象的可动部件 prim 是否有 `UsdPhysics.RigidBodyAPI`？
2) 对应 prim 是否有 `UsdPhysics.CollisionAPI` 且 `collisionEnabled = True`？
3) joint prim 的 `physics:body0/body1` 是否指向有效 prim path？
4) joint prim 的 `physics:jointEnabled` 最终是否为 True？

---

## 7. navigation（导航）策略：静态碰撞 + 语义

导航任务通常只需要：
- 足够准确的静态碰撞（用于规划/避障）；
- 语义标签（用于分割、语义地图、或规则过滤）。

因此典型策略是：
- 对大多数对象：只做静态 triangle mesh collider；
- 对带 joints 的模型：通常先禁用 joints，必要时对 joint 连接的 body 单独上静态 collider；
- 对 door：经常选择禁用（`SetActive(False)`）或做“只保留主门”的特殊逻辑（仓库原脚本里有更完整处理）。

语义标签写入：
- 使用 `omni.isaac.core.utils.semantics.add_update_semantics`，递归给 Mesh/Xform 标记 `class`。
- label 常用 `{category}/{instance_name}` 形式。

---

## 8. 参数与取舍：准确性、稳定性、性能三角形

你可以把碰撞近似理解为一个三角形：
- 越准确（更贴合几何）通常越慢；
- 越快通常越“粗”；
- 稳定性常常与“凸性、数量、尺度一致性”强相关。

项目默认策略（经验取舍）：
- 动态物体：`convexDecomposition`（性能与精度折中）
- 静态物体：`triangle mesh`（`approx = none`）
- SDF：保留作为选项，但默认不作为动态物体首选。

常见可调参数：
- convex hull vertex limit（默认 64）
- SDF resolution（默认 256）
- convex decomposition 的 max hulls（在一键脚本里与 leaf mesh 数相关）

---

## 9. 常见问题（踩坑清单）

### 9.1 我能不能把整个场景（静态 + 动态）都统一用 SDF 碰撞？

可以“技术上这么做”，但通常 **不建议把整个场景都统一改成 SDF**。更实用的经验是：**SDF 作为特例使用**，而不是默认策略。

原因（新手最容易忽略的几点）：
- **生成成本与加载成本更高**：SDF 需要从网格生成体素/距离场表示（或由引擎在加载时生成/缓存），分辨率越高越慢、占用越大。
- **规模化会拖垮性能**：如果整个场景几百/几千个物体都用 SDF，碰撞检测与内存占用会明显上升，交互帧率会掉得很快。
- **静态环境没必要**：对静态环境，triangle mesh（`approx = none`）通常已经足够准确且更常用；在“静态 + 大量几何”的场景里，triangle mesh 往往是更合理的默认。

什么时候更推荐 SDF？
- 少量、几何非常复杂、且你明显感觉 convex hull/convex decomposition 过于“粗糙”影响交互质量的物体。
- 你愿意用更高的计算/内存成本换更好的碰撞贴合。

结论：
- **静态大场景：优先 triangle mesh**。
- **动态可交互物体：优先 convex（hull 或 decomposition）**。
- **SDF：按需启用，别全场景默认**。

### 9.2 动态物体是不是“只能用 convex”，不能用三角网格？

你的理解基本是对的：在 PhysX/Isaac Sim 的常见最佳实践里，**动态刚体（dynamic rigid body）优先使用 convex 类型的碰撞（convex hull / convex decomposition）**。

为什么我们还会强调“动态用 triangle mesh 会不稳定/性能差”？
- 很多新手会尝试把 `approx = none`（triangle mesh）直接用在动态刚体上，因为它“看起来最准确”。
- 但在实际引擎实现与约束里：
  - 动态三角网格在很多情况下 **要么不被支持/会被引擎拒绝/被自动降级**，
  - 要么即使能跑，也经常带来 **接触不稳定、穿透、抖动**，并且 **碰撞代价很高**。

所以可以把它当成一个更稳妥的规则：
- **动态：convex hull / convex decomposition（优先）**
- **静态：triangle mesh（优先）**

如果你确实需要“更贴合”的动态碰撞：
- 先尝试 `convexDecomposition`（凸分解）而不是 triangle mesh。
- 如果仍不够贴合，再考虑对少量对象启用 SDF（并接受更高成本）。

1) 动态物体使用 triangle mesh collider 导致不稳定/性能差
- 建议：动态物体用 convex decomposition 或 convex hull。

2) 非均匀缩放/复杂 transform 导致碰撞生成异常
- 建议：尽量使用 TRS 表达；避免把 scale 混在矩阵里。

3) 网格质量问题（非流形、面索引错误）导致 collider 生成失败
- 需要先做 mesh 清理，或降级近似方式。

4) articulated 物体 joints 没启用
- 检查 joint prim 的 `physics:jointEnabled` 是否在最后被设置为 True。

5) 性能过慢
- 优先减少 convex hull 数量（maxConvexHulls）、减少 dynamic 物体数量、或更换近似为 convex hull。

---

## 10. 与本项目代码的对应关系

- 最小物理绑定工具：`docs/reference/modules/usd_physics.md`（对应 `set_physics/pxr_utils/usd_physics.py`）
- 交互预处理：`docs/reference/modules/preprocess_interaction.md`（对应 `set_physics/preprocess_for_interaction.py`）
- 导航预处理：`docs/reference/modules/preprocess_navigation.md`（对应 `set_physics/preprocess_for_navigation.py`）
- 一键 CLI（本篇最相关）：`set_physics/simready.py`

如果你希望进一步“统一输出格式”（导出到 specs 目录结构），可以结合：
- `python -m specs_normalizer`（见 `docs/reference/specs/normalizer_usage.md`）
