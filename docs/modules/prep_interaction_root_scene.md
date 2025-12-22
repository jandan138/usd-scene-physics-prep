# `prep_interaction_root_scene.py`（原理与代码导读）

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../scripts/prep_interaction_root_scene.py
> - ../../scripts/list_draggable_prims.py
> - ../../scripts/oneoff_force_draggable.py
> - ../../set_physics/preprocess_for_interaction.py
>
> 总索引：../overview/docs_index.md

## 索引
- [设计目标](#设计目标)
- [核心限制（为什么这么设计）](#核心限制为什么这么设计)
- [整体流程](#整体流程)
- [规则系统（static/dynamic 判定）](#规则系统staticdynamic-判定)
- [GLB payload 降级策略](#glb-payload-降级策略)
- [关键实现点](#关键实现点)

## 设计目标
该脚本解决一个非常具体的问题：
- 输入是外部数据集的 `/root` 结构场景（不满足本仓库规范化后的 `/Root/Meshes` 结构）。
- 希望输出一个“可以在 Isaac Sim 里直接做交互仿真”的 USD：动态对象可拖拽、静态环境有可靠碰撞。

它的取舍是：
- **不做**目录规范化/材质拆分/模型去重（这些是 `simready.py + parse_scene` 的工作）。
- **只做**在原始 stage 上 author `UsdPhysics` 的刚体/碰撞属性，并尽量规避 PhysX cooking 的常见失败点。

## 核心限制（为什么这么设计）
1) 一些 Isaac/Kit 环境缺少 `pxr.PhysxSchema`
- 因此脚本只依赖 `UsdPhysics`（CollisionAPI/MeshCollisionAPI/RigidBodyAPI/MassAPI）。

2) GLB payload 在无头 USD 环境可能无法加载
- 无法读取真实 Mesh points/topology，也无法可靠遍历 payload 子树。
- 但 Isaac GUI/运行时可能能解析 payload：这时只要在固定 prim path 上提前 author 物理属性，运行时仍会生效。

3) 场景里可能存在大量无效 Mesh
- 对没有 points/topology 的 Mesh 绑定 collider 会触发 PhysX 报错。
- 所以脚本默认只给“有效 Mesh（points 非空且拓扑一致）”绑定 collider。

## 整体流程
脚本的主流程可以概括为：
1. 复制输入 USD → 输出 USD（避免原地改写）。
2. 打开 stage，在 `--root` 子树下做一次“全量清理与强制关闭 physics flag”。
3. 对用户指定的 `--static` prim：绑定静态碰撞（不启用刚体）。
4. 对 `--root` 下一级 prim：按规则匹配出 dynamic 候选，为其绑定刚体、质量与碰撞。
5. 保存输出层，并打印统计信息（skipped meshes / forced untyped colliders）。

## 规则系统（static/dynamic 判定）
脚本把“要不要变成刚体”限制在 `--root` 的**一级子节点**，避免误把环境内部的子网格都设成刚体。

- static：由 `--static <primPath>` 明确指定。
- dynamic：默认匹配：
  - 名字前缀：`obj_`、`__`
  - 精确名字：`_`
  这些规则对应 SimBench 常见命名（`obj_*` 与 GLB 导入对象 `__xx` / `_`）。

## GLB payload 降级策略
当某个 dynamic prim：
- 由于 payload 无法加载/无法遍历，导致“找不到任何有效 Mesh”，
脚本会进入降级路径：
- 如果该 prim `HasPayload()`：直接探测常见几何 prim path，并在这些 path 上 author collider。

探测路径策略（按优先级）：
- 叶子 mesh 路径：
  - `<prim>/geometry_0/geometry_0`
  - `<prim>/geometry_0/geometry_01..geometry_09`
- 如果上述都不存在：退回到 `<prim>/geometry_0`

为什么要优先 leaf mesh：
- GLB payload 可能包含多个 sibling Mesh（例如 `geometry_01`），只写一个会漏碰撞导致穿透。
- 同时要避免 parent+child 都写 collider，否则会出现“重复碰撞体叠加→浮空/爆炸接触”。

## 关键实现点
- **覆盖引用层强 opinion**：清理 API 并不足以覆盖引用层里写死的 `physics:*Enabled=True`，因此脚本会在输出层显式 Set False，然后再对目标对象重启用。
- **质量兜底**：会清理已有 MassAPI，并写入 `physics:mass=1.0` + 一个保守惯性，规避负质量/无效惯性警告。
- **Mesh 有效性过滤**：只给 points/topology 合法的 Mesh 绑定 collider；无效 Mesh 直接统计到 `skipped_meshes`。
- **draggable 自检**：配套 `scripts/list_draggable_prims.py` 用“刚体启用 + 子树 collider 启用”作为经验判定，并对 GLB payload 做路径 probe。
