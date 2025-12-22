# simready：一键生成可仿真 USD（原理）

> 最后更新：2025-12-22
> 
> 相关代码：
> - ../../set_physics/simready.py
> - ../../set_physics/pxr_utils/data_clean.py（拆分/规范化：`parse_scene`）
> - ../../set_physics/preprocess_for_interaction.py（逻辑来源之一）
> - ../../set_physics/preprocess_for_navigation.py（逻辑来源之一）
> 
> 总索引：../overview/docs_index.md

## 索引
- [设计目标](#设计目标)
- [适用范围与前置假设](#适用范围与前置假设)
- [整体流程](#整体流程)
- [拆分与规范化（parse_scene）](#拆分与规范化parse_scene)
- [物理处理：interaction 模式](#物理处理interaction-模式)
- [物理处理：navigation 模式](#物理处理navigation-模式)
- [关键实现细节](#关键实现细节)
- [与现有脚本的关系](#与现有脚本的关系)

## 设计目标
`set_physics/simready.py` 把仓库里两段核心能力合并成一个“可复用的一次性入口”：
1. 通过 `parse_scene` 把“原始场景”整理成可复用的 `Materials/models/scenes` 结构（类似 `target/`）。
2. 在规范化后的场景上写入 PhysX/UsdPhysics schema（碰撞、刚体、关节启用/禁用）以及（可选）语义标签，使场景在 Isaac Sim 内可直接用于交互或导航仿真。

## 适用范围与前置假设
`simready.py` 的设计核心是“先规范化到仓库的统一结构，再做 physics”。因此它隐含了几条前置假设：
- 输入要么能被 `parse_scene` 规范化（通常意味着资源引用能解析、同级或可定位到 `Materials/`），要么你使用 `--skip-clean` 并确保输入已经是规范化结构。
- interaction 处理逻辑默认按 `/Root/Meshes` 的层级遍历实例与 scope。

如果你的输入属于外部数据集（常见根为 `/root` 小写、可能缺失 `Materials/`、或存在 GLB payload 在无头环境不可加载），更推荐使用专用脚本：
- `scripts/prep_interaction_root_scene.py`

相关排错与案例：
- `docs/operations/troubleshooting_interaction_preprocess.md`
- `docs/operations/simbench_interaction_preprocess_field_notes.md`
- `docs/operations/troubleshooting_glb_payload_multimesh.md`

## 整体流程
- CLI 解析参数后走两条路径：
  - 默认：先拆分规范化，再做 physics：
    1) `normalize_scene_to_target()` → 调 `parse_scene()` 在 `out-root` 下生成结构，并定位规范化后的场景 USD。
    2) 根据 `--mode` 调用：
       - `process_interaction_physics()` 输出 `start_result_dynamic.usd`
       - `process_navigation_static()` 输出 `start_result_navigation.usd`
  - `--skip-clean`：认为输入场景已是规范化结构，直接在输入 USD 上写 physics 输出到指定位置。

## 拆分与规范化（parse_scene）
- 入口：`normalize_scene_to_target()`。
- 核心点：复用本仓库现有拆分器 `set_physics/pxr_utils/data_clean.py:parse_scene`。
- 产物：
  - `out-root/Materials`：复制并统一材质/贴图
  - `out-root/models`：对同类实例做“指纹 + hash”去重，输出 `instance.usd`
  - `out-root/scenes/<scene_id>/`：场景 USD，内部以 reference 指向 `models/` 里的资产

这一步把输入 USD 从“场景大文件”变成“引用驱动、可复用”的结构，后续 physics 只需要在 scenes 下的布局文件上写 schema 即可。

## 物理处理：interaction 模式
- 入口：`process_interaction_physics(scene_usd, out_usd)`。
- Isaac Sim 启动：函数内创建 `SimulationApp(...)`，并在 finally 中 `kit.close()`，避免导入模块时就拉起 Kit。
- 主要遍历路径：假设场景存在 `/Root/Meshes`，按层级遍历：
  - scope（如 `BaseAnimation/Animation/Base/...`）
  - category（如 `door/oven/...`）
  - instance（模型实例 prim）
- 对每个实例：
  1) 先清理历史 physics（递归移除 collider/rigidbody/jointEnabled）。
  2) 找到 `Instance` 子 prim（若不存在则 fallback 用实例 prim 自身）。
  3) 根据 scope + 是否 pickable（`DEFAULT_PICKABLE_CATEGORIES`）决定：
     - `BaseAnimation/Animation`：更偏“可动/带关节”对象；对子组（`group_00/group_static` 等）绑定刚体或静态碰撞，并把所有关节 `jointEnabled=True`。
     - `Base`：静态三角网格碰撞近似。
     - 其他 scope：pickable → 刚体 + convex decomposition；非 pickable → 静态三角网格。

## 物理处理：navigation 模式
- 入口：`process_navigation_static(scene_usd, out_usd)`。
- 语义标注：用 `omni.isaac.core.utils.semantics.add_update_semantics` 递归写入 label（默认 `"{category}/{instance_name}"`）。
- door 处理：若 `--disable-doors`，对 `cate_name == "door"` 的实例直接 `SetActive(False)`。
- 物理策略：导航模式整体做“静态化”：
  - 清理历史 physics
  - 关节全部禁用（`jointEnabled=False`）
  - 如果存在关节连接的 body prim：对这些连接 body 单独绑定静态三角网格碰撞；否则对整个 instance 绑定静态三角网格碰撞

## 关键实现细节
- 变换规范化：`_transform_to_trs()` 将 `xformOp:transform` 拆为 `translate/orient/scale`，以更稳定地配合 rigid body。
- MeshMergeCollision：在支持 `pxr.PhysxSchema` 的 Isaac 环境中，可通过 `PhysxSchema.PhysxMeshMergeCollisionAPI` 把一个 Xform 下的 leaf meshes 作为碰撞集合，提高碰撞绑定一致性。
- 碰撞近似：通过 `UsdPhysics.MeshCollisionAPI.approximation` 选择 `convexDecomposition/none/...`，并按类型附加 PhysX schema（SDF/ConvexDecomposition 等）。
- 清理逻辑：在每次写 physics 前递归移除相关 schema 和属性，避免重复运行造成叠加。

## 与现有脚本的关系
- `simready.py` 的逻辑和本仓库的两份脚本高度同源：
  - interaction：接近 `set_physics/preprocess_for_interaction.py`
  - navigation：接近 `set_physics/preprocess_for_navigation.py`
- 主要差异：
  - 以 CLI 参数化方式运行，不再要求手改 `scene_path/dest_scene_path`。
  - 提供 `--skip-clean`，允许在已规范化的场景上重复生成不同 physics 输出。
  - 将 Isaac Sim 的启动/关闭包进函数内部，降低被 import 时的副作用。
