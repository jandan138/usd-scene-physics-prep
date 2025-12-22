# 交互预处理

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../set_physics/preprocess_for_interaction.py
> - ../../set_physics/pxr_utils/usd_physics.py
> - ../../scripts/prep_interaction_root_scene.py
> - ../../scripts/list_draggable_prims.py
>
> 总索引：../overview/docs_index.md

## 索引
- [目标](#目标)
- [关键函数](#关键函数)
- [处理逻辑](#处理逻辑)
- [外部数据集（`/root` 结构）](#外部数据集root-结构)
- [可拾取对象列表](#可拾取对象列表)

## 目标
- 为可交互对象（可拾取/带关节）绑定刚体与合适的碰撞近似，启用关节，建立碰撞分组并与基础静态组互斥，输出交互仿真场景。

## 关键函数
- 近似选项集合（`set_physics/preprocess_for_interaction.py:18-24`）
- 合并网格碰撞集合（`set_mesh_merge_collision`，`set_physics/preprocess_for_interaction.py:96-104`）
- 碰撞近似绑定（`set_collider_with_approx`，`set_physics/preprocess_for_interaction.py:106-128`）
- 刚体绑定与变换规范化（`set_rigidbody`，`set_physics/preprocess_for_interaction.py:130-146`）
- 关节收集与启用（`get_all_joints`、`bind_articulation`，`set_physics/preprocess_for_interaction.py:27-38,229-256`）

## 处理逻辑
- 写语义：`category/model_name`（`set_physics/preprocess_for_interaction.py:335-353`）。
- Base/Animation 分支：
  - Base：静态三角网格近似（`TRIANGLE_MESH`），并纳入基础碰撞组（`set_physics/preprocess_for_interaction.py:360-377`）。
  - BaseAnimation/Animation：按 `PICKABLE_OBJECTS` 决定是否刚体，启用关节（`set_physics/preprocess_for_interaction.py:258-284,335-371`）。
- 输出：保存为 `start_result_dynamic.usd`（`set_physics/preprocess_for_interaction.py:321-323,379-386`）。

## 外部数据集（`/root` 结构）
本仓库默认的交互预处理（`set_physics/preprocess_for_interaction.py`）主要面向规范化后的 GRScenes 输出结构；如果你的输入场景根节点是 `/root`（例如 SimBench GRSceneUSD/task10），并且没有 `Materials/` 目录，建议走专用脚本：
- 入口脚本：`scripts/prep_interaction_root_scene.py`
- 排错与参数推荐：`docs/operations/troubleshooting_interaction_preprocess.md`
 - 更完整用法（命令模板/参数说明）：`docs/usage/prep_interaction_root_scene.md`

如果你想快速复用一套“实战稳定策略/踩坑复盘”（包含 task9 的 GLB payload、浮空、消失等现象的处理思路），见：
- `docs/operations/simbench_interaction_preprocess_field_notes.md`

同时，如果你需要“列出可 Shift+左键拖拽（持续施力能改变位置）的 prim”，可以使用：
- `scripts/list_draggable_prims.py`

## 可拾取对象列表
- 列表定义见 `set_physics/preprocess_for_interaction.py:258-284`。

