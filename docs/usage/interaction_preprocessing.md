# 交互预处理

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

## 可拾取对象列表
- 列表定义见 `set_physics/preprocess_for_interaction.py:258-284`。

