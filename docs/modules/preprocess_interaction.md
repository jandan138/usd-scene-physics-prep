# 模块说明：preprocess_for_interaction

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../set_physics/preprocess_for_interaction.py
> - ../../set_physics/pxr_utils/usd_physics.py
>
> 总索引：../overview/docs_index.md

## 索引
- [近似选项](#近似选项)
- [关键能力](#关键能力)
- [可拾取对象](#可拾取对象)

## 近似选项
- SDF、ConvexHull、ConvexDecomposition、MeshSimplification、TriangleMesh（`set_physics/preprocess_for_interaction.py:18-24`）。

## 关键能力
- `transform_to_rt`：统一 `translate/orient/scale`（`set_physics/preprocess_for_interaction.py:67-94`）。
- `set_mesh_merge_collision`：合并网格碰撞集合（`set_physics/preprocess_for_interaction.py:96-104`）。
- `set_collider_with_approx`：按近似绑定碰撞体与 PhysX 细节（`set_physics/preprocess_for_interaction.py:106-128`）。
- `set_rigidbody`：绑定刚体并确保有 `xformOp:transform`（`set_physics/preprocess_for_interaction.py:130-146`）。
- `bind_articulation`：为带关节对象绑定刚体/碰撞近似，启用关节（`set_physics/preprocess_for_interaction.py:229-256`）。
- `solve_scene`：遍历场景并按 scope/cate 分支处理，输出交互场景（`set_physics/preprocess_for_interaction.py:287-323`）。

## 可拾取对象
- 列表见 `set_physics/preprocess_for_interaction.py:258-284`。

