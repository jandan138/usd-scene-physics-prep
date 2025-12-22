# 模块说明：usd_physics

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../set_physics/pxr_utils/usd_physics.py
>
> 总索引：../overview/docs_index.md

## 索引
- [碰撞体](#碰撞体)
- [刚体](#刚体)
- [说明](#说明)

## 碰撞体
- 单 mesh 绑定：`set_collider_`（`set_physics/pxr_utils/usd_physics.py:60-89`）。
- 递归绑定：`set_collider`（`set_physics/pxr_utils/usd_physics.py:92-101`）。
- 移除：`remove_collider`/`remove_collider_`（`set_physics/pxr_utils/usd_physics.py:103-121,116-122`）。
- 启用/禁用：`activate_collider`/`deactivate_collider`（`set_physics/pxr_utils/usd_physics.py:123-141`）。

## 刚体
- 绑定：`set_rigidbody`（`set_physics/pxr_utils/usd_physics.py:146-149`）。
- 移除：`remove_rigid`/`remove_rigid_`（`set_physics/pxr_utils/usd_physics.py:150-163,151-157`）。
- 启用/禁用：`activate_rigid`/`deactivate_rigid`（`set_physics/pxr_utils/usd_physics.py:166-175,179-189`）。

## 说明
- 该模块更贴近 USD 原生 API，适合进行最小化物理绑定与清理；复杂近似与碰撞分组在 `preprocess_for_interaction.py`/`preprocess_for_navigation.py` 中实现。

