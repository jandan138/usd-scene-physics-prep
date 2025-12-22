# 处理流水线

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../clean_data.py
> - ../../set_physics/pxr_utils/data_clean.py
> - ../../set_physics/preprocess_for_interaction.py
> - ../../set_physics/preprocess_for_navigation.py
> - ../../set_physics/get_all_references.py
> - ../../set_physics/export_scene.py
>
> 总索引：../overview/docs_index.md

## 索引
- [总览](#总览)
- [关键步骤详解](#关键步骤详解)

## 总览
1. 场景拆分与重组（`clean_data.py:5-24` → `set_physics/pxr_utils/data_clean.py:517-705`）
2. 基础资产规范化（变换、材质、贴图、引用）（`set_physics/pxr_utils/data_clean.py:121-149,367-397`）
3. 交互仿真预处理（刚体、碰撞近似、关节启用、碰撞分组）（`set_physics/preprocess_for_interaction.py:335-387`）
4. 导航仿真预处理（门处理、静态近似、语义标签）（`set_physics/preprocess_for_navigation.py:198-231,389-425`）
5. 依赖收集与场景打包（`set_physics/get_all_references.py:29-57,64-72`，`set_physics/export_scene.py:21-54`）

## 关键步骤详解
- 拆分：拷贝根结构，跳过 `Meshes/Looks/PhysicsMaterial`，为每个实例生成 `instance.usd` 并用 reference 引用（`set_physics/pxr_utils/data_clean.py:567-572,652-666`）。
- 变换：统一 `xformOp` 为 `translate/orient/scale`，便于 rigid/collider 使用（`set_physics/pxr_utils/data_clean.py:121-149`）。
- 材质：保持材质绑定关系，复制 `.mdl` 与贴图至目标路径（`set_physics/pxr_utils/data_clean.py:545-549,367-397`）。
- 交互：为可拾取与带关节对象绑定刚体与碰撞近似，建立碰撞分组并过滤（`set_physics/preprocess_for_interaction.py:229-256,335-387`）。
- 导航：禁用非主门或门模型，其他对象绑定静态三角网格碰撞近似，写语义（`set_physics/preprocess_for_navigation.py:198-231,389-425`）。

