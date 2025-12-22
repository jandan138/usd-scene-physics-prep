# 资产与材质引用

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../set_physics/pxr_utils/data_clean.py
> - ../../set_physics/get_all_references.py
> - ../../set_physics/export_scene.py
>
> 总索引：../overview/docs_index.md

## 索引
- [资产路径处理](#资产路径处理)
- [材质复制](#材质复制)
- [引用收集](#引用收集)

## 资产路径处理
- 相对路径资产去除父级 `..` 前缀，复制到 `target` 并更新属性引用（`set_physics/pxr_utils/data_clean.py:367-397`）。

## 材质复制
- 拷贝 `.mdl` 文件到场景目录下的材质链接位置（`set_physics/pxr_utils/data_clean.py:545-549`）。
- 在导出过程中重建 `Materials` 软链接，指向统一材质库（`set_physics/export_scene.py:45-54`）。

## 引用收集
- 模型引用：`get_model_reference`（`set_physics/get_all_references.py:5-12`）。
- 材质引用：`get_material_reference`（扫描 shader 属性中的 asset）（`set_physics/get_all_references.py:15-26`）。
- 汇总：`get_model_and_material_references_set`（`set_physics/get_all_references.py:29-57`）。

