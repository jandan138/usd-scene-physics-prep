---
title: Scene Export Usage
code_reference: set_physics/export_scene.py
created_at: 2025-12-19
updated_at: 2025-12-22
maintainer: Project Team
status: Active
---

# 使用指南：场景导出

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../set_physics/get_all_references.py
> - ../../set_physics/export_scene.py
>
> 总入口：../index.md

## 索引
- [依赖清单生成](#依赖清单生成)
- [导出](#导出)

## 依赖清单生成
- 扫描模型与材质引用，分别写入 JSON（`get_model_and_material_references_set`，`set_physics/get_all_references.py:29-57,64-72`）。

## 导出
- 根据引用清单复制模型与材质；复制 `start_result_dynamic.usd` 以及静态场景文件；重建 `Materials` 软链接（`set_physics/export_scene.py:21-54`）。
