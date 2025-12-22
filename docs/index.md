---
title: USD Scene Physics Prep 文档主页
---

# USD Scene Physics Prep

> 最后更新：2025-12-22
>
> 相关代码：
> - ../clean_data.py
> - ../set_physics/simready.py
> - ../set_physics/pxr_utils/data_clean.py
> - ../set_physics/preprocess_for_interaction.py
> - ../set_physics/preprocess_for_navigation.py
> - ../specs_normalizer/normalize.py
>
> 总索引：overview/docs_index.md

## 索引
- [文档索引](#文档索引)
- [快速链接](#快速链接)

一个用于 USD 场景的资产拆分与物理仿真预处理工具集，支持将原始场景规范为统一的模型/材质库与场景引用结构，并批量为对象绑定碰撞体、刚体、关节等物理属性，适配 Omniverse Isaac Sim 的交互与导航仿真。

## 文档索引
- 概述
  - [项目概述](overview/project_overview.md)
- 架构
  - [处理流水线](architecture/pipeline.md)
  - [目录结构](architecture/directory_structure.md)
- 使用
  - [快速开始](usage/quickstart.md)
  - [交互预处理](usage/interaction_preprocessing.md)
  - [导航预处理](usage/navigation_preprocessing.md)
  - [场景打包与导出](usage/export_scenes.md)
  - [simready（一键生成可仿真 USD）](usage/simready.md)
- 模块说明
  - [data_clean](modules/data_clean.md)
  - [usd_physics](modules/usd_physics.md)
  - [preprocess_for_interaction](modules/preprocess_interaction.md)
  - [preprocess_for_navigation](modules/preprocess_navigation.md)
  - [simready（原理）](modules/simready.md)
- 引用与依赖
  - [资产与材质引用](references/assets_and_materials.md)
  - [环境与依赖](references/dependencies.md)
- 规格说明
  - [数据目录结构规范解读](specs/dataset_structure_interpretation.md)
  - [目录结构对比：原始输出 vs 规范结构](specs/structure_comparison.md)
  - [规范化导出工具使用说明](specs/normalizer_usage.md)
- 运维
  - [Windows 使用注意](operations/windows_notes.md)
  - [环境准备建议](operations/environment_setup.md)
  - [交互预处理排错（SimBench/GRSceneUSD task10）](operations/troubleshooting_interaction_preprocess.md)
  - [SimBench（GRSceneUSD task9/task10）交互预处理：实战踩坑汇总](operations/simbench_interaction_preprocess_field_notes.md)
  - [排错：GLB payload 多 Mesh（geometry_01 等）未绑定 collider 导致穿透](operations/troubleshooting_glb_payload_multimesh.md)
- 示例与答疑
  - [工作流示例](examples/workflow_examples.md)
  - [常见问题](faq/faq.md)

## 快速链接
- 仓库主页：`https://github.com/jandan138/usd-scene-physics-prep`
- Pages（启用后）：`https://jandan138.github.io/usd-scene-physics-prep/`
