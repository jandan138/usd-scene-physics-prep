# USD Scene Physics Prep

一个围绕 USD 场景的资产拆分与物理仿真预处理工具集：将原始场景规范为可复用的模型/材质库与场景引用结构，并批量为对象绑定碰撞体、刚体、关节等物理属性，适配 Omniverse Isaac Sim 的交互与导航仿真。

## 文档索引
- 概述
  - `docs/overview/project_overview.md`
- 架构
  - `docs/architecture/pipeline.md`
  - `docs/architecture/directory_structure.md`
- 使用
  - `docs/usage/quickstart.md`
  - `docs/usage/interaction_preprocessing.md`
  - `docs/usage/navigation_preprocessing.md`
  - `docs/usage/export_scenes.md`
- 模块说明
  - `docs/modules/data_clean.md`
  - `docs/modules/usd_physics.md`
  - `docs/modules/preprocess_interaction.md`
  - `docs/modules/preprocess_navigation.md`
- 引用与依赖
  - `docs/references/assets_and_materials.md`
  - `docs/references/dependencies.md`
- 规格说明
  - `docs/specs/dataset_structure_interpretation.md`
  - `docs/specs/structure_comparison.md`
- 运维
  - `docs/operations/windows_notes.md`
  - `docs/operations/environment_setup.md`
- 示例与答疑
  - `docs/examples/workflow_examples.md`
  - `docs/faq/faq.md`

## 快速链接
- 仓库：`https://github.com/jandan138/usd-scene-physics-prep`
- 发布页（启用 GitHub Pages 后）：`https://jandan138.github.io/usd-scene-physics-prep/`

## 使用提要
- 拆分：运行 `python clean_data.py` 生成 `target/`。
- 交互：修改并运行 `set_physics/preprocess_for_interaction.py` 生成 `start_result_dynamic.usd`。
- 导航：修改并运行 `set_physics/preprocess_for_navigation.py` 生成 `start_result_navigation.usd`。
- 导出：运行 `set_physics/get_all_references.py` 与 `set_physics/export_scene.py` 进行打包。
