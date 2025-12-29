# 更新 README.md 使用提要

根据您的需求，我将在 `README.md` 的“使用提要”部分增加三种 SimReady 实现的适用场景及链接说明。

## 计划变更内容

我将把“使用提要”重构为如下结构：

### 1. 核心工作流 (Standard Pipeline)
保留原有的分步流程（拆分 -> 预处理 -> 导出），并明确其为**“标准批处理”**。

### 2. SimReady 方案选择 (SimReady Options)
新增一个子模块，详细列出三种方案的对比：

*   **方案 A：标准流水线 (Pipeline)**
    *   **适用场景**: 项目内部标准化数据，需要批量处理和精细控制。
    *   **入口**: `set_physics/preprocess_for_interaction.py`
    *   **文档**: [`docs/usage/interaction_preprocessing.md`](docs/usage/interaction_preprocessing.md)

*   **方案 B：一键工具 (CLI)**
    *   **适用场景**: 快速上手，无需关心中间步骤，直接从原始 USD 生成结果。
    *   **入口**: `set_physics/simready.py`
    *   **文档**: [`docs/usage/simready.md`](docs/usage/simready.md)

*   **方案 C：外部适配器 (External Adapter)**
    *   **适用场景**: 处理非标准结构（如 SimBench/GRScene 的 `/root` 结构）的外部数据集。
    *   **入口**: `scripts/prep_interaction_root_scene.py`
    *   **文档**: [`docs/usage/prep_interaction_root_scene.md`](docs/usage/prep_interaction_root_scene.md)

## 执行计划
1.  **编辑文件**: 修改 `D:\my_dev\usd-scene-physics-prep\README.md`。
2.  **提交更改**: 提交本次修改。