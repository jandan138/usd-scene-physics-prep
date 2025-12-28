# SimReady 脚本功能与关系分析报告

经过对代码的深入审查，我确认这三个脚本分别服务于不同的应用场景和工作流阶段。以下是详细的功能划分、实现确认及合理性评估。

## 1. 功能划分确认

| 脚本路径 | 角色定位 | 核心功能 | 适用场景 |
| :--- | :--- | :--- | :--- |
| `set_physics/preprocess_for_interaction.py` | **核心流水线组件** | 批量处理**已标准化**的场景。依赖 `clean_data.py` 的输出结构 (`/Root/Meshes/...`)，实现刚体/碰撞体绑定。 | 项目内部标准流水线（Batch Pipeline）。通常在数据清洗后运行。 |
| `scripts/prep_interaction_root_scene.py` | **外部数据适配器** | 处理**非标准结构**（如 SimBench/GRScene 的 `/root` 结构）的场景。包含独立的物理绑定逻辑，不依赖项目标准库。 | 处理外部数据集、非标准 USD 文件（One-off Adapter）。 |
| `set_physics/simready.py` | **新一代 CLI 工具** | **一站式**处理工具。整合了“数据清洗”和“物理预处理”两个步骤。支持 `interaction` 和 `navigation` 两种模式。 | 用户通过命令行快速生成 SimReady 资产（Unified CLI）。意在简化操作流程。 |

## 2. 具体实现检查

### A. `set_physics/preprocess_for_interaction.py`
*   **实现**: 依赖 `pxr_utils.usd_physics` 库。逻辑紧密耦合于项目的标准目录结构（遍历 `/Root/Meshes`）。
*   **状态**: 是目前文档中主要推荐的批处理脚本。

### B. `scripts/prep_interaction_root_scene.py`
*   **实现**: **完全独立实现**。它没有复用 `pxr_utils` 中的物理函数，而是内置了一套简化版的物理绑定逻辑（`_set_rigidbody`, `_set_collider_with_approx` 等）。
*   **合理性**: 合理。因为它需要处理的场景结构（`/root`）与主项目完全不同，独立实现避免了为了适配它而污染核心库代码。

### C. `set_physics/simready.py`
*   **实现**: **混合实现**。它调用了 `pxr_utils.data_clean.parse_scene` 进行数据清洗，但**重新实现**了物理绑定逻辑（与 `preprocess_for_interaction.py` 高度重复，但被封装在函数内部）。
*   **合理性**: 逻辑上合理（提供了一站式体验），但工程上存在**代码冗余**。物理绑定逻辑在 `preprocess_for_interaction.py` 和 `simready.py` 中被重复编写，维护时容易造成不一致。

## 3. 调用关系与执行顺序分析

*   **调用关系**:
    *   三者互不调用，属于**并行**的入口脚本。
    *   `simready.py` 和 `preprocess_for_interaction.py` 共享底层的 `data_clean` 逻辑（前者直接调用，后者依赖其输出）。
*   **执行顺序**:
    *   **路径 1 (标准批处理)**: `clean_data.py` (清洗) -> `preprocess_for_interaction.py` (物理)。
    *   **路径 2 (快速工具)**: 直接运行 `simready.py` (内部按顺序执行：清洗 -> 物理)。
    *   **路径 3 (外部数据)**: 直接运行 `prep_interaction_root_scene.py` (仅物理，针对特殊结构)。

## 4. 结论与建议

**确认结果**: 三个脚本的功能划分清晰，分别对应**标准批处理**、**特殊数据适配**和**便捷工具化**三种需求。

**潜在风险**:
*   `simready.py` 与 `preprocess_for_interaction.py` 存在大量重复的物理绑定代码。如果修改了物理规则（例如改变碰撞体近似参数），需要同时修改这两处。

**下一步建议**:
当前实现符合功能划分，可以保留现状。若未来需要降低维护成本，建议将 `simready.py` 中的物理逻辑重构为调用 `pxr_utils.usd_physics` 或 `preprocess_for_interaction.py` 中的共享函数。