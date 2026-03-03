# 文件归属表（File Ownership Map）

> 同一文件在一次 Agent Teams 会话中只能由一个 agent 修改。
> 若多个 agent 需触及同一文件，应串行而非并行。
> 本表由 team lead 手动维护；agent 不得自动修改本文件。

---

## 核心 Pipeline 代码

| 路径 | 负责 Agent | 说明 |
|---|---|---|
| `set_physics/pxr_utils/data_clean.py` | feature-implementer, bug-fixer | **HIGH RISK** — 核心拆分逻辑；改动频繁，需串行 |
| `set_physics/pxr_utils/usd_physics.py` | feature-implementer, bug-fixer | **HIGH RISK** — 所有物理脚本共享；多功能时串行 |
| `set_physics/preprocess_for_interaction.py` | feature-implementer, bug-fixer | **HIGH RISK** — 所有交互物理改动入口 |
| `set_physics/preprocess_for_navigation.py` | feature-implementer, bug-fixer | MEDIUM — 导航物理处理 |
| `set_physics/simready.py` | feature-implementer, code-refactorer | MEDIUM — 一键 CLI |
| `set_physics/pxr_utils/read_info.py` | feature-implementer, bug-fixer | LOW |
| `set_physics/pxr_utils/rendering.py` | feature-implementer | LOW |
| `clean_data.py` | feature-implementer, bug-fixer | LOW — 入口薄层；实际逻辑在 data_clean.py |

## Specs Normalizer

| 路径 | 负责 Agent | 说明 |
|---|---|---|
| `specs_normalizer/normalize.py` | feature-implementer, code-refactorer | MEDIUM — 导出编排器 |
| `specs_normalizer/exporters/materials.py` | feature-implementer, bug-fixer | LOW |
| `specs_normalizer/exporters/assets.py` | feature-implementer, bug-fixer | LOW |
| `specs_normalizer/exporters/scenes.py` | feature-implementer, bug-fixer | LOW |
| `specs_normalizer/validators/structure.py` | feature-implementer, bug-fixer | LOW |
| `specs_normalizer/check.py` | feature-implementer, bug-fixer | LOW |

## Scripts（工具脚本）

| 路径 | 负责 Agent | 说明 |
|---|---|---|
| `scripts/prep_interaction_root_scene.py` | feature-implementer, bug-fixer | MEDIUM — 外部数据集适配器 |
| `scripts/collect_textures_to_local_textures.py` | feature-implementer, bug-fixer | LOW |
| `scripts/oneoff_*.py` | feature-implementer, bug-fixer | LOW — 各自独立，可并行 |
| `scripts/doc_manager.py` | feature-implementer, code-refactorer | LOW |
| `scripts/isaac_python.sh` | feature-implementer | LOW — 环境包装器 |
| `scripts/run_phase2_full.sh` | feature-implementer | LOW |
| `scripts/run_phase3_full.sh` | feature-implementer | LOW |

## set_physics Tools

| 路径 | 负责 Agent | 说明 |
|---|---|---|
| `set_physics/export_scene.py` | feature-implementer, bug-fixer | LOW |
| `set_physics/get_all_references.py` | feature-implementer, bug-fixer | LOW |
| `set_physics/tools/` | feature-implementer | LOW — Isaac Sim 渲染/缩略图工具 |

## 文档

| 路径 | 负责 Agent | 说明 |
|---|---|---|
| `docs/` | docs-writer | 所有 .md 文档；其他 agent 只读 |
| `docs/agent-team-playbook.md` | team lead / 手动 | 本系统的设计手册，不由 agent 修改 |

## 基础设施（仅 team lead 或手动操作）

| 路径 | 负责 Agent | 说明 |
|---|---|---|
| `CLAUDE.md` | team lead / 手动 | 需人工审核，不由 agent 自动修改 |
| `.claude/agents/` | team lead / 手动 | Agent 定义文件 |
| `.claude/file-ownership.md` | team lead / 手动 | 本文件 |
| `.claude/settings.local.json` | team lead / 手动 | Agent Teams 配置 |

---

## 高冲突风险文件（High-Conflict-Risk Files）

| 文件 | 风险原因 | 推荐处理方式 |
|---|---|---|
| `set_physics/pxr_utils/data_clean.py` | 核心逻辑，几乎所有新功能都会修改 | 多功能时串行；同一 team session 只允许一个执行层 agent 修改 |
| `set_physics/pxr_utils/usd_physics.py` | 所有物理脚本共用；常被多功能同时依赖 | 同上 |
| `set_physics/preprocess_for_interaction.py` | 所有交互物理改动的集中点 | 同上 |
| `specs_normalizer/normalize.py` | 所有导出器的编排入口 | 多导出格式并行时串行合并 |

---

## 规则摘要

1. 各 agent 改动路径**不重叠** → 可并行调度
2. 两个 agent 都需改动**同一文件** → 必须串行
3. `version-commit-agent` 先合并改动范围最小、最独立的分支
4. Agent 改动超出本表所列范围 → `version-commit-agent` 须在合并前报告 team lead
5. `oneoff_*.py` 各自独立，可并行修改；但新增 oneoff 脚本应由一个 agent 完成后再调度下一个
