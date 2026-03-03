# Claude Code Agent Team 系统设计与复现手册

> **目的**：本文档是一份可执行的 playbook。另一个 Claude 实例读完后，能在新项目中从零搭建出同等严谨的 Agent Team 系统。
>
> **适用范围**：使用 Claude Code（claude.ai/code CLI）的任意软件项目。
>
> **语言约定**：中文为主；`front matter`、`isolation: worktree`、`MEMORY.md` 等技术词汇保留英文原词。

---

## 目录

1. [系统架构概览](#1-系统架构概览)
2. [Agent Front Matter 规范](#2-agent-front-matter-规范)
3. [Agent Body 设计规范](#3-agent-body-设计规范)
4. [支撑基础设施](#4-支撑基础设施)
5. [从零搭建 Checklist](#5-从零搭建-checklist)

---

## 1. 系统架构概览

### 1.1 Team Lead 不是一个 agent 文件

Claude Code Agent Teams 中，**team lead = 用户正在对话的主 Claude 会话本身**。

- 主 session 天然拥有全部协调工具：`TeamCreate`、`TaskCreate`、`TaskUpdate`、`TaskList`、`SendMessage`
- `.claude/agents/` 目录里定义的全是 **teammates**（被调度的执行者），不是 lead
- 不要新建 `team-lead.md`——lead 角色由主对话自动承担

### 1.2 三层分工模型

```
规划层（Plan）          执行层（Execute）         保障层（Assure）
──────────────         ─────────────────         ─────────────────
architecture-planner   feature-implementer        docs-writer
feature-designer       code-refactorer            asset-validator
                       bug-fixer                  isaac-sim-headless-tester
                                                  isaac-sim-full-tester
                                                  version-commit-agent
```

- **规划层**：只输出设计文档和架构建议，不写源码
- **执行层**：写源码、重构、修 bug，**必须启用 `isolation: worktree`**
- **保障层**：验证、文档、提交——不并行写源码，无需 worktree 隔离

### 1.3 启用 Agent Teams

在项目根目录创建 `.claude/settings.local.json`：

```json
{
  "env": {
    "CLAUDE_CODE_EXPERIMENTAL_AGENT_TEAMS": "1"
  },
  "permissions": {
    "allow": [
      "Bash(git:*)",
      "Bash(git push)",
      "WebSearch"
    ]
  }
}
```

> `permissions.allow` 根据项目实际需求调整。`CLAUDE_CODE_EXPERIMENTAL_AGENT_TEAMS` 是必须项。

---

## 2. Agent Front Matter 规范

每个 `.claude/agents/<name>.md` 文件的开头是 YAML front matter，控制 agent 的运行时行为。

### 2.1 完整字段说明

```yaml
---
name: my-agent-name          # kebab-case，必须与文件名一致（不含 .md）
description: "触发描述..."   # 主 session 据此决定何时调用此 agent（重要！）
model: sonnet                # sonnet（默认）| opus（复杂推理）| haiku（轻量）
color: blue                  # 可选：UI 颜色标签，便于区分角色
memory: project              # 必填：使 agent 能读取项目级 CLAUDE.md
isolation: worktree          # 可选：只有写源码的 agent 才加
---
```

### 2.2 `isolation: worktree` 决策矩阵

这是整个系统中**最重要的单条配置决策**。

| Agent 角色 | isolation | 理由 |
|---|---|---|
| 写源码（implementer / refactorer / bug-fixer）| **✅ 必须** | 并行运行时防止文件冲突 |
| 写设计/架构文档（designer / planner）| ❌ 不需要 | docs/ 与 src/ 无冲突 |
| 写项目文档（docs-writer）| ❌ 不需要 | 通常单独运行，无并行冲突 |
| 只读验证（explorer / validator / tester）| ❌ 不需要 | 不修改任何文件 |
| 合并所有 worktree（version-commit-agent）| **❌ 禁止** | 必须访问主仓库的全部分支 |

**实际例子**（本项目三个有 `isolation: worktree` 的 agent）：

```yaml
# feature-implementer.md
name: feature-implementer
model: sonnet
memory: project
isolation: worktree
```

```yaml
# code-refactorer.md
name: code-refactorer
model: sonnet
memory: project
isolation: worktree
```

```yaml
# bug-fixer.md
name: bug-fixer
model: sonnet
memory: project
isolation: worktree
```

### 2.3 `description` 字段写法

`description` 决定主 session 在什么情况下自动调用这个 agent。写法要求：

1. 一句话说明**触发场景**（用户说什么时调用）
2. 附 2-3 个 `<example>` 块，包含 `Context`、用户说的话、助手的响应、`<commentary>`
3. 必须明确写"何时用"和"何时不用"

```yaml
description: "Use this agent when fixing bugs based on requirements or design
documentation. This includes scenarios where a bug report, requirement spec, or
design doc is provided...

<example>
Context: The user has a bug report stating mesh simplification produces incorrect UVs.
user: \"UVs are getting scrambled. According to the design doc, face-varying UVs should be preserved.\"
assistant: \"I'll launch the bug-fixer agent to diagnose and fix this issue.\"
<commentary>
Since the user has a bug tied to a documented requirement, use the bug-fixer agent.
</commentary>
</example>"
```

---

## 3. Agent Body 设计规范

Agent 文件的 body（front matter 之后的正文）是 agent 的系统提示。
一个高质量的 agent body 必须包含以下 5 个章节。

### 3.1 章节一：角色声明

**格式**：一段话，说明"我是谁、我做什么、我不做什么"。

```markdown
You are an expert software engineer and bug analyst specializing in diagnosing
and fixing defects with surgical precision, always grounding your fixes in the
documented requirements, design documents, and architectural intent of the project.
```

**关键点**：明确边界。例如 docs-writer 的角色声明里要有 `You never modify source code`。

### 3.2 章节二：项目上下文

**格式**：列出本项目的入口文件、核心模块、运行命令。这部分**需要针对每个新项目替换**。

```markdown
## Project Context

You are working within the **ConvertAsset** project — a USD asset conversion toolkit.

- Entry point: `main.py` → `convert_asset/cli.py`
- USD bindings from Isaac Sim's Python environment (`pxr`)
- Core modules: `no_mdl/` (MDL→UsdPreviewSurface), `mesh/` (simplification), `glb/` (USD→GLB)
- Design docs in `docs/`; architecture in `CLAUDE.md`
- Chinese inline comments throughout — read and respect them
- Run commands via `./scripts/isaac_python.sh ./main.py <subcommand> [args]`
```

### 3.3 章节三：方法论（分阶段工作流）

**格式**：将工作拆成 3-5 个阶段，每阶段有具体行动清单。

以 `bug-fixer` 为例：

```markdown
## Your Bug-Fixing Methodology

### 1. Understand Before Acting
- Read the relevant requirement or design doc describing *intended* behavior
- Identify the delta between documented intent and observed behavior
- Trace the code path from entry point to the defective site

### 2. Root Cause Analysis
- Locate the minimal code region responsible for the bug
- Check cycle detection (`done` dict, `in_stack` set) and lazy import patterns

### 3. Design-Aligned Fix
- Implement the fix that restores documented behavior with minimal change
- Preserve existing patterns; respect `config.py` configuration system

### 4. Verification Plan
- Specify exact commands to reproduce the original bug
- Specify commands to verify the fix

### 5. Impact Assessment
- State which modules are modified and why
- Confirm no regressions in adjacent functionality
```

### 3.4 章节四：行为约束（NEVER / ALWAYS 清单）

**格式**：用 `**Never**` 和 `**Always**` 列出明确禁令和强制要求。

```markdown
## Behavioral Constraints

- **Never** flatten USD composition (references, payloads, variants must be preserved)
- **Never** move `pxr` imports to module top-level — lazy imports are intentional
- **Always** check `CLAUDE.md` and `docs/` before concluding what the correct behavior is
- **Always** prefer the smallest correct fix over a large refactor
- If requirements are ambiguous, state the ambiguity and propose the most conservative interpretation
```

这个清单防止 agent 在自动执行时"创造性地越界"。

### 3.5 章节五：持久化记忆（Persistent Agent Memory）

每个 agent 末尾必须有这一章节，指定记忆目录路径和规范。

```markdown
# Persistent Agent Memory

You have a persistent memory directory at
`/path/to/project/.claude/agent-memory/bug-fixer/`.
Its contents persist across conversations.

Guidelines:
- `MEMORY.md` is always loaded into your system prompt (lines after 200 truncated)
- Create topic files (`debugging.md`, `patterns.md`) for details, link from MEMORY.md
- Organize memory semantically by topic, not chronologically

What to save:
- Stable patterns confirmed across multiple interactions
- Key architectural decisions and important file paths
- Solutions to recurring problems

What NOT to save:
- Session-specific context (current task details, temporary state)
- Speculative conclusions from reading a single file
- Anything that duplicates CLAUDE.md instructions

## MEMORY.md

Your MEMORY.md is currently empty. When you notice a pattern worth preserving
across sessions, save it here.
```

> **注意**：路径使用绝对路径（项目绝对路径），这样 agent 无论在哪个工作目录下运行都能找到记忆文件。

---

## 4. 支撑基础设施

除了 agent 文件本身，还需要 3 个支撑文件让系统真正严谨运作。

### 4.1 文件归属表（`file-ownership.md`）

**位置**：`.claude/file-ownership.md`

**作用**：防止并行 agent 修改同一文件导致冲突；`version-commit-agent` 合并时依此规划顺序。

**格式模板**：

```markdown
# 文件归属表（File Ownership Map）

> 同一文件在一次 Agent Teams 会话中只能由一个 agent 修改。
> 若多个 agent 需触及同一文件，应串行而非并行。

## 代码模块归属

| 路径 | 负责 Agent | 说明 |
|---|---|---|
| `src/core/` | feature-implementer, code-refactorer | 核心逻辑 |
| `src/api/` | feature-implementer | API 层；新端点均在此注册 |
| `src/utils/` | feature-implementer, bug-fixer | 工具函数 |
| `src/cli.py` | feature-implementer | **高冲突风险**：多功能时串行处理 |

## 文档归属

| 路径 | 负责 Agent | 说明 |
|---|---|---|
| `docs/` | docs-writer | 所有文档；其他 agent 只读 |
| `CLAUDE.md` | team lead / 手动 | 需人工审核，不由 agent 自动修改 |

## 基础设施（仅 team lead 或手动操作）

| 路径 | 负责 Agent | 说明 |
|---|---|---|
| `.claude/agents/` | team lead / 手动 | Agent 定义文件 |
| `.claude/file-ownership.md` | team lead / 手动 | 本文件 |

## 高冲突风险文件

| 文件 | 风险原因 | 推荐处理方式 |
|---|---|---|
| `src/cli.py` | 所有新子命令均在此注册 | 多功能时串行，或 team lead 统一处理 |
| `src/config.py` | 全局配置，各模块都引用 | 同一 team 只允许一个 agent 修改 |

## 规则摘要

1. 各 agent 改动不重叠的路径 → 可并行
2. 两个 agent 都需要改同一文件 → 必须串行
3. version-commit-agent 先合并改动范围最小、最独立的分支
4. Agent 改动超出本表所列范围 → version-commit-agent 须在合并前报告 team lead
```

### 4.2 Agent 记忆目录

**位置**：`.claude/agent-memory/<agent-name>/`

为每个 agent 创建独立子目录：

```bash
mkdir -p .claude/agent-memory/feature-implementer
mkdir -p .claude/agent-memory/code-refactorer
mkdir -p .claude/agent-memory/bug-fixer
mkdir -p .claude/agent-memory/docs-writer
# ... 以此类推
```

各目录初始为空；agent 在工作过程中自行创建和更新 `MEMORY.md`。

### 4.3 version-commit-agent 的 Worktree Merge Coordinator

`version-commit-agent` 有一个特殊职责：合并所有 `isolation: worktree` agent 完成后留下的分支。

在该 agent 的 body 中必须包含以下章节：

```markdown
## Worktree Merge Coordinator（Agent Teams 模式）

带 `isolation: worktree` 的 agent 各自在独立分支完成工作后，由你负责集成。
worktree **创建**是自动的，你只负责**合并和清理**。

### 合并协议

**Step 1：收集已完成的 worktree 分支**
```bash
git branch | grep "worktree-"
git worktree list
```

**Step 2：检查改动文件，评估冲突风险**
```bash
git diff --name-only main..worktree-<name>
```
对照 `.claude/file-ownership.md` 确认改动在 agent 职责范围内。
改动文件**不重叠**的分支可并行合并；有重叠的必须串行。

**Step 3：按冲突风险从低到高合并**
```bash
# 无冲突预期时
git merge --no-ff worktree-<name> -m "merge: integrate <agent-name> changes"

# 若出现冲突
git merge --abort   # 回滚，向 team lead 报告
```

**Step 4：清理 worktree 和分支**
```bash
git worktree remove .claude/worktrees/<name>
git branch -d worktree-<name>
```

**Step 5：最终 push**
```bash
git push origin main
```

### 冲突处理原则

- **不要自动解决业务逻辑冲突**——报告 team lead，附冲突文件列表和 diff 摘要
- 格式/空白冲突（trailing whitespace、import 顺序）可直接解决
- 入口注册文件（如 `cli.py`）是高风险文件，冲突时优先串行重新执行
```

---

## 5. 从零搭建 Checklist

将这份 checklist 交给一个新项目的 Claude，它可以逐步执行完成整套系统搭建。

### Step 1：启用 Agent Teams

```bash
mkdir -p .claude
```

创建 `.claude/settings.local.json`：

```json
{
  "env": {
    "CLAUDE_CODE_EXPERIMENTAL_AGENT_TEAMS": "1"
  },
  "permissions": {
    "allow": [
      "Bash(git:*)",
      "Bash(git push)",
      "WebSearch"
    ]
  }
}
```

### Step 2：分析项目，确定 Agent 清单

根据项目性质，从下表选择需要的 agent：

| Agent | 何时需要 |
|---|---|
| `codebase-explorer` | 几乎所有项目都需要，用于代码库探索 |
| `feature-designer` | 项目有复杂新功能设计需求时 |
| `architecture-planner` | 项目有重构或架构演进需求时 |
| `feature-implementer` | 需要实现新功能时（必须加 `isolation: worktree`）|
| `code-refactorer` | 需要重构现有代码时（必须加 `isolation: worktree`）|
| `bug-fixer` | 需要修复 bug 时（必须加 `isolation: worktree`）|
| `docs-writer` | 项目有文档维护需求时 |
| `asset-validator` | 项目有特定格式资产（USD、JSON schema 等）需验证时 |
| `version-commit-agent` | 使用 Agent Teams 时必须有，负责合并所有 worktree |
| 领域专用 tester | 项目需要特定环境测试时（如 Isaac Sim） |

### Step 3：创建 Agent 文件

```bash
mkdir -p .claude/agents
```

为每个 agent 创建 `.claude/agents/<name>.md`，遵循第 2、3 节规范。

**最小可用模板**：

```markdown
---
name: feature-implementer
description: "Use this agent when implementing new features..."
model: sonnet
memory: project
isolation: worktree
---

You are a senior software engineer specializing in implementing features.

## Project Context

- Entry point: `main.py`
- Key modules: [列出项目核心模块]
- Run commands: [列出项目运行命令]

## Implementation Workflow

### Phase 1: Requirements Analysis
...

### Phase 2: Design
...

### Phase 3: Implementation
...

## Behavioral Constraints

- **Never** [列出禁止行为]
- **Always** [列出强制要求]

# Persistent Agent Memory

You have a persistent memory directory at
`/absolute/path/to/project/.claude/agent-memory/feature-implementer/`.

## MEMORY.md

Currently empty.
```

### Step 4：创建文件归属表

```bash
# 创建 .claude/file-ownership.md，按第 4.1 节模板填写
```

关键决策：
- 识别项目中的"高冲突风险文件"（通常是：入口注册文件、全局 config、共享工具函数）
- 把这些文件标注为单 agent 独占或串行处理

### Step 5：创建记忆目录

```bash
for agent in feature-implementer code-refactorer bug-fixer docs-writer \
             codebase-explorer architecture-planner feature-designer \
             version-commit-agent; do
  mkdir -p .claude/agent-memory/$agent
done
```

### Step 6：配置完整性自查

运行以下检查，确认系统配置无遗漏：

```bash
# 检查所有 agent 文件是否存在
ls .claude/agents/

# 检查 settings.local.json 是否包含 AGENT_TEAMS 配置
grep "EXPERIMENTAL_AGENT_TEAMS" .claude/settings.local.json

# 检查文件归属表是否存在
ls .claude/file-ownership.md

# 检查记忆目录是否为每个 agent 创建
ls .claude/agent-memory/
```

**配置一致性检查清单**：

- [ ] 每个写源码的 agent（implementer / refactorer / bug-fixer）都有 `isolation: worktree`
- [ ] `version-commit-agent` **没有** `isolation: worktree`
- [ ] 所有 agent 都有 `memory: project`
- [ ] `.claude/file-ownership.md` 覆盖了所有高冲突风险文件
- [ ] 每个 agent 的 body 末尾有 `Persistent Agent Memory` 章节，路径为绝对路径
- [ ] `version-commit-agent` 的 body 包含 Worktree Merge Coordinator 章节

### Step 7：提交到版本控制

```bash
git add .claude/agents/ .claude/file-ownership.md .claude/settings.local.json
git commit -m "chore(agents): add Claude Code Agent Team system"
```

> **注意**：`.claude/agent-memory/` 目录可以检入（共享跨会话记忆），也可以加入 `.gitignore`（本地独立记忆）。按团队协作需求决定。

---

## 附录：Agent 设计常见错误

| 错误 | 后果 | 修正方法 |
|---|---|---|
| 写源码的 agent 缺少 `isolation: worktree` | 并行运行时文件冲突 | 在 front matter 加 `isolation: worktree` |
| `version-commit-agent` 加了 `isolation: worktree` | 无法访问其他 worktree 分支，合并失败 | 移除该字段 |
| agent body 缺少项目上下文 | Agent 不了解项目架构，产出不符合惯例 | 在 body 开头加 Project Context 章节 |
| 记忆目录路径用相对路径 | Agent 在不同工作目录下找不到记忆 | 改为绝对路径 |
| `file-ownership.md` 不覆盖高冲突文件 | 合并时出现意外冲突，需手动解决 | 识别并显式列出所有高冲突风险文件 |
| `description` 字段过于笼统 | 主 session 不知何时调用，或频繁误调用 | 加具体触发场景描述和 example 块 |
