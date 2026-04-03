---
title: Team 工具通俗讲解
code_reference: N/A - 指南文档
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: Claude
status: complete
---

# Team 工具通俗讲解

## 一句话总结

- **TeamCreate** = 开公司（创建一个团队空间）
- **TaskCreate** = 派工单（给团队分配任务）
- **Agent** = 招员工（启动一个 AI 助手）
- **SendMessage** = 发消息（跟员工沟通）

---

## 详细解释

### 1. TeamCreate - 开公司

**是什么**：创建一个团队空间，包含任务列表和成员管理

**比喻**：就像注册一家公司，有了公司名、办公地点、组织架构

**什么时候用**：
- 任务需要多个 AI 协作时
- 想并行处理多个子任务时
- 需要跟踪复杂项目进度时

**实际例子**：
```json
{
  "team_name": "codex-mcp-investigation",
  "description": "调研 Codex MCP 问题"
}
```

**创建后会得到**：
- `~/.claude/teams/<team-name>/config.json` - 团队配置
- `~/.claude/tasks/<team-name>/` - 任务列表目录

---

### 2. TaskCreate - 派工单

**是什么**：在团队里创建一个具体任务

**比喻**：就像给工人发工单，写明要做什么、什么标准、交付什么

**什么时候用**：
- 把大任务拆成小任务时
- 给不同 Agent 分配具体工作时
- 需要跟踪任务状态时

**实际例子**：
```json
{
  "subject": "调研 MCP 配置",
  "description": "查找 ~/.codex/config.toml 中的 MCP 设置...",
  "status": "pending"  // pending / in_progress / completed
}
```

**任务状态流转**：
```
pending（待办） → in_progress（进行中） → completed（完成）
```

---

### 3. Agent - 招员工

**是什么**：启动一个 AI 子代理，加入团队干活

**比喻**：就像招聘员工，给他职位描述、工作范围、汇报对象

**什么时候用**：
- 需要并行处理多个独立任务时
- 需要专门技能的助手时（如探索型、执行型、审查型）
- 不想阻塞主会话时

**实际例子**：
```json
{
  "name": "mcp-config-researcher",
  "prompt": "调研 Codex MCP 自动加载机制...",
  "team_name": "codex-mcp-investigation"
}
```

**Agent 类型选择**：
| 类型 | 用途 | 比喻 |
|------|------|------|
| `Explore` | 查代码、找文件 | 侦查员 |
| `Plan` | 做架构设计 | 架构师 |
| `general-purpose` | 写代码、改文件 | 程序员 |
| `claude-code-guide` | 解答 Claude Code 问题 | 技术支持 |

---

### 4. SendMessage - 发消息

**是什么**：给 Agent 发送消息，或者广播给所有成员

**比喻**：就像发微信/钉钉消息给同事，可以私聊也可以群发

**什么时候用**：
- 给 Agent 分配新任务时
- 询问 Agent 进度时
- 通知所有 Agent 项目变更时

**实际例子**：
```json
// 私聊某个 Agent
{
  "to": "mcp-config-researcher",
  "message": "进度如何？"
}

// 广播给所有人
{
  "to": "*",
  "message": "项目需求变更，请注意..."
}
```

---

## 工具之间的关系

```
TeamCreate (开公司)
    │
    ├── TaskCreate (派工单 #1: 调研 A)
    │       │
    │       └── Agent (招员工 A 负责 #1)
    │               │
    │               └── SendMessage (跟 A 沟通)
    │
    ├── TaskCreate (派工单 #2: 调研 B)
    │       │
    │       └── Agent (招员工 B 负责 #2)
    │
    └── TaskCreate (派工单 #3: 汇总)
            │
            └── 等待 #1 #2 完成后执行
```

---

## 完整工作流示例

**场景**：调研一个复杂问题

```python
# 1. 开公司
TeamCreate(name="调研团队")

# 2. 拆任务
TaskCreate(subject="调研 A 部分")
TaskCreate(subject="调研 B 部分")
TaskCreate(subject="汇总报告", blockedBy=["1", "2"])

# 3. 招员工
Agent(name="研究员-A", prompt="调研 A...", team_name="调研团队")
Agent(name="研究员-B", prompt="调研 B...", team_name="调研团队")

# 4. 等他们完成...
SendMessage(to="研究员-A", message="进度如何？")

# 5. 汇总结果
# ...

# 6. 解散团队
TeamDelete()
```

---

## 常见误区

| 误区 | 正确理解 |
|------|----------|
| ❌ Agent 会自动开始工作 | ✅ Agent 启动后空闲，需要分配任务 |
| ❌ TaskCreate 会自动分配给 Agent | ✅ TaskCreate 只是创建工单，需要手动分配 owner |
| ❌ TeamDelete 会自动关闭 Agent | ✅ 必须先让 Agent 退出，再删除团队 |
| ❌ SendMessage 会阻塞等待回复 | ✅ 发送后立刻返回，消息异步送达 |

---

## 最佳实践

### 1. 任务拆分原则
- 每个任务应该是**独立可完成**的
- 任务之间可以用 `blockedBy` 指定依赖
- 任务描述要具体，包含交付标准

### 2. Agent 使用原则
- 一个 Agent 一次只做**一件事**
- 复杂任务拆给多个 Agent **并行处理**
- 探索型任务用 `Explore` agent，编码用 `general-purpose`

### 3. 沟通原则
- 启动 Agent 时就把要求说清楚（写在 prompt 里）
- 中途沟通用 SendMessage
- 完成后 Agent 会发 `idle_notification`，这时候可以收工

### 4. 收尾原则
```python
# 正确顺序：
1. SendMessage(agent, "任务完成，请关闭")  # 或发 shutdown_request
2. 等待 Agent 变成 idle（收到 idle_notification）
3. TeamDelete()  # 清理团队
```

---

## 快速决策表

| 场景 | 用什么工具 |
|------|-----------|
| 需要多个 AI 协作 | TeamCreate |
| 只是简单问个问题 | 直接用当前 Claude，不需要团队 |
| 要并行处理 3 个文件 | TeamCreate + 3x Agent |
| 跟踪任务进度 | TaskCreate + TaskList |
| 给 AI 分配具体工作 | Agent 的 prompt 参数 |
| 询问进度/分配新活 | SendMessage |
| 项目结束清理 | TeamDelete |
