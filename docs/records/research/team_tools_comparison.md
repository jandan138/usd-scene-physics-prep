---
title: Team 工具对比分析
code_reference: Claude Code Agent SDK
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: Claude
status: complete
---

# Team 工具对比分析

## 一句话总结

| 工具 | 一句话描述 |
|------|-----------|
| **TeamCreate** | 创建一个**团队容器**，把多个 Agent 组织在一起工作 |
| **TaskCreate** | 在当前会话中创建一个**待办任务**，用于跟踪工作进度 |
| **Agent** | 启动一个**专门的 AI 助手**（子代理）去执行具体任务 |
| **SendMessage** | 给**已经存在的 Agent 或团队**发送消息进行通信 |

---

## 1. TeamCreate - 创建团队

### 是什么？
`TeamCreate` 用于创建一个**团队容器**，将多个 Agent 组织在一起协同工作。它是最高层的组织单位。

### 类比理解
想象你要开一家餐厅：
- **TeamCreate** = 注册成立一家餐厅公司
- 公司本身不直接干活，但提供了一个组织框架

### 使用场景
```python
# 创建一个开发团队，包含前端、后端、测试三个角色
TeamCreate(
    name="web-dev-team",
    agents=["frontend-dev", "backend-dev", "qa-tester"],
    description="负责 Web 应用开发的团队"
)
```

### 关键特点
- **组织性**: 把相关的 Agent 分组管理
- **协作性**: 团队内的 Agent 可以互相通信
- **广播能力**: 可以给整个团队发送消息 (`to: "*"`)
- **生命周期**: 团队创建后持续存在，直到会话结束

---

## 2. TaskCreate - 创建任务

### 是什么？
`TaskCreate` 用于在当前会话中创建一个**待办任务卡片**，用于跟踪工作进度。它只是一个任务追踪工具，不执行任何实际工作。

### 类比理解
继续餐厅的例子：
- **TaskCreate** = 在厨房的白板上写一张任务卡片
- 比如："准备食材"、"清洗餐具"、"制作菜单"
- 卡片本身不会自动完成，需要有人去执行

### 使用场景
```python
# 创建一个任务来跟踪 Bug 修复进度
TaskCreate(
    subject="Fix login authentication bug",
    description="用户报告登录时 401 错误，需要调查并修复"
)

# 创建一个任务来规划调研工作
TaskCreate(
    subject="Research MCP token consumption",
    description="调查 MCP 工具对上下文窗口的影响"
)
```

### 关键特点
- **追踪性**: 记录需要完成的工作
- **状态管理**: 有 pending / in_progress / completed 等状态
- **非执行性**: 创建任务 ≠ 执行任务，任务只是记录
- **依赖关系**: 可以设置任务之间的依赖（A 阻塞 B）

### 常见误区
❌ **错误理解**: 创建任务后工作会自动进行  
✅ **正确理解**: 任务只是待办清单，需要手动或指派 Agent 去执行

---

## 3. Agent - 启动 Agent

### 是什么？
`Agent` 用于启动一个**专门的 AI 子代理**，让它独立执行具体的任务。这是实际干活的主力。

### 类比理解
餐厅例子：
- **Agent** = 雇佣一个具体的员工
- 比如：雇佣一个厨师、一个服务员、一个采购员
- 每个员工有自己的专业技能和职责

### 使用场景
```python
# 启动一个专门做代码审查的 Agent
Agent(
    name="code-reviewer",
    prompt="请审查以下代码的质量和安全性...",
    subagent_type="Review"
)

# 启动一个专门调研的 Agent
Agent(
    name="researcher",
    prompt="调研 MCP 协议的工作原理...",
    subagent_type="Explore"
)
```

### 关键特点
- **独立性**: Agent 在自己的上下文中运行，不污染主会话
- **专业性**: 可以指定不同类型的 Agent（Explore/Review/Code/等）
- **并行性**: 多个 Agent 可以同时运行
- **隔离性**: Agent 的错误不会导致主会话崩溃

### Agent 类型
| 类型 | 用途 |
|------|------|
| `Explore` | 代码库探索、研究调研 |
| `Review` | 代码审查、文档评审 |
| `Code` | 编写代码、实现功能 |
| `Test` | 编写测试、验证功能 |
| `Fix` | 修复 Bug、解决问题 |

---

## 4. SendMessage - 发送消息

### 是什么？
`SendMessage` 用于**给已经存在的 Agent 或团队发送消息**，进行异步通信。

### 类比理解
餐厅例子：
- **SendMessage** = 给某个员工或部门发消息
- 比如："厨师，3号桌点了一份牛排"、"服务员，5号桌需要加水"

### 使用场景
```python
# 给特定 Agent 发送任务指派
SendMessage(
    to="code-reviewer",
    message="请审查 src/auth.py 文件的登录逻辑",
    summary="指派代码审查任务"
)

# 给团队广播消息
SendMessage(
    to="*",
    message="项目 deadline 提前到周五，请调整优先级",
    summary="通知 deadline 变更"
)

# 给团队负责人汇报
SendMessage(
    to="team-lead",
    message="调研完成，发现 MCP 消耗 72% 上下文",
    summary="汇报调研结果"
)
```

### 关键特点
- **异步性**: 发送后不会等待回复，继续执行
- **定向性**: 可以发给特定 Agent (`to: "name"`) 或广播 (`to: "*"`)
- **通知性**: 适合汇报进度、指派任务、协调工作
- **非阻塞**: 不会暂停当前工作等待响应

---

## 5. 工具之间的关系

### 关系图

```
┌─────────────────────────────────────────────────────────────┐
│                      你的主会话 (You)                         │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐  │
│  │  TaskCreate │  │  TeamCreate │  │  SendMessage(to=...)│  │
│  │  (任务卡片)  │  │  (创建团队)  │  │  (发送消息)          │  │
│  └─────────────┘  └──────┬──────┘  └─────────────────────┘  │
│                          │                                   │
│                          ▼                                   │
│              ┌─────────────────────┐                         │
│              │    web-dev-team     │                         │
│              │    (团队容器)        │                         │
│              └──────────┬──────────┘                         │
│                         │                                    │
│         ┌───────────────┼───────────────┐                    │
│         ▼               ▼               ▼                    │
│  ┌─────────────┐ ┌─────────────┐ ┌─────────────┐            │
│  │   Agent     │ │   Agent     │ │   Agent     │            │
│  │ frontend-dev│ │ backend-dev │ │  qa-tester  │            │
│  │  (子代理)    │ │  (子代理)    │ │  (子代理)    │            │
│  └─────────────┘ └─────────────┘ └─────────────┘            │
└─────────────────────────────────────────────────────────────┘
```

### 调用关系

1. **TeamCreate** 可以包含多个 **Agent**
2. **Agent** 可以独立运行，也可以属于某个团队
3. **SendMessage** 可以向 **Agent** 或 **Team** 发送消息
4. **TaskCreate** 是独立的任务追踪，与其他工具没有直接调用关系

---

## 6. 实际使用中的最佳实践

### 场景 1：复杂项目开发

```python
# 1. 创建任务清单（规划）
TaskCreate(subject="实现用户登录功能")
TaskCreate(subject="编写单元测试")
TaskCreate(subject="代码审查")

# 2. 创建团队（组织）
TeamCreate(name="auth-team", agents=["dev", "reviewer"])

# 3. 启动 Agent 执行（执行）
Agent(name="dev", prompt="实现登录功能...")
Agent(name="reviewer", prompt="审查登录代码...")

# 4. 发送消息协调（通信）
SendMessage(to="reviewer", message="代码已完成，请审查")
```

### 场景 2：调研任务

```python
# 适合单人工作，不需要 TeamCreate

# 1. 创建任务跟踪
TaskCreate(subject="调研 MCP 上下文消耗")

# 2. 启动调研 Agent
Agent(name="researcher", subagent_type="Explore", prompt="调研 MCP...")

# 3. 等待结果后发送汇报
SendMessage(to="team-lead", message="调研完成，发现...")
```

### 场景 3：紧急 Bug 修复

```python
# 快速响应，最小化组织开销

# 直接启动 Fix Agent
Agent(name="fixer", subagent_type="Fix", prompt="修复生产环境 Bug...")

# 同时创建任务跟踪
TaskCreate(subject="修复生产 Bug #123")
```

---

## 7. 决策流程图

```
开始工作
    │
    ├─ 需要多人协作？ ──Yes──► TeamCreate 创建团队
    │                            │
    │                            ▼
    │                      Agent 启动成员
    │                            │
    │                            ▼
    │                      SendMessage 协调
    │
    No
    │
    ├─ 需要跟踪进度？ ──Yes──► TaskCreate 创建任务
    │                            │
    │                            ▼
    │                      Agent 执行工作
    │                            │
    │                            ▼
    │                      TaskUpdate 更新状态
    │
    No
    │
    └─ 直接 Agent 执行工作
```

---

## 8. 常见错误与避免方法

| 错误 | 说明 | 正确做法 |
|------|------|---------|
| 只创建任务不执行 | TaskCreate 后以为工作会自动完成 | TaskCreate 只是记录，需要 Agent 或自己去执行 |
| 创建团队但不启动 Agent | TeamCreate 后团队是空的 | TeamCreate 后要启动 Agent 加入团队 |
| 过度使用广播 | 所有消息都用 `to: "*"` | 只给需要的人发消息，减少干扰 |
| Agent 和 SendMessage 混淆 | 想用 Agent 但用了 SendMessage | Agent 是启动新代理，SendMessage 是给已有代理发消息 |
| 忘记更新任务状态 | TaskCreate 后状态一直是 pending | 及时用 TaskUpdate 更新为 in_progress / completed |

---

## 9. 总结对比表

| 维度 | TeamCreate | TaskCreate | Agent | SendMessage |
|------|-----------|-----------|-------|-------------|
| **本质** | 组织容器 | 任务卡片 | 执行实体 | 通信方式 |
| **创建什么** | 团队 | 待办事项 | AI 子代理 | 消息 |
| **是否执行工作** | ❌ 否 | ❌ 否 | ✅ 是 | ❌ 否 |
| **生命周期** | 会话期间 | 可更新状态 | 任务完成结束 | 即时发送 |
| **使用频率** | 低频（项目开始） | 中频（规划时） | 高频（执行时） | 高频（协调时） |
| **类比** | 成立公司 | 写任务清单 | 雇佣员工 | 发消息通知 |

---

## 10. 一句话选择指南

- **要组织多人协作** → 用 `TeamCreate`
- **要记录待办事项** → 用 `TaskCreate`
- **要实际执行工作** → 用 `Agent`
- **要通知/协调/汇报** → 用 `SendMessage`
