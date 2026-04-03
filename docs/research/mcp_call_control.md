---
title: Codex MCP 调用控制和过滤机制调研
code_reference: []
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: agent-team
status: completed
---

# Codex MCP 调用控制和过滤机制调研

## 概述

本文档总结了 OpenAI Codex CLI 中 MCP (Model Context Protocol) 工具的调用控制、过滤和用户控制机制。

---

## 1. Codex 内置的 MCP 调用决策机制

### 1.1 自动工具选择 vs 显式触发

Codex CLI 在工具调用上采用**混合策略**，平衡自动化和用户控制：

| 策略类型 | 描述 | 使用场景 |
|---------|------|---------|
| **自动检测** | 基于用户意图分析自动选择工具 | 减少常见场景的摩擦 |
| **半自动** | `approval_policy` 智能默认 | 平衡效率和安全性 |
| **显式触发** | 模式切换、命令触发 | 明确的用户意图 |
| **门控自动** | `on-request` 审批策略 | 用户保留否决权 |

### 1.2 "Recency Intent" 自动检测 (提议功能)

GitHub Issue [#2563](https://github.com/openai/codex/issues/2563) 讨论了基于用户意图的自动工具选择：

- **触发词**: "as of today", "latest version", "recent news", "this week", "new CVEs"
- **检测方法**: 关键词规则 + 轻量级分类器
- **模式**: `auto` (意图检测) / `on` (始终可用) / `off` (禁用)

```bash
# 自动模式 - 仅在检测到时效性意图时触发网络搜索
codex --enable-web --web-mode auto "What's the latest stable Next.js features?"

# TUI 命令控制
/web auto     # 自动检测时效性意图
/web on       # 始终允许网络工具
/web off      # 禁用网络工具
```

### 1.3 工具调解与控制架构

根据 [system prompt forensics 研究](https://system-prompts-forensics.rmax.ai/report.html)：

| 模式 | 工具行为 | 权限模型 |
|------|---------|---------|
| `vscode-codex agent/chat` | 审批门控升级 | 用户为最终决策者 |
| `vscode-codex agent-full-access` | 无升级，模型决定 | Agent 最终决策（有限自治） |
| `codex review` | 工具可用但禁止修复 | 角色限制模式 |

**核心洞察**: "Tools as the boundary of action" - 所有助手都通过声明的工具路由副作用，但在工具暴露、门控和排序方面有所不同。

---

## 2. 用户控制 MCP 调用范围的方法

### 2.1 审批策略 (approval_policy)

Codex CLI 使用**双层安全模型**结合沙盒模式和审批策略：

| 值 | 描述 |
|----|------|
| `untrusted` | 对不受信任的命令提示（推荐用于安全） |
| `on-request` | 模型决定何时请求审批（默认交互模式） |
| `never` | 自动批准所有命令（谨慎使用） |
| `on-failure` | ⚠️ 已弃用 - 仅在沙盒命令失败时提示 |

**配置示例** (`~/.codex/config.toml`):
```toml
# 基本审批策略
approval_policy = "on-request"

# 沙盒模式（与审批策略配合使用）
sandbox_mode = "workspace-write"

# 特定配置文件
[profiles.strict]
approval_policy = "on-request"
sandbox_mode = "read-only"

[profiles.auto]
approval_policy = "never"
sandbox_mode = "danger-full-access"
```

**CLI 标志** (临时覆盖):
```bash
# 基本标志
codex -a never -s workspace-write "task"
codex --ask-for-approval never --sandbox workspace-write "task"

# 便捷别名
codex --full-auto "task"                    # = -a on-request -s workspace-write
codex --dangerously-bypass-approvals-and-sandbox "task"  # 无限制 (--yolo)
```

### 2.2 细粒度审批策略 (高级)

```toml
approval_policy = { 
  granular = {
    sandbox_approval = true,      # 允许沙盒升级提示
    rules = true,                  # 允许 execpolicy `prompt` 规则审批
    mcp_elicitations = true,       # 允许 MCP 引导提示
    request_permissions = false,   # 自动拒绝权限请求
    skill_approval = false         # 自动拒绝技能脚本审批
  }
}
```

### 2.3 配置优先级

```
CLI 参数 (-c, -a, -s) > 配置文件 (-p) > config.toml > 默认值
```

---

## 3. "@" 提及或显式触发机制

### 3.1 "@" 提及触发器

根据 [OpenAI Codex 实践指南](https://www.linkedin.com/pulse/openai-codex-practitioners-guide-building-ai-agents-devon-coombs-jnmlc)：

> *"当你在提示中使用 @ 符号时，你可以标记特定连接，将该外部上下文带入当前会话。"*

**语法模式**:
```
@<connection-name> <your request>
```

**示例**:
- `@salesforce pull the top 20 closed deals from Q1 by ARR`
- `@github list open issues in this repository`
- `@figma get the latest design specs`

### 3.2 2026 年 Triggers 功能

OpenAI 在 **2025 年 3 月** 发布了 **"Triggers"** 功能，支持自动事件驱动响应：

| 触发事件 | Codex 自动操作 |
|---------|---------------|
| **创建新 Issue** | 自动分析问题，开始编码修复，打开 PR |
| **PR 收到审查评论** | 根据评论自动修改代码，推送更新 |
| **CI 测试失败** | 自动分析失败原因，尝试修复 |
| **通过 @ 提及** | **自动响应并执行请求** |

**触发配置示例**:
```yaml
trigger_phrase: "@ai"  # 或 "@codex" / "@"
```

### 3.3 斜杠命令

Codex CLI 支持通过斜杠命令显式触发功能：

| 命令 | 描述 |
|------|------|
| `/mcp` | 查看活跃的 MCP 服务器 |
| `/web auto` | 设置网络搜索为自动模式 |
| `/web on` | 启用网络搜索 |
| `/web off` | 禁用网络搜索 |
| `/review` | 启动代码审查 |
| `/plan` | 启动规划模式 |

### 3.4 显式 vs 自动工具使用对比

| 触发类型 | 行为 |
|---------|------|
| **显式 `@` 提及** | 用户手动标记特定 MCP 连接使用 |
| **自动/隐式** | Codex 可根据任务上下文自主选择和触发 MCP 工具 |

---

## 4. MCP 配置中的过滤和白名单设置

### 4.1 工具白名单 (`enabled_tools`)

**用途**: 仅暴露 MCP 服务器中的特定工具（白名单方法）

**配置**: 在 MCP 服务器配置中设置 `enabled_tools = ["tool1", "tool2"]`

**使用场景**: 最大安全性 - 仅启用必要的工具

```toml
[mcp_servers.my-server]
command = "npx"
args = ["-y", "@my-org/my-server"]
enabled_tools = ["read_file", "write_file"]  # 仅这些工具可用
```

### 4.2 工具黑名单 (`disabled_tools`)

**用途**: 阻止特定工具，同时保持其他工具可用

**配置**: 设置 `disabled_tools = ["tool1", "tool2"]`

**应用顺序**: 在 `enabled_tools` 之后应用（可同时使用）

**使用场景**: 移除危险操作，同时保留安全工具

```toml
[mcp_servers.my-server]
command = "npx"
args = ["-y", "@my-org/my-server"]
disabled_tools = ["delete_file", "execute_command"]  # 这些工具被阻止
```

### 4.3 组合使用

可以同时使用白名单和黑名单 - `enabled_tools` 先应用，然后 `disabled_tools` 从该子集中移除额外工具：

```toml
[mcp_servers.github]
command = "npx"
args = ["-y", "@modelcontextprotocol/server-github"]
enabled_tools = ["read_file", "list_issues", "get_pull_request"]
disabled_tools = ["delete_file"]  # 额外的安全层
```

### 4.4 完整服务器禁用

设置 `enabled = false` 可在不删除配置的情况下禁用服务器：

```toml
[mcp_servers.slack]
command = "npx"
args = ["-y", "@modelcontextprotocol/server-slack"]
enabled = false  # 服务器被禁用但配置保留
```

### 4.5 完整配置示例

**STDIO 服务器配置**:
```toml
[mcp_servers.context7]
enabled = true
required = true                         # 如果不可用则启动失败
command = "npx"
args = ["-y", "@upstash/context7-mcp"]
env = { "MY_VAR" = "value" }            # 静态环境变量
env_vars = ["PATH", "HOME"]             # 转发主机环境变量
cwd = "/path/to/project"                # 可选工作目录
startup_timeout_sec = 10
tool_timeout_sec = 60
enabled_tools = ["search", "summarize"]  # 工具白名单
disabled_tools = ["slow-tool"]           # 工具黑名单
```

**HTTP 服务器配置**:
```toml
[mcp_servers.custom-api]
url = "https://mcp.example.com/api"
bearer_token_env_var = "MCP_TOKEN"
enabled = true
tool_timeout_sec = 60
enabled_tools = ["query_data", "get_status"]  # 仅允许读取操作

[mcp_servers.custom-api.http_headers]
"X-Organization-ID" = "org-123"
```

### 4.6 其他控制选项

| 选项 | 描述 |
|------|------|
| `required = true` | 如果服务器无法连接则启动失败 |
| `startup_timeout_sec` | 服务器启动超时（默认：10秒） |
| `tool_timeout_sec` | 工具执行超时（默认：60秒） |
| `approval_policy` | 全局审批设置（非每工具） |

---

## 5. 为特定任务禁用 MCP 的方法

### 5.1 当前限制

GitHub Issue [#6049](https://github.com/openai/codex/issues/6049) 指出：

> 在 headless/自动化环境中运行 `codex exec` 时，目前**无法将代理限制为仅使用 MCP 工具**。

### 5.2 提议的解决方案

添加配置选项（CLI 标志或配置设置）以禁用 Codex 的内置工具，同时保持 MCP 工具可用：

| 需求 | 描述 |
|------|------|
| **最低要求** | 能够禁用 shell 命令执行 |
| **理想方案** | 能够禁用所有内置工具（shell、文件操作、plan、apply_patch）以实现真正的 MCP-only 模式 |

**建议实现**:
```bash
--disable-builtin-tools  # 禁用内置工具，保留 MCP 工具、web_search 和 view_image
```

### 5.3 当前可行的变通方法

1. **使用 `approval_policy = "on-request"`** - 模型决定何时请求审批
2. **使用 `sandbox_mode = "read-only"`** - 限制文件系统访问
3. **使用 `enabled_tools` 白名单** - 仅暴露特定的 MCP 工具
4. **禁用特定工具**:
```toml
[apps.github.tools.delete_repo]
enabled = false
approval_mode = "prompt"
```

### 5.4 应用级控制

Codex 还提供应用级工具控制：

```toml
[apps.github]
enabled = true
default_tools_enabled = true
default_tools_approval_mode = "auto"
destructive_enabled = false           # 禁用所有破坏性操作
open_world_enabled = true

[apps.github.tools.delete_repo]       # 细粒度工具控制
enabled = false
approval_mode = "prompt"
```

---

## 6. CLI 管理命令

| 命令 | 描述 |
|------|------|
| `codex mcp list` | 列出所有配置的 MCP 服务器 |
| `codex mcp add <name> -- <command>` | 添加新的 MCP 服务器 |
| `codex mcp get <name>` | 显示服务器配置 |
| `codex mcp remove <name>` | 移除服务器 |
| `codex mcp login <name>` | OAuth 认证 |
| `codex mcp logout <name>` | 移除 OAuth 凭证 |

---

## 7. 安全最佳实践

根据官方文档：

1. **使用前审查服务器代码**
2. **使用 `enabled_tools` 进行工具白名单**
3. **使用 `destructive_enabled = false` 禁用破坏性操作**
4. **尽可能使用 OAuth**
5. **安全存储凭证**（优先使用 keyring）
6. **设置超时**以防止挂起操作
7. **监控服务器日志**

---

## 8. 与其他 Agent 的比较

| 功能 | Codex CLI | Claude Code | OpenCode | Gemini CLI |
|------|-----------|-------------|----------|------------|
| 禁用工具 | ✅ `disabled_tools` | ❌ 无每工具字段 | ✅ `"mcp__server__*": false` | ✅ `excludeTools` |
| 白名单工具 | ✅ `enabled_tools` | ❌ 无 `includeTools` | ✅ 特定工具重新启用 | ✅ `includeTools` |
| Glob 模式 | ❌ 仅显式名称 | ✅ `mcp__server__*` 通配符 | ✅ 通配符 | ❌ 仅显式 |
| 每工具权限 | ❌ 仅全局 | ✅ `permissions.allow/deny` | ✅ 单独权限配置 | ❌ 仅服务器范围 |

---

## 9. 关键发现总结

### 9.1 调用控制机制

1. **白名单 (`enabled_tools`)**: 最安全的方法，仅显式启用需要的工具
2. **黑名单 (`disabled_tools`)**: 阻止特定危险工具
3. **服务器禁用 (`enabled = false`)**: 完全禁用服务器但保留配置
4. **审批策略 (`approval_policy`)**: 控制何时需要用户确认
5. **沙盒模式 (`sandbox_mode`)**: 限制文件系统/网络访问

### 9.2 显式触发机制

1. **"@" 提及**: `@<connection-name>` 显式触发特定 MCP 工具
2. **斜杠命令**: `/web`, `/mcp`, `/review`, `/plan` 等
3. **模式切换**: `Shift+Tab` 切换模式

### 9.3 当前限制

- 无法完全禁用内置工具实现 MCP-only 模式（Issue #6049）
- 不支持 Glob 模式匹配工具名称
- 无每工具权限控制（仅全局或白名单/黑名单）

---

## 参考来源

- [MCP Servers - Codex CLI](https://mintlify.com/openai/codex/configuration/mcp-servers)
- [Model Context Protocol – Codex](https://developers.openai.com/codex/mcp/)
- [Configuration Reference – Codex](https://developers.openai.com/codex/config-reference/)
- [Agent approvals & security – Codex](https://developers.openai.com/codex/agent-approvals-security/)
- [MCP Tools whitelist · Issue #4796 · openai/codex](https://github.com/openai/codex/issues/4796)
- [Ability to disable built-in tools for MCP-only execution #6049](https://github.com/openai/codex/issues/6049)
- [Auto Web Search on "Recency Intent" #2563](https://github.com/openai/codex/issues/2563)
- [OpenAI Codex: The Practitioner's Guide](https://www.linkedin.com/pulse/openai-codex-practitioners-guide-building-ai-agents-devon-coombs-jnmlc)
- [System Prompt Forensics Research](https://system-prompts-forensics.rmax.ai/report.html)
- [Codex CLI Mastery Guide](https://www.heyuan110.com/posts/ai/2026-02-12-codex-cli-mastery-guide/)
- [Codex CLI: The Definitive Technical Reference](https://blakecrosley.com/guides/codex)
