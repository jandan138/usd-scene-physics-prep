---
title: Codex 配置层级通俗讲解
code_reference: N/A - 指南文档
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: Claude
status: complete
---

# Codex 配置层级通俗讲解

## 一句话总结

- **MCP 服务器** = 外部工具集（像浏览器扩展，连外部服务）
- **插件** = 应用商店 App（有界面、技能、触发器）
- **功能开关** = 系统设置（启用/禁用内置功能）
- **codex_apps** = 系统核心（像 iOS 系统服务，无法禁用）

---

## 详细对比

### 1. MCP 服务器 [mcp_servers.figma]

**是什么**：Model Context Protocol 服务器，让 Codex 调用外部工具

**比喻**：就像 Chrome 浏览器扩展，连接外部服务（Figma、GitHub、数据库等）

**配置位置**：`~/.codex/config.toml`

```toml
[mcp_servers.figma]
url = "https://mcp.figma.com/mcp"
```

**包含什么**：
- 工具定义（JSON Schema）
- 服务端点 URL
- 认证信息

**禁用效果**：
- ✅ 不再连接 Figma 服务器
- ✅ 不再加载 Figma 的工具定义
- ✅ 节省 token（每个工具 500-1500 tokens）

---

### 2. 插件 [plugins."github@openai-curated"]

**是什么**：完整的应用包，包含技能、界面、触发器

**比喻**：就像手机 App Store 里的 App，有：
- 界面（图标、描述、启动画面）
- 技能（预定义的 agent 配置）
- 钩子（触发器，如保存后自动执行）

**配置位置**：`~/.codex/config.toml`

```toml
[plugins."github@openai-curated"]
enabled = true
```

**包含什么**：
```
~/.codex/.tmp/plugins/plugins/github/
├── .codex-plugin/plugin.json  # 应用信息、界面配置
├── .app.json                  # App ID 映射
├── skills/                    # 预定义技能
│   ├── gh-address-comments/   # 处理评论技能
│   ├── gh-fix-ci/            # 修复 CI 技能
│   └── github/               # 通用 GitHub 技能
├── hooks.json                # 触发器配置
└── assets/                   # 图标等资源
```

**禁用效果**：
- ✅ 插件技能不可用
- ✅ 插件触发器不执行
- ✅ 但底层的 MCP 连接可能仍存在（见下文）

---

### 3. 功能开关 [features].apps

**是什么**：Codex 内置功能的系统级开关

**比喻**：就像 iOS 的系统设置，控制核心功能

**配置位置**：`~/.codex/config.toml`

```toml
[features]
apps = false                    # 禁用 App 集成
tool_call_mcp_elicitation = false  # 禁用 MCP 引导
```

**所有功能列表**（运行 `codex features list` 查看）：
```
apply_patch_freeform    - 自由格式补丁
apps                    - App 集成（包括 codex_apps）
artifact                - 产物管理
code_mode               - 代码模式
fast_mode               - 快速模式
guardian_approval       - 守护者审批
multi_agent             - 多 Agent
plugins                 - 插件系统
shell_tool              - Shell 工具
...
```

**禁用效果**：
- ❓ `apps = false` 试图禁用 codex_apps，但它是核心功能
- ✅ `tool_call_mcp_elicitation = false` 禁用 MCP 自动调用提示

---

## 关键问题：为什么 codex_apps 仍然加载？

### 架构层级

```
┌─────────────────────────────────────────────────────────────┐
│  Layer 3: 功能开关 (features)                                │
│  ├── apps = false                    [尝试禁用]             │
│  └── tool_call_mcp_elicitation       [控制 MCP 调用方式]    │
├─────────────────────────────────────────────────────────────┤
│  Layer 2: 插件 (plugins)                                     │
│  ├── github@openai-curated (enabled=false)  [已禁用]        │
│  ├── figma (无 enabled 字段)                [通过 MCP 连接] │
│  └── build-web-apps, cloudflare 等         [可能启用]       │
├─────────────────────────────────────────────────────────────┤
│  Layer 1: MCP 服务器 (mcp_servers)                          │
│  ├── figma (已注释)                        [已禁用]         │
│  └── 其他外部 MCP 服务器                                    │
├─────────────────────────────────────────────────────────────┤
│  Layer 0: 系统核心 (无法禁用)                               │
│  └── codex_apps                          [总是启用]         │
│      ├── 内置 GitHub 工具集                                │
│      ├── 内置权限管理                                      │
│      └── 连接缓存 (~/.codex/cache/codex_apps_tools/)       │
└─────────────────────────────────────────────────────────────┘
```

### codex_apps 是什么？

**本质**：Codex 的内置 MCP 服务器，提供核心 GitHub 功能

**为什么禁用不了**：
1. **不是配置项** - 没有 `mcp_servers.codex_apps` 可以注释
2. **不是插件** - 不在 `plugins/` 目录
3. **不是功能开关** - `features.apps` 控制的是 App 集成方式，不是 codex_apps 本身
4. **缓存机制** - 工具定义缓存在 `~/.codex/cache/codex_apps_tools/`

**证据**：
```bash
$ codex mcp list
No MCP servers configured yet.

# 但启动时仍然显示：
# Booting MCP server: codex_apps
```

这说明 codex_apps 是**系统级内置服务**，不在用户配置的 MCP 列表中。

---

## 各层级禁用效果总结

| 配置项 | 禁用方法 | 实际效果 | 节省 Token |
|--------|----------|----------|------------|
| `mcp_servers.figma` | 注释掉 | ✅ 完全禁用 Figma MCP | ~10,000-20,000 |
| `plugins.github@openai-curated` | `enabled = false` | ✅ 禁用插件技能 | ~5,000-10,000 |
| `features.apps` | `= false` | ❓ 无效（codex_apps 仍加载） | 0 |
| `features.tool_call_mcp_elicitation` | `= false` | ⚠️ 改变调用方式，不减少 token | 0 |
| codex_apps（内置）| 无法禁用 | ❌ 始终启用 | ~15,000-30,000 |

---

## 实际建议

### 如果要最小化 MCP 上下文消耗：

```toml
# ~/.codex/config.toml

# 1. 禁用所有外部 MCP
# [mcp_servers.figma]
# url = "https://mcp.figma.com/mcp"

# 2. 禁用所有插件
[plugins."github@openai-curated"]
enabled = false

# 3. 功能开关不影响 codex_apps，但可禁用其他功能
[features]
tool_call_mcp_elicitation = false  # 减少 MCP 调用提示
```

### 无法避免的消耗：

- **codex_apps** 的工具定义：~15,000-30,000 tokens（每次请求都发送）
- 这是 OpenAI 设计的核心功能，用户无法禁用

---

## 类比：餐厅厨房

| Codex 组件 | 餐厅类比 | 能否关闭 |
|------------|----------|----------|
| **codex_apps** | 厨房核心设备（炉灶、冰箱） | ❌ 不能，否则餐厅无法运营 |
| **MCP 服务器** | 外送服务（美团、饿了么） | ✅ 可以，不用外送就行 |
| **插件** | 特色菜品（披萨、寿司） | ✅ 可以，从菜单移除 |
| **功能开关** | 服务方式（堂食/外卖） | ⚠️ 调整方式，不影响核心 |

---

## 结论

1. **figma MCP** - 外部服务，完全禁用 ✅
2. **github 插件** - 应用包，完全禁用 ✅
3. **apps 功能** - 控制 App 集成方式，不影响 codex_apps ❌
4. **codex_apps** - 系统核心，无法禁用 ❌

**实际节省**：禁用 figma + github 插件后，上下文消耗从 ~40,000 tokens 降至 ~15,000-30,000 tokens（仅剩 codex_apps 内置工具）。
