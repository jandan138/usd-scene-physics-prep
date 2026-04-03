---
title: Codex MCP 自动加载机制调研
code_reference: N/A
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: agent
tags: [codex, mcp, config, autoload, plugins]
status: completed
---

# Codex MCP 自动加载机制调研

## 1. 配置文件位置

### 1.1 全局配置目录
Codex 的配置存储在 `~/.codex/` 目录下：

```
~/.codex/
├── config.toml          # 主配置文件
├── auth.json            # 认证信息
├── version.json         # 版本信息
├── agents/              # Agent 配置
│   ├── default.toml
│   ├── explorer.toml
│   ├── worker.toml
│   └── reviewer.toml
├── plugins/             # 插件缓存
│   └── cache/
├── skills/              # 自定义技能
├── rules/               # 规则文件
│   └── default.rules
└── .tmp/plugins/plugins/ # 插件临时文件（实际插件存储）
```

### 1.2 项目级配置
项目级 Codex 配置位于项目目录下的 `.codex/`：

```
<project>/.codex/
└── worklogs/            # 工作日志
```

**注意**：项目级配置主要用于工作日志，主要配置仍在 `~/.codex/config.toml`。

## 2. MCP 服务器配置方式

### 2.1 全局 MCP 配置 (config.toml)
在 `~/.codex/config.toml` 中直接定义 MCP 服务器：

```toml
[mcp_servers.figma]
url = "https://mcp.figma.com/mcp"
```

### 2.2 插件级 MCP 配置 (.mcp.json)
插件通过 `.mcp.json` 文件声明 MCP 服务器：

**位置**: `~/.codex/.tmp/plugins/plugins/<plugin-name>/.mcp.json`

**示例**:
```json
{
  "mcpServers": {
    "vercel": {
      "type": "http",
      "url": "https://mcp.vercel.com",
      "note": "Official Vercel MCP server. Uses OAuth..."
    }
  }
}
```

**多服务器示例** (build-web-apps):
```json
{
  "mcpServers": {
    "stripe": {
      "type": "http",
      "url": "https://mcp.stripe.com"
    },
    "vercel": {
      "type": "http",
      "url": "https://mcp.vercel.com"
    },
    "supabase": {
      "type": "http",
      "url": "https://mcp.supabase.com/mcp"
    }
  }
}
```

### 2.3 插件声明 MCP 配置
插件的 `plugin.json` 通过 `mcpServers` 字段指向 `.mcp.json`：

```json
{
  "name": "vercel",
  "version": "0.21.0",
  "skills": "./skills/",
  "mcpServers": "./.mcp.json",
  "interface": { ... }
}
```

## 3. MCP 自动加载触发条件

### 3.1 插件启用时加载
根据配置分析，MCP 自动加载的触发条件：

1. **插件启用**: 当插件在 `config.toml` 中被标记为 `enabled = true` 时
   ```toml
   [plugins."github@openai-curated"]
   enabled = true
   ```

2. **插件包含 MCP 配置**: 插件目录下存在 `.mcp.json` 文件

3. **全局 MCP 服务器**: `config.toml` 中 `[mcp_servers.*]` 部分定义的服务器

### 3.2 已发现的 MCP 服务器

| 插件/来源 | MCP 服务器 | URL | 认证方式 |
|-----------|-----------|-----|----------|
| config.toml | figma | https://mcp.figma.com/mcp | OAuth |
| build-web-apps | stripe | https://mcp.stripe.com | - |
| build-web-apps | vercel | https://mcp.vercel.com | OAuth |
| build-web-apps | supabase | https://mcp.supabase.com/mcp | - |
| cloudflare | cloudflare-api | https://mcp.cloudflare.com/mcp | OAuth/Bearer |
| hugging-face | hf-mcp-server | https://huggingface.co/mcp?login | Login |
| vercel | vercel | https://mcp.vercel.com | OAuth |

### 3.3 无 MCP 配置的插件
以下插件没有 `.mcp.json` 文件：
- github (使用 connector 而非 MCP)
- slack
- linear
- notion
- stripe (作为依赖被 build-web-apps 引用)

## 4. 禁用 MCP 自动加载的选项

### 4.1 禁用特定插件
在 `~/.codex/config.toml` 中禁用插件：

```toml
[plugins."plugin-name"]
enabled = false
```

### 4.2 移除 MCP 服务器配置
从 `config.toml` 中删除 `[mcp_servers.*]` 部分。

### 4.3 无全局禁用开关
**重要发现**: 在调研的配置文件中，**没有发现全局禁用 MCP 自动加载的开关**。例如：
- 没有 `mcp_enabled = false` 全局设置
- 没有 `autoload_mcp = false` 选项

### 4.4 可能的禁用方法
根据配置结构，可能的禁用方式：

1. **注释掉 MCP 服务器配置**:
   ```toml
   # [mcp_servers.figma]
   # url = "https://mcp.figma.com/mcp"
   ```

2. **禁用包含 MCP 的插件**:
   ```toml
   [plugins."vercel"]
   enabled = false
   
   [plugins."cloudflare"]
   enabled = false
   
   [plugins."build-web-apps"]
   enabled = false
   
   [plugins."hugging-face"]
   enabled = false
   ```

## 5. 配置层级总结

```
┌─────────────────────────────────────────────────────────────┐
│  ~/.codex/config.toml                                       │
│  ├── [mcp_servers.xxx]        # 全局 MCP 服务器             │
│  └── [plugins."name"]         # 插件启用/禁用               │
│      └── enabled = true/false                               │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  ~/.codex/.tmp/plugins/plugins/<name>/.mcp.json            │
│  └── mcpServers: { ... }      # 插件级 MCP 配置             │
└─────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────┐
│  ~/.codex/.tmp/plugins/plugins/<name>/.codex-plugin/       │
│  └── plugin.json                                              │
│      └── "mcpServers": "./.mcp.json"  # 声明 MCP 配置路径   │
└─────────────────────────────────────────────────────────────┘
```

## 6. 调研结论

1. **自动加载机制**: MCP 服务器在插件启用时自动加载，无需显式配置
2. **配置位置**: 全局在 `config.toml`，插件级在 `.mcp.json`
3. **禁用方式**: 只能通过禁用插件或删除 MCP 配置来禁用，**无全局开关**
4. **当前激活的 MCP**: figma (全局), vercel, cloudflare, stripe, supabase, hugging-face (通过插件)

## 7. 相关文件路径

- 全局配置: `~/.codex/config.toml`
- 插件缓存: `~/.codex/.tmp/plugins/plugins/`
- MCP 配置: `~/.codex/.tmp/plugins/plugins/<name>/.mcp.json`
- 插件声明: `~/.codex/.tmp/plugins/plugins/<name>/.codex-plugin/plugin.json`
