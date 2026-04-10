---
title: "Codex Proxy Configuration Research"
code_reference:
  - "/root/.local/bin/codex_vps"
  - "/root/.config/claude-providers/vps-http-proxy.env"
  - "/usr/local/bin/codex"
created_at: 2026-04-09
updated_at: 2026-04-09
maintainer: "GitHub Copilot"
status: completed
doc_class: record
---

# Codex 网络代理配置方案调研报告

## 1. use_win_clash 命令分析

### 定义位置
`/root/.bashrc` (第 111-122 行)

### 功能说明
`use_win_clash` 是一个 bash 函数，用于设置 Windows Clash 代理环境变量。

### 设置的环境变量
```bash
export WIN_CLASH_PORT="$port"           # 默认端口 17890
export http_proxy="http://127.0.0.1:${port}"
export https_proxy="http://127.0.0.1:${port}"
export HTTP_PROXY="$http_proxy"
export HTTPS_PROXY="$https_proxy"
```

### 相关辅助函数
- `unset_win_clash()`: 清除所有代理环境变量
- `test_win_clash()`: 测试代理连接
- `win_clash_status()`: 显示当前代理状态

### 自动启用
当 `WIN_CLASH_AUTO=1` 时，会在 shell 启动时自动调用 `use_win_clash`。

---

## 2. 当前 Codex 包装方式

### 2.1 实际 Codex 二进制位置
- 入口脚本: `/root/.local/bin/codex` (指向 `../lib/node_modules/@openai/codex/bin/codex.js`)
- 实际二进制: `/root/.local/lib/node_modules/@openai/codex-linux-x64/vendor/x86_64-unknown-linux-musl/codex/codex`

### 2.2 包装函数定义
在 `/root/.bashrc` 中定义了以下 codex 包装函数 (第 509-590 行):

#### 核心包装函数 `_codex_run()`
```bash
_codex_run() {
    local http_proxy_value="$1"
    local https_proxy_value="$2"
    shift 2

    if [ -z "$_codex_real_bin" ] || [ ! -x "$_codex_real_bin" ]; then
        echo "codex wrapper: missing codex binary" >&2
        return 1
    fi

    (
        unset OPENAI_API_KEY
        unset OPENAI_BASE_URL
        unset OPENAI_API_BASE

        if [ -n "$http_proxy_value" ]; then
            export http_proxy="$http_proxy_value"
            export HTTP_PROXY="$http_proxy_value"
        else
            unset http_proxy
            unset HTTP_PROXY
        fi

        if [ -n "$https_proxy_value" ]; then
            export https_proxy="$https_proxy_value"
            export HTTPS_PROXY="$https_proxy_value"
        else
            unset https_proxy
            unset HTTPS_PROXY
        fi

        unset ALL_PROXY
        unset all_proxy

        exec "$_codex_real_bin" "$@"
    )
}
```

#### 可用的 codex 命令变体
1. **`codex_direct`**: 直接运行 codex，不设置代理
2. **`codex_vps_proxy`**: 通过 VPS 代理运行 codex
3. **`codex_local_tunnel_start`**: 启动本地 SSH 隧道
4. **`codex_local_tunnel_stop`**: 停止本地 SSH 隧道
5. **`codex_local_tunnel_status`**: 查看隧道状态
6. **`codex_local_tunnel`**: 启动隧道并通过隧道运行 codex
7. **`codex`** (默认): 等同于 `codex_local_tunnel`

### 2.3 代理配置来源
代理配置从 `/root/.config/claude-providers/official-proxy.env` 读取:
- `CLAUDE_OFFICIAL_VPS_PROXY_HOST`: 45.32.95.77
- `CLAUDE_OFFICIAL_VPS_PROXY_PORT`: 18443
- `CLAUDE_OFFICIAL_VPS_PROXY_USER`: claudeproxy
- `CLAUDE_OFFICIAL_VPS_PROXY_PASS`: f2YAiSmLrUW9KjJTP7cfIwrw
- `CLAUDE_OFFICIAL_LOCAL_TUNNEL_PORT`: 28790

---

## 3. Codex CLI 本身的代理支持

### 3.1 环境变量支持
Codex CLI (OpenAI Codex) 基于 OpenAI API，支持标准的 HTTP 代理环境变量:
- `HTTP_PROXY` / `http_proxy`
- `HTTPS_PROXY` / `https_proxy`
- `ALL_PROXY` / `all_proxy`

### 3.2 配置文件
Codex CLI 的配置文件位于 `~/.codex/config.toml`，但**没有直接的代理配置选项**。

### 3.3 命令行选项
通过 `codex --help` 查看，没有专门的 `--proxy` 选项。

### 3.4 底层实现
Codex CLI 使用 Rust 编写，通过 Node.js wrapper 调用。代理支持依赖于:
1. Rust 的 `reqwest` 库会读取 `HTTP_PROXY`/`HTTPS_PROXY` 环境变量
2. OpenAI Rust SDK 遵循标准代理环境变量约定

---

## 4. 可行的代理配置方案

### 方案 A: 使用现有的 `codex` 包装函数 (推荐)
```bash
# 默认行为 - 自动启动 SSH 隧道并通过代理运行
codex "your prompt"

# 直接运行，不经过代理
codex_direct "your prompt"

# 通过 VPS 代理直接运行 (不经过 SSH 隧道)
codex_vps_proxy "your prompt"
```

### 方案 B: 手动设置环境变量后运行
```bash
# 使用 use_win_clash
use_win_clash 17890
codex_direct "your prompt"

# 或手动设置
export HTTP_PROXY="http://127.0.0.1:17890"
export HTTPS_PROXY="http://127.0.0.1:17890"
codex_direct "your prompt"
```

### 方案 C: 创建自定义包装脚本
```bash
#!/bin/bash
# ~/.local/bin/codex-with-proxy
export HTTP_PROXY="http://127.0.0.1:17890"
export HTTPS_PROXY="http://127.0.0.1:17890"
exec /root/.local/bin/codex "$@"
```

### 方案 D: 修改 codex 配置文件 (不推荐)
Codex CLI 本身不支持在配置文件中设置代理，必须通过环境变量。

---

## 5. 与 Claude Code 代理方案的对比

| 特性 | Claude Code | Codex CLI |
|------|-------------|-----------|
| 包装函数 | `claude_official`, `claude_*` | `codex`, `codex_*` |
| 环境变量 | `ANTHROPIC_BASE_URL`, `ANTHROPIC_API_KEY` | `OPENAI_API_KEY` |
| 代理支持 | HTTP_PROXY/HTTPS_PROXY | HTTP_PROXY/HTTPS_PROXY |
| 配置文件 | `~/.claude/config.json` | `~/.codex/config.toml` |
| 代理配置位置 | 包装函数/环境变量 | 包装函数/环境变量 |

---

## 6. 总结与建议

### 当前状态
用户已经有完善的 codex 代理包装方案:
1. `codex` 命令默认通过 SSH 本地隧道 + VPS 代理运行
2. `codex_direct` 可直接运行，绕过代理
3. `use_win_clash` 可用于设置本地 Clash 代理

### 使用建议
1. **日常使用**: 直接使用 `codex` 命令（已配置自动代理）
2. **使用本地 Clash**: 运行 `use_win_clash` 后再使用 `codex_direct`
3. **调试/直连**: 使用 `codex_direct` 绕过所有代理

### 注意事项
- `codex` 包装函数会 `unset OPENAI_API_KEY`，确保使用配置文件中的 API key
- 代理配置与 Claude Code 共享同一个 `official-proxy.env` 文件
- SSH 隧道使用端口 28790，与 Clash 默认端口 17890 不同

---

## 7. 2026-04-09 本机官方 Codex 安装后的最新入口

### 当前命令现状

- 官方 Codex 当前安装在 `/usr/local/bin/codex`
- 默认 `codex` 不再走旧的 shell 包装函数，而是直接使用官方 npm 全局安装版本
- 新增 `codex_vps` 作为显式代理入口，用于在启动官方 Codex 之前加载 VPS HTTP 代理环境

### `codex_vps` 的最终实现原则

2026-04-09 的最终落地方案，不再让 Codex 直接复用 Claude 的 `18443 -> iproyal` 住宅代理链。

原因是这条旧链路对 Anthropic 可用，但对 OpenAI/ChatGPT 传输层会出现 reset，具体表现为：

- `codex_apps` MCP 初始化失败
- `chatgpt.com/backend-api/wham/apps` 请求 reset
- 后续 Codex 主请求也会 `Connection reset by peer`

最终采用的稳定方案是：

```text
codex_vps
  -> 本地 SSH 隧道 127.0.0.1:28791
  -> VPS 127.0.0.1:18445 tinyproxy
  -> VPS 原生直连 OpenAI / ChatGPT
```

也就是说：

1. `codex_vps` 先调用本地 tunnel helper，确保 SSH LocalForward 已建立
2. 本地代理地址固定为 `127.0.0.1:28791`
3. VPS 上单独使用 `18445` 端口的 tinyproxy，和 Claude 的 `18443` 入口彻底隔离
4. 最终仍然执行官方 `/usr/local/bin/codex`

相关本地文件：

- `/root/.local/bin/codex_vps`
- `/root/.local/bin/codex_vps_tunnel_start`
- `/root/.local/bin/codex_vps_tunnel_status`
- `/root/.local/bin/codex_vps_tunnel_stop`
- `/root/.local/share/zhuzihou-cli/codex_vps_tunnel.py`
- `/root/.config/claude-providers/codex-openai-tunnel.env`

### 使用方式

```bash
# 默认官方 Codex（不附加 Codex 专用代理）
codex

# 显式通过本地隧道 + VPS tinyproxy 启动官方 Codex
codex_vps

# 手动控制隧道
codex_vps_tunnel_start
codex_vps_tunnel_status
codex_vps_tunnel_stop

# 无副作用验证
codex_vps --version
```

### 验证结果

2026-04-09 本机验证结果：

- `command -v codex` -> `/usr/local/bin/codex`
- `command -v codex_vps` -> `/root/.local/bin/codex_vps`
- `codex_vps_tunnel_start` -> `running 127.0.0.1:28791`
- 通过 `127.0.0.1:28791` 代理测试：
    - `https://api.openai.com/v1/models` -> `HTTP 401`
    - `https://chatgpt.com/backend-api/wham/apps` -> `HTTP 405`
- `codex_vps --version` -> `codex-cli 0.118.0`
- `codex_vps exec --color never --output-last-message ... "Reply with exactly: ok"` -> `ok`
- `codex_vps_tunnel_stop` 后，`codex_vps_tunnel_status` -> `stopped`

### 结论

对于当前机器，推荐把代理需求明确分成两条入口：

- `codex`：官方默认入口
- `codex_vps`：本地 SSH 隧道 + VPS tinyproxy 直连 OpenAI/ChatGPT 的专用入口

这样不会再把默认官方安装和 Claude 专用住宅代理绑死在一起，排障和日常使用都更清晰。
