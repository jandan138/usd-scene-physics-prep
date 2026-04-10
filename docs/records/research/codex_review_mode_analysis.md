---
title: Codex Review Mode Analysis
code_reference: ~/.local/lib/node_modules/@openai/codex/
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: cli-agent
status: completed
---

# Codex CLI Review Mode vs Regular Mode Analysis

## Executive Summary

The `codex review` command fails with `failed to refresh available models: timeout waiting for child process to exit` while regular interactive `codex` sessions work. This is a **known widespread issue** affecting Codex CLI v0.118.0 and multiple earlier versions.

## Key Findings

### 1. Root Cause: "Timeout Waiting for Child Process to Exit"

The error originates from `codex_core::models_manager::manager` when attempting to refresh the models list:

```
2026-04-03T05:33:11.464690Z ERROR codex_core::models_manager::manager: failed to refresh available models: timeout waiting for child process to exit
```

This error occurs in:
- `codex review` mode
- `codex exec` mode
- `codex resume` mode

But **NOT** in:
- Regular interactive `codex` TUI mode (when started via Claude Code plugin)

### 2. Architecture Difference: App-Server vs Direct CLI

| Mode | Entry Point | Models Refresh Behavior |
|------|-------------|------------------------|
| Interactive TUI | `codex` (no args) | Uses existing app-server, cache-first strategy |
| Review | `codex review` | Spawns new process, attempts model refresh |
| Exec | `codex exec` | Spawns new process, attempts model refresh |
| Claude Plugin | `codex-companion.mjs` | Uses `app-server` protocol, broker-based |

**Critical Difference:**

The Claude Code plugin (`/codex:review` command) uses a **different code path**:
- Script: `/root/.claude/plugins/marketplaces/openai-codex/plugins/codex/scripts/codex-companion.mjs`
- It connects to a running `codex app-server` instance via the App-Server Protocol
- It uses `review/start` API call instead of `codex review` CLI
- The app-server is already running (started by VS Code extension or previous session)

### 3. Models Cache Refresh Strategy

From binary string analysis and logs:

```rust
// From codex-rs/core/src/models_manager/manager.rs
codex_core::models_manager::manager
list_models{refresh_strategy=online_if_uncached}
models cache: using cached models for OnlineIfUncached
```

The refresh strategy `online_if_uncached` is used, but the child process timeout suggests:
1. A subprocess is spawned to fetch models from `api.openai.com/v1/models`
2. The subprocess hangs (likely due to proxy/network issues)
3. Parent process times out waiting for child

### 4. Known GitHub Issues

This is a **confirmed widespread bug** tracked in multiple issues:

| Issue | Version | Date | Notes |
|-------|---------|------|-------|
| [#16491](https://github.com/openai/codex/issues/16491) | v0.118.0 | Mar 2026 | `codex resume` timeout |
| [#10070](https://github.com/openai/codex/issues/10070) | v0.92.0 | Jan 2026 | Child process timeout |
| [#9211](https://github.com/openai/codex/issues/9211) | v0.80.0 | Jan 2026 | SQLite WAL/SHM workaround |
| [#14321](https://github.com/openai/codex/issues/14321) | v0.114.0 | Mar 2026 | Recurring timeout |
| [#14783](https://github.com/openai/codex/issues/14783) | v0.115.0-alpha.24 | Mar 2026 | Linux aarch64 |
| [#15596](https://github.com/openai/codex/issues/15596) | v0.116.0-alpha.10 | Mar 2026 | macOS Apple Silicon |

### 5. Proxy and Network Considerations

Binary string analysis reveals proxy handling:

```
proxy_url
HTTP_PROXY
http-connect
https_connect
socks5_tcp
socks5_udp
```

The child process may not inherit proxy environment variables correctly, causing the models fetch to hang.

**Related Issues:**
- [#16079](https://github.com/openai/codex/issues/16079) - Linux + HTTP proxy failures
- [#6764](https://github.com/openai/codex/issues/6764) - NO_PROXY not passed to CLI

### 6. SQLite WAL/SHM File Issue

Workaround from [#9211](https://github.com/openai/codex/issues/9211):

```bash
rm ~/.codex/*.wal ~/.codex/*.shm
```

Current state:
```
-rw-r--r-- 1 root root 32768 Apr  3 05:33 /root/.codex/state_5.sqlite-shm
-rw-r--r-- 1 root root 4128272 Apr  3 05:33 /root/.codex/state_5.sqlite-wal
-rw-r--r-- 1 root root 32768 Apr  3 05:34 /root/.codex/logs_1.sqlite-shm
-rw-r--r-- 1 root root 4873992 Apr  3 05:34 /root/.codex/logs_1.sqlite-wal
```

These WAL/SHM files exist and may contribute to the child process timeout.

## Code Evidence

### Binary Strings Analysis

From `/root/.local/lib/node_modules/@openai/codex/node_modules/@openai/codex-linux-x64/vendor/x86_64-unknown-linux-musl/codex/codex`:

```
# Models manager strings
models cache: attempting load_fresh
models cache: loaded cache file
models cache: cache hit
models cache: cache version mismatch
models cache: cache miss, fetching remote models
models cache: using cached models for OnlineIfUncached
failed to refresh available models: timeout waiting for child process to exit

# API endpoints
/v1/models
api.openai.com

# Source file references
/home/runner/work/codex/codex/codex-rs/core/src/models_manager/cache.rs
/home/runner/work/codex/codex/codex-rs/core/src/models_manager/manager.rs
```

### Log Evidence

From `~/.codex/log/codex-tui.log`:

```
2026-04-03T05:17:17.310978Z  INFO list_models{refresh_strategy=online_if_uncached}: codex_core::models_manager::manager: new
2026-04-03T05:17:17.311787Z  INFO list_models{refresh_strategy=online_if_uncached}: codex_core::models_manager::cache: models cache: loaded cache file
2026-04-03T05:17:17.311810Z  INFO list_models{refresh_strategy=online_if_uncached}: codex_core::models_manager::cache: models cache: cache hit
2026-04-03T05:17:17.312556Z  INFO list_models{refresh_strategy=online_if_uncached}: codex_core::models_manager::manager: models cache: cache entry applied
```

Note: These successful logs are from the **app-server** mode used by Claude Code plugin, not direct CLI.

## Why Regular Mode Works

1. **App-Server Reuse**: The VS Code extension (`/root/.vscode-server/extensions/openai.chatgpt-26.5401.11717/bin/linux-x86_64/codex app-server`) is already running
2. **Models Cache Warm**: The app-server has already fetched and cached models
3. **No Child Process Spawn**: Uses IPC/websocket to existing process instead of spawning new subprocess
4. **Broker Architecture**: Claude plugin uses `app-server-broker.mjs` with Unix socket communication

## Workarounds and Fixes

### Immediate Workarounds

1. **Use Claude Code Plugin** (Recommended)
   ```
   /codex:review --wait
   ```
   This uses the app-server protocol and bypasses the CLI child process issue.

2. **Clear SQLite WAL/SHM Files**
   ```bash
   rm ~/.codex/*.wal ~/.codex/*.shm
   ```

3. **Ensure Proxy Environment Variables**
   ```bash
   export HTTP_PROXY=...
   export HTTPS_PROXY=...
   export NO_PROXY=...
   codex review --uncommitted
   ```

4. **Use App-Server Mode Directly**
   ```bash
   # Start app-server in background
   codex app-server &
   # Then use companion script or API calls
   ```

### Configuration Options

No direct config option exists to skip model refresh, but related settings in `~/.codex/config.toml`:

```toml
# Model settings (does not control refresh behavior)
model = "gpt-5.4"
model_reasoning_effort = "xhigh"

# Network settings
[network]
# proxy settings may help if child process inherits them
```

## Recommendations

1. **For Users**: Use `/codex:review` via Claude Code instead of `codex review` CLI
2. **For Debugging**: Monitor `~/.codex/log/codex-tui.log` for child process errors
3. **For Proxy Environments**: Ensure `HTTP_PROXY`/`HTTPS_PROXY` are exported before running Codex
4. **For Automation**: Use the app-server protocol directly instead of CLI subprocess

## Sources

- [Codex CLI Issue #16491 - codex resume timeout](https://github.com/openai/codex/issues/16491)
- [Codex CLI Issue #10070 - timeout waiting for child process](https://github.com/openai/codex/issues/10070)
- [Codex CLI Issue #9211 - SQLite WAL workaround](https://github.com/openai/codex/issues/9211)
- [Codex CLI Issue #16079 - Linux HTTP proxy issues](https://github.com/openai/codex/issues/16079)
- [Codex CLI Issue #14335 - exec review timeout](https://github.com/openai/codex/issues/14335)
