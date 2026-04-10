---
title: Codex 插件实现方式调研报告
code_reference: |
  /root/.claude/plugins/marketplaces/openai-codex/plugins/codex/scripts/codex-companion.mjs
  /root/.claude/plugins/marketplaces/openai-codex/plugins/codex/scripts/lib/app-server.mjs
  /root/.claude/plugins/marketplaces/openai-codex/plugins/codex/scripts/lib/process.mjs
  /root/.claude/plugins/marketplaces/openai-codex/plugins/codex/scripts/lib/codex.mjs
  /root/.claude/plugins/marketplaces/openai-codex/plugins/codex/commands/setup.md
  /root/.claude/plugins/marketplaces/openai-codex/plugins/codex/agents/codex-rescue.md
created_at: 2026-04-01
updated_at: 2026-04-01
maintainer: researcher
status: completed
---

# Codex 插件实现方式调研报告

## 1. 插件目录结构

```
/root/.claude/plugins/marketplaces/openai-codex/
├── plugins/codex/
│   ├── .claude-plugin/
│   │   └── plugin.json              # 插件元数据
│   ├── agents/
│   │   └── codex-rescue.md          # subagent 定义 (codex:rescue)
│   ├── commands/                    # 命令定义 (/codex:* 命令)
│   │   ├── setup.md                 # /codex:setup
│   │   ├── review.md                # /codex:review
│   │   ├── rescue.md                # /codex:rescue
│   │   ├── status.md                # /codex:status
│   │   ├── result.md                # /codex:result
│   │   ├── cancel.md                # /codex:cancel
│   │   └── adversarial-review.md    # /codex:adversarial-review
│   ├── skills/                      # 技能定义
│   │   ├── codex-cli-runtime/SKILL.md
│   │   ├── codex-result-handling/SKILL.md
│   │   └── gpt-5-4-prompting/SKILL.md
│   ├── scripts/
│   │   ├── codex-companion.mjs      # 主入口脚本
│   │   ├── app-server-broker.mjs    # broker 进程
│   │   └── lib/
│   │       ├── codex.mjs            # Codex 运行时核心
│   │       ├── app-server.mjs       # app-server 客户端
│   │       ├── process.mjs          # 进程管理
│   │       ├── broker-lifecycle.mjs # broker 生命周期
│   │       └── ...
│   └── hooks/
│       └── hooks.json
```

## 2. Codex 调用的具体位置和方式

### 2.1 命令映射方式

`/codex:*` 命令通过 `commands/*.md` 文件定义，每个 markdown 文件包含 YAML frontmatter 描述命令元数据，正文描述执行逻辑。

例如 `/codex:setup` 命令的定义在 `commands/setup.md` 中：

```yaml
---
description: Check whether the local Codex CLI is ready...
argument-hint: '[--enable-review-gate|--disable-review-gate]'
allowed-tools: Bash(node:*), Bash(npm:*), AskUserQuestion
---

Run:

```bash
node "${CLAUDE_PLUGIN_ROOT}/scripts/codex-companion.mjs" setup --json $ARGUMENTS
```
```

### 2.2 核心调用入口

所有 Codex 调用最终都通过 `codex-companion.mjs` 脚本完成。

**关键调用路径：**

| 命令 | 调用方式 |
|------|----------|
| `/codex:setup` | `node codex-companion.mjs setup --json $ARGUMENTS` |
| `/codex:review` | `node codex-companion.mjs review "$ARGUMENTS"` |
| `/codex:rescue` | 路由到 `codex-rescue` subagent，然后调用 `node codex-companion.mjs task ...` |
| `/codex:status` | `node codex-companion.mjs status ...` |
| `/codex:result` | `node codex-companion.mjs result ...` |
| `/codex:cancel` | `node codex-companion.mjs cancel ...` |

### 2.3 直接调用 codex CLI 的位置

Codex 插件在以下位置直接调用 `codex` 命令：

#### (1) 检查 Codex 可用性 - `lib/codex.mjs:655-673`

```javascript
export function getCodexAvailability(cwd) {
  const versionStatus = binaryAvailable("codex", ["--version"], { cwd });
  // ...
  const appServerStatus = binaryAvailable("codex", ["app-server", "--help"], { cwd });
  // ...
}
```

#### (2) 检查登录状态 - `lib/codex.mjs:694-726`

```javascript
export function getCodexLoginStatus(cwd) {
  const result = runCommand("codex", ["login", "status"], { cwd });
  // ...
}
```

#### (3) 启动 app-server - `lib/app-server.mjs:187-191`

```javascript
// SpawnedCodexAppServerClient.initialize()
this.proc = spawn("codex", ["app-server"], {
  cwd: this.cwd,
  env: this.options.env,
  stdio: ["pipe", "pipe", "pipe"]
});
```

**这是最关键的调用点** - 所有实际任务执行都通过 `codex app-server` 子进程完成。

### 2.4 环境变量传递

Codex 插件在以下位置传递环境变量：

#### (1) 直接 spawn 调用

在 `lib/app-server.mjs:187-191`：

```javascript
this.proc = spawn("codex", ["app-server"], {
  cwd: this.cwd,
  env: this.options.env,  // 传递自定义环境变量
  stdio: ["pipe", "pipe", "pipe"]
});
```

#### (2) broker 进程启动

在 `lib/broker-lifecycle.mjs:59-70`：

```javascript
export function spawnBrokerProcess({ scriptPath, cwd, endpoint, pidFile, logFile, env = process.env }) {
  const child = spawn(process.execPath, [scriptPath, "serve", ...], {
    cwd,
    env,  // 传递环境变量
    detached: true,
    stdio: ["ignore", logFd, logFd]
  });
}
```

#### (3) 环境变量来源

在 `lib/tracked-jobs.mjs:60-68`：

```javascript
export function createJobRecord(base, options = {}) {
  const env = options.env ?? process.env;
  const sessionId = env[options.sessionIdEnv ?? SESSION_ID_ENV];
  return {
    ...base,
    createdAt: nowIso(),
    ...(sessionId ? { sessionId } : {})
  };
}
```

#### (4) 关键环境变量

- `CODEX_COMPANION_APP_SERVER_ENDPOINT` - broker 端点
- `CODEX_COMPANION_APP_SERVER_PID_FILE` - PID 文件路径
- `CODEX_COMPANION_APP_SERVER_LOG_FILE` - 日志文件路径
- `CODEX_COMPANION_SESSION_ID` - 会话 ID

## 3. 可能的代理注入点

### 3.1 最佳注入点：`lib/app-server.mjs`

在 `SpawnedCodexAppServerClient.initialize()` 方法中（第 187-191 行）：

```javascript
this.proc = spawn("codex", ["app-server"], {
  cwd: this.cwd,
  env: this.options.env,
  stdio: ["pipe", "pipe", "pipe"]
});
```

**注入方式：**

修改 `this.options.env` 以包含代理配置：

```javascript
const envWithProxy = {
  ...this.options.env,
  HTTP_PROXY: "http://proxy.example.com:8080",
  HTTPS_PROXY: "http://proxy.example.com:8080",
  // 或其他 Codex CLI 支持的代理环境变量
};

this.proc = spawn("codex", ["app-server"], {
  cwd: this.cwd,
  env: envWithProxy,
  stdio: ["pipe", "pipe", "pipe"]
});
```

### 3.2 备选注入点：`lib/broker-lifecycle.mjs`

在 `spawnBrokerProcess` 函数中（第 59-70 行），可以修改传递给 broker 进程的环境变量。

### 3.3 全局环境变量方式

由于插件通过 `process.env` 继承父进程环境变量，可以在启动 Claude Code 之前设置：

```bash
export HTTP_PROXY=http://proxy.example.com:8080
export HTTPS_PROXY=http://proxy.example.com:8080
claude
```

### 3.4 配置方式建议

考虑到代理配置可能需要根据网络环境变化，建议：

1. **插件级配置**：在插件配置文件中添加代理设置
2. **环境变量继承**：依赖系统环境变量（最简单）
3. **Claude Code 配置**：通过 Claude Code 的设置机制传递代理配置

## 4. 总结

| 项目 | 详情 |
|------|------|
| **调用方式** | 通过 `node codex-companion.mjs` 脚本间接调用 |
| **实际 Codex 调用** | `spawn("codex", ["app-server"], ...)` 在 `lib/app-server.mjs:188` |
| **环境变量传递** | 通过 `options.env` 参数传递，默认继承 `process.env` |
| **最佳代理注入点** | `lib/app-server.mjs` 中的 `SpawnedCodexAppServerClient.initialize()` |
| **代理配置方式** | 修改 spawn 的 `env` 参数，或依赖全局环境变量 |
