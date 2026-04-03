# Codex 插件代理方案 1B 可行性调研报告

---
title: "Codex Plugin Proxy Solution 1B Feasibility Analysis"
code_reference: 
  - "/root/.claude/plugins/marketplaces/openai-codex/plugins/codex/scripts/lib/app-server.mjs"
  - "/root/.claude/plugins/marketplaces/openai-codex/plugins/codex/scripts/lib/broker-lifecycle.mjs"
  - "/root/.claude/plugins/marketplaces/openai-codex/plugins/codex/scripts/session-lifecycle-hook.mjs"
created_at: 2026-04-01
updated_at: 2026-04-01
maintainer: "researcher"
status: "completed"
---

## 1. 执行摘要

本报告深入调研了**方案 1B**（修改 codex 插件的 `app-server.mjs` 注入代理环境变量）的可行性和持久性。研究发现：

1. **技术上可行**：可以在 `app-server.mjs` 第 188-191 行修改 spawn 调用的 env 参数
2. **更新机制风险高**：插件通过 git 从上游仓库拉取更新，直接修改会被覆盖
3. **推荐替代方案**：使用 Claude Code 的 `settings.json` env 配置（方案 D）或 patch + 自动化脚本（方案 C）

---

## 2. 当前插件代码分析

### 2.1 关键代码位置

**文件**: `/root/.claude/plugins/marketplaces/openai-codex/plugins/codex/scripts/lib/app-server.mjs`

**spawn codex 的位置**（第 187-192 行）：

```javascript
class SpawnedCodexAppServerClient extends AppServerClientBase {
  // ...
  async initialize() {
    this.proc = spawn("codex", ["app-server"], {
      cwd: this.cwd,
      env: this.options.env,  // <-- 关键：环境变量来源
      stdio: ["pipe", "pipe", "pipe"]
    });
    // ...
  }
}
```

### 2.2 env 参数的来源和传递链

环境变量的传递路径：

1. **`CodexAppServerClient.connect()`**（第 316-331 行）
   ```javascript
   export class CodexAppServerClient {
     static async connect(cwd, options = {}) {
       let brokerEndpoint = null;
       if (!options.disableBroker) {
         // 从 options.env 或 process.env 读取 BROKER_ENDPOINT_ENV
         brokerEndpoint = options.env?.[BROKER_ENDPOINT_ENV] ?? 
                          process.env[BROKER_ENDPOINT_ENV] ?? null;
         if (!brokerEndpoint) {
           // 创建 broker session，传递 env
           const brokerSession = await ensureBrokerSession(cwd, { env: options.env });
           // ...
         }
       }
       // ...
     }
   }
   ```

2. **`ensureBrokerSession()`**（`broker-lifecycle.mjs` 第 113-171 行）
   - 接收 `options.env` 并传递给 `spawnBrokerProcess()`

3. **`spawnBrokerProcess()`**（`broker-lifecycle.mjs` 第 59-70 行）
   ```javascript
   export function spawnBrokerProcess({ scriptPath, cwd, endpoint, pidFile, logFile, env = process.env }) {
     const logFd = fs.openSync(logFile, "a");
     const child = spawn(process.execPath, [scriptPath, ...], {
       cwd,
       env,  // <-- 使用传入的 env
       detached: true,
       stdio: ["ignore", logFd, logFd]
     });
     // ...
   }
   ```

### 2.3 修改方案

**方案 1B 的具体修改**（第 187-191 行）：

```javascript
// 修改前
this.proc = spawn("codex", ["app-server"], {
  cwd: this.cwd,
  env: this.options.env,
  stdio: ["pipe", "pipe", "pipe"]
});

// 修改后 - 注入代理环境变量
this.proc = spawn("codex", ["app-server"], {
  cwd: this.cwd,
  env: {
    ...this.options.env,  // 保留原有环境变量
    HTTP_PROXY: process.env.HTTP_PROXY || "",
    HTTPS_PROXY: process.env.HTTPS_PROXY || "",
    http_proxy: process.env.http_proxy || "",
    https_proxy: process.env.https_proxy || "",
    ALL_PROXY: process.env.ALL_PROXY || "",
    all_proxy: process.env.all_proxy || "",
    NO_PROXY: process.env.NO_PROXY || "",
    no_proxy: process.env.no_proxy || ""
  },
  stdio: ["pipe", "pipe", "pipe"]
});
```

---

## 3. 插件更新机制调研

### 3.1 插件目录结构

```
/root/.claude/plugins/
├── cache/
│   └── openai-codex/
│       └── 1.0.1/                    # 缓存版本
├── marketplaces/
│   ├── claude-plugins-official/      # 官方插件
│   └── openai-codex/                 # Codex 插件（git 仓库）
│       ├── .git/                     # Git 仓库
│       ├── .claude-plugin/
│       │   └── plugin.json           # 插件清单
│       ├── package.json              # npm 配置
│       ├── plugins/
│       │   └── codex/
│       │       ├── .claude-plugin/
│       │       ├── hooks/
│       │       │   └── hooks.json    # Claude hooks 配置
│       │       ├── scripts/
│       │       │   ├── lib/
│       │       │   │   └── app-server.mjs   # <-- 目标文件
│       │       │   └── ...
│       │       └── ...
│       └── ...
```

### 3.2 插件来源和版本管理

**Git 远程仓库**：
```bash
$ cd /root/.claude/plugins/marketplaces/openai-codex && git remote -v
origin  https://github.com/openai/codex-plugin-cc.git (fetch)
origin  https://github.com/openai/codex-plugin-cc.git (push)
```

**当前版本**：1.0.1（`package.json` 和 `plugin.json`）

**Claude Code 配置**（`/root/.claude/settings.json`）：
```json
{
  "extraKnownMarketplaces": {
    "openai-codex": {
      "source": {
        "source": "github",
        "repo": "openai/codex-plugin-cc"
      }
    }
  },
  "enabledPlugins": {
    "codex@openai-codex": true
  }
}
```

### 3.3 更新机制分析

根据插件配置，Claude Code 的插件更新机制：

1. **Git 基础**：插件目录是一个完整的 git 仓库
2. **GitHub 来源**：从 `openai/codex-plugin-cc` 仓库拉取更新
3. **更新方式**：可能是以下之一：
   - Claude Code 自动检查并拉取更新
   - 用户手动触发更新（如 `/plugin update` 命令）
   - 重启 Claude Code 时自动更新

**关键风险**：
- 任何直接修改 `app-server.mjs` 的行为都会在下次 `git pull` 时被覆盖
- 如果用户运行 `git restore` 或重新克隆，修改会丢失
- 没有内置的"用户自定义补丁"机制

---

## 4. 修改持久性评估

### 4.1 直接修改源文件（方案 A）

| 维度 | 评估 |
|------|------|
| 实施难度 | 低 - 只需修改几行代码 |
| 持久性 | **极差** - 下次更新必定被覆盖 |
| 维护成本 | 高 - 每次更新后需重新修改 |
| 风险 | 高 - 可能忘记重新应用，导致代理失效 |

**结论**：不推荐作为长期方案

### 4.2 Git Fork + 自定义分支（方案 B）

| 维度 | 评估 |
|------|------|
| 实施难度 | 中 - 需要 fork 仓库并维护分支 |
| 持久性 | 高 - 修改在 fork 中永久保存 |
| 维护成本 | 中 - 需要定期同步上游更新 |
| 风险 | 中 - 可能遗漏上游重要更新 |

**实施步骤**：
1. Fork `openai/codex-plugin-cc` 到个人/组织账号
2. 在 fork 中创建 `proxy-support` 分支
3. 修改 `app-server.mjs` 注入代理变量
4. 更新 `settings.json` 指向 fork：
   ```json
   {
     "extraKnownMarketplaces": {
       "openai-codex": {
         "source": {
           "source": "github",
           "repo": "your-username/codex-plugin-cc",
           "branch": "proxy-support"
         }
       }
     }
   }
   ```

**结论**：可行，但需要长期维护 fork

### 4.3 Patch 文件 + Apply 脚本（方案 C）

| 维度 | 评估 |
|------|------|
| 实施难度 | 中 - 需要创建 patch 和自动化脚本 |
| 持久性 | 中 - 更新后自动重新应用 |
| 维护成本 | 低 - 脚本自动处理 |
| 风险 | 低 - 如果上游代码变化，patch 可能失效 |

**实施步骤**：
1. 创建 patch 文件：
   ```bash
   cd /root/.claude/plugins/marketplaces/openai-codex
   git diff plugins/codex/scripts/lib/app-server.mjs > proxy-inject.patch
   ```

2. 创建自动应用脚本：
   ```bash
   #!/bin/bash
   # apply-proxy-patch.sh
   PLUGIN_DIR="/root/.claude/plugins/marketplaces/openai-codex"
   cd "$PLUGIN_DIR"
   
   # 如果工作区有修改，先 stash
   git stash push -m "proxy-patch-stash"
   
   # 拉取更新
   git pull origin main
   
   # 应用 patch
   git apply proxy-inject.patch || {
     echo "Patch failed, manual intervention needed"
     exit 1
   }
   
   echo "Proxy patch applied successfully"
   ```

3. 添加到 shell 配置文件或 cron 定期执行

**结论**：平衡了持久性和维护成本，推荐

### 4.4 Claude Code settings.json env（方案 D）

| 维度 | 评估 |
|------|------|
| 实施难度 | 极低 - 只需修改配置文件 |
| 持久性 | **高** - 不受插件更新影响 |
| 维护成本 | 极低 - 一次配置永久有效 |
| 风险 | 低 - 依赖 Claude Code 的 env 传递机制 |

**当前 settings.json**：
```json
{
  "env": {
    "API_TIMEOUT_MS": "600000",
    "CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC": "1"
  }
}
```

**添加代理变量**：
```json
{
  "env": {
    "API_TIMEOUT_MS": "600000",
    "CLAUDE_CODE_DISABLE_NONESSENTIAL_TRAFFIC": "1",
    "HTTP_PROXY": "http://proxy.example.com:8080",
    "HTTPS_PROXY": "http://proxy.example.com:8080",
    "NO_PROXY": "localhost,127.0.0.1"
  }
}
```

**关键问题**：Claude Code 的 `settings.json` env 是否会传递给 codex 子进程？

根据代码分析：
- `SpawnedCodexAppServerClient` 使用 `this.options.env` 或默认值
- 如果 `options.env` 未设置，Node.js 的 `spawn` 默认继承 `process.env`
- Claude Code 启动时应该继承系统环境变量

**但是**：需要验证 `settings.json` 中的 env 是否会影响 `process.env`

**结论**：**最推荐** - 无需修改插件代码，配置简单持久

### 4.5 包装 codex 二进制（方案 E）

已在之前的调研中详细分析，见 `docs/research/codex_proxy_configuration_research.md`。

| 维度 | 评估 |
|------|------|
| 实施难度 | 中 - 需要创建包装脚本 |
| 持久性 | 高 - 不依赖插件代码 |
| 维护成本 | 低 - 包装脚本稳定 |
| 风险 | 低 - 但可能遗漏某些调用路径 |

---

## 5. 方案对比汇总

| 方案 | 实施难度 | 持久性 | 维护成本 | 风险 | 推荐度 |
|------|----------|--------|----------|------|--------|
| A - 直接修改 | 低 | 极差 | 高 | 高 | ⭐ |
| B - Git Fork | 中 | 高 | 中 | 中 | ⭐⭐⭐ |
| C - Patch + 脚本 | 中 | 中 | 低 | 低 | ⭐⭐⭐⭐ |
| D - settings.json | 极低 | **高** | **极低** | **低** | ⭐⭐⭐⭐⭐ |
| E - 包装二进制 | 中 | 高 | 低 | 低 | ⭐⭐⭐⭐ |

---

## 6. 推荐方案

### 首选：方案 D（settings.json env 配置）

**理由**：
1. **零代码修改** - 不需要修改插件文件
2. **完全持久** - 不受插件更新影响
3. **配置简单** - 只需编辑 JSON 文件
4. **官方支持** - 使用 Claude Code 原生配置机制

**验证步骤**：
1. 在 `settings.json` 中添加代理环境变量
2. 重启 Claude Code
3. 运行 `/codex:setup` 验证 codex 是否能通过代理访问网络

**如果方案 D 无效**：
- 可能是 Claude Code 未将 settings.json 的 env 传递给子进程
- 此时退而求其次选择 **方案 C（Patch + 脚本）**

### 备选：方案 C（Patch + 自动化脚本）

**适用场景**：
- 方案 D 验证失败
- 需要更细粒度的环境变量控制

**实施要点**：
1. 创建可靠的 patch 文件
2. 设置 Git post-merge hook 自动应用
3. 添加失败告警机制

---

## 7. 实施建议

### 立即行动

1. **测试方案 D**：
   ```bash
   # 编辑 settings.json
   cat >> /root/.claude/settings.json << 'EOF'
   {
     "env": {
       "HTTP_PROXY": "${HTTP_PROXY}",
       "HTTPS_PROXY": "${HTTPS_PROXY}",
       "NO_PROXY": "${NO_PROXY}"
     }
   }
   EOF
   ```

2. **验证生效**：
   - 重启 Claude Code
   - 运行 `/codex:setup` 检查网络连通性

### 长期方案

如果方案 D 有效：
- 文档化配置步骤
- 添加到团队 onboarding 指南

如果方案 D 无效：
- 实施方案 C
- 设置自动化监控

---

## 8. 附录：关键文件路径

| 文件 | 路径 |
|------|------|
| 插件 app-server.mjs | `/root/.claude/plugins/marketplaces/openai-codex/plugins/codex/scripts/lib/app-server.mjs` |
| 插件 broker-lifecycle.mjs | `/root/.claude/plugins/marketplaces/openai-codex/plugins/codex/scripts/lib/broker-lifecycle.mjs` |
| 插件 hooks.json | `/root/.claude/plugins/marketplaces/openai-codex/plugins/codex/hooks/hooks.json` |
| Claude Code settings | `/root/.claude/settings.json` |
| 上游仓库 | `https://github.com/openai/codex-plugin-cc.git` |

---

## 9. 结论

**方案 1B（直接修改 app-server.mjs）技术上可行，但不推荐作为长期方案**，因为：

1. 插件通过 git 管理，直接修改会被更新覆盖
2. 存在更优雅的替代方案（settings.json env 配置）
3. 维护成本高，容易遗忘重新应用修改

**最终推荐**：
- **首选方案 D**：`settings.json` env 配置
- **备选方案 C**：Patch + 自动化脚本
