---
title: Claude 美区住宅 IP 代理部署完整文档
created_at: 2026-03-24
updated_at: 2026-03-24
maintainer: zhuzihou
status: sensitive
doc_class: archive
---

# Claude 美区住宅 IP 代理部署完整文档

> ⚠️ **安全警告**：本文档包含所有真实凭据，请妥善保管，勿泄露！
> 
> 部署时间：2026-03-24
> 部署路径：Vultr Los Angeles + IPRoyal Arizona Residential

---

## 📋 资源清单

### 1. IPRoyal 静态住宅 IP（入口代理）

```yaml
服务商: IPRoyal
类型: ISP Static Residential Proxy
IP地址: 168.158.155.72
端口: 12323
协议: HTTP/HTTPS (实测 HTTP 可用，SOCKS5 被禁用)
用户名: 14af8dc6c6fe7
密码: b83f83bed2
地理位置: Mesa, Arizona, US
ISP: Interworks Networking Services (AS20012)
过期时间: 2026-05-23
月费: $2.40
```

### 2. Vultr VPS（中转服务器）

```yaml
服务商: Vultr
类型: Cloud Compute - Shared CPU
机房: Los Angeles, California (LAX)
IP地址: 45.32.95.77
操作系统: Ubuntu 22.04 LTS x64
用户名: root
密码: zN]3PoTxnMj#EtP$
SSH端口: 22
配置: 1 vCPU / 1 GB RAM / 25 GB SSD
月费: $6.00
主机名: claude
```

### 3. 3X-UI 面板（Xray 管理）

```yaml
访问地址: https://45.32.95.77:55555/yX3Lb249F7aJomBtAE
协议: HTTPS (自签名证书，浏览器会提示不安全，点击继续访问)
端口: 55555
WebBasePath: yX3Lb249F7aJomBtAE
用户名: S0U8XUs0EO
密码: jzZ9ykIYXT
Xray版本: 26.2.6
面板版本: v2.8.11
```

### 4. VPS 标准 HTTP 代理入口（供 Claude CLI 使用）

```yaml
备注: claude-http-proxy
协议: HTTP Proxy
监听地址: 45.32.95.77
端口: 18443
用户名: claudeproxy
密码: f2YAiSmLrUW9KjJTP7cfIwrw
对应入站标签: inbound-18443
出站路由: 强制走 IPRoyal 住宅 IP
```

---

## 🔗 客户端连接信息

### VLESS + Reality 节点链接

```
vless://27012732-5f1e-4a6e-840e-e67550d542a3@45.32.95.77:443?type=tcp&encryption=none&security=reality&pbk=Ib5lfAD2NuiVwv-1yYN_1g7fko1wBiZeCq3wZ56vnRo&fp=chrome&sni=amazon.com&sid=d6b06750&spx=%2F&flow=xtls-rprx-vision#claude
```

### 节点参数详情

| 参数 | 值 | 说明 |
|------|-----|------|
| 协议 | VLESS | 高性能代理协议 |
| UUID | 27012732-5f1e-4a6e-840e-e67550d542a3 | 用户唯一标识 |
| 地址 | 45.32.95.77 | VPS 公网 IP |
| 端口 | 443 | HTTPS 标准端口，防封锁 |
| 加密 | none | VLESS 默认 |
| 流控 | xtls-rprx-vision | XTLS 最新流控，降低延迟 |
| 安全 | reality | Reality 伪装，防检测 |
| SNI | amazon.com | 伪装目标域名 |
| PublicKey | Ib5lfAD2NuiVwv-1yYN_1g7fko1wBiZeCq3wZ56vnRo | Reality 公钥 |
| ShortID | d6b06750 | Reality 短 ID |
| 指纹 | chrome | 模拟 Chrome TLS 指纹 |

### 当前 Bash 环境中的 Claude 命令入口

当前机器的 `~/.bashrc` 中已经补充了 3 个清晰区分的命令入口：

| 命令 | 作用 | 是否走中转 | 说明 |
|------|------|------------|------|
| `claude_official_direct` | 官方原始入口 | 否 | 只清理 `ANTHROPIC_*` 环境变量，不挂代理 |
| `claude_official_local_tunnel` | 本地 SSH 隧道入口 | 是 | 本地 `127.0.0.1:28790` → SSH 转发到 VPS `18443` HTTP 代理 |
| `claude_official_vps_proxy` | 直接使用 VPS HTTP 代理 | 是 | `HTTP_PROXY/HTTPS_PROXY` 直接指向 `45.32.95.77:18443` |
| `claude_official` | 默认官方入口 | 是 | 当前已改为默认走 `claude_official_local_tunnel`，避免误用直连 |

兼容入口说明：

- `claude_official` 当前默认等价于 `claude_official_local_tunnel`
- 也就是说，直接执行 `claude_official` 就会优先走本地 SSH 隧道中转
- 如果你需要显式直连排查，请改用：
  - `claude_official_direct`
- 如果你想显式跳过本地隧道、直接使用 VPS 标准代理，请改用：
  - `claude_official_vps_proxy`

本地辅助命令：

```bash
claude_official_local_tunnel_start
claude_official_local_tunnel_status
claude_official_local_tunnel_stop
```

相关配置文件：

```bash
~/.config/claude-providers/official-proxy.env
```

### 当前 Bash 环境中的 Codex 命令入口

2026-04-09 更新：Codex 的最终可用路径已经和 Claude 分流，不再复用当前 `18443 -> IPRoyal` 住宅代理链路。

当前机器中与 Codex 相关的实际入口如下：

| 命令 | 作用 | 是否走中转 | 说明 |
|------|------|------------|------|
| `codex` | Codex 官方默认入口 | 否 | 当前官方 npm 全局安装，直接执行 `/usr/local/bin/codex` |
| `codex_vps` | Codex 专用代理入口 | 是 | 本地 `127.0.0.1:28791` → SSH 转发到 VPS `127.0.0.1:18445` tinyproxy → VPS 直连 OpenAI/ChatGPT |

本地辅助命令：

```bash
codex_vps_tunnel_start
codex_vps_tunnel_status
codex_vps_tunnel_stop
```

结论说明：

- 当前 `codex_vps` **不是**走住宅 IP 链路。
- 具体来说，它**不是**通过 `45.32.95.77:18443 -> IPRoyal 168.158.155.72` 访问 OpenAI/ChatGPT。
- 最终可用路径是：`本地 SSH 隧道 -> VPS tinyproxy -> VPS 原生出口 -> OpenAI/ChatGPT`。
- 这不等于“所有住宅 IP 都不能用 ChatGPT/OpenAI”；只表示**当前这条 IPRoyal 住宅代理链在本部署中没有跑通 Codex/OpenAI/ChatGPT**。
- 因此当前策略是：Claude 继续走住宅链路，Codex 改为走 VPS 直连链路。

---

## ⚙️ 服务端配置文件

### Xray 核心配置 (/usr/local/x-ui/bin/config.json)

```json
{
  "log": {
    "access": "none",
    "dnsLog": false,
    "error": "",
    "loglevel": "warning",
    "maskAddress": ""
  },
  "routing": {
    "domainStrategy": "AsIs",
    "rules": [
      {
        "type": "field",
        "inboundTag": ["api"],
        "outboundTag": "api"
      },
      {
        "type": "field",
        "outboundTag": "blocked",
        "ip": ["geoip:private"]
      },
      {
        "type": "field",
        "outboundTag": "blocked",
        "protocol": ["bittorrent"]
      },
      {
        "type": "field",
        "domain": [
          "claude.ai",
          "anthropic.com",
          "*.claude.ai",
          "*.anthropic.com"
        ],
        "outboundTag": "iproyal-residential"
      }
    ]
  },
  "dns": null,
  "inbounds": [
    {
      "listen": "127.0.0.1",
      "port": 62789,
      "protocol": "tunnel",
      "settings": {
        "address": "127.0.0.1"
      },
      "streamSettings": null,
      "tag": "api",
      "sniffing": null
    }
  ],
  "outbounds": [
    {
      "tag": "direct",
      "protocol": "freedom",
      "settings": {
        "domainStrategy": "AsIs",
        "redirect": "",
        "noises": []
      }
    },
    {
      "tag": "blocked",
      "protocol": "blackhole",
      "settings": {}
    },
    {
      "tag": "iproyal-residential",
      "protocol": "http",
      "settings": {
        "servers": [
          {
            "address": "168.158.155.72",
            "port": 12323,
            "users": [
              {
                "user": "14af8dc6c6fe7",
                "pass": "b83f83bed2"
              }
            ]
          }
        ]
      }
    }
  ],
  "transport": null,
  "policy": {
    "levels": {
      "0": {
        "statsUserDownlink": true,
        "statsUserUplink": true
      }
    },
    "system": {
      "statsInboundDownlink": true,
      "statsInboundUplink": true,
      "statsOutboundDownlink": false,
      "statsOutboundUplink": false
    }
  },
  "api": {
    "tag": "api",
    "services": [
      "HandlerService",
      "LoggerService",
      "StatsService"
    ]
  },
  "stats": {},
  "reverse": null,
  "fakedns": null,
  "observatory": null,
  "burstObservatory": null,
  "metrics": {
    "tag": "metrics_out",
    "listen": "127.0.0.1:11111"
  }
}
```

**关键配置说明：**
- **入站**: VLESS + Reality @ 端口 443
- **新增入站**: HTTP Proxy @ 端口 18443（专门给 Claude CLI / curl / shell 命令走标准代理）
- **出站分流**: 
  - `claude.ai` / `anthropic.com` → 走 IPRoyal 住宅 IP
  - 其他流量 → 直连（VPS 原生 IP）
- **协议修正**: 使用 HTTP 代理连接 IPRoyal（SOCKS5 在该端口被禁用）
- **命令行专用路由**:
  - `inbound-443` → `iproyal`
  - `inbound-18443` → `iproyal`

### 命令行中转链路说明

当前已经同时支持两种 Claude CLI 中转路径：

#### 方案 A：本地隧道（推荐）

```text
claude_official_local_tunnel
  -> 本地 127.0.0.1:28790
  -> SSH LocalForward
  -> VPS 45.32.95.77:18443 HTTP Proxy
  -> IPRoyal 168.158.155.72
```

特点：

- 不需要在本地安装 VLESS 客户端
- 本地只暴露 `127.0.0.1` 端口，不对公网开放
- 适合长期作为默认中转方式使用

#### 方案 B：直接走 VPS 标准代理

```text
claude_official_vps_proxy
  -> VPS 45.32.95.77:18443 HTTP Proxy
  -> IPRoyal 168.158.155.72
```

特点：

- 配置最简单
- 不依赖本地 SSH 隧道
- 适合快速验证或临时使用

#### 环境变量行为说明

以上两个命令都会在启动 `claude` 之前：

- 清理 `ANTHROPIC_API_KEY`
- 清理 `ANTHROPIC_BASE_URL`
- 清理 `ANTHROPIC_AUTH_TOKEN`
- 设置 `http_proxy` / `https_proxy`

因此它们仍然走**官方 Claude Code 登录方式**，只是网络出口改成了中转链路。

---

## 🖥️ Bash / Claude CLI 使用指南

### 1. 让新命令生效

```bash
source ~/.bashrc
```

这一步对 `claude_official` 和 `codex` 都生效。

### 2. 直接使用 VPS HTTP 代理

```bash
claude_official_vps_proxy
```

适用场景：

- 只想最快跑通中转
- 不想额外维护本地 SSH 隧道

### 3. 通过本地 SSH 隧道使用（推荐）

```bash
claude_official_local_tunnel
```

这个命令会自动确保本地 `127.0.0.1:28790` 可用，然后再启动 `claude`。

如果你平时直接敲：

```bash
claude_official
```

当前也会默认走这一条本地隧道链路。

Codex 当前对应写法：

```bash
codex
# 官方默认直连入口

codex_vps
# Codex 专用代理入口：127.0.0.1:28791 -> VPS 127.0.0.1:18445 tinyproxy -> VPS 直连 OpenAI/ChatGPT
```

如需手动控制 Codex 隧道：

```bash
codex_vps_tunnel_start
codex_vps_tunnel_status
codex_vps_tunnel_stop
```

Claude 如需手动控制：

```bash
claude_official_local_tunnel_start
claude_official_local_tunnel_status
claude_official_local_tunnel_stop
```

### 4. 仅在排查问题时使用直连入口

```bash
claude_official_direct
codex
# Codex 当前默认就是官方直连入口
```

注意：

- `claude_official_direct` 和 `codex` 都**不会**走住宅 IP 中转
- 如果目标是让 Codex 始终通过中转访问，请使用 `codex_vps`

### 5. 直接使用 VPS HTTP 代理的显式命令

```bash
claude_official_vps_proxy
```

注意：

- 当前 `codex_vps` 不再使用 `18443` 这个 Claude 专用住宅代理入口。
- Codex 现在使用的是单独的本地隧道 + VPS `127.0.0.1:18445` tinyproxy 直连路径。

### 6. curl 验证命令

验证 VPS 标准代理：

```bash
curl -x http://claudeproxy:f2YAiSmLrUW9KjJTP7cfIwrw@45.32.95.77:18443 https://api.ipify.org
```

验证本地 SSH 隧道：

```bash
curl -x http://claudeproxy:f2YAiSmLrUW9KjJTP7cfIwrw@127.0.0.1:28790 https://api.ipify.org
```

预期返回：

```text
168.158.155.72
```

---

## 💻 客户端使用指南

### Windows (v2rayN)

**1. 下载安装**
- 地址：https://github.com/2dust/v2rayN/releases
- 选择：`v2rayN-windows-64-desktop.zip`
- 解压到任意目录，运行 `v2rayN.exe`

**2. 导入节点**
- 复制链接：`vless://27012732-5f1e-4a6e-840e-e67550d542a3@45.32.95.77:443?type=tcp&encryption=none&security=reality&pbk=Ib5lfAD2NuiVwv-1yYN_1g7fko1wBiZeCq3wZ56vnRo&fp=chrome&sni=amazon.com&sid=d6b06750&spx=%2F&flow=xtls-rprx-vision#claude`
- 在 v2rayN 主界面按 `Ctrl+V`
- 自动识别并添加节点

**3. 连接使用**
- 右键节点 `claude` → 设为活动服务器
- 右键托盘图标 → 系统代理 → 自动配置系统代理
- 浏览器访问 https://ipinfo.io 验证 IP 是否为 `168.158.155.72`

### macOS (V2RayU)

```bash
# 1. 安装
brew install --cask v2rayu

# 2. 导入节点
# 复制 vless:// 链接，点击菜单栏 V2RayU 图标 → 配置 → 导入

# 3. 启用
点击菜单栏图标 → 系统代理模式 → 全局模式
```

### iOS (Shadowrocket)

```
1. App Store (美区) 购买 Shadowrocket ($2.99)
2. 点击右上角 "+"
3. 类型选择 "VLESS"
4. 手动填写：
   - 地址: 45.32.95.77
   - 端口: 443
   - UUID: 27012732-5f1e-4a6e-840e-e67550d542a3
   - 流控: xtls-rprx-vision
   - 传输: tcp
   - 安全: reality
   - SNI: amazon.com
   - PublicKey: Ib5lfAD2NuiVwv-1yYN_1g7fko1wBiZeCq3wZ56vnRo
   - ShortID: d6b06750
5. 开启连接
```

### Clash.Meta (mihomo) 配置

```yaml
proxies:
  - name: "claude-vps"
    type: vless
    server: 45.32.95.77
    port: 443
    uuid: 27012732-5f1e-4a6e-840e-e67550d542a3
    flow: xtls-rprx-vision
    network: tcp
    tls: true
    servername: amazon.com
    reality-opts:
      public-key: Ib5lfAD2NuiVwv-1yYN_1g7fko1wBiZeCq3wZ56vnRo
      short-id: d6b06750
    client-fingerprint: chrome
    udp: true

proxy-groups:
  - name: "PROXY"
    type: select
    proxies:
      - "claude-vps"
      - "DIRECT"

rules:
  - DOMAIN-SUFFIX,claude.ai,PROXY
  - DOMAIN-SUFFIX,anthropic.com,PROXY
  - DOMAIN-SUFFIX,claudeusercontent.com,PROXY
  - MATCH,DIRECT
```

---

## 🧪 验证测试

### 1. IP 出口测试

```bash
# 应该返回 168.158.155.72 (Mesa, Arizona, US)
curl -s https://ipinfo.io
```

**预期输出：**
```json
{
  "ip": "168.158.155.72",
  "city": "Mesa",
  "region": "Arizona",
  "country": "US",
  "loc": "33.4223,-111.8226",
  "org": "AS20012 Interworks Networking Services",
  "postal": "85210",
  "timezone": "America/Phoenix"
}
```

### 2. Claude 访问测试

```bash
# 通过 VPS 标准代理测试出口
curl -x http://claudeproxy:f2YAiSmLrUW9KjJTP7cfIwrw@45.32.95.77:18443 https://api.ipify.org

# 通过本地 SSH 隧道测试出口
curl -x http://claudeproxy:f2YAiSmLrUW9KjJTP7cfIwrw@127.0.0.1:28790 https://api.ipify.org
```

**预期输出：**

```text
168.158.155.72
```

### 3. DNS 泄露测试

访问：https://www.dnsleaktest.com/
- 应该显示美国 DNS 服务器
- 不应出现中国 DNS

---

## 🔐 安全与维护

### 备份文件位置

```bash
/etc/x-ui/x-ui.db.bak-20260324-034153-before-claude-inbound
/etc/x-ui/x-ui.db.bak-20260324-032253
/usr/local/x-ui/bin/config.json.bak-20260324-031636
```

### 常用管理命令

```bash
# SSH 登录
ssh root@45.32.95.77
# 密码: zN]3PoTxnMj#EtP$

# 查看 Xray 状态
x-ui status

# 重启服务
x-ui restart

# 查看日志
x-ui log

# 更新面板
x-ui update
```

### 防火墙规则

```bash
# 已开放的端口
ufw allow 22/tcp      # SSH
ufw allow 443/tcp     # VLESS Reality
ufw allow 18443/tcp   # HTTP Proxy (Claude CLI / curl)
ufw allow 55555/tcp   # 3X-UI 面板
ufw allow 80/tcp      # HTTP (ACME 证书验证)
```

---

## 📱 Claude 账号注册与使用

### 注册流程

```
1. 确保 v2rayN 已连接，IP 显示为 168.158.155.72
2. 浏览器打开 https://claude.ai
3. 使用 Gmail / Outlook 邮箱注册
4. 验证手机号（推荐 Google Voice 或其他海外号码）
5. 完成注册
```

### 升级 Claude Pro（推荐 iOS 内购）

```
1. 准备美区 Apple ID
2. 支付宝购买美区礼品卡（支付宝 → 切换旧金山 → 搜索 Pockyt Shop）
3. 充值到 Apple ID
4. iPhone 下载 Claude App（美区 App Store）
5. App 内订阅 Claude Pro ($20/月)
6. 订阅后网页版自动同步 Pro 权限
```

**注意：**
- 避免使用 +86 手机号注册（可能触发风控）
- 保持 IP 稳定，不要频繁切换节点
- 系统时区建议改为 America/Los_Angeles
- 浏览器语言设置为 en-US

---

## ⚠️ 故障排查

### 问题 1：无法连接节点

```bash
# 检查 VPS 防火墙
ufw status
# 确保 443 端口开放

# 检查 Xray 是否运行
x-ui status

# 查看错误日志
x-ui log
```

### 问题 2：能连接但 IP 不对（显示 45.32.95.77）

```bash
# 检查出站规则配置
cat /usr/local/x-ui/bin/config.json | grep -A 10 "iproyal"

# 检查 IPRoyal 代理是否可用
curl -x http://14af8dc6c6fe7:b83f83bed2@168.158.155.72:12323 https://ipinfo.io

# 检查 VPS HTTP 代理是否可用
curl -x http://claudeproxy:f2YAiSmLrUW9KjJTP7cfIwrw@45.32.95.77:18443 https://api.ipify.org

# 检查本地 SSH 隧道是否可用
source ~/.bashrc
claude_official_local_tunnel_status
curl -x http://claudeproxy:f2YAiSmLrUW9KjJTP7cfIwrw@127.0.0.1:28790 https://api.ipify.org
```

如果本地 `127.0.0.1:28790` 返回连接失败：

- 先执行 `claude_official_local_tunnel_start`
- 再执行 `claude_official_local_tunnel_status`
- 确认状态是 `running`

### 问题 3：Claude 提示地区不支持

- 检查 IP 是否为住宅 IP（不是 VPS IP）
- 清除浏览器 Cookie 和缓存
- 使用隐私模式/无痕模式访问
- 检查 DNS 是否泄露

---

## 💰 费用统计

| 项目 | 月费 | 备注 |
|------|------|------|
| IPRoyal 住宅 IP | $2.40 | 静态 ISP 代理，到期 2026-05-23 |
| Vultr VPS | $6.00 | 1GB RAM，Los Angeles 机房 |
| **总计** | **$8.40/月** | 约 ¥60 RMB |

---

## 📞 关键信息速查

```
【住宅 IP 代理】
IP: 168.158.155.72:12323
User: 14af8dc6c6fe7
Pass: b83f83bed2

【VPS SSH】
IP: 45.32.95.77
User: root
Pass: zN]3PoTxnMj#EtP$

【面板管理】
URL: https://45.32.95.77:55555/yX3Lb249F7aJomBtAE
User: S0U8XUs0EO
Pass: jzZ9ykIYXT

【VPS HTTP 代理】
Proxy: http://45.32.95.77:18443
User: claudeproxy
Pass: f2YAiSmLrUW9KjJTP7cfIwrw

【本地 SSH 隧道代理】
Proxy: http://127.0.0.1:28790
Auth : 使用同一组 HTTP 代理账号
Start: claude_official_local_tunnel_start
Check: claude_official_local_tunnel_status
Stop : claude_official_local_tunnel_stop

【Bash 命令入口】
Direct      : claude_official_direct
LocalTunnel : claude_official_local_tunnel
VPSProxy    : claude_official_vps_proxy
Compat      : claude_official (= local_tunnel)

【Codex 命令入口】
Default     : codex
Proxy       : codex_vps
TunnelStart : codex_vps_tunnel_start
TunnelCheck : codex_vps_tunnel_status
TunnelStop  : codex_vps_tunnel_stop
Note        : Codex 当前不走住宅 IP 18443 链路，而是走 VPS 直连 OpenAI/ChatGPT

【VLESS 节点】
vless://27012732-5f1e-4a6e-840e-e67550d542a3@45.32.95.77:443?type=tcp&encryption=none&security=reality&pbk=Ib5lfAD2NuiVwv-1yYN_1g7fko1wBiZeCq3wZ56vnRo&fp=chrome&sni=amazon.com&sid=d6b06750&spx=%2F&flow=xtls-rprx-vision#claude
```

---

*文档生成时间: 2026-03-24*  
*部署版本: v1.1*  
*维护者: root@claude (45.32.95.77)*
