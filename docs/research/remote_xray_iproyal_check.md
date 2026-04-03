---
title: Remote Xray and Iproyal Configuration Check
code_reference: xray-linux-amd64, iproyal proxy
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: remote-agent
status: completed
---

# Remote Xray 服务器和 Iproyal 代理配置检查报告

## 执行时间
2026-04-03

## 检查目标
- SSH 主机: 45.32.95.77
- Xray 工作目录: /usr/local/x-ui
- Xray HTTP 入站端口: 18443
- Iproyal 代理: 168.158.155.72:12323

---

## 1. Xray 配置检查

### 配置文件位置
`/usr/local/x-ui/bin/config.json`

### HTTP 入站配置 (18443 端口)
```json
{
  "listen": "0.0.0.0",
  "port": 18443,
  "protocol": "http",
  "settings": {
    "accounts": [
      {
        "user": "claudeproxy",
        "pass": "f2YAiSmLrUW9KjJTP7cfIwrw"
      }
    ],
    "allowTransparent": false
  },
  "tag": "inbound-18443"
}
```

**状态**: 配置正确
- 监听地址: 0.0.0.0 (所有接口)
- 端口: 18443
- 协议: HTTP
- 认证: 已启用 (claudeproxy/f2YAiSmLrUW9KjJTP7cfIwrw)

### Iproyal 出站配置
```json
{
  "tag": "iproyal",
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
```

**状态**: 配置正确
- 代理地址: 168.158.155.72:12323
- 认证: 已配置

### 路由规则
```json
{
  "type": "field",
  "inboundTag": ["inbound-18443"],
  "outboundTag": "iproyal"
}
```

**状态**: 路由规则正确 - 18443 端口的流量被路由到 iproyal 出站

---

## 2. Iproyal 代理状态检查

### 直接连接测试 (从远程服务器)

#### 测试 1: HTTP 基本连接
```bash
curl -x http://14af8dc6c6fe7:b83f83bed2@168.158.155.72:12323 http://httpbin.org/ip
```
**结果**: 成功
```json
{
  "origin": "168.158.155.72"
}
```

#### 测试 2: HTTPS 连接 (OpenAI API)
```bash
curl -x http://14af8dc6c6fe7:b83f83bed2@168.158.155.72:12323 https://api.openai.com/v1/models
```
**结果**: 成功 (返回 401，说明需要认证，但代理连接正常)

#### 测试 3: 带认证头的 API 测试
```bash
curl -x http://14af8dc6c6fe7:b83f83bed2@168.158.155.72:12323 \
  https://api.openai.com/v1/models \
  -H 'Authorization: Bearer sk-fake'
```
**结果**: HTTP 401 (预期行为，API key 无效但连接成功)
```json
{
  "error": {
    "message": "Incorrect API key provided: sk-fake...",
    "type": "invalid_request_error",
    "code": "invalid_api_key"
  }
}
```

#### 测试 4: Anthropic API
```bash
curl -x http://14af8dc6c6fe7:b83f83bed2@168.158.155.72:12323 \
  https://api.anthropic.com/v1/models
```
**结果**: HTTP 401 (预期行为，需要 x-api-key 头)
```json
{
  "type": "error",
  "error": {
    "type": "authentication_error",
    "message": "x-api-key header is required"
  }
}
```

#### 测试 5: 网站访问测试
- **Google**: 成功
- **Anthropic 官网**: 成功
- **GitHub API**: 成功
- **ChatGPT (chat.openai.com)**: 返回 308 重定向 (正常)
- **OpenAI Platform (platform.openai.com)**: Cloudflare 挑战页面 (正常，需要浏览器)
- **Claude.ai**: Cloudflare 挑战页面 (正常，需要浏览器)

**结论**: Iproyal 代理工作正常，可以访问 OpenAI API 和 Anthropic API。

---

## 3. Xray 代理状态检查

### 进程状态
```
root 30626 0.0 2.3 1263680 22516 ? Sl Mar24 6:57 bin/xray-linux-amd64 -c bin/config.json
```
**状态**: 进程正常运行 (PID 30626，自 Mar 24 启动)

### 端口监听状态
```
LISTEN 0 4096 127.0.0.1:62789 (api)
LISTEN 0 4096 127.0.0.1:11111 (metrics)
LISTEN 0 4096 *:443 (vless inbound)
LISTEN 0 4096 *:18443 (http inbound)
```
**状态**: 18443 端口正常监听

### Xray 代理测试 (本地回环)

#### 测试 1: HTTP 基本连接
```bash
curl -x http://claudeproxy:f2YAiSmLrUW9KjJTP7cfIwrw@127.0.0.1:18443 http://httpbin.org/ip
```
**结果**: 成功 (返回 iproyal IP: 168.158.155.72)

#### 测试 2: IP 信息
```bash
curl -x http://claudeproxy:f2YAiSmLrUW9KjJTP7cfIwrw@127.0.0.1:18443 https://ipinfo.io/json
```
**结果**: 
```json
{
  "ip": "168.158.155.72",
  "city": "Mesa",
  "region": "Arizona",
  "country": "US",
  "org": "AS20012 Interworks Networking Services"
}
```

#### 测试 3: OpenAI API
```bash
curl -x http://claudeproxy:f2YAiSmLrUW9KjJTP7cfIwrw@127.0.0.1:18443 \
  https://api.openai.com/v1/models \
  -H 'Authorization: Bearer sk-fake'
```
**结果**: HTTP 401 (预期行为，API key 无效但代理连接成功)

#### 测试 4: Anthropic API
```bash
curl -x http://claudeproxy:f2YAiSmLrUW9KjJTP7cfIwrw@127.0.0.1:18443 \
  https://api.anthropic.com/v1/models
```
**结果**: HTTP 401 (预期行为，需要 x-api-key 头)

**结论**: Xray HTTP 入站代理工作正常，流量正确转发到 iproyal。

---

## 4. 日志检查

### Xray/x-ui 服务日志
```bash
journalctl -u x-ui --since '1 hour ago' --no-pager
```
**结果**: 无最近日志条目 (服务稳定运行，无错误)

### 访问日志
配置中 `log.access` 设置为 `"none"`，未记录访问日志。

### 错误日志
配置中 `log.error` 设置为空字符串，未启用错误日志记录。

---

## 5. 关键问题解答

### Q1: Iproyal 代理是否正常工作?
**答案**: 是。所有测试均通过：
- HTTP/HTTPS 连接正常
- 可以访问 api.openai.com (返回 401 是预期行为，说明代理连接成功)
- 可以访问 api.anthropic.com
- 出口 IP: 168.158.155.72 (美国 Arizona)

### Q2: Iproyal 是否被 OpenAI 限制?
**答案**: 否。测试显示：
- 可以正常连接到 api.openai.com
- 返回 401 是因为没有提供有效的 API key，这是预期行为
- 如果 IP 被限制，会返回 403 或连接被拒绝

### Q3: Xray 配置是否正确?
**答案**: 是。配置正确：
- HTTP 入站 (18443) 配置正确
- Iproyal 出站配置正确
- 路由规则正确
- 进程正常运行
- 端口监听正常

---

## 6. 修复建议

当前配置**无需修复**，所有组件工作正常。

### 可选优化建议

1. **启用访问日志** (用于调试)
   ```json
   "log": {
     "access": "/var/log/xray/access.log",
     "error": "/var/log/xray/error.log",
     "loglevel": "warning"
   }
   ```

2. **添加健康检查路由** (可选)
   可以添加一个直接出站 (direct) 的路由用于测试：
   ```json
   {
     "type": "field",
     "inboundTag": ["inbound-18443"],
     "domain": ["httpbin.org", "ipinfo.io"],
     "outboundTag": "direct"
   }
   ```

3. **备份配置**
   当前已有备份: `/usr/local/x-ui/bin/config.json.bak-20260324-031636`

---

## 7. 本地 SSH 隧道测试

从本地机器通过 SSH 隧道测试代理链：

```bash
# 建立 SSH 隧道
ssh -i ~/.ssh/claude_vps_proxy_ed25519 -N -L 18443:127.0.0.1:18443 root@45.32.95.77

# 在另一个终端测试
curl -x http://claudeproxy:f2YAiSmLrUW9KjJTP7cfIwrw@127.0.0.1:18443 \
  https://api.openai.com/v1/models \
  -H 'Authorization: Bearer YOUR_API_KEY'
```

---

## 总结

| 检查项 | 状态 | 备注 |
|--------|------|------|
| Xray 进程 | 正常 | PID 30626，稳定运行 |
| Xray 配置 | 正确 | HTTP 18443 入站配置正确 |
| Iproyal 出站 | 正确 | 代理配置正确 |
| 路由规则 | 正确 | 18443 → iproyal |
| 端口监听 | 正常 | 0.0.0.0:18443 监听中 |
| Iproyal 连接 | 正常 | 可以访问 OpenAI/Anthropic API |
| Xray 代理功能 | 正常 | 本地测试通过 |
| OpenAI 限制 | 无 | IP 未被限制 |

**最终结论**: 代理链 `本地 → SSH 隧道 → Xray (18443) → iproyal → OpenAI` 配置正确，所有组件工作正常。如果遇到连接问题，请检查本地 SSH 隧道是否正确建立。