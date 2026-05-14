---
title: OSS Upload Documentation Update - New DSW Quickstart
code_reference: scripts/setup_rclone_for_oss.sh, docs/operations/grscenes_oss_rclone_runbook.md
created_at: 2026-05-07
updated_at: 2026-05-07
maintainer: Codex
status: completed
---

# OSS Upload Documentation Update - New DSW Quickstart

## Summary

本次更新解决了"换了新 DSW 后如何快速恢复 OSS 上传能力"的问题。通过新增一键安装脚本和更新 runbook，实现了全新 DSW 环境下的无障碍 OSS 上传。

## Changes

### 1. 新增一键安装脚本

**文件**: `scripts/setup_rclone_for_oss.sh`

功能：
- 自动检测并安装 rclone v1.68.1+（支持 Alibaba Cloud OSS provider）
- 自动配置内网 endpoint（`oss-cn-beijing-internal.aliyuncs.com`）
- 通过环境变量或交互式输入配置 AK/SK，不在仓库中保存凭证
- 验证 bucket 连通性
- 提供清晰的踩坑说明和解决方案

使用方法：
```bash
# 使用自己的凭证
export ALIBABA_ACCESS_KEY_ID=your-key
export ALIBABA_SECRET_ACCESS_KEY=your-secret
bash scripts/setup_rclone_for_oss.sh

# 或交互式输入凭证
bash scripts/setup_rclone_for_oss.sh

# 非交互式（CI/自动化）
bash scripts/setup_rclone_for_oss.sh --non-interactive
```

### 2. 更新 GRScenes OSS Rclone Runbook

**文件**: `docs/operations/grscenes_oss_rclone_runbook.md`

新增内容：
- **New DSW Quickstart** 章节：从零到上传的完整步骤
- **踩坑记录表**：5 个主要问题的现象和解决方案
  1. apt 版本太老（v1.53.3 不支持 Alibaba Cloud）
  2. DSW 外网下载极慢（~20 KB/s）
  3. IP 白名单（公网 endpoint 返回 403）
  4. 代理变量导致 hang（非交互 shell 中）
  5. 权限不足（AccessDenied）

### 3. 更新文档索引

**文件**: `docs/INDEX.md`

- 更新 runbook 最后更新日期为 2026-05-07
- 新增相关代码引用：`setup_rclone_for_oss.sh`

## Problems Solved

| 问题 | 之前的做法 | 现在的做法 |
|---|---|---|
| 新 DSW 没有 rclone | 手动 apt install，然后发现版本不对 | 运行脚本，自动安装正确版本 |
| 不知道 endpoint 该用哪个 | 试错公网 endpoint，被 403 | 脚本自动配置内网 endpoint |
| 不知道版本要求 | apt 装的 v1.53.3 不支持 Alibaba Cloud | 脚本检测并升级 |
| 代理导致 hang | 排查很久才发现是代理问题 | 脚本自动用 `env -u` 绕过 |
| 下载速度极慢 | 等 15+ 分钟下载 21MB | 脚本支持预下载包检测 |

## Testing

- 脚本语法检查通过：`bash -n scripts/setup_rclone_for_oss.sh`
- 脚本已赋予执行权限：`chmod +x scripts/setup_rclone_for_oss.sh`
- Runbook 格式验证通过（YAML frontmatter + Markdown）

## Next Steps

下次换 DSW 时：
1. 打开新 DSW
2. 运行 `bash scripts/setup_rclone_for_oss.sh`
3. 按脚本提示操作
4. 即可开始 OSS 上传

## References

- [GRScenes OSS Rclone Runbook](../../operations/grscenes_oss_rclone_runbook.md)
- [Project OSS Commands](../../../../../../.config/opencode/skills/oss-rclone-ops/references/project-commands.md)
