---
title: GRScenes OSS Rclone Runbook
code_reference: skills/oss-rclone-ops/SKILL.md
created_at: 2026-03-14
updated_at: 2026-03-14
maintainer: Codex
status: Active
---

# GRScenes OSS Rclone Runbook

## Summary

本页定义当前项目通过 `rclone` 操作阿里云 OSS 的标准做法，覆盖只读探测、上传、下载、同步、校验与受控清理。本文档面向当前环境和当前 bucket 结构，不尝试抽象成通用 S3 教程。

## Environment Baseline

- 日期：2026-03-14
- 已验证工具：`rclone v1.68.1`
- 当前机器未发现：`ossutil`、`aws`、`s3cmd`
- 当前可见 remote：`aliyun-a-oss-demo:`
- 本文统一以 `aliyun-a-oss-demo:` 为准；若历史记录里出现 `aliyun-oss-demo:`，按旧别名或口误处理

## Safety Rules

- 禁止在终端记录、文档、聊天、截图中展示 `rclone config show` 输出。
- 默认复用本机已有 `rclone` remote，不在本仓库记录明文 Access Key 或 Secret。
- 任何会改动远端状态的操作，优先执行 `--dry-run` 或等价只读探测。
- `sync` 只用于“目标端必须镜像源端”的场景；普通上传优先用 `copy`。
- `delete`、`deletefile`、`purge` 执行前，必须先用 `lsjson` 或 `lsf` 确认精确作用范围。
- 如果 `rclone` 只在 agent/自动化 shell 里挂起，而在交互 shell 里正常，优先排查并临时去掉 `HTTP_PROXY`、`HTTPS_PROXY`、`ALL_PROXY` 及其小写变体。

## Standard Variables

```bash
REMOTE='aliyun-a-oss-demo:'
BUCKET_ROOT='pjlab-bjpai-zhuzihou-assets'
DATASET='GRScenes/GRScenes-test1'
LOCAL_ROOT='/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test1'
REMOTE_PATH="${REMOTE}${BUCKET_ROOT}/${DATASET}"
```

## Read-Only Probes

先确认工具和 remote 存在：

```bash
rclone version
rclone listremotes
rclone backend features "${REMOTE}"
```

推荐的远端探测顺序：

```bash
timeout 20s rclone lsf "${REMOTE}${BUCKET_ROOT}"
timeout 20s rclone lsf "${REMOTE}${BUCKET_ROOT}/GRScenes"
timeout 20s rclone lsjson "${REMOTE_PATH}" --max-depth 1
timeout 20s rclone size "${REMOTE_PATH}"
```

如果自动化执行通道存在代理注入且 `rclone` 在创建 backend 后挂起，可先用无代理前缀复测：

```bash
env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy -u ALL_PROXY -u all_proxy \
  rclone lsf "${REMOTE}${BUCKET_ROOT}"
```

本次实测记录：

- `rclone version` 成功
- `rclone listremotes` 成功，返回 `aliyun-a-oss-demo:` 与 `h-objstore:`
- `rclone backend features aliyun-a-oss-demo:` 成功，确认该 remote 支持 `copy`、`copyto`、`ls*`、`size`、`cat`、`check`、`delete*`、`purge`、`link`、`settier` 等能力
- `rclone lsf aliyun-a-oss-demo:pjlab-bjpai-zhuzihou-assets` 可以正常返回这一层的条目，例如 `GRScenes/`、`GRScenes-test1-20260309/`、`GRScenes-v1-1/`、`InternScenes/`、`oss_test.txt`
- 对同一层使用 `lsd`、`lsf --dirs-only`、`size` 的结果曾出现空输出或超时，因此不要把这些命令的失败直接解释成“路径不存在”
- 对 `copyto --dry-run` 与 `sync --dry-run` 的在线 smoke test 也曾在创建远端 backend 后超时，说明 dry-run 仍可能受远端探测延迟影响

## Upload

标准目录上传：

```bash
rclone copy "${LOCAL_ROOT}" "${REMOTE_PATH}" \
  --transfers 16 \
  --checkers 32 \
  --stats 10s \
  --progress
```

单文件上传时使用 `copyto`：

```bash
rclone copyto README.md "${REMOTE}${BUCKET_ROOT}/ops-smoke/README.md" --dry-run
```

说明：

- `copy` 是增量上传，不会删除远端多余文件。
- 首次跑通新路径时，建议先对单文件执行 `copyto --dry-run`，再对全目录执行正式命令。

## Bucket-Root Release Upload Template

适用于“把本地标准包根目录上传为 bucket root 下一个新的发布目录”，而不是上传到 `GRScenes/` 子目录。

```bash
REMOTE='aliyun-a-oss-demo:'
BUCKET_ROOT='pjlab-bjpai-zhuzihou-assets'
LOCAL_ROOT='/abs/path/GRScenes-test0-rebuilt'
RELEASE_NAME='GRScenes-test0-rebuilt-standard-20260314'
REMOTE_PATH="${REMOTE}${BUCKET_ROOT}/${RELEASE_NAME}"

find "${LOCAL_ROOT}" -mindepth 1 -maxdepth 1 -type d -printf '%f/\n' | sort

env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy -u ALL_PROXY -u all_proxy \
  rclone lsf "${REMOTE}${BUCKET_ROOT}"

env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy -u ALL_PROXY -u all_proxy \
  rclone lsf "${REMOTE_PATH}"

env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy -u ALL_PROXY -u all_proxy \
  rclone copy "${LOCAL_ROOT}" "${REMOTE_PATH}" \
  --transfers 16 \
  --checkers 32 \
  --stats 10s \
  --progress

env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy -u ALL_PROXY -u all_proxy \
  rclone lsf "${REMOTE_PATH}"

env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy -u ALL_PROXY -u all_proxy \
  rclone check "${LOCAL_ROOT}" "${REMOTE_PATH}" --one-way
```

使用要点：

- 如果标准包根目录预期是三层顶级目录，上传前先确认本地顶层正好包含 `Material/`、`GRScenes_assets/`、`GRScenes100/`。
- 如果 bucket-root `lsf` 已经出现同名 `RELEASE_NAME/`，不要直接复用该路径；先确认是否已有历史发布。
- 对精确目标前缀执行 `rclone lsf` 时，“空输出且正常退出”可视为目标不存在或为空；“超时或挂起”仍属于不充分证据，需要继续缩小范围或切换到无代理执行。

## Download

将远端数据集拉回本地：

```bash
rclone copy "${REMOTE_PATH}" /abs/path/to/local/GRScenes-test1 \
  --transfers 16 \
  --checkers 32 \
  --stats 10s \
  --progress
```

只读查看单个远端文件时可用：

```bash
rclone cat "${REMOTE}${BUCKET_ROOT}/ops-smoke/README.md" | sed -n '1,20p'
```

## Sync And Check

仅在目标端应严格镜像本地目录时使用 `sync`：

```bash
rclone sync "${LOCAL_ROOT}" "${REMOTE_PATH}" --dry-run --progress
rclone sync "${LOCAL_ROOT}" "${REMOTE_PATH}" \
  --transfers 16 \
  --checkers 32 \
  --stats 10s \
  --progress
```

传输后校验：

```bash
rclone check "${LOCAL_ROOT}" "${REMOTE_PATH}" --one-way
```

## Controlled Cleanup

删除前先列举：

```bash
rclone lsf "${REMOTE}${BUCKET_ROOT}/ops-smoke"
rclone lsjson "${REMOTE}${BUCKET_ROOT}/ops-smoke" --max-depth 2
```

删除单文件：

```bash
rclone deletefile "${REMOTE}${BUCKET_ROOT}/ops-smoke/README.md"
```

删除某个前缀下的文件：

```bash
rclone delete "${REMOTE}${BUCKET_ROOT}/ops-smoke"
```

彻底删除某个前缀及其内容：

```bash
rclone purge "${REMOTE}${BUCKET_ROOT}/ops-smoke"
```

## Troubleshooting

### Root listing 慢或无结果

- 先用 `rclone lsf "${REMOTE}${BUCKET_ROOT}"` 做不带过滤的基础列举，再缩小到更深的 prefix。
- 不要优先依赖 `--dirs-only` 判断“这一层是否有内容”；该过滤在这个后端上可能漏结果。
- 用 `timeout 20s` 限制探测时间，避免长时间挂起终端。
- 仅在定位问题时追加 `-vv`，例如：

```bash
timeout 20s rclone lsf "${REMOTE}${BUCKET_ROOT}" -vv
```

### 命令能识别 remote，但列举不稳定

- 先用 `rclone backend features "${REMOTE}"` 判断 remote 能力是否正常。
- 再用单文件或窄路径的 `copyto --dry-run`、`lsjson --max-depth 1` 做更小范围探测。
- 如果问题持续，优先检查网络路径、endpoint 可达性和 bucket/prefix 权限，而不是直接改传输参数。

### 需要更高确定性

- 上传后立即执行 `rclone check`。
- 对小规模对象可追加 `--checksum`，但不要默认对大目录启用。

## Recorded Release Uploads

### 2026-03-14: `GRScenes-test0-rebuilt-standard-20260314`

- 本地源：`/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt`
- 远端目标：`aliyun-a-oss-demo:pjlab-bjpai-zhuzihou-assets/GRScenes-test0-rebuilt-standard-20260314`
- 放置策略：bucket root 顶层新发布目录，不放到 `GRScenes/` 子目录
- 预检结论：
  - 本地顶层目录确认只有 `Material/`、`GRScenes_assets/`、`GRScenes100/`
  - bucket-root `rclone lsf aliyun-a-oss-demo:pjlab-bjpai-zhuzihou-assets` 未出现目标目录名
  - 对精确目标前缀执行无代理 `rclone lsf`，得到空输出并正常退出，按“目标不存在或为空”处理
- 执行命令：`rclone copy`，参数为 `--transfers 16 --checkers 32 --stats 10s --progress`
- 结果：
  - `rclone copy` 退出码为 `0`
  - 传输总量 `147.374 GiB`
  - 匹配对象数 `193897`
  - 总耗时约 `8m14.6s`
- 验收：
  - 远端顶层列举结果为 `GRScenes100/`、`GRScenes_assets/`、`Material/`
  - `rclone check --one-way` 结果为 `0 differences found`，`193897 matching files`

## Skill

本仓库已提供配套 skill：`skills/oss-rclone-ops/`。当前机器同步安装了一份同名 skill 到本机 Codex skills 目录，供后续直接触发使用。
