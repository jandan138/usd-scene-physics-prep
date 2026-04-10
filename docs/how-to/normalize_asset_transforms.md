---
title: Asset Transform Normalization (Recenter + Y-up to Z-up)
code_reference: scripts/normalize_asset_transforms.py
created_at: 2026-03-04
updated_at: 2026-03-05
maintainer: Project Team
status: Active
---

# 使用指南：Asset Transform Normalization

> 最后更新：2026-03-04
>
> 相关代码：
> - ../../scripts/normalize_asset_transforms.py
>
> 总入口：../index.md

## 索引
- [功能概述](#功能概述)
- [数学原理](#数学原理)
- [运行环境](#运行环境)
- [CLI 参数说明](#cli-参数说明)
- [快速开始](#快速开始)
- [DLC 集群提交](#dlc-集群提交)
- [验证工作流](#验证工作流)
- [输出结构](#输出结构)
- [常见问题](#常见问题)

## 功能概述

`scripts/normalize_asset_transforms.py` 将 GRScenes 资产统一为 Z-up、原点居中的标准形态，同时自动补偿场景布局中的引用变换，确保归一化前后场景中物体的世界空间位置不变。

脚本分两阶段执行：

**Phase 1 — 归一化资产** (`normalize_asset`):
1. 将 `/Root/Instance` 下所有内部变换（scale/orient/translate）烘焙进 mesh 顶点
2. 应用 Y-up → Z-up 旋转 (`R_y2z = Rot_X(+90°)`)
3. 以包围盒中心点为基准将几何体重新居中到原点
4. 将变换后的顶点写回 mesh，`/Root/Instance` 仅保留原始 scale
5. 设置 stage `upAxis = Z`

**Phase 2 — 补偿场景布局** (`compensate_scene`):
1. 遍历场景 layout USD 中所有引用了已归一化资产的 prim
2. 调整每个 prim 的 `xformOp:transform`，使物体在世界空间中保持原始位置
3. 保存修改后的 layout USD

## 数学原理

### Y-up → Z-up 旋转

```
R_y2z = Rot_X(+90°)
作用: (x, y, z) → (x, -z, y)
```

对应矩阵（4x4，row-vector convention）：

```
     ┌ 1   0   0  0 ┐
     │ 0   0   1  0 │
     │ 0  -1   0  0 │
     └ 0   0   0  1 ┘
```

其逆矩阵 `R_y2z_inv = Rot_X(-90°)`。

### 资产归一化

对于每个 mesh 顶点 `p`：

```
p_root    = M_internal.Transform(p)       # 烘焙内部变换，映射到 /Root 空间
p_rotated = R_y2z.Transform(p_root)       # Y-up → Z-up
p_centered = p_rotated - center           # 减去包围盒中心
p_local   = p_centered / original_scale   # 除以 Instance 原始 scale
```

其中 `center = (bbox_min + bbox_max) / 2`，取所有 rotated 顶点的包围盒中点。

### 场景补偿公式

```
M_scene_new = T(center) * R_y2z_inv * M_scene_old
```

推导（row-vector convention, `q = p * M`）：
- 归一化前：`q = p * M_internal * M_scene_old`
- 归一化后资产内部变换变为：`p' = p * M_internal * R_y2z * T(-center)`
- 要求 `p' * M_scene_new = p * M_internal * M_scene_old`
- 解出：`M_scene_new = T(center) * R_y2z_inv * M_scene_old`

## 运行环境

- **Python 3.8+**
- **依赖**：`usd-core`（pxr 模块）
- **不需要** Isaac Sim — 纯 pxr 操作，可在标准 Python 环境中运行

```bash
# 确认 pxr 可用
python3 -c "from pxr import Usd, UsdGeom, Gf; print('OK')"
```

## CLI 参数说明

```
python3 scripts/normalize_asset_transforms.py [OPTIONS]
```

| 参数 | 必选 | 说明 |
|------|------|------|
| `--assets-root` | 是 | 资产根目录（如 `GRScenes-test1/GRScenes_assets`） |
| `--scenes-root` | 是 | 场景根目录（如 `GRScenes-test1/GRScenes100`） |
| `--output-root` | 是 | 输出根目录（镜像输入结构） |
| `--category` | 否 | 仅处理指定分类（如 `plate`）；不指定则处理全部分类 |
| `--dry-run` | 否 | 模拟运行：计算 center 但不写入任何文件 |
| `--symlink-copy` | 否 | 对非 USD 文件（PNG、GLB、JSON）使用符号链接替代复制，大幅减少 I/O |
| `--report-dir` | 否 | JSON 报告输出目录；不指定则写入 `output-root/normalize_report.json` |

## 快速开始

### 1. Dry-run 试运行（推荐先执行）

```bash
python3 scripts/normalize_asset_transforms.py \
  --assets-root GRScenes-test1/GRScenes_assets \
  --scenes-root GRScenes-test1/GRScenes100 \
  --output-root /tmp/test-normalize \
  --category plate --dry-run \
  --report-dir check_reports/normalize
```

检查输出报告中的 error 数是否为 0：

```bash
python3 -c "
import json
r = json.load(open('check_reports/normalize/normalize_report.json'))
print(f\"Assets: {r['meta']['assets_processed']}/{r['meta']['assets_total']}\")
print(f\"Errors: {r['meta']['errors_count']}\")
"
```

### 2. 单分类真实运行（plate 示例）

```bash
python3 scripts/normalize_asset_transforms.py \
  --assets-root GRScenes-test1/GRScenes_assets \
  --scenes-root GRScenes-test1/GRScenes100 \
  --output-root GRScenes-test1-normalized \
  --category plate --symlink-copy \
  --report-dir check_reports/normalize
```

**实测结果**（plate 分类，426 个资产）：

| 指标 | 值 |
|------|-----|
| 资产归一化 | 426/426，0 errors |
| 场景补偿 | 99 layouts，701 prims compensated |
| Phase 1 耗时 | 79.48s |
| Phase 2 耗时 | 17.63s |
| 总耗时 | ~97s |

> **注意**：单分类运行时，stderr 会出现 "Could not open asset" 警告，这是因为场景布局引用了其他未处理分类的资产，属于正常现象。

### 3. 全量运行（所有分类）

```bash
python3 scripts/normalize_asset_transforms.py \
  --assets-root GRScenes-test1/GRScenes_assets \
  --scenes-root GRScenes-test1/GRScenes100 \
  --output-root GRScenes-test1-normalized \
  --symlink-copy \
  --report-dir check_reports/normalize
```

**注意**：
- 输出目录请使用持久存储路径（**不要用 `/tmp`**，重启会丢失）
- 全量数据（~52,907 资产）建议使用 `--symlink-copy` 减少磁盘 I/O
- 脚本具有幂等性：如果目标文件已存在会跳过复制

## DLC 集群提交

对于大规模全量处理，可通过 DLC（PAI-DLC）集群提交。`scripts/dlc/run_task.sh` 提供了 `normalize_assets` 模式（见 `run_task.sh:94-99`），通过 Isaac Sim Python 运行归一化脚本，接受所有 CLI 参数。

> 相关代码：
> - `scripts/dlc/run_task.sh:94-99` — `normalize_assets` 模式分支
> - `CLAUDE.md:107` — DLC 模式列表

### 用法

```bash
# 在 DLC worker 中执行
bash run_task.sh normalize_assets \
  --assets-root /cpfs/shared/data/GRScenes_assets \
  --scenes-root /cpfs/shared/data/GRScenes100 \
  --output-root /cpfs/shared/output/GRScenes-normalized \
  --symlink-copy \
  --report-dir /cpfs/shared/output/reports
```

该模式内部调用：

```bash
./scripts/isaac_python.sh scripts/normalize_asset_transforms.py [args...]
```

所有 CLI 参数（`--assets-root`、`--scenes-root`、`--output-root`、`--category`、`--dry-run`、`--symlink-copy`、`--report-dir`）均透传至归一化脚本。

详细 DLC 提交流程（`submit_batch.py`、`launch_job.sh` 等）参见 `docs/operations/dlc/README.md`。

## 验证工作流

归一化完成后，按以下步骤验证结果。

> 相关代码：
> - `scripts/verify_normalized_assets.py` — 自动化验证脚本

### 1. 检查归一化报告

```bash
# 确认 0 errors
python3 -c "
import json
r = json.load(open('check_reports/normalize/normalize_report.json'))
print(f\"Assets: {r['meta']['assets_processed']}/{r['meta']['assets_total']}\")
print(f\"Errors: {r['meta']['errors_count']}\")
if r['errors']:
    for e in r['errors'][:5]:
        print(f\"  - {e}\")
"
```

### 2. 运行自动化验证脚本

`scripts/verify_normalized_assets.py` 自动检查所有归一化不变量：

**资产检查项**：
- `upAxis == Z`
- `/Root/Instance` prim 存在且有效
- `/Root/Instance` 变换为纯 scale 矩阵（无旋转/平移分量）
- 所有 mesh 的包围盒中心在 `/Root` 空间中近似 `(0, 0, 0)`（可配置容差）

**场景检查项**（可选）：
- 每个带 authored references 的 prim 拥有有效的 `xformOp:transform`（Matrix4d）
- `xformOpOrder` 包含 `xformOp:transform`

```bash
# 仅验证资产（单分类）
python3 scripts/verify_normalized_assets.py \
  --assets-root GRScenes-test1-normalized/GRScenes_assets \
  --category plate \
  --tolerance 0.01 \
  --report-dir check_reports/normalize

# 验证资产 + 场景布局
python3 scripts/verify_normalized_assets.py \
  --assets-root GRScenes-test1-normalized/GRScenes_assets \
  --scenes-root GRScenes-test1-normalized/GRScenes100 \
  --tolerance 0.01 \
  --report-dir check_reports/normalize
```

| 参数 | 必选 | 说明 |
|------|------|------|
| `--assets-root` | 是 | 归一化后的资产根目录 |
| `--scenes-root` | 否 | 归一化后的场景根目录（不指定则跳过场景验证） |
| `--category` | 否 | 仅验证指定分类 |
| `--tolerance` | 否 | 包围盒中心容差，默认 `0.01` |
| `--report-dir` | 否 | JSON 报告输出目录 |

**输出**：`verification_report.json`，包含每个资产的 pass/fail 详情和汇总。退出码 0 表示全部通过，1 表示存在失败项。

**依赖**：仅需 `pxr`（Usd, UsdGeom, Gf），不需要 Isaac Sim。

**实测结果**（plate 分类验证）：

| 指标 | 值 |
|------|-----|
| 资产通过 | 426/426 |
| 场景布局通过 | 99/99 |
| 总检查项 | 525 |
| 失败数 | 0 |
| 容差 | 0.01 |

所有不变量确认通过：`upAxis=Z`、`instance_scale_only=true`、bbox center 在原点附近。报告：`check_reports/normalize/verification_report.json`。

### 3. 手动抽样检查（可选）

```python
from pxr import Usd, UsdGeom, Gf

stage = Usd.Stage.Open("GRScenes-test1-normalized/GRScenes_assets/plate/<uid>/usd/<uid>.usd")

# 检查 upAxis
print("upAxis:", UsdGeom.GetStageUpAxis(stage))  # 应为 "Z"

# 检查 Instance 变换
inst = stage.GetPrimAtPath("/Root/Instance")
xf = UsdGeom.Xformable(inst)
m = xf.GetLocalTransformation(Usd.TimeCode.Default())
print("Instance transform:", m)  # 应为纯 scale 矩阵
```

### 4. 确认符号链接有效

```bash
# 检查 symlink 是否指向正确的源文件
ls -la GRScenes-test1-normalized/GRScenes_assets/plate/<uid>/
# annotation.json, *.png 等应为 symlink → 源目录
```

## 输出结构

```
<output-root>/
  GRScenes_assets/
    <category>/
      <uid>/
        annotation.json      → (symlink to source)
        *.png                 → (symlink to source)
        glb/                  → (symlink to source)
        usd/
          <uid>.usd           # 归一化后的真实文件
          textures            → (symlink to source)
  GRScenes100/               # 或 scenes-root 的 basename
    <scene_type>/
      <scene_id>/
        layout.usd            # 补偿后的场景布局
  normalize_report.json       # 处理报告（若未指定 --report-dir）
```

## 常见问题

### Q: 为什么需要 `--symlink-copy`？
全量数据集包含 ~52,907 个资产，每个资产含 annotation.json、4 张 PNG、glb/ 目录等。直接复制会产生巨大 I/O 负载，可能导致机器崩溃。使用 `--symlink-copy` 仅对 USD 文件进行真实写入，其余文件通过符号链接指向源目录。

### Q: 有些资产的 center 偏移很大（如 1192.75），正常吗？
正常。这说明原始几何体在 Y-up 空间中远离原点。归一化后几何体将居中到原点，场景补偿公式会确保最终位置不变。

### Q: 可以只跑 Phase 1 不跑 Phase 2 吗？
当前脚本总是两阶段一起执行。如果只需要归一化资产，可以指定一个空的 `--scenes-root`（不含 layout.usd 的目录），Phase 2 会自动跳过。

### Q: 脚本是否需要 Isaac Sim？
不需要。脚本仅依赖 `usd-core`（pxr 模块），可在标准 Python 环境中运行。

---

## 全量验证结果 (2026-03-04)

GRScenes 数据集全量归一化已于 2026-03-04 通过 DLC 完成。

### 执行摘要

| 指标 | 数值 |
|------|------|
| **DLC Job ID** | `dlc16ykhe1dcit8s` |
| **处理资产数** | 52,904 / 52,907 (99.994%) |
| **场景数** | 99 个 layout.usd |
| **场景 prim 补偿** | 101,919 个 |
| **Phase 1 耗时** | 30,828 秒 (~8.6 小时) |
| **Phase 2 耗时** | 34.4 秒 |
| **总耗时** | ~8 小时 35 分钟 |

### 数据错误说明

3 个资产因原始数据问题未能归一化（非脚本错误）：

| 资产路径 | 错误原因 |
|----------|----------|
| `cabinet/b98d6ccbeb75dfdeb60e27649a5b055a` | 原始 USD 无 `/Root/Instance` 下 mesh |
| `other/d41d8cd98f00b204e9800998ecf8427e` | 原始 USD 无 `/Root/Instance` 下 mesh |
| `person/351316cbb083f9f4df0cccd60cbfa848` | 原始 USD 无 `/Root/Instance` 下 mesh |

这些资产在归一化输出目录中存在，但 `usd/` 子目录为空（仅包含 textures 符号链接）。

### 输出位置

- **归一化资产**: `GRScenes-test1-normalized/GRScenes_assets/`
- **补偿场景**: `GRScenes-test1-normalized/GRScenes100/`
- **处理报告**: `check_reports/normalize/normalize_report.json`
- **验证报告**: `check_reports/normalize/verification_report.json` (验证完成后生成)

### 后续验证

如需验证归一化结果：

```bash
python3 scripts/verify_normalized_assets.py \
  --assets-root GRScenes-test1-normalized/GRScenes_assets \
  --scenes-root GRScenes-test1-normalized/GRScenes100 \
  --report-dir check_reports/normalize
```

验证脚本将检查：
- 每个资产的 `upAxis = Z`
- 每个资产的 `/Root/Instance` 为纯 scale 变换
- 每个资产的包围盒中心接近原点 (tolerance 0.01)
- 每个场景的引用 prim 具有有效的 `xformOp:transform`
