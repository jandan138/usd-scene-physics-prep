# DLC Scripts Documentation / DLC 脚本使用文档

本文档介绍如何使用 DLC (Deep Learning Cluster) 脚本在阿里云集群上提交和运行 USD 场景物理预处理任务。

This document explains how to use the DLC scripts to submit and run USD scene physics preprocessing tasks on the Alibaba Cloud cluster.

---

## 1. 概述 / Overview

`usd-scene-physics-prep` 的 DLC 脚本提供了在阿里云 PAI-DLC 集群上提交和运行物理预处理任务的自动化工具链。支持交互物理、导航物理、SimReady 处理、场景拆分、规范化导出等多种任务模式。

The DLC scripts provide an automated toolchain for submitting and running physics preprocessing tasks on the Alibaba Cloud PAI-DLC cluster. They support multiple task modes including interaction physics, navigation physics, SimReady processing, scene splitting, normalized export, and more.

### 调用链 / Call Chain Diagram

```
DSW/本地终端                    阿里云 DLC 集群                   Worker 容器
DSW/Local Terminal             Alibaba Cloud DLC Cluster        Worker Container

+-------------------+         +----------------------+         +-------------------------+
| submit_batch.py   |  dlc    | DLC Scheduler        |  启动   | launch_job.sh           |
| 循环提交每个 chunk | ------> | 调度 & 创建容器       | ------> | 设置环境变量             |
| Loop & submit     |  submit | Schedule & create    |  start  | Setup env vars          |
+-------------------+         +----------------------+         +------------+------------+
                                                                            |
                                                                            v
                                                               +-------------------------+
                                                               | run_task.sh             |
                                                               | 检测模式 & 分发任务      |
                                                               | Detect mode & dispatch  |
                                                               +------------+------------+
                                                                            |
                                                         +------------------+------------------+
                                                         |                                     |
                                                         v
                                              +---------------------+
                                              | isaac_python.sh     |
                                              | 所有任务模式         |
                                              | (interaction, nav,  |
                                              |  simready, custom,  |
                                              |  normalize, clean)  |
                                              +---------------------+
```

---

## 2. 快速开始 / Quick Start

3 步提交你的第一个 DLC 任务 / Submit your first job in 3 steps:

### Step 1: 确认 DLC CLI 可用 / Verify DLC CLI

```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep

# 检查 dlc 二进制是否存在
ls -la ./dlc

# 验证认证 (应返回任务列表)
./dlc get jobs
```

如果 `dlc` 不在项目根目录或认证失败，参见 [常见问题](#9-常见问题--faq--troubleshooting)。

### Step 2: 提交单个任务 / Submit a single job

```bash
# 提交一个 interaction 物理预处理任务
python scripts/dlc/submit_batch.py --total 1 --name interaction_test \
    --command_args "interaction --input /path/to/scene.usd --output /path/to/output"
```

### Step 3: 检查任务状态 / Check job status

```bash
./dlc get job <job_id>
```

---

## 3. 脚本说明 / Script Reference

### 3.1 `submit_batch.py` (批量调度器 / Batch Dispatcher)

**位置 / Location**: `scripts/dlc/submit_batch.py`
**运行环境 / Runs on**: DSW / 本地开发机

循环提交多个分块任务到 DLC 集群。每个分块调用一次 `launch_job.sh`。

Loops through chunks and submits each as a separate DLC job by calling `launch_job.sh`.

**参数 / Arguments**:

| 参数 | 类型 | 必填 | 默认值 | 说明 |
|------|------|------|--------|------|
| `--total` | int | Yes | - | 总分块数 / Total chunk count |
| `--name` | str | No | `physics_prep` | 任务基础名称 / Base task name |
| `--data_sources` | str | No | (launch_job.sh 默认值) | 逗号分隔的数据源 ID / Comma-separated data source IDs |
| `--command_args` | str | No | (batch 模式) | 覆盖 run_task.sh 的运行模式 / Override run_task.sh mode |

**用法示例 / Usage Examples**:

```bash
# 批量提交 10 个 interaction 任务
python scripts/dlc/submit_batch.py --total 10 --name interaction_batch \
    --command_args "interaction --input /path/to/assets"

# 提交单个 simready 任务
python scripts/dlc/submit_batch.py --total 1 --name simready_job \
    --command_args "simready --input-usd /path/to/asset.usd"

# 使用自定义数据源
python scripts/dlc/submit_batch.py --total 5 --name my_task \
    --data_sources "d-xxx,d-yyy,d-zzz"
```

### 3.2 `launch_job.sh` (提交包装器 / Submission Wrapper)

**位置 / Location**: `scripts/dlc/launch_job.sh`
**运行环境 / Runs on**: 由 `submit_batch.py` 自动调用 (通过 `dlc submit`)

构造 DLC 提交命令并调用 `dlc submit pytorchjob`。

Constructs the DLC submission command and calls `dlc submit pytorchjob`.

**参数 / Arguments**:

```bash
bash launch_job.sh <TASK_NAME> <CHUNK_ID> <CHUNK_TOTAL> [DATA_SOURCES] [COMMAND_ARGS]
```

| 位置 | 参数 | 默认值 | 说明 |
|------|------|--------|------|
| $1 | TASK_NAME | (必填) | 任务名称, 用于构造 Job Name |
| $2 | CHUNK_ID | (必填) | 当前分块 ID |
| $3 | CHUNK_TOTAL | (必填) | 总分块数 |
| $4 | DATA_SOURCES | `d-mzps5b7joy2axmqpa8,d-d49o5g0h2818sw8j1g,d-8wz4emfs21s5ajs9oz` | 数据源 ID 列表 |
| $5 | COMMAND_ARGS | `$CHUNK_ID $CHUNK_TOTAL` | 传给 run_task.sh 的参数 |

**容器资源配置 / Container Resources** (硬编码在脚本中):

| 资源 | 值 |
|------|-----|
| GPU | 1 |
| CPU | 16 |
| Memory | 118Gi |
| Shared Memory | 118Gi |
| Priority | 7 |
| Workers | 1 |
| Oversold Type | ForbiddenQuotaOverSold |
| Max Running Time | 0 (无限制 / unlimited) |

### 3.3 `run_task.sh` (任务执行器 / Task Executor)

**位置 / Location**: `scripts/dlc/run_task.sh`
**运行环境 / Runs on**: DLC Worker 容器内部

根据第一个参数分发到不同的处理模式。自动设置环境变量 (`OMNI_KIT_ACCEPT_EULA`, `PYTHONPATH`) 并通过 `isaac_python.sh` 调用 Isaac Sim Python。

Dispatches to different processing modes based on the first argument. Auto-sets environment variables and calls Isaac Sim Python via `isaac_python.sh`.

**用法 / Usage**:

```bash
bash run_task.sh <mode> [args...]
```

完整模式列表见 [任务模式](#5-任务模式--task-modes)。

---

## 4. 配置参考 / Configuration

### 环境变量 / Environment Variables

以下环境变量可在运行 `submit_batch.py` 或 `launch_job.sh` 前设置，用于覆盖默认配置。

These environment variables can be set before running `submit_batch.py` or `launch_job.sh` to override defaults.

| 环境变量 | 默认值 | 说明 |
|----------|--------|------|
| `DLC_WORKSPACE_ID` | `270969` | DLC 工作空间 ID / DLC workspace ID |
| `DLC_RESOURCE_ID` | `quotalplclkpgjgv` | DLC 资源配额 ID / Resource quota ID |
| `DLC_IMAGE` | `dsw-registry-vpc.cn-beijing.cr.aliyuncs.com/pai-training-algorithm/isaac-sim:isaacsim450-vnc-v8` | Docker 镜像地址 / Docker image URI |
| `DLC_CODE_ROOT` | `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep` | 代码在容器内的挂载路径 / Code mount path in container |
| `DLC_BIN` | `$CODE_ROOT/dlc` | DLC CLI 二进制路径 / DLC CLI binary path |
| `ISAAC_SIM_ROOT` | (自动检测) | Isaac Sim 安装目录 / Isaac Sim install path |
| `OMNI_KIT_ACCEPT_EULA` | `YES` (run_task.sh 自动设置) | 接受 Isaac Sim EULA |
| `PYTHONUNBUFFERED` | `1` (run_task.sh 自动设置) | Python 无缓冲输出 / Unbuffered Python output |
| `PYTHONPATH` | `${CODE_ROOT}:...` (run_task.sh 自动设置) | 项目根目录自动添加 / Auto-added by run_task.sh |

### Docker 镜像 / Docker Image

| 项目 | 值 |
|------|-----|
| 镜像 | `dsw-registry-vpc.cn-beijing.cr.aliyuncs.com/pai-training-algorithm/isaac-sim:isaacsim450-vnc-v8` |
| Isaac Sim 版本 | 4.5.0 (源码安装版 / Source install) |
| Isaac Sim 路径 | `/isaac-sim/` (容器内) |
| Python 运行方式 | `isaac_python.sh` -> `/isaac-sim/python.sh` |

**重要**: 本项目使用 Isaac Sim **源码安装版** (不同于 render-usd 使用的 conda 环境)。运行 Isaac Sim 相关任务时通过 `isaac_python.sh` 包装器调用，而非直接使用 `python`。

**Important**: This project uses the Isaac Sim **source install** (unlike render-usd which uses a conda environment). Isaac Sim tasks are invoked via the `isaac_python.sh` wrapper, not `python` directly.

### 数据源 / Data Sources

默认数据源 ID (在 `launch_job.sh` 中配置):

```
d-mzps5b7joy2axmqpa8,d-d49o5g0h2818sw8j1g,d-8wz4emfs21s5ajs9oz
```

如需使用不同的数据源，可通过以下方式覆盖:

```bash
# 方式 1: submit_batch.py 参数
python scripts/dlc/submit_batch.py --total 5 --name my_task --data_sources "d-aaa,d-bbb"

# 方式 2: launch_job.sh 第 4 个参数
bash scripts/dlc/launch_job.sh my_task 0 1 "d-aaa,d-bbb"
```

**注意**: 数据源 ID 不可重复, 否则 DLC 会报错 "different datasource can't have the same mount path"。

**Note**: Data source IDs must be unique, otherwise DLC returns error "different datasource can't have the same mount path".

---

## 5. 任务模式 / Task Modes

`run_task.sh` 支持以下运行模式。通过第一个参数指定模式。

`run_task.sh` supports the following modes, selected by the first argument.

### 5.1 `interaction` - 交互物理预处理

为交互场景设置物理属性 (碰撞体、刚体等)。

```bash
# 本地运行
bash scripts/dlc/run_task.sh interaction --input /path/to/scene.usd --output /path/to/output

# DLC 提交
python scripts/dlc/submit_batch.py --total 1 --name interaction_prep \
    --command_args "interaction --input /path/to/scene.usd --output /path/to/output"
```

**底层调用 / Underlying call**: `isaac_python.sh set_physics/preprocess_for_interaction.py [args...]`

### 5.2 `navigation` - 导航物理预处理

为导航场景设置物理属性。

```bash
# 本地运行
bash scripts/dlc/run_task.sh navigation --input /path/to/scene.usd --output /path/to/output

# DLC 提交
python scripts/dlc/submit_batch.py --total 1 --name navigation_prep \
    --command_args "navigation --input /path/to/scene.usd --output /path/to/output"
```

**底层调用 / Underlying call**: `isaac_python.sh set_physics/preprocess_for_navigation.py [args...]`

### 5.3 `simready` - SimReady 一键处理

通过 SimReady CLI 执行一键式物理属性设置。

```bash
# 本地运行
bash scripts/dlc/run_task.sh simready --input-usd /path/to/asset.usd

# DLC 提交
python scripts/dlc/submit_batch.py --total 1 --name simready_job \
    --command_args "simready --input-usd /path/to/asset.usd"
```

**底层调用 / Underlying call**: `isaac_python.sh -m set_physics.simready [args...]`

### 5.4 `prep_root_scene` - Root 结构场景预处理

处理外部 `/root` 结构的场景 (如 SimBench、GRScene)。

```bash
# 本地运行
bash scripts/dlc/run_task.sh prep_root_scene --input /path/to/scene --output /path/to/output

# DLC 提交
python scripts/dlc/submit_batch.py --total 1 --name root_scene_prep \
    --command_args "prep_root_scene --input /path/to/scene --output /path/to/output"
```

**底层调用 / Underlying call**: `isaac_python.sh scripts/prep_interaction_root_scene.py [args...]`

### 5.5 `normalize` - 规范化导出

通过 `specs_normalizer` 执行规范化导出。**需要 Isaac Sim Python**（specs_normalizer 依赖 pxr）/ **Requires Isaac Sim Python** (specs_normalizer depends on pxr)

```bash
# 本地运行
bash scripts/dlc/run_task.sh normalize [args...]

# DLC 提交
python scripts/dlc/submit_batch.py --total 1 --name normalize_job \
    --command_args "normalize [args...]"
```

**底层调用 / Underlying call**: `isaac_python.sh -m specs_normalizer [args...]`

### 5.6 `clean` - 场景拆分 / 数据清洗

执行场景拆分和数据清洗。**需要 Isaac Sim Python**（clean_data.py 依赖 pxr）/ **Requires Isaac Sim Python** (clean_data.py depends on pxr)

```bash
# 本地运行
bash scripts/dlc/run_task.sh clean [args...]

# DLC 提交
python scripts/dlc/submit_batch.py --total 1 --name clean_job \
    --command_args "clean [args...]"
```

**底层调用 / Underlying call**: `isaac_python.sh clean_data.py [args...]`

### 5.7 `custom` - 通用自定义模式

直接传递任意脚本和参数给 Isaac Sim Python。适用于实验性脚本或一次性任务。

```bash
# 本地运行
bash scripts/dlc/run_task.sh custom my_script.py --arg1 value1

# DLC 提交
python scripts/dlc/submit_batch.py --total 1 --name custom_job \
    --command_args "custom my_script.py --arg1 value1"
```

**底层调用 / Underlying call**: `isaac_python.sh <script.py> [args...]`

### 5.8 batch (默认模式) - 批量处理

当第一个参数不匹配任何命名模式时进入 batch 模式（默认 else 分支）。需要额外参数指定具体任务。

Enters batch mode when the first argument doesn't match any named mode (default else branch). Requires extra arguments.

```bash
# 本地运行
bash scripts/dlc/run_task.sh 0 10 my_script.py --some-arg

# DLC 提交 (默认模式, 无需 --command_args)
# 注意: batch 模式目前需要额外参数, 否则会报错
python scripts/dlc/submit_batch.py --total 10 --name batch_job \
    --command_args "0 10 my_script.py --some-arg"
```

**底层调用 / Underlying call**: `isaac_python.sh [extra_args...]`

**注意**: Batch 模式目前为保留模式, 需要额外参数定义具体行为。不传额外参数会报错退出。

---

## 6. 环境依赖 / Dependencies

### 容器内 / Inside Container

| 依赖项 | 路径 / 来源 | 必须 | 说明 |
|--------|-------------|------|------|
| Isaac Sim 4.5.0 | `/isaac-sim/` (Docker 镜像自带) | Yes (Isaac Sim 模式) | 源码安装版, 包含 `python.sh` |
| `isaac_python.sh` | `$CODE_ROOT/scripts/isaac_python.sh` | Yes (Isaac Sim 模式) | 自动定位 Isaac Sim 并设置环境 |
| DLC CLI 二进制 | `$CODE_ROOT/dlc` | Yes (提交任务时) | 用于 `dlc submit` 命令 |
| Python 3 | 系统 Python | No (normalize/clean 不够用) | `normalize` 和 `clean` 均需要 Isaac Sim Python (依赖 pxr) |
| CPFS 挂载 | `/cpfs/` | Yes | 代码和数据的共享存储 |

### 提交环境 / Submission Environment (DSW/Local)

| 依赖项 | 说明 |
|--------|------|
| `dlc` CLI | 已安装且认证完成 |
| Python 3.10+ | 运行 `submit_batch.py` |
| CPFS 访问 | 代码目录可访问 |

### 与 render-usd 的关键差异 / Key Differences from render-usd

| | render-usd | usd-scene-physics-prep |
|--|-----------|------------------------|
| Python 运行方式 | Conda 环境 (`miniconda/`) | `isaac_python.sh` -> `/isaac-sim/python.sh` |
| Isaac Sim 版本 | 4.1 (conda) | 4.5.0 (源码安装) |
| Docker 镜像 | `isaacsim41-cuda118` | `isaacsim450-vnc-v8` |
| 非 Isaac 模式 | 无 | 无（所有模式均需要 Isaac Sim Python） |

---

## 7. DSW 配置 / DSW Setup

如果在 DSW 上提交任务时遇到 "command not found" 或认证错误, 参考以下步骤。

### 7.1 检查 `dlc` CLI

```bash
# 检查 dlc 是否在 PATH 或项目根目录
which dlc || ls -la /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/dlc
```

如果项目根目录有 `dlc` 二进制但不在 PATH, `launch_job.sh` 会自动使用 `$CODE_ROOT/dlc`。

### 7.2 配置认证

```bash
# 首次使用需要配置 AccessKey
./dlc config

# 需要输入:
# - AccessKey ID
# - AccessKey Secret
# - Endpoint: dlc.cn-beijing.aliyuncs.com
# - Region: cn-beijing
```

### 7.3 验证

```bash
./dlc get jobs
# 如果返回任务列表, 说明配置成功
```

---

## 8. 本地测试 / Local Testing

在提交到 DLC 之前, 可以在本地运行脚本来验证逻辑。

You can run scripts locally to verify logic before submitting to DLC.

```bash
# 确保 Isaac Sim 可用 (设置 ISAAC_SIM_ROOT 或使用容器)
export ISAAC_SIM_ROOT="/path/to/isaac-sim"

# 测试 interaction 模式
bash scripts/dlc/run_task.sh interaction --input /path/to/scene.usd --output ./test_output

# 测试 normalize 模式
bash scripts/dlc/run_task.sh normalize --some-arg

# 测试 custom 模式
bash scripts/dlc/run_task.sh custom my_script.py --arg value
```

---

## 9. 常见问题 / FAQ / Troubleshooting

### Q1: `isaac_python.sh not found`

**错误信息**: `ERROR: isaac_python.sh not found at /cpfs/.../scripts/isaac_python.sh`

**原因**: `DLC_CODE_ROOT` 设置不正确, 或代码未正确挂载到容器。

**解决**:
```bash
# 检查代码路径
ls -la /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/isaac_python.sh

# 如果路径不同, 设置环境变量
export DLC_CODE_ROOT="/correct/path/to/usd-scene-physics-prep"
```

### Q2: `Could not locate Isaac Sim installation`

**错误信息**: `ERROR: Could not locate Isaac Sim installation (python.sh not found).`

**原因**: `isaac_python.sh` 无法在默认路径找到 Isaac Sim。

**解决**:
```bash
# 设置 Isaac Sim 安装路径
export ISAAC_SIM_ROOT="/isaac-sim"  # 容器内通常是这个路径

# 验证
ls -la $ISAAC_SIM_ROOT/python.sh
```

`isaac_python.sh` 按以下顺序搜索 Isaac Sim:
1. `$ISAAC_SIM_ROOT` 环境变量
2. `/isaac-sim/` (Docker 容器默认路径)
3. `~/.local/share/ov/pkg/isaac_sim-*` (本地安装)
4. `/opt/nvidia/isaac-sim`, `/opt/NVIDIA/isaac-sim`, `/opt/omniverse/isaac-sim`

### Q3: `dlc` 命令找不到

**错误信息**: `dlc: command not found`

**原因**: `dlc` 二进制不在 PATH 中。

**解决**:
```bash
# launch_job.sh 默认使用 $CODE_ROOT/dlc
# 确保二进制存在
ls -la /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/dlc

# 或手动设置路径
export DLC_BIN="/path/to/dlc"
```

### Q4: 数据源 ID 重复导致提交失败

**错误信息**: HTTP 400 `different datasource can't have the same mount path`

**原因**: `--data_sources` 参数中有重复的 ID。

**解决**: 检查并去重数据源 ID 列表。每个 ID 只能出现一次。

### Q5: Conda 相关警告

**说明**: 本项目**不使用 Conda 环境**, 与 `render-usd` 不同。如果看到 conda 相关的警告或错误, 通常可以忽略。所有任务模式（包括 `normalize` 和 `clean`）均通过 `isaac_python.sh` 调用，因为它们依赖 pxr。

### Q6: Batch 模式报错 "requires additional arguments"

**错误信息**: `ERROR: Batch mode requires additional arguments to define the task.`

**原因**: Batch 模式 (默认模式) 需要额外参数来定义具体任务。

**解决**: 使用 `--command_args` 传入具体的任务参数, 或使用命名模式 (interaction, navigation, etc.)。

```bash
# 正确: 使用命名模式
python scripts/dlc/submit_batch.py --total 1 --name job \
    --command_args "interaction --input /path/to/scene.usd"

# 正确: batch 模式 + 额外参数
python scripts/dlc/submit_batch.py --total 10 --name job \
    --command_args "0 10 my_script.py --arg value"
```

### Q7: 任务提交后如何查看日志?

```bash
# 查看任务状态
./dlc get job <job_id>

# 查看任务列表
./dlc get jobs
```

---

## 10. 完整用法示例 / Complete Examples

### 示例 1: 批量处理交互物理

```bash
# 提交 10 个分块任务
python scripts/dlc/submit_batch.py --total 10 --name interaction_grscenes \
    --command_args "interaction --input /cpfs/shared/simulation/assets/scenes"
```

### 示例 2: 单个 SimReady 处理

```bash
python scripts/dlc/submit_batch.py --total 1 --name simready_single \
    --command_args "simready --input-usd /cpfs/shared/simulation/assets/object.usd"
```

### 示例 3: 使用自定义数据源和脚本

```bash
python scripts/dlc/submit_batch.py --total 5 --name custom_task \
    --data_sources "d-aaa,d-bbb" \
    --command_args "custom scripts/my_custom_script.py --flag"
```

### 示例 4: 直接调用 launch_job.sh (调试用)

```bash
bash scripts/dlc/launch_job.sh debug_task 0 1 "" "interaction --input /path/to/scene.usd"
```

### 示例 5: 本地调试 (不提交到 DLC)

```bash
# 直接运行 run_task.sh
export ISAAC_SIM_ROOT="/isaac-sim"
bash scripts/dlc/run_task.sh simready --input-usd /path/to/asset.usd
```
