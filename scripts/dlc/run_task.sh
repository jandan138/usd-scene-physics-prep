#!/bin/bash
set -euo pipefail
# 任务执行脚本 (run_task.sh)
# 该脚本在 DLC 容器内部运行，负责设置环境并执行物理预处理命令
# This script runs inside the DLC container, sets up the environment
# and dispatches physics preprocessing commands.

# 定义代码根目录 (Define code root directory)
# 默认为 /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
# 该路径必须与 DLC 挂载路径或本地测试路径一致
CODE_ROOT=${DLC_CODE_ROOT:-"/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep"}

# ============================================================
# 设置环境 (Setup environment)
# ============================================================

# Isaac Sim 源码安装版 - 使用 isaac_python.sh 包装器 (NOT conda)
# Isaac Sim source install - use isaac_python.sh wrapper (NOT conda)
ISAAC_PYTHON="$CODE_ROOT/scripts/isaac_python.sh"

# 检查 Isaac Sim 是否可用 (Verify Isaac Sim is available)
if [ ! -f "$ISAAC_PYTHON" ]; then
    echo "ERROR: isaac_python.sh not found at $ISAAC_PYTHON"
    exit 1
fi

# 接受 Isaac Sim EULA 协议 (Accept Isaac Sim EULA)
export OMNI_KIT_ACCEPT_EULA=YES
# 设置 Python 输出无缓冲，确保日志实时打印 (Unbuffered output for real-time logs)
export PYTHONUNBUFFERED=1
# 将项目根目录添加到 PYTHONPATH (Add project root to PYTHONPATH)
export PYTHONPATH="${CODE_ROOT}:${PYTHONPATH:-}"
# Ensure all relative paths in scripts resolve from the project root
cd "$CODE_ROOT"

ensure_isaac_module() {
    local module_name="$1"
    local rc=0
    "$ISAAC_PYTHON" - <<PY || rc=$?
import importlib.util
import json
import site
import sys

module_name = ${module_name@Q}
spec = importlib.util.find_spec(module_name)
payload = {
    "module": module_name,
    "found": bool(spec),
    "executable": sys.executable,
    "prefix": sys.prefix,
    "sitepackages": site.getsitepackages(),
    "path_head": sys.path[:10],
}
if spec is not None:
    payload["origin"] = spec.origin
    print("ISAAC_MODULE_OK " + json.dumps(payload, ensure_ascii=False))
else:
    print("ISAAC_MODULE_MISSING " + json.dumps(payload, ensure_ascii=False))
    raise SystemExit(1)
PY
    return $rc
}

# Early dependency preflight: bbox-gated cert jobs depend on ijson.
# Make the failure surface explicit here so DLC logs are actionable.
ensure_isaac_module "ijson"

# ============================================================
# 检查运行模式 (Check run mode)
# $1 是第一个参数，决定运行哪种处理模式
# $1 is the first argument, determines which processing mode to run
# ============================================================

if [ $# -eq 0 ]; then
    echo "Usage: run_task.sh <mode> [args...]"
    echo ""
    echo "Modes:"
    echo "  interaction     Physics preprocessing for interaction"
    echo "  navigation      Physics preprocessing for navigation"
    echo "  simready        One-shot SimReady CLI (--input-usd <path>)"
    echo "  prep_root_scene External /root-structured scene prep"
    echo "  normalize       Normalized export via specs_normalizer"
    echo "  normalize_assets  Asset transform normalization (recenter + Y-up→Z-up)"
    echo "  clean           Scene splitting / data cleaning"
    echo "  dedup           Asset dedup report by category chunk"
    echo "  custom          Pass any command to Isaac Sim Python"
    echo "  <chunk_id> <chunk_total> [script.py ...]  Batch mode"
    exit 0
fi

if [ "$1" == "interaction" ]; then
    # 交互物理预处理模式 (Physics preprocessing for interaction)
    # 用法: bash run_task.sh interaction [args...]
    shift
    echo "Running Interaction Physics Preprocessing..."
    "$ISAAC_PYTHON" set_physics/preprocess_for_interaction.py "$@"

elif [ "$1" == "navigation" ]; then
    # 导航物理预处理模式 (Physics preprocessing for navigation)
    # 用法: bash run_task.sh navigation [args...]
    shift
    echo "Running Navigation Physics Preprocessing..."
    "$ISAAC_PYTHON" set_physics/preprocess_for_navigation.py "$@"

elif [ "$1" == "simready" ]; then
    # SimReady 一键处理模式 (One-shot SimReady CLI)
    # 用法: bash run_task.sh simready --input-usd <path> [args...]
    shift
    echo "Running SimReady CLI..."
    "$ISAAC_PYTHON" -m set_physics.simready "$@"

elif [ "$1" == "prep_root_scene" ]; then
    # 外部 root 结构场景预处理 (External /root-structured scene prep, SimBench/GRScene)
    # 用法: bash run_task.sh prep_root_scene --input <path> --output <path> [args...]
    shift
    echo "Running Root Scene Preparation..."
    "$ISAAC_PYTHON" scripts/prep_interaction_root_scene.py "$@"

elif [ "$1" == "normalize" ]; then
    # 规范化导出模式 (Normalized export via specs_normalizer)
    # 注意: 需要 Isaac Sim Python (pxr 依赖) / Requires Isaac Sim Python (pxr dependency)
    # 用法: bash run_task.sh normalize [args...]
    shift
    echo "Running Normalize (specs_normalizer)..."
    "$ISAAC_PYTHON" -m specs_normalizer "$@"

elif [ "$1" == "normalize_assets" ]; then
    # Asset transform normalization (recenter + Y-up→Z-up)
    # 用法: bash run_task.sh normalize_assets [args...]
    shift
    echo "Running Asset Transform Normalization..."
    "$ISAAC_PYTHON" "$CODE_ROOT/scripts/normalize_asset_transforms.py" "$@"

elif [ "$1" == "clean" ]; then
    # 场景拆分模式 (Scene splitting / data cleaning)
    # 注意: 需要 Isaac Sim Python (pxr 依赖) / Requires Isaac Sim Python (pxr dependency)
    # 用法: bash run_task.sh clean [args...]
    shift
    echo "Running Clean (scene splitting)..."
    "$ISAAC_PYTHON" "$CODE_ROOT/clean_data.py" "$@"

elif [ "$1" == "dedup" ]; then
    # 去重扫描模式 (Asset dedup report by category chunk)
    # 用法: bash run_task.sh dedup --chunk-id <id> --chunk-total <total> [args...]
    shift
    echo "Running Dedup Report (by category chunk)..."
    "$ISAAC_PYTHON" "$CODE_ROOT/scripts/dlc/dedup_by_category.py" "$@"

elif [ "$1" == "custom" ]; then
    # 通用模式 - 直接传递命令给 Isaac Sim Python (Generic mode - pass any command)
    # 用法: bash run_task.sh custom <script.py> [args...]
    shift
    echo "Running Custom Command: $*"
    "$ISAAC_PYTHON" "$@"

else
    # 批量模式 (Batch mode) - DLC 默认模式
    # 用法: bash run_task.sh <chunk_id> <chunk_total> [args...]
    # 预留给后续批量处理任务 (Reserved for future batch processing tasks)
    if [ $# -lt 2 ]; then
        echo "ERROR: Batch mode requires at least 2 arguments: <chunk_id> <chunk_total>"
        echo "Usage: run_task.sh <chunk_id> <chunk_total> <script.py> [args...]"
        exit 1
    fi
    CHUNK_ID=$1
    CHUNK_TOTAL=$2
    shift 2

    echo "Running Batch Task: Chunk $CHUNK_ID / $CHUNK_TOTAL"
    echo "NOTE: Batch mode is reserved for future use. Pass additional args to define behavior."

    # 如果有额外参数，通过 Isaac Sim Python 执行 (If extra args, run via Isaac Sim Python)
    if [ $# -gt 0 ]; then
        "$ISAAC_PYTHON" "$@"
    else
        echo "ERROR: Batch mode requires additional arguments to define the task."
        echo "Usage: bash run_task.sh <chunk_id> <chunk_total> <script.py> [args...]"
        exit 1
    fi
fi
