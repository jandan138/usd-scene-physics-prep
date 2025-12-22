# simready：一键生成可仿真 USD（用法）

> 最后更新：2025-12-22
> 
> 相关代码：
> - ../../set_physics/simready.py
> - ../../clean_data.py（底层会调用拆分逻辑）
> - ../../set_physics/pxr_utils/data_clean.py（`parse_scene`/引用与目录规范化）
> 
> 总索引：../overview/docs_index.md

## 索引
- [适用场景](#适用场景)
- [运行环境](#运行环境)
- [快速开始](#快速开始)
- [参数说明](#参数说明)
- [输出结构](#输出结构)
- [常用命令示例](#常用命令示例)
- [常见问题](#常见问题)
- [更新文档的触发点](#更新文档的触发点)

## 适用场景
`set_physics/simready.py` 的目标是把本仓库“拆分 target + 物理预处理”的流程封装成一个可参数化的一次性命令：
- 输入：单个场景 USD（通常来自数据集目录，且同级有 `Materials/*.mdl`）
- 输出：
  - 一份“规范化后的场景”（在 `out-root/scenes/<scene_id>/` 下）
  - 一份“可直接用于仿真”的场景 USD：
    - `interaction`：生成 `start_result_dynamic.usd`（包含可交互刚体/关节启用等）
    - `navigation`：生成 `start_result_navigation.usd`（全静态 + 语义标签 + 可选禁用门）
  - 同时生成所需的 `Materials/`、`models/` 等依赖目录（由拆分阶段创建）

它相当于把你原先需要“改脚本路径 + 分别跑 clean_data / preprocess_*”的工作，收敛成一个命令行入口。

## 运行环境
- **必须**在 Omniverse Isaac Sim 的 Python 环境运行（脚本内部会用 `isaacsim.SimulationApp`，导航模式还会用 `omni.isaac.core.utils.semantics`）。
- 若你只想执行“拆分/规范化（parse_scene）”，理论上只需要 `usd-core (pxr)`；但 `simready` CLI 的默认流程会进入 physics 处理，因此整体仍建议在 Isaac 环境执行。

## 快速开始
1. 准备一个场景 USD 文件路径，例如：
   - `.../home_scenes/<scene_id>/start_result_new.usd` 或 `start_result_fix.usd`
2. 在 Isaac Sim 环境执行：
   - 交互模式：
     ```bash
     <ISAAC_SIM_ROOT>/python.sh -m set_physics.simready \
       --input-usd /abs/path/to/start_result_new.usd \
       --out-root  /abs/path/to/out_root \
       --mode interaction \
       --headless
     ```
   - 导航模式（并禁用 door 类实例）：
     ```bash
     <ISAAC_SIM_ROOT>/python.sh -m set_physics.simready \
       --input-usd /abs/path/to/start_result_new.usd \
       --out-root  /abs/path/to/out_root \
       --mode navigation \
       --disable-doors \
       --headless
     ```

执行成功会在 stdout 打印生成的 USD 路径。

## 参数说明
- `--input-usd`：输入场景 USD 路径。
- `--out-root`：输出根目录（将创建 `Materials/`、`models/`、`scenes/`）。
- `--scene-id`：输出的场景目录名（默认从 `input-usd` 的父目录名推断，兜底用文件名）。
- `--mode`：`interaction` 或 `navigation`。
- `--headless`：无 UI 模式（推荐批处理使用）。
- `--disable-doors`：仅 `navigation` 使用：把 `door` 类实例设为 inactive。
- `--skip-clean`：跳过拆分/规范化阶段，把 `--input-usd` 当作已经规范化好的场景直接处理。
- `--normalized-usd`：仅在 `--skip-clean` 时生效，指定 physics 输出 USD 的路径（不指定则写到 `out-root/<scene_id>_<mode>.usd`）。

## 输出结构
默认（不加 `--skip-clean`）会生成类似结构：
- `out-root/Materials/`：统一材质与贴图
- `out-root/models/`：去重后的模型库
- `out-root/scenes/<scene_id>/`：
  - 规范化后的场景（与输入 USD 同名，或 fallback 到 `start_result_fix.usd` / `start_result_new.usd`）
  - `start_result_dynamic.usd`（interaction 模式）
  - `start_result_navigation.usd`（navigation 模式）

## 常用命令示例
- 已经有规范化 target 场景，只想重新打不同的 physics（跳过拆分）：
  ```bash
  <ISAAC_SIM_ROOT>/python.sh -m set_physics.simready \
    --input-usd /abs/path/to/target/scenes/<sid>/start_result_new.usd \
    --out-root  /abs/path/to/out \
    --mode interaction \
    --skip-clean \
    --normalized-usd /abs/path/to/out/dynamic.usd \
    --headless
  ```

## 常见问题
- `ModuleNotFoundError: isaacsim` 或 `omni.*`：说明你不是在 Isaac Sim 的 python 环境运行。
- 报错 `Expected /Root/Meshes in scene`：通常表示输入 USD 不是本仓库规范化后的结构，或输入场景层级不符合预期。
- 资产/材质找不到：确认输入 USD 的同级（或引用路径）能找到 `Materials/*.mdl`；若是外部数据集路径，建议先用 `--out-root` 做一次规范化输出。

## 更新文档的触发点
当以下内容变化时，建议同步更新本文：
- `set_physics/simready.py` 的 CLI 参数变更（增删参数、默认值、输出文件名）。
- interaction/navigation 的处理逻辑（碰撞近似策略、pickable 列表、door 规则）。
- `parse_scene` 的输出目录结构或引用修正规则变更。
