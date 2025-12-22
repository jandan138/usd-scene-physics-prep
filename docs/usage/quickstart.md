# 快速开始

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../clean_data.py
> - ../../set_physics/pxr_utils/data_clean.py
> - ../../set_physics/preprocess_for_interaction.py
> - ../../set_physics/preprocess_for_navigation.py
> - ../../set_physics/get_all_references.py
> - ../../set_physics/export_scene.py
>
> 总索引：../overview/docs_index.md

## 索引
- [依赖](#依赖)
- [步骤](#步骤)

## 依赖
- Python 包：`usd-core`（pxr）、`numpy`、`pandas`（可选，读取 CSV）。
- 交互/导航预处理：需要 Omniverse Isaac Sim 与 `omni.*` SDK。

## 步骤
1. 准备输入场景：将 `home_scenes/<scene_id>/start_result_fix.usd` 或 `start_result_new.usd` 放到仓库根目录下的 `home_scenes/`。
2. 执行拆分：运行 `python clean_data.py`（`clean_data.py:5-24,68-69`），生成 `target/` 目录。
3. 交互预处理：在 `set_physics/preprocess_for_interaction.py` 中修改 `scene_path/dest_scene_path`，运行后生成 `start_result_dynamic.usd`（`set_physics/preprocess_for_interaction.py:324-343,335-387`）。
4. 导航预处理：在 `set_physics/preprocess_for_navigation.py` 中修改路径，运行后生成 `start_result_navigation.usd`（`set_physics/preprocess_for_navigation.py:234-253,389-425`）。
5. 打包导出：使用 `set_physics/get_all_references.py` 生成依赖清单，再运行 `set_physics/export_scene.py` 将场景与依赖复制到目标目录（`set_physics/get_all_references.py:64-72`，`set_physics/export_scene.py:21-54`）。

