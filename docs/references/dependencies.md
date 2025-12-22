# 环境与依赖

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../clean_data.py
> - ../../set_physics/preprocess_for_interaction.py
> - ../../set_physics/preprocess_for_navigation.py
>
> 总索引：../overview/docs_index.md

## 索引
- [基础依赖](#基础依赖)
- [仿真依赖](#仿真依赖)
- [注意](#注意)

## 基础依赖
- `usd-core`（pxr）：USD Python API。
- `numpy`：几何与矩阵处理。
- `pandas`：读取 CSV（`set_physics/pxr_utils/read_info.py:1-14`）。

## 仿真依赖
- Omniverse Isaac Sim：`isaacsim`、`omni.*` 模块；相关脚本依赖渲染与视口（如 `tools/random_models.py`、`tools/thumb_img.py`）。

## 注意
- Windows 环境下软链接与 `cp` 命令可能不可用，参考 `docs/operations/windows_notes.md`。

