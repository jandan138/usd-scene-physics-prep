---
title: 环境准备建议
code_reference:
- set_physics/simready.py
- set_physics/preprocess_for_interaction.py
- set_physics/preprocess_for_navigation.py
created_at: '2025-11-26'
updated_at: '2025-12-22'
maintainer: Codex
status: Active
---

# 环境准备建议

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../clean_data.py
> - ../../set_physics/simready.py
> - ../../set_physics/preprocess_for_interaction.py
> - ../../set_physics/preprocess_for_navigation.py
>
> 总入口：../index.md

## 索引
- [Python 环境](#python-环境)
- [Isaac Sim](#isaac-sim)
- [验证](#验证)

## Python 环境
- 建议使用 Conda/venv，安装：`usd-core`、`numpy`、`pandas`。

## Isaac Sim
- 安装 Omniverse Isaac Sim 并确保 `isaacsim` 与 `omni.*` 模块可用；脚本中包含视口与传感器示例（`set_physics/tools/random_models.py`、`set_physics/tools/thumb_img.py`）。

## 验证
- 运行 `python clean_data.py` 生成 `target/` 后，按交互/导航脚本生成相应 USD 并用 Isaac Sim 打开验证碰撞与刚体效果。
