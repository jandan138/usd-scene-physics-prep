# 工作流示例

## 资产拆分与交互仿真
1. 放置输入场景到 `home_scenes/`。
2. 运行 `python clean_data.py`，生成 `target/`。
3. 在 `set_physics/preprocess_for_interaction.py` 中设置 `scene_path/dest_scene_path`。
4. 运行脚本生成 `start_result_dynamic.usd`，打开 Isaac Sim 仿真。

## 导航仿真
1. 同上完成拆分。
2. 设置并运行 `set_physics/preprocess_for_navigation.py`，生成 `start_result_navigation.usd`。

## 打包导出
1. 运行 `set_physics/get_all_references.py` 生成依赖 JSON。
2. 运行 `set_physics/export_scene.py` 复制场景与依赖到发布目录，并重建材质软链接。

