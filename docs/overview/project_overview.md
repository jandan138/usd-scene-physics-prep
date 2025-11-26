# 项目概述

- 本项目是围绕 USD（Universal Scene Description）的“资产拆分与物理仿真预处理”工具集：把原始场景规范化为可复用的模型/材质库与场景引用结构，并批量为对象绑定碰撞体、刚体、关节等物理属性，以便在 Omniverse Isaac Sim 中进行交互或导航仿真。
- 核心拆分入口在 `clean_data.py:5-24`，调用 `parse_scene` 完成重组与导出（`set_physics/pxr_utils/data_clean.py:517-705`）。
- 物理预处理分为交互与导航两类，分别见 `set_physics/preprocess_for_interaction.py:335-387` 与 `set_physics/preprocess_for_navigation.py:198-231,389-425`。

## 输入与输出
- 输入：`home_scenes/<scene_id>/start_result_fix.usd` 或 `home_scenes/<scene_id>/start_result_new.usd`；源材质库 `Materials/*.mdl`（按场景目录）。
- 输出：
  - 统一的材质与贴图库 `target/Materials`（含 `Textures`）。
  - 模型资产库 `target/models/<scope>/<articulated|others>/<category>/<model_hash>/instance.usd`。
  - 场景文件 `target/scenes/<scene_id>/start_result_new.usd` 或 `start_result_fix.usd`，内部以 reference 指向上述模型。

## 处理要点
- 变换规范化：将 `xformOp:transform` 拆解为 `translate/orient/scale`（`set_physics/pxr_utils/data_clean.py:121-149`），更适配物理 API。
- 引用与复制：相对路径资产复制到 `target` 并修正引用（`set_physics/pxr_utils/data_clean.py:367-397`）。
- 命名稳定性：按引用链和变换生成模型指纹并用 hash 命名（`set_physics/pxr_utils/data_clean.py:612-616,642-653`）。
- 语义标注：写入 `category/model_name` 语义标签，便于下游任务（`set_physics/preprocess_for_interaction.py:335-353`，`set_physics/preprocess_for_navigation.py:198-216`）。

