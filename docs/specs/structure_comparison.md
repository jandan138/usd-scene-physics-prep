# 目录结构对比：原始输出 vs 规范结构

## 概述
- 本文对比当前项目在处理后生成的目录结构（`target/Materials | target/models | target/scenes`）与规范化结构提案（`docs/specs/dataset_structure_interpretation.md` 中的 Materials/Assets/Scenes）。
- 目标是明确一一映射关系、差异点与迁移建议，便于后续发布与数据集化。

## 原始输出结构（当前）
```
target/
├─ Materials/
│  ├─ Textures/
│  └─ *.mdl
├─ models/
│  ├─ layout/
│  │  ├─ articulated/
│  │  └─ others/
│  └─ object/
│     ├─ articulated/
│     └─ others/
│     └─ <category>/<model_hash>/
│        ├─ instance.usd
│        └─ Materials -> ../../../../../Materials  (软链接)
└─ scenes/
   └─ <scene_id>/
      ├─ Materials -> ../../Materials  (软链接)
      ├─ models    -> ../../models     (软链接)
      └─ start_result_new.usd | start_result_fix.usd
```
- 生成逻辑参考：
  - 目录与软链接创建：`set_physics/pxr_utils/data_clean.py:527-542`
  - 模型实例化与引用：`set_physics/pxr_utils/data_clean.py:642-666`
  - 模型指纹（MD5）：`set_physics/pxr_utils/data_clean.py:612-616`
  - 交互/导航额外输出：
    - 交互场景：`start_result_dynamic.usd`（`set_physics/preprocess_for_interaction.py:321-323`）
    - 导航场景：`start_result_navigation.usd`（`set_physics/preprocess_for_navigation.py:424-425`）

## 规范结构（提案）
```
Material/
├─ mdl/
│  ├─ textures/
│  └─ {mid}.mdl
├─ README.txt
└─ LICENSE

Asset_name/
├─ Asset_category/
│  ├─ {uid}.glb/usd
│  ├─ [optional] {uid}_annotation.json
│  └─ ...
├─ Asset_annotation.json
├─ [optional] Asset_features
├─ README.txt
└─ LICENSE

Scene_name/
├─ Scene_category/
│  ├─ {sid}/
│  │  ├─ layout.json/usd
│  │  ├─ rendering.png
│  │  ├─ {sid}_annotation.json
│  │  └─ [optional] StructureMesh
│  └─ ...
├─ [optional] Scene_features
├─ README.txt
└─ LICENSE
```
- 命名示例（来自讨论与约定）：
  - 资产库：`GRScenes_assets`、`MesaTask_assets`（或通用 `models`）
  - 场景集：`GRScenes100`、`MesaTask`
  - 场景类别：`home | commercial`（GRScenes 当前更常用；可按需细化如 `bedroom`），`office_table | dinning_table`（MesaTask 示例）

## 一一映射关系
- Materials
  - 原始：`target/Materials` → 规范：`Material/mdl`；`target/Materials/Textures` → `Material/mdl/textures`
  - `.mdl` 文件对应 `{mid}.mdl`；贴图目录对应 `textures/`
- Assets
  - 原始：`target/models/<scope>/<articulated|others>/<category>/<model_hash>/instance.usd`
  - 规范：`Asset_name/Asset_category/{uid}.usd`
  - 对应规则：
    - `Asset_name = models` 或数据集名（如 `GRScenes_assets`）
    - `Asset_category = <category>`（例如 `cabinet`、`plant`）
    - `{uid} = <model_hash>`；文件名 `{uid}.usd = instance.usd`
    - 单资产注释 `{uid}_annotation.json`：当前未生成；可由预处理脚本或统计流程补充
    - 汇总注释 `Asset_annotation.json`：可依据 `get_inst_model_mapping`（`set_physics/pxr_utils/data_clean.py:706-725`）或生成流程聚合
- Scenes
  - 原始：`target/scenes/<scene_id>/start_result_new.usd | start_result_fix.usd`
  - 交互/导航：`start_result_dynamic.usd`、`start_result_navigation.usd`
  - 规范：`Scene_name/Scene_category/{sid}/layout.usd | layout.json` + `rendering.png` + 注释 JSON
  - 对应规则：
    - `Scene_name = GRScenes100 | MesaTask`
    - `Scene_category = home | commercial | office_table | dinning_table`
    - `{sid} = <scene_id>`；`layout.usd` 对应 `start_result_new.usd | start_result_fix.usd`
    - `{sid}_annotation.json`：当前未输出；可根据语义/物理统计生成（参考交互处理中的语义设置 `set_physics/preprocess_for_interaction.py:335-353` 与对象计数）

## 关键差异点
- 命名维度
  - 原始结构以 `target/` 为根，未区分数据集名；规范强调数据集驱动的命名（`Asset_name`、`Scene_name`）。
- 注释与特征
  - 原始结构无 `{uid}_annotation.json`、`Asset_annotation.json`、`{sid}_annotation.json`、`Scene_features` 等；规范化结构预留了系统性的标注与特征位置。
- 资源引用方式
  - 原始结构大量使用软链接（`Materials`、`models`），规范化结构将材质与资产位于固定位置，场景按需引用。
- 结构网格
  - 规范结构中 `StructureMesh` 为可选；原始结构未单独导出墙体/地面等静态结构，相关元素混在 `Base` 范围（可由脚本提取）。
- 文件类型
  - 规范结构允许 `layout.json` 搭配 `layout.usd`；原始结构主要为 USD。

## 迁移建议（不改代码前提）
- 材质
  - 将 `target/Materials` 拷贝/映射到 `Material/mdl`，贴图至 `Material/mdl/textures`；增补 `README.txt` 与 `LICENSE`。
- 资产
  - 将 `target/models/.../<category>/<model_hash>/instance.usd` 映射为 `Asset_name/Asset_category/{uid}.usd`；生成 `{uid}_annotation.json`（语义、物理近似、材质引用等）；按类别生成 `Asset_annotation.json`。
- 场景
  - 将 `target/scenes/<scene_id>/start_result_*.usd` 映射为 `Scene_name/Scene_category/{sid}/layout.usd`；生成 `rendering.png`（参考 `set_physics/tools/thumb_img.py`）；生成 `{sid}_annotation.json`（对象统计/语义分布/物理对象计数）。
- 结构网格
  - 依据 `Meshes/Base` 提取墙体与地面，输出为 `StructureMesh`。
- Windows 注意
  - 软链接与 `cp` 在 Windows 上的兼容性问题参考 `docs/operations/windows_notes.md`。

## 风险与兼容性
- 软链接替换：需保证引用路径在新结构下仍有效；推荐在生成阶段更新 USD 的相对路径（参考资产路径处理 `set_physics/pxr_utils/data_clean.py:367-397`）。
- 体量扩展：规范结构的注释与特征目录会增加文件数量，建议批处理生成与校验。

## 结论
- 原始结构适合生产管线的高效拼装与复用；规范结构更适合发布与数据集管理。
- 二者可通过稳定映射平滑迁移；建议以新增导出脚本方式实现，使处理主干保持不变。
