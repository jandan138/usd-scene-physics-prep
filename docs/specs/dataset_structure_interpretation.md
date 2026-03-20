---
title: 数据目录结构规范解读（Materials / Assets / Scenes）
code_reference:
- set_physics/pxr_utils/data_clean.py
- set_physics/preprocess_for_interaction.py
- set_physics/preprocess_for_navigation.py
- set_physics/tools/thumb_img.py
created_at: '2025-11-26'
updated_at: '2026-01-13'
maintainer: Codex
status: Active
---

# 数据目录结构规范解读（Materials / Assets / Scenes）

> 最后更新：2026-01-13
>
> 相关代码：
> - ../../set_physics/pxr_utils/data_clean.py
> - ../../set_physics/preprocess_for_interaction.py
> - ../../set_physics/preprocess_for_navigation.py
> - ../../set_physics/tools/thumb_img.py
>
> 总索引：../overview/docs_index.md

## 索引
- [目标](#目标)
- [Materials](#materials)
- [Assets](#assets)
- [Scenes](#scenes)
- [命名示例与对话结论](#命名示例与对话结论)
- [占位符命名规范](#占位符命名规范)
- [与现有流程的契合点](#与现有流程的契合点)
- [后续落地建议](#后续落地建议)

## 目标
- 将现有项目的资产与场景组织规范化为“Materials / Assets / Scenes”三大板块，提升可分发性、可检索性与可扩展性。
- 对图示结构的各层级与占位符进行逐条解释，并给出与当前仓库输出的对齐映射。

> 重要约定（textures 软链接）：
> - 最终发布包中，**每个 `.usd` 文件的同目录下**可以存在一个 `textures` 软链接，指向顶层统一材质库：`Material/mdl/textures`。
> - 该软链接**不在目录导出第一步生成**（specs_normalizer 导出阶段忽略软链接），后续由独立脚本统一创建。

## Materials
```
Material/
├─ mdl/
│  ├─ textures/          # 纹理贴图集合目录
│  ├─ {mid}.mdl          # 单个材质定义（MDL 文件）
│  └─ ...
├─ README.txt            # 数据规模、树结构说明、其他信息
└─ LICENSE               # 许可证信息
```
- 说明
  - `mdl/`：存放材质定义与其依赖的贴图；`textures/`为贴图目录。
  - `{mid}.mdl`：材质标识（mid）的 MDL 文件，一个材质一个文件。
  - `README.txt`：建议记录材质数量、贴图大小、目录树、命名规范等。
  - `LICENSE`：对应材质的授权条款。
- 与现有结构的映射
  - 现有：`target/Materials` 与 `target/Materials/Textures`（`set_physics/pxr_utils/data_clean.py:527-534,535-542`）。
  - 建议将 `target/Materials/*.mdl` 与 `target/Materials/Textures` 对应 `Material/mdl/{mid}.mdl` 与 `Material/mdl/textures/`。

## Assets
```
Asset_name/
├─ Asset_category/
│  ├─ {uid}/
│  │  ├─ glb/
│  │  │  └─ {uid}.glb               # 可选：单资产文件（GLB）
│  │  ├─ usd/
│  │  │  ├─ {uid}.usd               # 单资产文件（USD）
│  │  │  └─ textures -> ../../../Material/mdl/textures  # 软链接：每个 USD 同目录 textures
│  │  ├─ urdf/
│  │  │  ├─ {uid}.urdf              # 可选：单资产文件（URDF）
│  │  │  └─ parts/                  # 可选：URDF 子部件资源
│  │  ├─ {uid}_annotation.json      # 单资产注释（含 asset_type）
│  │  ├─ front.png                  # 正视图预览（推荐）
│  │  ├─ [optional] left/right/back.png
│  │  └─ [optional] rendering.mp4   # 演示视频
│  └─ ...
├─ Asset_annotation.json         # 该 Asset_name 下所有资产的汇总注释
├─ [optional] Asset_features     # 资产级特征（统计/向量等）
├─ README.txt                    # 数据规模、树结构说明、其他信息
└─ LICENSE                       # 许可证信息
```
-- 说明
  - 顶层 `Asset_name`：资产大类或数据集资产库名（如 `GRScenes_assets`、`MesaTask_assets`，也可为通用 `models`）。
  - 二级 `Asset_category`：具体类别（如 `cabinet`、`plant`）。
  - 三级 `{uid}`：单资产专属目录。
  - `glb/`、`usd/`、`urdf/`：按格式分子目录；最低要求为存在 `usd/{uid}.usd`。
  - `usd/textures`：软链接，指向顶层 `Material/mdl/textures`，方便在资产目录内独立打开 USD。
  - `{uid}_annotation.json`：该资产的标注文件。
    - 必须包含字段 `asset_type`，取值为 `{'rigid', 'soft', 'part_aware', 'articulation'}` 之一。
    - 本仓库同时约定一组“稳定字段”，用于描述资产大小、姿态与材质组织方式（见下方“单资产标注标准”）。
  - `front.png`：资产正视图缩略图（推荐）。
  - `Asset_annotation.json`：该类别下所有资产的聚合注释。
  - `Asset_features`：可选的特征目录，存储类别级或资产级的数值特征。
- 与现有结构的映射
  - 现有模型输出：`target/models/<scope>/<articulated|others>/<category>/<model_hash>/instance.usd`（`set_physics/pxr_utils/data_clean.py:586-597,642-666`）。
  - 建议映射：
    - `Asset_name = models`，或按数据集命名为 `GRScenes_assets`、`MesaTask_assets` 等
    - `Asset_category = <category>`（如 `cabinet`、`plant`）
    - `{uid} = <model_hash>`（`set_physics/pxr_utils/data_clean.py:612-616` 的 MD5）
    - 文件：`{uid}/usd/{uid}.usd` (对应源 `instance.usd`)
    - 软链接：`{uid}/usd/textures -> ../../../Material/mdl/textures`（由后续脚本生成；导出第一步不生成）
    - 单资产标注：`{uid}/{uid}_annotation.json`，其中 `asset_type` 字段根据源路径 (`articulated`/`others`) 自动填充。
    - 汇总标注：`Asset_annotation.json` 可引用 `get_inst_model_mapping` 或自建聚合（`set_physics/pxr_utils/data_clean.py:706-725`）。

### 单资产标注标准（{uid}_annotation.json）

该文件用于“资产级（物体级）”元数据描述，路径固定为：`<Asset_name>/<category>/<uid>/{uid}_annotation.json`。

字段规范（推荐所有字段都出现；不可用时填 `null`）：
- `uid` (string, required): 资产唯一 ID（与目录名一致）。
- `category` (string, required): 资产类别（如 `bed`、`cabinet`）。
- `asset_type` (string, required): 资产物理类型，取值：`rigid | soft | part_aware | articulation`。
- `glb_size` (number|null, required): `glb/{uid}.glb` 的文件大小（单位：MB，按 $1\,\text{MB}=1024\times1024$ bytes 计算）。不存在则为 `null`。
- `usd_size` (number|null, required): 该资产 USD 的“可渲染包大小”（单位：MB，按 $1\,\text{MB}=1024\times1024$ bytes 计算）。
  - 计算方式：`usd/{uid}.usd` 文件大小 + 该 USD 引用到的 MDL 文件 + 这些 MDL 引用到的贴图（去重后累计）。
  - 说明：材质库集中存储时，多个资产可能共享同一组 MDL/贴图；`usd_size` 以“该资产单独可渲染所需依赖”为准，可能与整体包的增量下载大小不同。
  - 若 `usd/{uid}.usd` 不存在则为 `null`。
- `urdf_size` (number|null, required): `urdf/` 目录总大小（单位：MB，递归累计，按 $1\,\text{MB}=1024\times1024$ bytes 计算）。不存在则为 `null`。
- `orientation` (number, required): 资产默认朝向（角度制，单位：degree）。当前约定默认值为 `0`，后续可从 `{0, 15, 30, ..., 345, 360}` 中选择。
- `usd_material_softlink` (bool, required):
  - `true`：USD 使用“集中材质库”（通常为顶层 `Material/mdl`，USD 内引用为相对路径，且可选存在 `usd/textures -> ../../../Material/mdl/textures` 软链接）。
  - `false`：USD 使用“就地材质”（材质与贴图与 `{uid}.usd` 同目录/同层级存放）。

兼容/占位字段（历史上已存在，允许为空字符串或空数组）：
- `description` (string)
- `material` (string)
- `dimensions` (string)
- `mass` (string)
- `placement` (array)

示例：

```json
{
  "uid": "0a85b986de35ccfdec7c686d791fd747",
  "category": "bed",
  "description": "",
  "material": "",
  "dimensions": "",
  "mass": "",
  "placement": [],
  "asset_type": "rigid",
  "glb_size": null,
  "usd_size": 12345678,
  "urdf_size": null,
  "orientation": 0,
  "usd_material_softlink": true
}
```

## Scenes
```
Scene_name/
├─ Scene_category/
│  ├─ {sid}/
│  │  ├─ layout.json/usd          # 场景布局（JSON 或 USD）
│  │  ├─ front.png                # 缩略图（推荐）
│  │  ├─ [optional] rendering.mp4 # 视角演示
│  │  ├─ textures -> ../../../Material/mdl/textures  # 软链接：若存在 layout.usd，则同目录可放 textures
│  │  ├─ {sid}_annotation.json    # 场景标注（语义/物理/统计等）
│  │  └─ [optional] StructureMesh # 结构网格（如墙体、地面）
│  └─ ...
├─ [optional] Scene_features      # 场景级特征（统计/向量等）
├─ README.txt                     # 数据规模、树结构说明、其他信息
└─ LICENSE                        # 许可证信息
```
-- 说明
  - 顶层 `Scene_name`：数据集场景集名称（例如 `GRScenes100`、`MesaTask`）。
  - 二级 `Scene_category`：场景类别（如 `home` 或 `commercial`；也可按更细粒度如 `bedroom`，视数据集而定）。
  - `{sid}`：场景唯一 ID 文件夹，包含布局、缩略图、标注与可选结构网格。
  - `layout.json/usd`：场景结构描述；USD 可直接用于 Isaac Sim，JSON 可用于外部工具链。
  - `front.png`：缩略图，可由 `set_physics/tools/thumb_img.py` 生成（文件名在规范中为 `front.png`）。
  - `textures`：软链接（可选），指向顶层 `Material/mdl/textures`，用于场景目录内独立打开 `layout.usd`。
  - `StructureMesh`：将墙体、地面等结构单独输出，便于导航/碰撞分析。
- 与现有结构的映射
  - 现有场景输出：`target/scenes/<scene_id>/start_result_new.usd|start_result_fix.usd`（`set_physics/pxr_utils/data_clean.py:551-566,703-705`）。
  - 建议映射（示例）：
    - GRScenes：`Scene_name = GRScenes100`；`Scene_category = home | commercial`
    - MesaTask：`Scene_name = MesaTask`；`Scene_category = office_table | dinning_table`
    - `{sid} = <scene_id>`（目录名，如 `MV7J6NIKTKJZ2AABAAAAADA8_usd`）
    - `layout.usd = start_result_new.usd | start_result_fix.usd`
    - `front.png` 可通过渲染工具生成并放置在同级目录。
    - `{sid}_annotation.json` 可记录场景级统计、物理对象数量、语义分布等（可参考 `preprocess_for_interaction.py:335-387` 的语义与物理绑定逻辑）。

## 命名示例与对话结论
- 资产库命名：
  - GRScenes 使用 `Asset_name = GRScenes_assets`
  - MesaTask 使用 `Asset_name = MesaTask_assets`
- 场景集命名：
  - GRScenes 使用 `Scene_name = GRScenes100`
  - MesaTask 使用 `Scene_name = MesaTask`
- 场景类别：
  - GRScenes 目前按 `home | commercial`，暂未确认更细的分类；若未来需要，可引入如 `bedroom` 等更细粒度分类。
  - MesaTask 场景类别示例为 `office_table | dinning_table`。
-- 原则：
  - 不仅面向 GRScenes，规范需适配多个数据集与任务；`Asset_name`/`Scene_name`/`Scene_category` 采用数据集驱动的命名以支持扩展。

## 占位符命名规范
- `{mid}` 材质标识：建议小写字母与下划线，或原始 MDL 名；确保在 `mdl/` 范围内唯一。
- `{uid}` 资产唯一标识：建议使用稳定的哈希（如 MD5），来自引用链与变换组合（`set_physics/pxr_utils/data_clean.py:612-616`）。
- `{sid}` 场景唯一标识：沿用现有场景目录名（如 `MV…_usd`）。
- 命名规则建议：仅使用 `a-z0-9_-`，避免空格与大写（便于跨平台一致性）。

## 与现有流程的契合点
- 目录生成：现有脚本已生成 `target/Materials`、`target/models`、`target/scenes`（`set_physics/pxr_utils/data_clean.py:527-542,551-566`）。
- 模型实例化：`instance.usd` 与引用关系已建立（`set_physics/pxr_utils/data_clean.py:652-666`）。
- 语义与物理：交互/导航预处理可为后续标注文件提供信息来源（`set_physics/preprocess_for_interaction.py:335-387`，`set_physics/preprocess_for_navigation.py:198-231,389-425`）。
- 缩略图：`set_physics/tools/thumb_img.py` 可批量生成 `front.png`。


## 后续落地建议
- 在不改代码的前提下，可先新增导出脚本，将现有 `target/` 组织映射到本规范目录，并产出空的注释/特征占位文件，逐步填充。
- Windows 平台注意：软链接与 `cp` 命令请参考 `docs/operations/windows_notes.md`，采用 `shutil` 与相对路径替代。
