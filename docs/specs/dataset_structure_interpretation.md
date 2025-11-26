# 数据目录结构规范解读（Materials / Assets / Scenes）

## 目标
- 将现有项目的资产与场景组织规范化为“Materials / Assets / Scenes”三大板块，提升可分发性、可检索性与可扩展性。
- 对图示结构的各层级与占位符进行逐条解释，并给出与当前仓库输出的对齐映射。

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
│  ├─ {uid}.glb/usd              # 单资产文件（GLB 或 USD）
│  ├─ [optional] {uid}_annotation.json  # 单资产注释
│  └─ ...
├─ Asset_annotation.json         # 该 Asset_name 下所有资产的汇总注释
├─ [optional] Asset_features     # 资产级特征（统计/向量等）
├─ README.txt                    # 数据规模、树结构说明、其他信息
└─ LICENSE                       # 许可证信息
```
-- 说明
  - 顶层 `Asset_name`：资产大类或数据集资产库名（如 `GRScenes_assets`、`MesaTask_assets`，也可为通用 `models`）。
  - 二级 `Asset_category`：具体类别（如 `cabinet`、`plant`）。
  - `{uid}.glb/usd`：具体的资产文件，GLB 或 USD 二选一；`{uid}_annotation.json` 为该资产的标注（例如尺寸、可交互性、物理近似等）。
  - `Asset_annotation.json`：该类别下所有资产的聚合注释（方便批量检索与统计）。
  - `Asset_features`：可选的特征目录，存储类别级或资产级的数值特征。
- 与现有结构的映射
  - 现有模型输出：`target/models/<scope>/<articulated|others>/<category>/<model_hash>/instance.usd`（`set_physics/pxr_utils/data_clean.py:586-597,642-666`）。
  - 建议映射：
    - `Asset_name = models`，或按数据集命名为 `GRScenes_assets`、`MesaTask_assets` 等
    - `Asset_category = <category>`（如 `cabinet`、`plant`）
    - `{uid} = <model_hash>`（`set_physics/pxr_utils/data_clean.py:612-616` 的 MD5）
    - 文件：`{uid}.usd = instance.usd`
    - 单资产标注：`{uid}_annotation.json` 可记录语义、物理属性（碰撞近似/刚体）、引用材质等。
    - 汇总标注：`Asset_annotation.json` 可引用 `get_inst_model_mapping` 或自建聚合（`set_physics/pxr_utils/data_clean.py:706-725`）。

## Scenes
```
Scene_name/
├─ Scene_category/
│  ├─ {sid}/
│  │  ├─ layout.json/usd          # 场景布局（JSON 或 USD）
│  │  ├─ rendering.png            # 渲染缩略图（单视角或少量视角）
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
  - `rendering.png`：缩略图，可由 `set_physics/tools/thumb_img.py` 生成。
  - `StructureMesh`：将墙体、地面等结构单独输出，便于导航/碰撞分析。
- 与现有结构的映射
  - 现有场景输出：`target/scenes/<scene_id>/start_result_new.usd|start_result_fix.usd`（`set_physics/pxr_utils/data_clean.py:551-566,703-705`）。
  - 建议映射（示例）：
    - GRScenes：`Scene_name = GRScenes100`；`Scene_category = home | commercial`
    - MesaTask：`Scene_name = MesaTask`；`Scene_category = office_table | dinning_table`
    - `{sid} = <scene_id>`（目录名，如 `MV7J6NIKTKJZ2AABAAAAADA8_usd`）
    - `layout.usd = start_result_new.usd | start_result_fix.usd`
    - `rendering.png` 可通过渲染工具生成并放置在同级目录。
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
- 缩略图：`set_physics/tools/thumb_img.py` 可批量生成 `rendering.png`。

## 后续落地建议
- 在不改代码的前提下，可先新增导出脚本，将现有 `target/` 组织映射到本规范目录，并产出空的注释/特征占位文件，逐步填充。
- Windows 平台注意：软链接与 `cp` 命令请参考 `docs/operations/windows_notes.md`，采用 `shutil` 与相对路径替代。
