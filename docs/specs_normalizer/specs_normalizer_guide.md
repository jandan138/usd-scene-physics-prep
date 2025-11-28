# specs_normalizer 使用指南

## 名称含义
- “spec”指规范化的目录与引用结构，目的是把来源目录（包含`Materials`、`models`、`scenes`）整理为统一、可移植、无软链接依赖的结构，保证单资产或场景在独立打开时也能正确找到材质与贴图。

## 新目录结构
- `Material/mdl` 存放所有`.mdl`材质文件与贴图目录：`Material/mdl/Textures`
- `models/<category>/<uid>.usd` 统一模型输出位置，每个`usd`内的`mdl:sourceAsset`会改写为相对`Material/mdl`的路径
- `Scenes/<category>/...` 场景输出位置（可选），按类别聚合

## 环境要求
- 需在 Omniverse Isaac Sim 的 Python 环境运行以使用`pxr`（USD Python API）。本项目提供`scripts/isaac_python.sh`脚本自动定位并启用该环境。

## 快速开始
- 仅导出材质与模型（同时改写模型内 MDL 为相对路径）：
  ```bash
  ./scripts/isaac_python.sh -m specs_normalizer \
    --src-target /abs/path/to/source_root \
    --dst-root   /abs/path/to/output_root \
    --asset-name models \
    --models-only
  ```
- 完整导出（材质 + 模型 + 场景）：
  ```bash
  ./scripts/isaac_python.sh -m specs_normalizer \
    --src-target /abs/path/to/source_root \
    --dst-root   /abs/path/to/output_root \
    --asset-name models \
    --scene-name Scenes \
    --scene-category <home_scenes|commercial_scenes>
  ```
- 可选兼容软链接（仅用于过渡或对旧工具兼容）：
  ```bash
  ./scripts/isaac_python.sh -m specs_normalizer ... --compat-links
  ```

## 功能要点
- 材质导出与贴图复制：`specs_normalizer/exporters/materials.py:14-26`
  - 复制源`Materials`下所有`.mdl`到`Material/mdl`
  - 复制源`Materials/Textures`到目标`Material/mdl/Textures`（注意大小写为`Textures`）
- 模型导出与 MDL 相对路径改写：`specs_normalizer/exporters/assets.py:52-62`
  - 使用`pxr`遍历`Usd.Stage`属性，将引用`Materials/...`的`.mdl`统一改写为相对`Material/mdl`路径
  - 同时把包含`::.::Materials::<name>`的模块占位改写为文件引用`<name>.mdl`
- 可复用改写工具：`specs_normalizer/utils/mdl_rewrite.py:28-104`
  - 提供`rewrite_usd_mdl_paths(src_usd, dst_usd, materials_dir, use_relative=True, ...)`
  - 原地改写可用`rewrite_usd_mdl_paths_inplace(usd_path, materials_dir, ...)`
- 入口与参数：`specs_normalizer/normalize.py:21-38,44-62`
  - 支持`--models-only`只导出材质与模型（跳过结构校验与场景导出）
  - 标准参数：`--src-target`、`--dst-root`、`--asset-name`、`--scene-name`、`--scene-category`、`--with-annotations`、`--compat-links`

## 验证示例
- 检查模型`usd`内的`.mdl`是否已相对到`Material/mdl`：
  ```bash
  ./scripts/isaac_python.sh -c 'from pxr import Usd,Sdf; p="/abs/output/models/<category>/<uid>.usd"; s=Usd.Stage.Open(p); c=0
  
  for prim in s.Traverse():
    for attr in prim.GetAttributes():
      v=attr.Get();
      if isinstance(v,Sdf.AssetPath) and v.path.startswith("../../"):
        c+=1
  print("relative_mdl_refs", c)'
  ```
- 统计材质与纹理完整性（示例思路）：遍历`usd`收集`.mdl`文件名，再解析`Material/mdl/*.mdl`中出现的`Textures/...`相对路径，确认对应文件存在于`Material/mdl/Textures`。

## 常见问题
- 纹理目录大小写：当前库中的`.mdl`统一使用`Textures/`，请确保目标贴图目录为`Material/mdl/Textures`（大写）。
- `.mdl`与`USD`职责：`.mdl`内部持有纹理路径与参数默认值；`USD`中只保存对`.mdl`的引用（`mdl:sourceAsset`）。
- 软链接依赖：规范化后不再需要同目录`Materials`软链接；相对路径改写保证单个资产独立打开也能解析材质。

## 示例场景
- 旧结构根目录包含：`Materials/`、`models/`、`scenes/`
- 运行“仅模型与材质”导出：
  ```bash
  ./scripts/isaac_python.sh -m specs_normalizer \
    --src-target /cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/commercial_scenes \
    --dst-root   /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/export_specs_commercial \
    --asset-name models \
    --models-only
  ```

