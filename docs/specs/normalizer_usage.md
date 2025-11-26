# 规范化导出工具使用说明（specs_normalizer）

## 工具概览
- 作用：将现有处理结果（如 `target/`）导出为规范结构（Materials / Assets / Scenes），不改动原始输出主干。
- 入口：`python -m specs_normalizer`
- 位置：`specs_normalizer/`

## 参数
- `--src-target` 源目录（默认 `target`）
- `--dst-root` 规范化输出根目录（默认 `export_specs`）
- `--asset-name` 资产库顶层名（默认 `models`；可设为 `GRScenes_assets`、`MesaTask_assets`）
- `--scene-name` 场景集顶层名（默认 `Scenes`；可设为 `GRScenes100`、`MesaTask`）
- `--scene-category` 场景类别（默认 `default`；如 `home`、`commercial`、`office_table`）
- `--with-annotations` 生成最小化注释 JSON（可选）

## 校验
- 函数：`specs_normalizer/validators/structure.py:3`
- 检查项：
  - `Materials` 是否存在 `.mdl` 或 `Textures`（`specs_normalizer/validators/structure.py:13-16`）
  - `models` 是否存在 `layout|object` 与 `articulated|others` 的任意组合（`specs_normalizer/validators/structure.py:21-30`）
  - `scenes/<sid>` 是否存在 `start_result_fix.usd | start_result_new.usd`（`specs_normalizer/validators/structure.py:35-45`）
- 入口调用：`specs_normalizer/normalize.py:16-21`

## 导出映射
- 材质
  - 入口：`specs_normalizer/exporters/materials.py:4`
  - 映射：`target/Materials/*.mdl` → `Material/mdl/{mid}.mdl`（`specs_normalizer/exporters/materials.py:5-11`）
  - 贴图：`target/Materials/Textures/**` → `Material/mdl/textures/**`（`specs_normalizer/exporters/materials.py:12-16`）
- 资产
  - 迭代器：`specs_normalizer/exporters/assets.py:5-23`
  - 导出：`target/models/.../<category>/<model_hash>/instance.usd` → `Asset_name/<category>/<model_hash>.usd`（`specs_normalizer/exporters/assets.py:25-40`）
  - 注释：`{uid}_annotation.json` 与 `Asset_annotation.json`（`specs_normalizer/exporters/assets.py:41-49`）
- 场景
  - 选择布局：`specs_normalizer/exporters/scenes.py:5-12`
  - 导出：`Scene_name/Scene_category/{sid}/layout.usd`（`specs_normalizer/exporters/scenes.py:14-31`）
  - 注释：可选 `{sid}_annotation.json`，包含 `/Root/Meshes` 子层级计数（`specs_normalizer/exporters/scenes.py:32-45`）

## 执行示例
- GRScenes：
  - `python -m specs_normalizer --src-target ./target --dst-root ./export_specs --asset-name GRScenes_assets --scene-name GRScenes100 --scene-category home --with-annotations`
- MesaTask：
  - `python -m specs_normalizer --src-target ./target --dst-root ./export_mesa_specs --asset-name MesaTask_assets --scene-name MesaTask --scene-category office_table`

## 输出结构示例
```
export_specs/
├─ Material/mdl/
│  ├─ textures/
│  └─ *.mdl
├─ GRScenes_assets/
│  └─ <category>/<model_hash>.usd
└─ GRScenes100/home/
   └─ <scene_id>/
      ├─ layout.usd
      └─ <scene_id>_annotation.json  # 可选
```

## 运行环境与注意
- 纯复制与重排：不修改 USD 内容、不创建软链接。
- Windows 兼容：使用 `shutil` 复制，避免 `cp`；参考 `docs/operations/windows_notes.md`。
- 注释生成：若未安装 `pxr.Usd`，场景注释的计数将为空结构。

## 入口编排
- 主入口：`specs_normalizer/normalize.py:7-28`
- 依次调用：校验 → 材质导出 → 资产导出 → 场景导出

