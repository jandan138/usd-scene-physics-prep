# 统一导出流水线（三阶段说明与预期）

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../specs_normalizer/normalize.py
> - ../../specs_normalizer/exporters/materials.py
> - ../../specs_normalizer/exporters/assets.py
> - ../../specs_normalizer/exporters/scenes.py
>
> 总索引：../overview/docs_index.md

## 索引
- [总目标](#总目标)
- [阶段一：合并并规范化材质库（Materials → Material/mdl）](#阶段一合并并规范化材质库materials-materialmdl)
- [阶段二：导出资产库并改写 MDL 引用](#阶段二导出资产库并改写-mdl-引用)
- [阶段三：导出场景并重写引用指向新库](#阶段三导出场景并重写引用指向新库)
- [验证与检查](#验证与检查)

## 总目标
- 将现有 `target/` 输出规范化为可分发结构：`Material/mdl`、`<Asset_name>/...`、`<Scene_name>/<Scene_category>/{sid}`。
- 所有 USD 内的 MDL 与模型引用统一改写为“相对路径”，仓库内自足，无需软链接。
- 产出基础校验与注释，便于后续检索与统计。

## 阶段一：合并并规范化材质库（Materials → Material/mdl）
- 操作内容：
  - 合并两来源材质（commercial/home）到统一输出根。
  - 复制 `.mdl` 与贴图目录，规范成 `export_specs_unified/Material/mdl` 与小写 `textures`。
  - 处理超大贴图（如需，取消阈值并完整复制）。
- 命令示例：
  - `python -m specs_normalizer --src-target target --dst-root export_specs_unified`（完整流水线）
  - 或仅材质：`python -m specs_normalizer --src-target target --dst-root export_specs_unified --models-only`（会先导出材质再导出资产）
- 预期结果：
  - `export_specs_unified/Material/mdl` 下 `.mdl` 数量与贴图数量与源数据一致；贴图目录名为小写 `textures`。
  - 无零字节文件；校验报告更新，记录超大贴图与其来源。

## 阶段二：导出资产库并改写 MDL 引用
- 操作内容：
  - 遍历来源：
    - `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes/models/.../{category}/{uid}/instance.usd`
    - `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/commercial_scenes/models/.../{category}/{uid}/instance.usd`
    - 复制为 `{asset_name}/{category}/{uid}.usd`。
  - 将 USD 中所有 MDL 引用改写为相对指向 `export_specs_unified/Material/mdl`。
  - 可选生成 `{uid}_annotation.json` 与 `Asset_annotation.json`。
- 命令示例：
  - 使用 Isaac Sim Python（确保 `ISAAC_SIM_ROOT` 配置或退出conda）：
    - `./scripts/isaac_python.sh -c "from specs_normalizer.normalize import main; main()" --src-target /cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes --dst-root export_specs_unified --asset-name GRScenes_assets --models-only`
    - `./scripts/isaac_python.sh -c "from specs_normalizer.normalize import main; main()" --src-target /cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/commercial_scenes --dst-root export_specs_unified --asset-name GRScenes_assets --models-only`
  - 加注释：在上述命令末尾追加 `--with-annotations`
- 预期结果：
  - `export_specs_unified/GRScenes_assets/{category}/{uid}.usd` 成立，数量与类别分布与源数据一致（去重后）。
  - 任意资产 USD 的 MDL 引用改写为相对路径（形如 `@../../Material/mdl/<mdl_name>.mdl@`），贴图引用解析到 `Material/mdl/textures/...`。
  - 若启用注释，生成单资产与类别聚合注释文件。

## 阶段三：导出场景并重写引用指向新库
- 操作内容：
  - 对 `target/scenes/<sid>/` 选择布局文件（优先 `start_result_raw.usd`，其次 `fix`/`new` 或任意 USD）。
  - 复制为 `export_specs_unified/<Scene_name>/<Scene_category>/<sid>/layout.usd`，同级复制其它 USD。
  - 将场景内对材质与模型的引用改写为相对路径：`Material/mdl` 与 `<Asset_name>`。
  - 可选生成 `{sid}_annotation.json`（若 `pxr` 可用，统计 `/Root/Meshes` 子层级）。
- 命令示例：
  - 全量：`python -m specs_normalizer --src-target target --dst-root export_specs_unified --scene-name GRScenes100 --scene-category home`
  - 指定场景：`python -m specs_normalizer --src-target target --dst-root export_specs_unified --scenes-only --scene-name GRScenes100 --scene-category home --scene-ids MV7J6NIKTKJZ2AABAAAAADA8_usd`
  - 加注释：在上述命令末尾追加 `--with-annotations`
- 预期结果：
  - `export_specs_unified/GRScenes100/home/{sid}/layout.usd` 建立；同级保留其它 USD 以便复核。
  - 场景内的引用改写为相对路径，模型指向 `<Asset_name>/{category}/{uid}.usd`，材质指向 `Material/mdl/<mdl_name>.mdl`。
  - 若启用注释，生成 `{sid}_annotation.json`，含基本统计信息。

## 验证与检查
- 材质计数：`find export_specs_unified/Material/mdl -maxdepth 1 -type f -name "*.mdl" | wc -l`
- 贴图计数：`find export_specs_unified/Material/mdl/textures -type f | wc -l`
- 资产计数：`find export_specs_unified/GRScenes_assets -type f -name "*.usd" | wc -l`
- 场景计数：`find export_specs_unified/GRScenes100/home -mindepth 1 -maxdepth 1 -type d | wc -l`
- 引用抽检：打开若干 USD，确认 MDL/模型路径为相对路径且可解析。
