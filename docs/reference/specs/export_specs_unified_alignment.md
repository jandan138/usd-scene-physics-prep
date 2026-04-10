---
title: 导出统一结构对齐报告（阶段一至三）
code_reference:
- scripts/check_phase2_assets.py
- scripts/check_phase3_scenes.py
- scripts/fix_mdl_textures_case.py
- specs_normalizer/normalize.py
created_at: '2025-12-08'
updated_at: '2026-01-13'
maintainer: Codex
status: Active
---

# 导出统一结构对齐报告（阶段一至三）

> 最后更新：2026-01-13
>
> 相关代码：
> - ../../scripts/check_phase2_assets.py
> - ../../scripts/check_phase3_scenes.py
> - ../../scripts/fix_mdl_textures_case.py
> - ../../specs_normalizer/normalize.py
>
> 总入口：../index.md

## 索引
- [输出结构概览](#输出结构概览)
- [合规性验证摘要](#合规性验证摘要)
- [与规范文档的差距](#与规范文档的差距)
- [结论](#结论)

## 输出结构概览
- 材质库 `Material/mdl`
  - 目录：`Material/mdl` 与小写 `textures/`
  - 数量统计：MDL 1721、Textures 20047
  - 大小写统一：已将所有引用中的 `Textures/` 统一为 `textures/`
  - 相关修复：`scripts/fix_mdl_textures_case.py`（文本级修复）；严格引用检查均通过
- 资产库 `GRScenes_assets`
  - 当前导出结构（2025-12-22）：`GRScenes_assets/{category}/{uid}/{uid}.usd`
  - 目标发布结构（2026-01-13）：`GRScenes_assets/{category}/{uid}/usd/{uid}.usd`，并在 `usd/` 下创建 `textures -> ../../../Material/mdl/textures`（后置脚本生成）
  - 数量统计：USD 85647
  - 引用规范：资产中的 MDL 与贴图引用均指向 `Material/mdl`（贴图位于其 `textures/`）
- 场景集 `GRScenes100`
  - 当前导出结构（2025-12-22）：`GRScenes100/{home|commercial}/{sid}/layout.usd`（并附带该场景目录下其它 USD）
  - 目标发布结构（2026-01-13）：同目录可选创建 `textures -> ../../../Material/mdl/textures`（后置脚本生成），缩略图文件名推荐为 `front.png`
  - 数量统计：`scene_dir_count=99`
  - 引用规范：场景 `layout.usd` 中模型引用指向导出目录 `GRScenes100/*`，材质与贴图指向顶层材质库

## 合规性验证摘要
- 阶段二资产检查（文本扫描）
  - 摘要：`usd_count=85647`、`zero_size_count=0`、`mdl_ok_count=85647`、`tex_ok_count=85647`
  - 报告：`check_reports/phase2_assets_report_failures.json`
- 阶段三场景检查（文本扫描）
  - 摘要：`scene_dir_count=99`、`missing_count=0`、`bad_refs_count=0`
  - 报告：`check_reports/phase3_scenes_report_failures.json`
- 阶段三严格检查（pxr 深检）
  - 摘要：`scene_dir_count=99`、`missing_count=0`、`bad_refs_count=0`
  - 报告：`check_reports/phase3_strict_refs_failures.json`
- 贴图一致性与引用分布（文本扫描）
  - 摘要：`refs.counts.lower=0`、`upper=0`、`other=0`；`recommend_delete="upper"`（与删除动作一致）
  - 报告：`check_reports/textures_consistency_report.json`、`check_reports/textures_consistency_after_delete.json`
- MDL 文本大小写修复（文本扫描）
  - 摘要：`total=1721`，最终干跑为 `changed=[]`、`reason=no_uppercase_found`
  - 报告：`check_reports/fix_mdl_textures_case_report.json`、`check_reports/fix_mdl_textures_case_report_dry_2.json`

## 与规范文档的差距
- 参考规范：`docs/reference/specs/dataset_structure_interpretation.md`
- Materials 差距
  - 规范含 `README.txt` 与 `LICENSE`；当前未提供
  - 目录结构对齐（`Material/mdl` 与小写 `textures`）已完成
- Assets 差距
  - 规范含 `{uid}_annotation.json`（单资产注释）与 `Asset_annotation.json`（汇总）；当前已支持生成（含 `asset_type` 字段）
  - 规范可选 `Asset_features`；当前未提供
  - 规范含 `README.txt` 与 `LICENSE`；当前未提供
  - 文件格式允许 `glb/usd`；当前为 `usd`
- Scenes 差距
  - 规范含 `front.png`（缩略图）、`{sid}_annotation.json`（场景注释）、可选 `StructureMesh` 与 `Scene_features`；当前未提供
  - 规范允许每个 USD 同目录创建 `textures` 软链接指向 `Material/mdl/textures`；该软链接将由后置脚本生成（导出第一步不生成）
  - 规范含 `README.txt` 与 `LICENSE`；当前未提供
  - 目录命名与分类（`Scene_name=GRScenes100`，`Scene_category=home|commercial`）已对齐

## 结论
- 三个阶段已完成，`export_specs_unified` 下的结构与引用规范均达成：
  - 所有 USD 就位：资产（`GRScenes_assets`）、场景（`GRScenes100`）、材质（`Material/mdl`）
  - 引用统一：场景与资产的模型/材质/贴图引用指向导出目录与顶层材质库；贴图目录大小写统一为小写 `textures`
- 与规范文档差距集中在“文档与注释/特征”类的附加内容：README、LICENSE、注释 JSON、特征目录与缩略图等；结构与命名已对齐，功能性引用已严格通过
