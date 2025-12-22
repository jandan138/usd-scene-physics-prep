# 操作记录（Materials / Assets / Scenes）

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../specs_normalizer/normalize.py
> - ../../scripts/check_phase2_assets.py
> - ../../scripts/check_phase3_scenes.py
> - ../../scripts/fix_mdl_textures_case.py
>
> 总索引：../overview/docs_index.md

## 索引
- [2025-12-02 第一次操作：合并两来源的 Materials 到统一目录](#2025-12-02-第一次操作合并两来源的-materials-到统一目录)
- [2025-12-03 第五次操作：阶段二资产导出与检查结论](#2025-12-03-第五次操作阶段二资产导出与检查结论)
- [2025-12-03 第六次操作：阶段三场景导出与检查结论](#2025-12-03-第六次操作阶段三场景导出与检查结论)
- [2025-12-08 第八次操作：修复 MDL 文件中贴图大小写残留](#2025-12-08-第八次操作修复-mdl-文件中贴图大小写残留)

## 2025-12-02 第一次操作：合并两来源的 Materials 到统一目录
- 来源目录：
  - /cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/commercial_scenes/scenes
  - /cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes/scenes
- 目标根：/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/export_specs_unified
- 合并内容：
  - 复制 .mdl 到 `export_specs_unified/Materials`，贴图到 `export_specs_unified/Materials/Textures`
  - 规范视图创建：
    - 将 `export_specs_unified/Materials` 规范化为 `export_specs_unified/Material/mdl`
    - 将 `Textures` 重命名为小写 `textures`
- 统计（首次执行）：
  - MDL: 1721（合并后统计）
  - Textures: 20047（合并后统计；其中空文件 1 个）
- 备注：
  - 超大贴图（>64MB）采用占位文件以加速拷贝；如需完整复制可取消该限制重新执行。
  - 该结构与规范文档 `docs/specs/dataset_structure_interpretation.md` 的 Materials 部分一致（`Material/mdl` 与小写 `textures`）。
  - 校验报告：`check_reports/materials_merge_report.json`，字段包含目标存在性、MDL/Textures 计数与缺失比对，状态 `completed` 表示两来源的 MDL 全部覆盖到目标目录。

### 超大贴图说明（未写入原因与来源）
- 目标空文件：`export_specs_unified/Materials/Textures/Day.png`（检测为 0 字节占位）
- 未写入原因：源贴图大小约 69.8MB（超过 64MB 阈值），为加速合并过程与避免 I/O 阻塞，使用占位文件代替；可调整阈值或取消限制后重试完整写入。
- 来源样例（均指向同名贴图）：
  - `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes/scenes/MWAX5JYKTKJZ2AABAAAAAAI8_usd/Materials/Textures/Day.png`（73167895 字节）
  - `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes/scenes/MVUCSQAKTKJ5EAABAAAAABA8_usd/Materials/Textures/Day.png`（73167895 字节）
  - `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes/scenes/MVUHLWYKTKJ5EAABAAAAABQ8_usd/Materials/Textures/Day.png`（73167895 字节）
  - 更多来源详见：`check_reports/materials_zero_size_details.json`
  
### 第二次操作：取消阈值并完整复制超大贴图
- 执行时间：2025-12-02
- 操作：取消 64MB 阈值，将 `Day.png` 从来源目录完整复制到目标目录。
- 结果统计：
  - `Day.png` 目标大小：73167895 字节（验证通过）
  - MDL 总数：1721
  - Textures 总数：20047
  - 零字节文件：0
- 校验报告更新：`check_reports/materials_merge_report.json` 已追加 `copied_large_textures` 记录，指明来源与目标路径及大小。

## 2025-12-02 第三次操作：规范化目录命名（Materials → Material/mdl，Textures → textures）
- 操作：
  - 删除旧的 `Material/mdl` 软链接
  - 移动 `export_specs_unified/Materials` 到 `export_specs_unified/Material/mdl`
  - 将 `Textures` 重命名为小写 `textures`
- 结果统计：
  - MDL 总数：1721（不变）
  - Textures 总数：20047（不变）
- 零字节文件：0（不变）

## 2025-12-02 第四次操作：准备阶段二资产导出（来源修正）
- 来源确认：
  - `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes/models/.../{category}/{uid}/instance.usd`
  - `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/commercial_scenes/models/.../{category}/{uid}/instance.usd`
- 计划：按上述来源导出为 `export_specs_unified/GRScenes_assets/{category}/{uid}.usd`，并相对改写 MDL 引用指向 `Material/mdl`。
- 当前执行状态：环境提示需退出 conda 或设置 ISAAC_SIM_ROOT 以加载 `pxr`，将于执行阶段二时使用 `scripts/isaac_python.sh` 运行。

## 2025-12-03 第五次操作：阶段二资产导出与检查结论
- 执行命令：
  - `bash -c 'set -euo pipefail; cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep; ./scripts/isaac_python.sh -c "from specs_normalizer.normalize import main; main()" --src-target /cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes --dst-root export_specs_unified --asset-name GRScenes_assets --models-only'`
  - `bash -c 'set -euo pipefail; cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep; ./scripts/isaac_python.sh -c "from specs_normalizer.normalize import main; main()" --src-target /cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/commercial_scenes --dst-root export_specs_unified --asset-name GRScenes_assets --models-only'`
- 目标输出：`export_specs_unified/GRScenes_assets/{category}/{uid}.usd`
- 检查命令：
  - 全量失败清单（仅失败项）：`python3 scripts/check_phase2_assets.py --assets-dir export_specs_unified/GRScenes_assets --materials-dir export_specs_unified/Material/mdl --fail-only --output check_reports/phase2_assets_report_failures.json`
- 检查结果：
  - `usd_count=85647`
  - `zero_size_count=0`
  - `mdl_ok_count=85647`
  - `tex_ok_count=85647`
  - `mdl_bad_examples=0`
  - `tex_bad_examples=0`
  - `results=[]`（无失败项）
- 结论：阶段二导出资产完整；所有导出 USD 的 MDL 与贴图引用均指向统一材质库 `export_specs_unified/Material/mdl`（贴图位于其 `textures` 目录）。

## 2025-12-03 第六次操作：阶段三场景导出与检查结论
- 执行命令：
  - `bash -c 'set -euo pipefail; export ISAAC_SIM_ROOT=/isaac-sim; cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep; ./scripts/isaac_python.sh -c "from specs_normalizer.normalize import main; main()" --src-target /cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes --dst-root export_specs_unified --scene-name GRScenes100 --scene-category home --scenes-only'`
  - `bash -c 'set -euo pipefail; export ISAAC_SIM_ROOT=/isaac-sim; cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep; ./scripts/isaac_python.sh -c "from specs_normalizer.normalize import main; main()" --src-target /cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/commercial_scenes --dst-root export_specs_unified --scene-name GRScenes100 --scene-category commercial --scenes-only'`
- 目标输出：`export_specs_unified/GRScenes100/{home|commercial}/{sid}/layout.usd`（并附带该场景目录下其它 USD 文件）
- 检查命令：
  - 失败项清单：`python3 scripts/check_phase3_scenes.py --export-root export_specs_unified --fail-only --output check_reports/phase3_scenes_report_failures.json`
- 检查结果：
  - `scene_dir_count=99`
  - `missing_count=0`
  - `bad_refs_count=0`
  - `missing=[]`
  - `bad_refs=[]`
- 报告路径：`check_reports/phase3_scenes_report_failures.json`
- 结论：阶段三场景导出完整；所有场景的 `layout.usd` 模型引用指向导出目录 `export_specs_unified/GRScenes100/*`，未发现缺失文件与错误引用。

### 严格深检（pxr）
- 工具：`scripts/check_phase3_strict_refs.py`（使用 `pxr.Usd/Sdf` 解析子层、引用、payload 与 Asset 属性，并将相对路径按 `layout.usd` 所在目录展开为绝对路径后校验）
- 允许前缀：
  - 场景：`export_specs_unified/GRScenes100`
  - 资产：`export_specs_unified/GRScenes_assets`
  - 材质：`export_specs_unified/Material/mdl`（贴图位于其 `textures`）
- 结论：
  - `scene_dir_count=99`
  - `missing_count=0`
  - `bad_refs_count=0`
  - 报告：`check_reports/phase3_strict_refs_failures.json`（为空集，全部通过）

## 2025-12-03 第七次操作：删除大写 Textures 目录与复核
- 操作：将 `export_specs_unified/Material/mdl/Textures` 删除（仅保留小写 `textures` 作为规范目录）。
- 复核一：严格引用深检（pxr）
  - 命令：`ISAAC_SIM_ROOT=... ./scripts/isaac_python.sh scripts/check_phase3_strict_refs.py --export-root export_specs_unified --fail-only --output check_reports/phase3_strict_refs_after_delete.json`
  - 结论：`scene_dir_count=99`、`missing_count=0`、`bad_refs_count=0`（无异常）
- 复核二：贴图一致性与引用分布检查（文本扫描）
  - 命令：`python3 scripts/check_textures_consistency.py --mdl-root export_specs_unified/Material/mdl --scenes-root export_specs_unified/GRScenes100 --assets-root export_specs_unified/GRScenes_assets --output check_reports/textures_consistency_after_delete.json`
  - 结论：`refs.counts.lower=0`、`upper=0`、`other=0`；`recommend_delete="upper"`，与删除动作一致，且无引用影响。
- 收尾结论：阶段三完成；材质贴图目录统一为小写 `textures`，所有 USD 引用通过严格（pxr）与文本双重复核，无问题。

## 2025-12-08 第八次操作：修复 MDL 文件中贴图大小写残留
- 背景：Isaac 打开资产时出现 `rtx.mdltranslator.plugin` 报错，路径残留 `Material/mdl/Textures/...`；严格 USD 检查显示资产指向小写 `textures`，问题来源于 MDL 文本内嵌默认参数。
- 操作：执行 MDL 文本修复脚本，统一将 `Textures/` 改为 `textures/` 并生成备份。
  - 干跑（差异清单）：`python3 scripts/fix_mdl_textures_case.py --mdl-root export_specs_unified/Material/mdl --dry-run --output check_reports/fix_mdl_textures_case_report_dry.json`
  - 实修：`python3 scripts/fix_mdl_textures_case.py --mdl-root export_specs_unified/Material/mdl --output check_reports/fix_mdl_textures_case_report.json`
  - 复核（再次干跑）：`python3 scripts/fix_mdl_textures_case.py --mdl-root export_specs_unified/Material/mdl --dry-run --output check_reports/fix_mdl_textures_case_report_dry_2.json`
- 结果：`total=1721`，最终报告显示 `changed=[]`，所有项 `reason=no_uppercase_found`（库内 MDL 已无大写 `Textures/`）。
- 结论：材质库（MDL 与贴图）大小写已统一；结合之前 USD 严格检查，场景与资产均落在小写 `textures`，可在 Isaac 中正常解析。
