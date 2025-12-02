# 操作记录（Materials / Assets / Scenes）

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
