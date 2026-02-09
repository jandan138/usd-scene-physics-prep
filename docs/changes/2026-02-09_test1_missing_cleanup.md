# 2026-02-09 — GRScenes-test1: 目录别名统一、door_* 缺失引用清理、Material/mdl 缺失排查

## 背景
在对 `GRScenes-test1/GRScenes_assets` 做类别别名统一（例如 `nightstand -> night_stand`）后，针对 validate 报告中的 `missing` 做进一步排查与清理，避免把“历史残留/占位引用”误认为 rename 引入的破坏。

## 变更概览
### 1) 类别合并脚本增强
- 更新 `scripts/merge_asset_categories_test1.py` 的合并映射 `CATEGORY_MERGES`：
  - 新增多组 underscore canonical（例如 `bookshelf -> book_shelf`、`trashcan -> trash_can`、`Musical_instrument -> musical_instrument` 等）。
- 增加 USD 扫描进度输出参数 `--progress-every`：对大规模 USD 重写/扫描时提供可观测性。

### 2) 仅清理“缺失 door_* reference”的最小改动（方案 A）
- 发现 validate 的一部分 `missing` 来源于 `layout.usd` 内 `/Root/Meshes/BaseAnimation/door_<GUID...>/...` 的 reference，目标资产目录在 test1 下已不存在（且这些 door_* USD 在历史上常为无 mesh 占位）。
- 实施“最小手术”：只移除具体 prim 上指向不存在文件的 reference item，不删除 scope/prim 本身，也不影响其他有效 door 原型。
- 效果：validate `missing` 从 78 降到 47；`reference` 类 missing 归零，剩余均为 `Material/mdl`。

相关脚本与输出：
- `scripts/oneoff_clear_missing_door_references.py`
- `check_reports/test1_clear_missing_door_refs_apply.json`
- `check_reports/test1_category_merge_underscore_v3_validate_after_clear_missing_door_refs.json`

### 3) Material/mdl 缺失排查（存在性检查）
- 对剩余 `Material/mdl` 缺失做去重与落盘路径检查：
  - validate 中剩余缺失 47 条，去重后目标路径 13 个。
  - 在 `GRScenes-test0/Material` 未发现这 13 个路径对应文件。
  - 在源目录中：
    - `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes/Materials` 找到 4 个 `textures/.../*.jpg`（对应 validate 中 `Material/mdl/textures/...` 的引用）。
    - `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/commercial_scenes/Materials` 未找到对应文件。
  - 剩余 9 个（Base/vMaterials_2/eyes03A 等）在上述三处均未找到。

相关输出：
- `check_reports/test1_mdl_missing_existence_check.md`
- `check_reports/test1_mdl_missing_existence_check_v2.json`

### 4) 辅助检查脚本（one-off inspectors）
为快速定位“某个场景 prim 引用了什么、引用 arc 是什么、defaultPrim/元数据是什么”等，新增多个一次性脚本：
- `scripts/oneoff_inspect_layout_prim.py`
- `scripts/oneoff_find_prims_in_layout.py`
- `scripts/oneoff_find_prims_in_stage.py`
- `scripts/oneoff_find_layout_prim_for_renamed_category.py`
- `scripts/oneoff_inspect_usd_mesh_content.py`

另：新增一个通用的数据结构检查脚本（非 one-off）：
- `scripts/data_check.py`

### 5) test1 资产类别计数（同格式 JSON）
- 生成与 `check_reports/GRScenes_assets_category_cnt.json` 同格式的 test1 版本：
  - `check_reports/GRScenes-test1_GRScenes_assets_category_cnt.json`

## 备注
- 本次清理只针对“指向不存在文件的 door_* reference”（方案 A），不触碰 `Material/mdl` 的 assetPath 引用；后者需单独决定是同步文件到 subset、还是改写引用路径/提供替代材质库。
