---
title: "Plan C Execution: Patch Dedup Placement Displacement"
code_reference: scripts/rewrite_layout_asset_refs_with_compensation.py
created_at: 2026-03-12
updated_at: 2026-03-12
maintainer: claude-agent
status: approved
---

# Plan C Execution: Patch Dedup Placement Displacement

## Overview

C1 dedup 将 old 资产替换为 canonical 资产后，补偿代码是 no-op（两个 bug），导致 18.2% 物品位置偏移。本文档描述修补方案（Plan C：原地 patch，不回滚）。

### 发现的两个 Bug

| # | Bug | 位置 | 影响 |
|---|-----|------|------|
| 1 | `_get_asset_internal_matrix()` 读 `/Root`（identity）而非 `/Root/Instance` | L165-188 | 补偿公式变 no-op |
| 2 | 补偿矩阵乘法顺序错误：用了右乘 `old_world * old_inst * canon^{-1}`，应为左乘 `canon^{-1} * old_inst * old_local` | L539 | 即使修了 Bug1，补偿结果仍然错误 |

### 正确公式推导（row-vector 惯例）

```
USD row-vector: p' = p * M
链式变换: p_world = p_mesh * M_instance * M_prim_local * M_parent_world

补偿条件: 换引用后世界坐标不变
  p_mesh * M_old_inst * M_old_local = p_mesh * M_canon_inst * M_new_local

解出 (对所有 p_mesh):
  M_new_local = M_canon_inst^{-1} * M_old_inst * M_old_local    ← 左乘

验证: normalize_asset_transforms.py L458 的场景补偿也用左乘:
  M_scene_new = T_center * R_y2z_inv * M_scene_old             ← 同样模式
```

---

## 前置条件检查清单

在执行前逐一确认：

- [ ] `GRScenes-test1-normalized/` 存在，包含 `GRScenes_assets/` 和 `GRScenes100/`
- [ ] `GRScenes-test1-normalized_bak/_dedup_assets/` 存在，包含 78 个备份子目录
- [ ] `check_reports/c1_bulk/` 存在，包含 79 个 `*_geom_only_mapping.json` 文件
- [ ] `GRScenes-test0/GRScenes100/` 存在，包含 99 个场景（用于验证）
- [ ] 所有 99 个场景目录下存在 `.pre_c1_*.usd` 布局备份文件
- [ ] `pxr` Python 模块可用（通过 `scripts/isaac_python.sh` 运行）

---

## 执行步骤

### Step 1: 修复 `_get_asset_internal_matrix()`

**文件**: `scripts/rewrite_layout_asset_refs_with_compensation.py`

将 L165-188 替换为：

```python
def _get_asset_internal_matrix(asset_usd_abs: str) -> Gf.Matrix4d:
    """Return local xform of asset's /Root/Instance as 'asset internal' matrix."""
    stage = Usd.Stage.Open(asset_usd_abs, load=Usd.Stage.LoadNone)
    if stage is None:
        raise RuntimeError(f"Failed to open asset USD: {asset_usd_abs}")

    default_prim = stage.GetDefaultPrim()
    if not default_prim or not default_prim.IsValid():
        children = [c for c in stage.GetPseudoRoot().GetChildren() if c and c.IsValid()]
        default_prim = children[0] if children else None

    if not default_prim or not default_prim.IsValid():
        return Gf.Matrix4d(1.0)

    # Look for "Instance" child under the default prim (e.g. /Root/Instance)
    # This is where normalize_asset_transforms.py writes the alignment transform.
    instance_prim = None
    for child in default_prim.GetChildren():
        if child.GetName() == "Instance":
            instance_prim = child
            break

    # Use Instance prim if found, otherwise fall back to default prim
    target_prim = instance_prim if (instance_prim and instance_prim.IsValid()) else default_prim

    xf = UsdGeom.Xformable(target_prim)
    if not xf:
        return Gf.Matrix4d(1.0)

    res = xf.GetLocalTransformation(Usd.TimeCode.Default())
    if isinstance(res, tuple):
        return res[0]
    return res
```

**Edge cases**:
- 无 `/Root/Instance` 的资产 → fallback 到 defaultPrim（identity），安全
- 3 个已知坏资产（empty mesh, empty-string MD5, no child prims） → fallback 到 identity，安全

### Step 2: 修复 L539 矩阵乘法顺序

将 L524-541 的补偿逻辑中：

```python
# WRONG (current):
new_world = old_world * old_internal * canonical_internal.GetInverse()
new_local = parent_world.GetInverse() * new_world
```

替换为直接操作 local transform（更简洁，与 normalize_asset_transforms.py 一致）：

```python
# CORRECT:
old_local = xform_cache.GetLocalTransformation(prim)
if isinstance(old_local, tuple):
    old_local = old_local[0]
new_local = canonical_internal.GetInverse() * old_internal * old_local
_set_local_matrix(prim, new_local)
```

同时更新文件开头注释（L19）：
```
# 旧: "We currently treat 'asset internal' as the local transform authored on the asset stage's defaultPrim."
# 新: "We treat 'asset internal' as the local transform on the Instance child of the asset stage's defaultPrim (/Root/Instance)."
```

### Step 2.5: Preflight Validation

在修改任何 layout 文件之前，运行预检验证脚本确认所有前置数据完整可靠。

**脚本**: `scripts/preflight_placement.py`

```bash
./scripts/isaac_python.sh scripts/preflight_placement.py \
  --dataset-root ./GRScenes-test1-normalized \
  --bak-root ./GRScenes-test1-normalized_bak \
  --mapping-dir ./check_reports/c1_bulk/ \
  --report-dir ./check_reports/placement_patch/
```

**6 项指标**:

| # | 指标 | 说明 | Pass 标准 |
|---|------|------|-----------|
| 1 | Earliest `.pre_c1` selection | 每个场景能否正确找到最早的 `.pre_c1_*.usd` 备份 | 100% 场景有 pre_c1 备份 |
| 2 | Old-ref locatability rate | 原始引用的 old 资产可在 live 或 `_dedup_assets/` 备份中定位 | > 95% |
| 3 | Mapping coverage rate | 原始引用中 `ref_changed` 的 prim 在 consolidated_mapping 中有对应条目 | > 95% |
| 4 | Current-ref consistency rate | 当前 layout.usd 中的引用与 mapping 中 canonical 路径一致 | > 95% |
| 5 | Canonical live-member resolution rate | 每个 canonical 资产在 live 数据集中存在且可打开 | > 95% |
| 6 | `/Root/Instance` anomaly bucketing | 统计无 `/Root/Instance` 子节点的资产并分类（identity fallback 安全 vs 需关注） | 报告输出即可 |

**Pass 条件**: 所有比率指标 > 95%，脚本退出码 = 0。

**必须在 Step 3 之前通过 preflight，否则不继续执行。**

---

### Step 3: 编写 Patch 脚本 `scripts/patch_dedup_placement.py`

#### 输入参数

```
--dataset-root    GRScenes-test1-normalized 根目录 (默认: ./GRScenes-test1-normalized)
--bak-root        备份根目录 (默认: ./GRScenes-test1-normalized_bak)
--mapping-dir     C1 mapping 目录 (默认: ./check_reports/c1_bulk/)
--dry-run         只报告不修改
--workers N       并行度 (默认: 8)
--report-dir      报告输出目录 (默认: ./check_reports/placement_patch/)
--scene-filter    可选场景过滤 (如 MV7J6*)
```

#### 算法

```
Phase 0: 构建数据结构
  0a. 加载所有 <category>_geom_only_mapping.json → consolidated_mapping: {old_abs: canonical_abs}
  0b. 扫描 _dedup_assets/ 构建 old_asset_locator: {original_abs: backup_abs}
      注意: 229 个 group 的 canonical 被不同批次 soft-delete 了,
      必须检测 live member 而非盲目用 usd_paths[0]

Phase 1: 逐场景处理 (可并行, 99 scenes)
  对每个场景:
  1a. 找到最早的 .pre_c1_*.usd 备份 → original_layout (dedup 前的引用)
  1b. 打开 original_layout, 记录 {prim_path: old_ref_abs}
  1c. 打开 current layout.usd
  1d. 备份 layout.usd → layout.pre_patch.<timestamp>.usd
  1e. 对每个 prim:
      - 获取 current_ref (当前引用的 canonical 资产)
      - 获取 orig_ref (从 1b 的备份中读取的原始引用)
      - 跳过 if orig_ref == current_ref (未被 dedup 换过)
      - 跳过 if orig_ref 不在 consolidated_mapping 中
      - 读取 M_old_inst: old 资产 /Root/Instance transform (从 backup)
      - 读取 M_canon_inst: canonical 资产 /Root/Instance transform (live)
      - 跳过 if delta ≈ identity (tolerance 1e-6)
      - 计算: new_local = M_canon_inst^{-1} * M_old_inst * old_local
      - 写入 prim
  1f. 保存 layout.usd
  1g. 输出 per-scene JSON report
```

#### 关键函数: `_get_root_instance_matrix()`

```python
def _get_root_instance_matrix(asset_usd_abs, old_asset_locator, cache):
    """读取资产 /Root/Instance 的 local transform."""
    if asset_usd_abs in cache:
        return cache[asset_usd_abs]

    # 确定实际文件路径 (live 或 backup)
    if os.path.exists(asset_usd_abs):
        actual_path = asset_usd_abs
    elif old_asset_locator and asset_usd_abs in old_asset_locator:
        actual_path = old_asset_locator[asset_usd_abs]
    else:
        raise FileNotFoundError(f"Asset not found: {asset_usd_abs}")

    stage = Usd.Stage.Open(actual_path, load=Usd.Stage.LoadNone)

    # 查找 /Root/Instance
    default_prim = stage.GetDefaultPrim()
    instance_prim = None
    if default_prim and default_prim.IsValid():
        for child in default_prim.GetChildren():
            if child.GetName() == "Instance":
                instance_prim = child
                break

    if not instance_prim or not instance_prim.IsValid():
        cache[asset_usd_abs] = Gf.Matrix4d(1.0)
        return Gf.Matrix4d(1.0)

    xf = UsdGeom.Xformable(instance_prim)
    result = xf.GetLocalTransformation(Usd.TimeCode.Default())
    m = result[0] if isinstance(result, tuple) else result
    cache[asset_usd_abs] = m
    return m
```

#### 幂等性设计

```python
# 如果已经 patch 过 (存在 .pre_patch.* 备份):
pre_patch_backups = sorted(scene_dir.glob("layout.pre_patch.*.usd"))
if pre_patch_backups:
    # 从第一个 pre_patch 备份恢复未修补状态, 然后重新 patch
    shutil.copy2(pre_patch_backups[0], layout_path)
```

#### 报告格式

**Per-scene**: `<scene_id>_patch_report.json`
```json
{
  "scene": "MV7J6..._usd",
  "patched": 15,
  "skipped_same_ref": 890,
  "skipped_identity_delta": 3,
  "errors": 0,
  "entries": [{"prim": "/Root/...", "action": "compensated", ...}]
}
```

**Summary**: `patch_summary.json`
```json
{
  "total_scenes": 99,
  "total_prims_patched": 1234,
  "total_errors": 0
}
```

#### Fail-Closed Design

Patch 脚本遇到以下任一条件时 **跳过该 prim 并记录 error/skip**，绝不静默写入错误数据：

| 条件 | 动作 | 日志级别 |
|------|------|----------|
| `orig_ref → canonical` 映射不唯一（一个 old 对应多个 canonical） | skip + error | ERROR |
| `orig_ref` 不在 `consolidated_mapping` 中 | skip | WARN |
| `current_ref` 与 mapping 中 canonical 路径不一致 | skip + error | ERROR |
| Old 资产既不在 live 也不在 `_dedup_assets/` 备份中（不可定位） | skip + error | ERROR |
| Canonical 资产不存在或无法打开 | skip + error | ERROR |
| 资产无 `/Root/Instance` 且 defaultPrim 非 identity transform | skip + warn | WARN |

原则：**宁可漏补也不错补**。所有 skip/error 计入报告，由操作员审查后决定是否需要人工处理。

#### Manifest

每次运行生成 manifest 文件 `<report-dir>/manifest_<run_id>.json`：

```json
{
  "run_id": "patch_20260312_143000",
  "patch_version": "1.0",
  "timestamp": "2026-03-12T14:30:00+08:00",
  "dry_run": false,
  "scenes": [
    {
      "scene_id": "MV7J6..._usd",
      "earliest_pre_c1": "layout.pre_c1_geom_only.20260311_140031.usd",
      "input_hash": "sha256:abc123...",
      "patched": 15,
      "skipped": 890,
      "errors": 0
    }
  ],
  "totals": {
    "scenes": 99,
    "patched": 1234,
    "skipped": 85000,
    "errors": 0
  }
}
```

`input_hash` 是每个场景 layout.usd 在 patch 前的 SHA-256，用于追溯和幂等性校验。

#### Atomic Write

layout.usd 写入采用原子操作，避免 patch 中途崩溃导致文件损坏：

```python
# 写入临时文件，然后原子替换
tmp_path = layout_path.with_suffix(".usd.tmp")
stage.Export(str(tmp_path))
os.rename(str(tmp_path), str(layout_path))  # POSIX atomic on same filesystem
```

不使用 in-place 修改（`stage.Save()`），确保任何时刻磁盘上只有完整的 layout.usd 或原始未修改的版本。

### Step 4: 执行 Patch

#### Step 4.0: Shadow Preview（高风险场景预览）

在全量 dry-run 之前，先对 3-5 个高风险场景生成预览文件，做三方比较：

```bash
# 生成 preview 文件（不修改原始 layout.usd）
./scripts/isaac_python.sh scripts/patch_dedup_placement.py \
  --dataset-root ./GRScenes-test1-normalized \
  --bak-root ./GRScenes-test1-normalized_bak \
  --mapping-dir ./check_reports/c1_bulk/ \
  --report-dir ./check_reports/placement_patch/ \
  --scene-filter "MV7J6*" \
  --preview-output  # 输出 layout.plan_c_preview.usd，不覆盖 layout.usd
```

**选取场景**: MV7J6（已知最多位移） + 含 curtain/towel/book prim 最多的 2-3 个场景。

**三方比较**:
| 文件 | 含义 |
|------|------|
| `layout.plan_c_preview.usd` | patch 后的预览结果 |
| `layout.usd` | 当前状态（有位移 bug） |
| test0 对应场景 `layout.usd` | 原始 ground truth |

使用 `compare_test0_vs_normalized.py` 对 preview 文件验证位移是否归零。确认无误后再进入全量 dry-run。

---

```bash
# 4a. Dry-run 先验证
./scripts/isaac_python.sh scripts/patch_dedup_placement.py \
  --dataset-root ./GRScenes-test1-normalized \
  --bak-root ./GRScenes-test1-normalized_bak \
  --mapping-dir ./check_reports/c1_bulk/ \
  --report-dir ./check_reports/placement_patch/ \
  --dry-run

# 4b. 检查 dry-run 报告
# 期望: MV7J6 场景 ~201 个 prim 需要 patch
cat check_reports/placement_patch/patch_summary.json | python -m json.tool

# 4c. 正式执行
./scripts/isaac_python.sh scripts/patch_dedup_placement.py \
  --dataset-root ./GRScenes-test1-normalized \
  --bak-root ./GRScenes-test1-normalized_bak \
  --mapping-dir ./check_reports/c1_bulk/ \
  --report-dir ./check_reports/placement_patch/ \
  --workers 8
```

### Step 5: 全量验证

编写批量验证脚本 `scripts/verify_all_scenes_vs_test0.py`，对所有 99 个场景（69 home + 30 commercial）做顶点世界坐标比较。

```bash
# 5a. Pre-patch baseline (可选, 记录修补前状态)
./scripts/isaac_python.sh scripts/verify_all_scenes_vs_test0.py \
  --test0-root ./GRScenes-test0 \
  --normalized-root ./GRScenes-test1-normalized \
  --out ./check_reports/placement_investigation/verify_pre_patch.json

# 5b. Post-patch 验证 (修补后)
./scripts/isaac_python.sh scripts/verify_all_scenes_vs_test0.py \
  --test0-root ./GRScenes-test0 \
  --normalized-root ./GRScenes-test1-normalized \
  --out ./check_reports/placement_investigation/verify_post_patch.json
```

#### Pass 标准

| 指标 | 要求 |
|------|------|
| 所有 prim 质心位移 | < 0.01 (浮点精度) |
| `displaced_gt_0.01` | 0 (across all 99 scenes) |
| 总 prim 数 | 与 pre-patch 一致 |
| `only_in_test0` / `only_in_normalized` | 0 |
| `total_no_mesh` | 0（或与 pre-patch 一致） |
| `max_per_mesh_displacement > 0.01` count | 0 |

#### 重点验证品类

| 品类 | 修补前位移数 | 期望修补后 |
|------|-------------|-----------|
| curtain | 19/21 (90.5%) | 0 |
| towel | 9/9 (100%) | 0 |
| book | 7/7 (100%) | 0 |
| desk | 2/3 (66.7%) | 0 |
| other | 57/149 (38.3%) | 0 |

---

## 回滚方案

如果 patch 结果不符合预期：

```bash
# 方案 1: 逐场景回滚 (从 .pre_patch 备份)
for d in GRScenes-test1-normalized/GRScenes100/home/*_usd/; do
  pre=$(ls "$d"layout.pre_patch.*.usd 2>/dev/null | head -1)
  [ -f "$pre" ] && cp "$pre" "$d"layout.usd
done

# 方案 2: 全量回滚到 dedup 前 (从 .pre_c1 备份)
# 注意: 这会撤销所有 dedup 引用替换
for d in GRScenes-test1-normalized/GRScenes100/home/*_usd/; do
  pre=$(ls "$d"layout.pre_c1_*.usd 2>/dev/null | sort | head -1)
  [ -f "$pre" ] && cp "$pre" "$d"layout.usd
done
```

---

## 风险评估

| 风险 | 级别 | 缓解 |
|------|------|------|
| Old asset 备份不完整 | 低 | 99.2% 覆盖，剩余 204 个仍 live |
| 229 个 broken canonical group | 中 | patch 脚本需检测 live member，非盲目用第一个 |
| 矩阵精度问题 | 低 | Scale-only 变换，条件数好 |
| 无 /Root/Instance 的资产 | 低 | fallback 到 identity（无需补偿） |
| 场景缺少 .pre_c1 备份 | 低 | 已确认 3,795 个 .pre_ 文件覆盖所有 69 home 场景 |

> **注意**: `placement_investigation_v2_report.md` L153 的公式 (`M_layout_new = M_layout_old * M_old_instance * M_canon_instance^{-1}`) 是右乘旧公式，**已确认错误**。执行以本文档 L33 左乘公式为准：`M_new_local = M_canon_inst^{-1} * M_old_inst * M_old_local`。

---

## 需要写的脚本清单

| # | 脚本 | 用途 | 运行方式 |
|---|------|------|----------|
| 1 | `scripts/preflight_placement.py` | Preflight 验证（Step 2.5） | `isaac_python.sh` |
| 2 | `scripts/patch_dedup_placement.py` | 核心 patch 脚本 | `isaac_python.sh` |
| 3 | `scripts/verify_all_scenes_vs_test0.py` | 全量验证 | `isaac_python.sh` |

---

## 数据位置速查

| 数据 | 路径 |
|------|------|
| Normalized 数据集 | `GRScenes-test1-normalized/` |
| test0 基准 | `GRScenes-test0/` |
| Dedup 备份 | `GRScenes-test1-normalized_bak/_dedup_assets/` |
| C1 映射 | `check_reports/c1_bulk/*_geom_only_mapping.json` |
| 3-way 合并报告 | `check_reports/union_merged_3way/all_categories_union_merged.json` |
| Layout .pre_ 备份 | `GRScenes-test1-normalized/GRScenes100/home/*_usd/layout.pre_c1_*.usd` |
| V2 调研报告 | `docs/operations/placement_investigation_v2_report.md` |
| 补偿 bug 代码 | `scripts/rewrite_layout_asset_refs_with_compensation.py:165-188, 524-541` |
