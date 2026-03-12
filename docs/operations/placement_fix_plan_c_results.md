---
title: "Plan C Placement Patch: Execution Results & Root Cause Reframe"
code_reference: scripts/patch_dedup_placement.py, scripts/verify_all_scenes_vs_test0.py
created_at: 2026-03-12
updated_at: 2026-03-12
maintainer: claude-agent
status: complete
---

# Plan C Placement Patch: Execution Results & Root Cause Reframe

## TL;DR

**原始假设错误。** 27,687 个 displaced prims 并非由 dedup 补偿 bug 导致，而是 **V2 asset normalization 场景补偿** (`normalize_asset_transforms.py`) 的遗留问题。Plan C patch 本身正确执行（0 errors, 3,187 prims 补偿），但对位移几乎无影响，因为 dedup 几乎没有引入位移。

---

## 时间线

| 时间 | 事件 |
|------|------|
| 2026-03-11 | V2 Re-normalization 完成：52,904 资产 normalize + 99 场景补偿 |
| 2026-03-11 | C1 3-way dedup: 25,361 资产 soft-deleted，引用替换为 canonical |
| 2026-03-12 | 6-agent 调查团队发现两个 bug，产出 Plan C |
| 2026-03-12 | 11-agent 实现团队完成 Plan C 代码（bug fixes + 4 脚本 + 文档） |
| 2026-03-12 | Plan C 执行：preflight → dry-run → patch → verify |
| 2026-03-12 | 发现 `stage.Export()` 扁平化 bug，修复并重跑 |
| 2026-03-12 | **根因重新定位**：位移来自 V2 normalization，非 dedup |

---

## 根因重新定位

### 原始假设 (V2 Investigation Report)

> "C1 dedup 换引用后补偿代码是 no-op，导致 18.2% 物品世界坐标位移。"
> — `docs/operations/placement_investigation_v2_report.md`

V2 报告比较了 **test0 vs normalized**（post-dedup），发现 27,687 displaced prims，并归因于 dedup 补偿的两个 bug。

### 实际根因

位移 **在 dedup 之前就已存在**。证据：

```
Pre-C1 backup (dedup 前) vs test0:
  场景 MV7J6...DA8: 1105 compared, 795 displaced > 0.01 (71.9%)

Post-patch (dedup + Plan C 补偿后) vs test0:
  场景 MV7J6...DA8: 1105 compared, 795 displaced > 0.01 (71.9%)

数字完全相同。Dedup 没有引入位移，Plan C 补偿也没有减少位移。
```

### 为什么 V2 报告结论有误

V2 报告的方法论是：
1. test1 vs normalized → 零位移 → "normalization 正确"
2. test0 vs normalized → 27,687 位移 → "dedup 导致位移"

推理错误在第 2 步：test0 vs normalized 的差异包含了 **normalization + dedup** 两个步骤。V2 报告假设 normalization 正确（基于第 1 步），所以把所有差异归因于 dedup。但实际上第 1 步只证明 test1→normalized 变换自洽，不能证明 normalized 的世界坐标与 test0 一致。

**真正的位移源是 V2 场景补偿 (`normalize_asset_transforms.py:458`)**：
```python
M_scene_new = T_center * _R_Y2Z_INV * M_scene_old
```
这个公式对 27% 的 prims 产生了错误的世界坐标。

### 两个 Bug 确实存在但影响极小

Bug 1 (`_get_asset_internal_matrix` 读 `/Root` 而非 `/Root/Instance`) 和 Bug 2 (矩阵右乘) 确实存在且已修复。但由于 dedup 的 canonical 和 old 资产的 `/Root/Instance` transform 几乎相同：

- 101,934 total prims with refs
- 54,178 same ref (未被 dedup 换过) → 不需要补偿
- 44,569 identity delta (换过但 M_canon ≈ M_old) → 不需要补偿
- **3,187 prims 需要补偿** → 已正确补偿，但不影响位移

---

## Plan C 执行详情

### Preflight (E1) — PASS

所有 6 项指标 ≥ 99.99%：

| 指标 | 结果 | Rate |
|------|------|------|
| M1 Pre-C1 backup selection | 69/69 scenes | 100.0% |
| M2 Old-ref locatability | 51,747/51,753 | 99.99% |
| M3 Mapping coverage | 18,812/18,812 | 100.0% |
| M4 Current-ref consistency | 18,812/18,812 | 100.0% |
| M5 Canonical live resolution | 3,862/3,862 | 100.0% |
| M6 /Root/Instance bucketing | 4,319 with_transform, 3 identity, 0 missing | 100.0% |

Report: `check_reports/preflight/preflight_report.json`

### Dry Run (E3)

| Metric | Value |
|--------|-------|
| Scenes processed | 99 |
| Prims to patch | 3,187 |
| Skipped (same ref) | 54,178 |
| Skipped (identity delta) | 44,569 |
| Errors | 0 |

### Actual Patch (E4, re-run with Export fix)

| Metric | Value |
|--------|-------|
| Scenes processed | 99 |
| Prims patched | 3,187 |
| Errors | 0 |
| Backups created | 99 `.pre_patch.*.usd` files |
| File sizes | 1.1-1.3 MB (正常, 非扁平化) |
| Elapsed | 71.9s |

### Verification (E5, final v2)

| Metric | Pre-Patch (E2) | Post-Patch v2 | Delta |
|--------|----------------|---------------|-------|
| total_compared | 101,919 | 101,919 | **0** (正确) |
| displaced_gt_0.01 | 27,687 | 27,719 | +32 (浮点噪声) |
| max_per_mesh_gt_0.01 | 13,008 | 12,999 | -9 (浮点噪声) |
| only_in_test0 | 31 | 31 | **0** (正确) |
| only_in_normalized | 0 | 0 | 0 |

**结论**：Pre-patch 与 post-patch 的位移数几乎完全一致 (±32, 属浮点精度噪声)。Patch 对位移无实质影响。

---

## `stage.Export()` 扁平化 Bug

### 问题

`patch_dedup_placement.py` 首次执行使用了 `stage.Export(tmp_path)`（L591），这会将整个 composed stage（包括所有引用的 mesh/material）扁平化写入单个文件，导致：

- 82 个 scene layout 从 ~2MB 膨胀至 ~2.5GB
- 引用结构被破坏（`/Flattened_Prototype_1` 替代了 `/Root`）
- 验证脚本只能比较 11,093 prims（90,826 因引用丢失无法比较）

### 修复

```python
# 旧 (错误): 扁平化整个 composed stage
stage.Export(tmp_path)

# 新 (正确): 只导出 root layer, 保留引用结构
stage.GetRootLayer().Export(tmp_path)
```

Commit: `2e0afb2`

### 为什么首次验证显示 "93.4% 修复"

首次验证（使用扁平化后的文件）的 "93.4% 修复" 是假象：
- `total_compared` 从 101,919 降到 11,093（因引用丢失）
- 剩余可比较的 11,093 prims 恰好位移较少（样本偏差）
- `only_in_test0` 从 31 飙升到 90,868（引用丢失的 prims）

修复 Export bug 后，`total_compared = 101,919`，`only_in_test0 = 31`，位移数与 pre-patch 一致。

---

## 已确认的 17 个问题场景

这 17 个场景在 **所有状态**（pre-C1, pre-patch, post-patch）下都有相同位移，确认是 V2 normalization 遗留问题：

| 场景族 | 场景数 | 总 displaced |
|--------|--------|-------------|
| MWBGLKQKTKJZ2 | 5 | 490 |
| MVUHLWYKTKJ5E | 4 | 425 |
| MWAX5JYKTKJZ2 | 4 | 396 |
| MVUCSQAKTKJ5E | 2 | 203 |
| MWF4WLIKTIFZI | 1 | 132 |
| MWHLEPQKTIFZI | 1 | 72 |
| **总计** | **17** | **1,718** |

> 注：V2 全量验证中 displaced = 27,719 覆盖全部 99 个场景，不仅仅是这 17 个。
> 这 17 个是"全部 prims 都被 dedup 换过 + identity delta"的特殊子集。
> 其余 82 个场景同样有位移，只是分布不同。

---

## 文件清单

### 要了解目前状况，应阅读这些文件：

| 文件 | 说明 | 重要度 |
|------|------|--------|
| 本文档 | 执行结果 + 根因重新定位 | **必读** |
| `docs/operations/placement_fix_plan_c_execution.md` | Plan C 方案设计（含 preflight/fail-closed/manifest） | 参考 |
| `docs/operations/placement_investigation_v2_report.md` | V2 调研报告（**结论已过时**，位移归因有误） | 参考，注意 WARNING |
| `check_reports/placement_patch_v2/verify_post_patch_v2.json` | 最终验证结果（Export fix 后） | 数据 |
| `check_reports/placement_patch/verify_pre_patch.json` | Pre-patch baseline | 数据 |
| `check_reports/preflight/preflight_report.json` | Preflight 6 项指标 | 数据 |

### 代码文件：

| 文件 | 行数 | 说明 |
|------|------|------|
| `scripts/rewrite_layout_asset_refs_with_compensation.py` | 795 | Bug 1 + Bug 2 已修 (commit `18df762`) |
| `scripts/patch_dedup_placement.py` | 828 | 核心 patch (Export fix: commit `2e0afb2`) |
| `scripts/preflight_placement.py` | 718 | 只读 preflight (6 指标) |
| `scripts/verify_all_scenes_vs_test0.py` | 579 | 全量验证 (99 scenes vs test0) |
| `tests/test_patch_placement.py` | 452 | 27 unit tests |

### 数据/报告文件：

| 文件 | 说明 |
|------|------|
| `check_reports/placement_patch_v2/` | V2 patch run (Export fix 后的正确结果) |
| `check_reports/placement_patch/` | V1 patch run (Export bug, 结果不可信) |
| `check_reports/preflight/` | Preflight 报告 |
| `GRScenes-test1-normalized/GRScenes100/*/layout.pre_patch.*.usd` | 99 个 scene 的 pre-patch 备份 |
| `GRScenes-test1-normalized_bak/_dedup_assets/` | Dedup soft-delete 的资产备份 |

### Git Commits：

| Commit | 说明 |
|--------|------|
| `18df762` | Plan C 实现：bug fixes + 4 脚本 + docs |
| `dd0136c` | 执行结果文档（结论已过时，被本次更新覆盖） |
| `2e0afb2` | Export bug fix: `stage.Export()` → `GetRootLayer().Export()` |

---

## 下一步

**位移的真正修复方向不是 dedup 补偿，而是 V2 normalization 场景补偿。**

需要调查 `normalize_asset_transforms.py` 的 `compensate_scene()` 函数（L377-477），特别是 L458：
```python
M_scene_new = T_center * _R_Y2Z_INV * M_scene_old
```

27,719 个 prims (27.2%) 在此公式下产生了错误的世界坐标。需要确定：
1. 是公式本身有问题，还是输入参数（center、rotation）有问题？
2. 是否所有 99 个场景都受影响，还是只有特定场景族？
3. V2 当时的验证为什么没发现这个问题？（因为用了 test1 而非 test0 作为基准）

---

## 回滚方案

每个场景都有 `.pre_patch.*.usd` 备份。如需回滚 patch 效果：
```bash
for f in $(find GRScenes-test1-normalized/GRScenes100 -name 'layout.pre_patch.*.usd'); do
  dir=$(dirname "$f")
  cp "$f" "$dir/layout.usd"
done
```

注：patch 的实际效果极小（3,187 prims 补偿，大部分是 identity delta），回滚与否对位移无影响。
