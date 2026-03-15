---
title: "Handoff: test0-rebuilt normalize pipeline — chain transform fix applied, ready for re-run"
code_reference: "scripts/normalize_asset_transforms.py:152, scripts/test0_rebuilt_normalize.py"
created_at: "2026-03-15"
updated_at: "2026-03-15"
maintainer: "team-lead"
status: "handoff"
---

# Handoff: test0-rebuilt Normalize Pipeline

## 当前状态

### 已完成

1. **Phase 1 allowlist 修复** — `scripts/check_test0_phase1_allowlist.py`
   - `_submit_logs` 过滤、`door_*` 动态模式、`book` 允许列表
   - 8/8 测试通过

2. **全量 Phase 1 + Phase 2 执行** — run ID `20260314_rebuilt_full_dlc_v1`
   - Phase 1: 79 Succeeded / 35 allowlisted failures
   - Phase 2: Succeeded (DLC job `dlc1wjemtlyp8uz1`, 2 min)
   - Pairwise + Audit: 均完成 (exit 0)
   - **Hard gate: FAILED** — 487 displaced prims

3. **Root cause 调查完成** — `_get_chain_transform` 矩阵乘法顺序 bug
   - Bug 位置: `scripts/normalize_asset_transforms.py:152`
   - 问题: `result = result * _get_local_matrix(p)` 应为 `result = _get_local_matrix(p) * result`
   - 原因: 祖先优先 → 后代优先 (USD row-vector convention)
   - 影响: 仅影响 Instance 和 mesh 之间有中间 prim (Group 节点) 的资产
   - 验证: desk/5e55f06f 修复后误差 18.618m → 0.000000m

4. **修复已应用**
   - Line 152 一行改动 + docstring 更新
   - 回归测试 `test_get_chain_transform_multiply_order` 已添加
   - 5/5 测试通过
   - Smoke test gap 分析完成（文档已写）

### 未提交 (unstaged changes)

运行 `git diff --stat` 查看所有改动。关键改动文件：
- `scripts/normalize_asset_transforms.py` — line 152 修复 + docstring
- `scripts/check_test0_phase1_allowlist.py` — allowlist 扩展
- `tests/test_normalize_asset_transforms.py` — 回归测试
- `tests/test_check_test0_phase1_allowlist.py` — allowlist 测试
- `docs/test0_full/` — 多份调查文档
- `docs/changes/2026-03-15_matrix_multiply_order_fix.md`
- `docs/operations/smoke_test_gap_analysis.md`

## 下一步: 重跑 Phase 1 + Phase 2

### 为什么需要全量重跑

`_get_chain_transform` 同时被 Phase 1 (center 计算) 和 Phase 2 (场景补偿) 调用。修复后：
- Phase 1 会算出正确的 center 值
- Phase 2 会用正确的 center 做补偿
- 两边都要重跑，不能只跑 Phase 2

### 执行方式

**选项 A: 使用 orchestrator (推荐)**

```bash
# 1. 先 commit 修复
git add scripts/normalize_asset_transforms.py tests/test_normalize_asset_transforms.py
git commit -m "fix(normalize): correct matrix multiply order in _get_chain_transform"

# 2. 新建 run (需要新 run_id 避免覆盖旧结果)
python scripts/test0_rebuilt_normalize.py submit-full \
  --source-root GRScenes-test0-rebuilt \
  --normalized-root GRScenes-test0-rebuilt-normalized \
  --report-root check_reports/test0_rebuilt_full \
  --run-id 20260315_chain_fix_v1

# 3. 等待完成
python scripts/test0_rebuilt_normalize.py wait-full \
  --manifest check_reports/test0_rebuilt_full/20260315_chain_fix_v1/run_manifest.json \
  --poll-seconds 30
```

**注意**: 需要先清理或重命名 `GRScenes-test0-rebuilt-normalized/GRScenes_assets/` 目录，因为 Phase 1 需要写入新的 normalized 资产。

**选项 B: 手动分步执行**

```bash
# Phase 1: 提交 114 个 DLC 任务 (每个 category 一个)
bash scripts/dlc/submit_normalize_phase1.sh 20260315_chain_fix_v1

# 等 Phase 1 完成后 merge centers
python scripts/merge_phase1_centers.py \
  --phase1-root check_reports/test0_rebuilt_full/20260315_chain_fix_v1/normalize/phase1 \
  --out-dir check_reports/test0_rebuilt_full/20260315_chain_fix_v1/normalize/phase1/centers_merged

# Phase 2: 提交 1 个 DLC 任务 (场景补偿)
bash scripts/dlc/submit_normalize_phase2.sh 20260315_chain_fix_v1

# 后续: pairwise + audit + hard gate
```

### 预期结果

- 487 displaced prims → **0**
- 126 missing centers → **不变** (数据质量问题，非代码 bug)
- Hard gate: **PASS** (如果 missing centers 不阻断)

### 关键文件路径

| 文件 | 用途 |
|------|------|
| `scripts/normalize_asset_transforms.py` | 修复所在文件 |
| `scripts/test0_rebuilt_normalize.py` | orchestrator |
| `scripts/check_test0_phase1_allowlist.py` | Phase 1 allowlist 验证 |
| `check_reports/test0_rebuilt_full/20260314_rebuilt_full_dlc_v1/` | 旧 run 结果 (buggy) |
| `GRScenes-test0-rebuilt/` | 源数据 (干净 baseline) |
| `GRScenes-test0-rebuilt-normalized/` | 输出目录 |

### 关键文档

| 文档 | 内容 |
|------|------|
| `docs/test0_full/displacement_root_cause_chain_transform_20260315.md` | Root cause 分析 (正确版) |
| `docs/test0_full/grscenes_test0_rebuilt_phase1_allowlist_verdict_20260315.md` | Phase 1 allowlist 判定 |
| `docs/changes/2026-03-15_matrix_multiply_order_fix.md` | 修复变更记录 |
| `docs/operations/smoke_test_gap_analysis.md` | Smoke test 覆盖缺口分析 |
| `docs/test0_full/grscenes_test0_normalize_only_runbook.md` | 全量执行 runbook |

### 126 missing centers 说明

这是独立于 487 displaced 的问题，**零重叠**：
- 36 个 book 资产: Phase 1 归一化失败 (meshless)
- 29 个 other 资产: Phase 1 静默跳过
- 31 个 door_* 资产: 空字符串 MD5 (d41d8cd98f00b204e9800998ecf8427e)
- cabinet/person: 已知坏资产

这些是源数据质量问题，hard gate 的 `center_found` 检查可能需要容忍这 126 个。
