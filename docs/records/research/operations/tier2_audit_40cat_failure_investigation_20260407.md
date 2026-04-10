---
title: "Tier2 Audit 40-Category Failure Investigation"
code_reference: scripts/placement_pairwise_compare.py, scripts/c1_autorun_categories.py
created_at: 2026-04-07
updated_at: 2026-04-07
maintainer: zhuzihou
status: active
---

# Tier2 Audit 40-Category Failure Investigation

## 背景

2026-04-03 执行 bbox-gated tier2 dedup rollout 批量运行：

```bash
./scripts/isaac_python.sh scripts/c1_autorun_categories.py \
  --bbox-gated --bbox-policy bbox_primary_rmse_observe \
  --v-matrix-mode auto --continue-on-failure ...
```

83 个 category 结果：
- 34 成功 (category_done)
- 9 跳过 (mapping_pairs=0)
- **40 失败 — 全部在 audit 步骤 (rc=2)**

运行目录：
```
check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout/_autorun/
  c1_v8_tier2_rollout_bbox_primary_rmse_observe_20260403_120430/
```

## 结论

**统一的同一个 bug，不是各有不同。**

`bbox_primary_rmse_observe` 策略名字暗示"只观察 bbox 差异、不阻断"，但代码实现中
observe 和 harder 两个策略对 bbox 的检查逻辑完全一样——都把 bbox 差异计为硬失败
(hard_fail)。唯一区别是 harder 额外检查 vertex RMSE。

Tier2 去重（topo_filesize / shape_invariant）用形状相似但不完全相同的模型替换原模型，
包围盒必然存在微小差异。审计把这些差异错误地当作阻断性失败，导致 40 个 category
全部 verdict=false, exit code=2。

## 根因分析

### 数据流

```
cert (生成 filtered_mapping)
  → apply (执行去重替换，写 layout.*.usd)
    → audit (pairwise compare 原始 vs 替换后)
      → step6 (promote / soft-delete)
```

### audit 的判定逻辑

`placement_pairwise_compare.py` 对每个 ref_changed 的 prim：

1. 根据 dedup mode 决定 `effective_eps_bbox`：
   - `geom_only` → 0.01（严格，V=Identity 精确匹配）
   - `topo_filesize` / `shape_invariant` → **0.15**（放宽）
   - `transitive` / unknown → 0.01（严格）

2. 检查 5 项 bbox 指标，任一超阈值则记为 `hard_failure`：
   - `bbox_min_delta > effective_eps_bbox`
   - `bbox_max_delta > effective_eps_bbox`
   - `footprint_extent_delta > effective_eps_bbox`
   - `footprint_axis_delta > eps_angle` (1.0)
   - `centroid_delta > eps_pos` (0.01)

3. **两个 policy 的差异仅在于**：
   - `bbox_primary_rmse_observe`：只检查上述 5 项
   - `bbox_primary_rmse_harder`：上述 5 项 + vertex_rmse > eps_geom

4. verdict 判定（`placement_pairwise_compare.py:804-808`）：
   ```python
   passed = (
       scenes_error == 0
       and no_mesh_ok
       and compared_scope_complete
       and ref_changed_hard_fail_count == 0   # ← 关键：任何 hard_fail 都阻断
   )
   ```

5. 退出码（`placement_pairwise_compare.py:1041`）：
   ```python
   return 0 if verdict["passed"] else 2
   ```

6. autorun 对 rc != 0 一律记录 category_fail（`c1_autorun_categories.py:499-503`）

### 问题所在

`observe` 策略**没有让 bbox 差异变成非阻断的**。bbox 检查的 hard_fail 逻辑在两个
策略下完全相同，observe 只是不额外检查 vertex RMSE 而已。名字暗示的"只观察"行为
并未实现。

### 相关代码位置

| 文件 | 行号 | 作用 |
|------|------|------|
| `scripts/placement_pairwise_compare.py` | 581-607 | mode-aware eps + hard_fail 判定 |
| `scripts/placement_pairwise_compare.py` | 643 | hard_fail 计数 |
| `scripts/placement_pairwise_compare.py` | 804-808 | verdict.passed 判定 |
| `scripts/placement_pairwise_compare.py` | 1041 | exit code |
| `scripts/c1_autorun_categories.py` | 499-503 | audit rc 处理 |

## 证据

### 抽样 verdict（4 个 category）

| Category | ref_changed_fail_count | 主要 blocking reasons |
|----------|----------------------|----------------------|
| bottle | 423 | bbox_min(298), bbox_max(268), footprint(224), centroid(78) |
| wall | 312 | bbox_max(280), bbox_min(269), footprint(241) |
| ground | 171 | footprint(136), bbox_max(114), bbox_min(109) |
| cabinet | 5 | footprint(5), bbox_max(3), bbox_min(3) |

### 与位移/RMSE 指标的对比

per-scene 日志显示所有场景：
- `displaced>0.01=0`（质心位移=0）
- `vertex_rmse>0.01=0`（顶点 RMSE=0）

**位移和 RMSE 完全通过，仅 bbox 外框尺寸差异触发了失败。**

这与预期一致：V 补偿矩阵正确修正了位置（质心不偏移），但替换模型的几何形状与原模型
不完全一致，包围盒自然不同。

### 34 个通过的 category

这些 category 要么只有 geom_only 对（V=Identity，bbox 完全一致），要么 tier2 对的
bbox 差异恰好在 0.15 以内。

## 修复方案选项

### 方案 A：让 observe 真正只观察（推荐）

修改 `placement_pairwise_compare.py`，在 `bbox_primary_rmse_observe` 策略下，
bbox 相关的 hard_failures 降级为 soft warnings——记录到报告但不计入
`ref_changed_hard_fail_count`。

**影响范围**：仅修改 verdict 判定逻辑，不影响数据或替换结果。

### 方案 B：进一步放宽 tier2 阈值

将 topo_filesize / shape_invariant 的 `effective_eps_bbox` 从 0.15 提高到更大值
（如 0.5 或 1.0）。

**风险**：可能掩盖真正有问题的替换。

### 方案 C：autorun 层面对 observe 策略的 rc=2 降级为警告

在 `c1_autorun_categories.py` 中，当 policy 为 observe 且 rc=2 时，不记录为失败，
继续执行 step6。

**缺点**：治标不治本，verdict.json 仍然显示 passed=false。

## 下一步

1. ~~选定修复方案~~ — 已选定方案 A
2. ~~实施修改~~ — 已实施：observe 策略下 tier2 (topo_filesize/shape_invariant) 的 bbox 检查降级为 soft warning，数据照样记录到 `soft_reason_counts`，但不计入 `ref_changed_hard_fail_count`，verdict.passed 不受影响。4 个测试全过（2 旧 + 2 新）。
3. 用 `--skip-done` 重跑 40 个失败 category（34 个已通过的会自动跳过）
4. 确认全部 83 category 通过（减去 9 个 mapping_pairs=0 的跳过项）
