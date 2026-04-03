---
title: "Topo NN Shuffle-Gated Fallback — Phase 3 实现记录"
code_reference: scripts/compute_vertex_transform.py
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: zhuzihou
status: implemented
---

# Topo NN Shuffle-Gated Fallback — 变更记录

## 一句话总结

对 `topo_filesize` 去重配对中「顶点被打乱」的情况，新增了一个 **自动检测 + 自动修复** 的 fallback 路径，修复了 72 对原本会失败的资产配对，且对已有的正常配对 **零影响**。

---

## 问题是什么？（通俗版）

想象你有两个一模一样的乐高模型，但其中一个的积木块被随机重新编号了。如果你按编号一一对应去比较，结果会乱七八糟——明明是同一个模型，对比结果却说差异巨大。

这就是 `topo_filesize` 去重模式的问题：两个资产几何完全一样，但 USD 文件内的 **顶点顺序被打乱了**（vertex shuffle）。原来的 `procrustes_full()` 函数假设顶点是按顺序对齐的，所以一遇到 shuffle 就失败，算出来的变换矩阵 V 完全错误（bbox 误差高达 86）。

## 怎么修的？

**核心思路**：先用 KDTree 最近邻（NN）把顶点重新配对回去，再做 Procrustes 对齐。

但不能无脑对所有配对都走 NN——spike 实验证明对 **非 shuffle** 的配对，NN 反而会引入回归（31/100 变差）。所以我们加了 **三重门控**：

```
1. baseline 结果够好（bbox ≤ 0.15）？ → 直接用，不碰
2. NN 找到的对应关系质量高（close_pct > 95%）？ → 确认是真 shuffle
3. NN 修正后结果确实好（bbox ≤ 0.01）？ → 采纳
任一条件不满足 → 保持原来的 baseline 结果
```

## 改了什么文件？

### `scripts/compute_vertex_transform.py`（+133 行）

| 新增内容 | 说明 |
|---------|------|
| 3 个常量 | `_NN_FALLBACK_BASELINE_THRESHOLD=0.15`, `_NN_FALLBACK_CLOSE_PCT_THRESHOLD=95.0`, `_NN_FALLBACK_ACCEPT_THRESHOLD=0.01` |
| `_bbox_delta_max_abs()` | 计算 V 矩阵应用后的 bbox 偏差 |
| `_nn_procrustes_in_normalized_space()` | bbox 归一化 → KDTree NN 配对 → Procrustes → 反归一化 |
| `_topo_same_vtx_with_nn_fallback()` | 共享 helper：baseline → 三重门控 → 选 V |

**两处调用点**都换成了共享 helper：
- `compute_V_for_pair()` — 直接 V 计算路径
- `_accumulate_V_along_path()` — transitive BFS 路径

### `docs/operations/topo_vertex_shuffle_correspondence_fix_plan.md`

Phase 3 章节从「设计草案」更新为「实际实现记录」，包含阈值来源和与原 plan 的差异说明。status 从 `draft` 改为 `implemented`。

## 新增产物（spike 实验）

| 文件 | 说明 |
|------|------|
| `scripts/debug_topo_nn_correspondence_spike.py` | Phase 2 独立 spike 脚本（不改核心库，只做实验） |
| `check_reports/test0_rebuilt_dedup/topo_nn_spike_results.csv` | 350 对实验数据（100 poor + 100 gap + 100 good + 50 geom_only） |
| `check_reports/test0_rebuilt_dedup/topo_nn_spike_analysis.md` | 分析报告（四条成功标准逐条验证） |
| `check_reports/test0_rebuilt_dedup/topo_nn_spike_analysis_plots.png` | 可视化图表 |

## 效果

| 指标 | 修复前 | 修复后 |
|------|--------|--------|
| 72 对 shuffle 配对的 bbox 误差 | 0.15 ~ 86（全部失败） | < 0.004（全部通过） |
| good/geom_only 回归 | — | **0**（门控从不触发） |
| 单对耗时 | ~0ms | max 0.77s（< 2s 阈值） |

## 安全机制

- **Kill-switch**：设置环境变量 `DEDUP_DISABLE_NN_FALLBACK=1` 可一键禁用 NN fallback，回到纯 Procrustes
- **默认行为不变**：门控只在 baseline 已经很差时才尝试 NN，绝大多数配对走原路径
- **三重检查**：NN 结果必须同时通过 shuffle 确认 + 质量验收才会被采纳

## 未覆盖的部分

128/200 non-shuffle 的 poor/gap 配对不在本次修复范围内。这些配对的误差根因不是顶点 shuffle，可能是拓扑差异、真几何差异等，需要下一轮单独调查。

## 验证

- 17/17 现有单元测试通过
- 合成数据测试：shuffle 修复 ✓、无回归 ✓、kill-switch ✓
