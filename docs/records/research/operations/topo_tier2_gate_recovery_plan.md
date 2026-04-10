---
title: "Topo Precheck Recovery — 第二档门控改进计划"
code_reference:
  - scripts/compute_vertex_transform.py
  - scripts/debug_topo_precheck_recovery_spike.py
  - scripts/c1_autorun_categories.py
created_at: 2026-04-05
updated_at: 2026-04-05
maintainer: zhuzihou
status: draft
doc_class: record
---

# Topo Precheck Recovery — 第二档门控改进计划

## 一、当前 Runbook 阶段与瓶颈

**Runbook 状态**：Step 0（v8 报告重生成）已完成，Step 1（union 重建）已完成，Step 2 的 bottle dry_run 探针在 shuffle fix 后已完成（`c1_bulk_v8_shuffle_fix`）。**Step 3（apply + pairwise）和 Step 5（全量 rollout）尚未执行**；other 类目探针也因 bottle audit exit(1) 未跑。

**瓶颈 1：topo precheck 仍有 297 条被拒。** Shuffle fix 把 595→297（救回 298 条纯 shuffle），但剩余 297 条的根因不是顶点重排序——它们 `nn_close_pct` 大多只有 40-90%（而非 100%），`nn_unique_ratio` 中位数仅 0.59。这意味着 NN 匹配是「多对一 / 部分覆盖」的近似匹配，不是双射 shuffle。然而 spike 表明其中 223 条若强制采用 NN 的 V，世界空间 bbox 可压到 ≤0.5，41 条甚至 ≤0.01——**信号是存在的，但当前 Gate1（close_pct > 95%）完全拦死了这条路**。

**瓶颈 2：audit 仍为 false。** 这是 `observe` 策略的已知行为：`ref_changed` 类型的 bbox delta 在 observe 模式下触发 hard fail（390 条），与门控策略无关。这不是本轮需要解决的问题——observe 策略就是设计成「看看但不强制通过」的，不影响 cert 阶段的 eligible 判定。

**瓶颈 3：shape_invariant 的 14 条 bbox_precheck_failed 未动。** 数量小，但 shape_invariant 同顶点数分支同样走的是索引对齐 Procrustes（`compute_V_shape_invariant:671-689`），也会受非 shuffle 但需要 NN 辅助的情况影响。这是 Phase C 的潜在改进区域。

## 二、第二档门控设计

### 问题本质

当前三重门控（Gate 0 → Gate 1 → Gate 2）为纯 shuffle 设计，逻辑是：

```
baseline 差(>0.15) → NN 确认双射 shuffle(close_pct>95%) → NN 结果极好(bbox≤0.01) → 采纳
```

297 条 spike 数据表明存在第二类配对：**不是纯 shuffle（close_pct 30-95%），但 NN 对齐后 V 的世界空间 bbox 仍可达可接受范围**。这类配对的 NN 是「近似对应」而非「精确双射」，需要更宽松但有安全兜底的门控。

### 多对一 NN 下 Procrustes V 的稳定性风险

**关键数据**：297 条配对的 `nn_unique_ratio` 中位数仅 0.59，意味着约 41% 的 canon 点被映射到已被其他 canon 点占用的 old 点（多对一）。

**数学影响**：Procrustes 求解 `min ||A @ R - B||`。当 B（reordered old 点）中有大量重复行时：

- 多个不同的 canon 点被强行对齐到同一个 old 点，SVD 产生的 R 和 scale 被这些"伪约束"拉偏。
- V 矩阵可能在 **bbox 极值点附近表现良好**（因为极值点的 NN 匹配通常更独特），但 **内部顶点位移可能显著高于 bbox 暗示的水平**。

**bbox 不等于安全**：bbox 只检查 6 个极值（min/max per axis），内部顶点可能有显著位移但不影响 bbox。因此 Phase A 中**必须增加 vertex_rmse 验证**（见 Phase A 详细设计）。

### 297 条配对的 canonical 集中度

spike CSV 中 297 条配对仅来自 **37 个 unique canonical**，且 top-5 canonical 各有 24-32 条配对。推荐组合的 ~77 条更是集中在仅 **12 个 unique canonical**。这意味着 Tier2 的效果高度依赖于少数 canonical 的几何特性——如果某个 canonical 本身有问题（如拓扑退化），会"批量"影响 Tier2 判定。Phase A 需要做 **per-canonical 分组分析**。

### A 类 vs B 类划分


| 类型             | 条件                              | 数量（bottle） | 处置                         |
| -------------- | ------------------------------- | ---------- | -------------------------- |
| **A 类（不可救回）**  | `nn_bbox > 0.5`（NN 的 V 也无法过粗预检） | ~74 条      | **保持拒绝**，大概率非同一刚性几何或变换差异过大 |
| **B 类（可救回候选）** | `nn_bbox ≤ 0.5` 且 baseline 失败   | ~223 条     | 进一步分层评估                    |


B 类内部进一步分层：


| 子层       | 条件                      | 数量  | 信号强度                          |
| -------- | ----------------------- | --- | ----------------------------- |
| B-1（高信心） | `nn_bbox ≤ 0.01`        | 41  | 极强：NN 后 V 精度已达 shuffle-fix 水平 |
| B-2（中信心） | `0.01 < nn_bbox ≤ 0.15` | 95  | 强：世界 bbox 偏差在视觉不可见范围          |
| B-3（低信心） | `0.15 < nn_bbox ≤ 0.5`  | 87  | 中等：能过粗预检但偏差较大，需要额外验证          |


### 候选策略

#### 策略 1：分层阈值门控（推荐先模拟）

首发配置（Phase A 默认目标）：

- `nn_bbox ≤ 0.05`
- `nn_mean_dist_norm ≤ 0.02`
- `nn_unique_ratio ≥ 0.5`
- `nn_close_pct ≥ 10%`

其他阈值仅用于必要时的敏感性分析，不做全参数笛卡尔积扫描。

在 `_topo_same_vtx_with_nn_fallback()` 的 Gate 1（`close_pct > 95%`）失败后，不直接返回 baseline，而是进入**第二档判定**：

```
伪逻辑（不是代码，只是决策树）：

Gate 0: baseline_bbox > 0.15 → 继续
Gate 1: nn_close_pct > 95% → 如果是，走现有 shuffle 路径（Gate 2: nn_bbox ≤ 0.01）
Gate 1 失败 → 进入 Gate 1B（第二档）：
  前置条件（排除完全不相关的配对）：
  - nn_close_pct ≥ TIER2_CLOSE_PCT_FLOOR（候选值：10% / 15%）
  条件全部满足时采纳 NN 的 V：
  - nn_bbox ≤ TIER2_BBOX_THRESHOLD（候选值：0.01 / 0.05 / 0.15）
  - nn_mean_dist_norm ≤ TIER2_MEAN_DIST_THRESHOLD（候选值：0.02）
  - nn_unique_ratio ≥ TIER2_UNIQUE_THRESHOLD（候选值：0.5 / 0.7 / 0.9）
  否则 → 返回 baseline（保持现状）
```

> **nn_close_pct 下限说明**：spike 数据中推荐组合的 77 条配对 close_pct 从 12.4% 到 93.8% 变化极大。其中约 9 条 close_pct < 20%，意味着归一化空间中不到 20% 的点 NN 距离 < 0.01——这是极弱的点级对应。虽然 nn_bbox 仍满足阈值，但在保守起步阶段应设下限排除（如 ≥ 10%），Phase A 中需专门分析这些低 close_pct 配对的 vertex_rmse。

**关键数据支撑**（来自 spike CSV 交叉分析）：


| 组合                            | 能救回的条数（/297） |
| ----------------------------- | ------------ |
| `nn_bbox ≤ 0.15` & `ur ≥ 0.5` | 115          |
| `nn_bbox ≤ 0.15` & `ur ≥ 0.7` | 54           |
| `nn_bbox ≤ 0.05` & `ur ≥ 0.5` | ~70          |
| `nn_bbox ≤ 0.01`（无额外约束）       | 41           |


**推荐保守起步**：先用 `nn_bbox ≤ 0.05` + `nn_mean_dist_norm ≤ 0.02` + `nn_unique_ratio ≥ 0.5` + `nn_close_pct ≥ 10%` ——预计救回约 70 条，且 mean_dist_norm 的 median 只有 0.014，天然形成了与真不一致配对的分隔带。

> **条件独立性说明**：在保守配置 `nn_bbox ≤ 0.05` 下，`nn_mean_dist_norm ≤ 0.02` 过滤了 0 条（完全冗余），`nn_unique_ratio ≥ 0.5` 仅过滤了 1 条（几乎冗余）。**在此配置下实际上只有 `nn_bbox` 一个条件在真正筛选。** `nn_mean_dist_norm` 和 `nn_unique_ratio` 是为未来放宽到 `nn_bbox ≤ 0.15` 预留的安全网——在 0.15 配置下，mean_dist > 0.02 会额外过滤 19 条，ur < 0.5 会额外过滤 21 条。

#### 策略 2：Shadow Label（先标记、后上线）

不改 `_topo_same_vtx_with_nn_fallback()` 的默认行为，而是在 cert 产出中额外打标签：

```
在 build_pair_certificate() 中：
如果 reject_reason == bbox_precheck_failed_topo_filesize：
  额外计算 nn_bbox / nn_unique_ratio / nn_mean_dist_norm
  写入 cert 的 metadata 字段：recoverable_candidate = true/false
  写入具体指标：tier2_nn_bbox, tier2_nn_unique_ratio, tier2_nn_mean_dist_norm
```

优势：零风险，可与现有 cert 审计流程完全兼容，先积累数据再做阈值决策。
劣势：需要跑两轮（先标记、再决定上线），时间成本更高。

**建议**：策略 1 和策略 2 可以结合——**Phase A 用策略 2 的方式做离线模拟**（不改 compute_vertex_transform.py，只用 CSV 模拟），**Phase B 上线时用策略 1 写入代码**。

### 审核指标（上线前后对比清单）


| 指标                                 | 来源                              | 安全阈值                                         |
| ---------------------------------- | ------------------------------- | -------------------------------------------- |
| eligible_count 变化                  | `pair_certificate_summary.json` | 应增加（≈ Tier2 救回数）                             |
| bbox_precheck_failed_topo_filesize | `pair_certificate_summary.json` | 应减少对应数量                                      |
| displaced > 0.01                   | pairwise compare audit          | **必须 = 0**                                   |
| vertex_rmse > 0.01                 | pairwise compare audit          | 不应新增                                         |
| ref_changed_hard_fails             | audit                           | 预期减少（更多 eligible → 更多 ref 被替换 → 更少 fallback） |


**判定规则**：displaced > 0.01 新增任何一条即为 **FAIL**，需回滚。eligible 增加但 displaced=0 即为 **PASS**。

## 三、Transitive / Shape_Invariant 的边界

### Transitive 不需要专门的 NN 路径

Transitive V 是 BFS 沿路径逐步累积的：`V_total = V_step1 * V_step2 * ... * V_stepN`。每一步 `V_step` 调用的就是 `_topo_same_vtx_with_nn_fallback()`（`compute_vertex_transform.py:912`）。

因此：

- **topo 步骤的 NN 改进自动传导到 transitive 路径**——无需发明 transitive 专用 NN。
- 只要每一步的 V 更准，累积误差自然更小。
- Transitive 的 256 条 `transitive_not_supported` 是因为没有中间路径（图不连通），不是 V 精度问题，不在本轮范围内。

### Shape_Invariant 同顶点数分支的 NN 门控（Phase C）

`compute_V_shape_invariant()` 的同顶点数分支（L671-689）走的是**索引对齐 Procrustes（在 bbox 归一化空间里）**。虽然 bbox 归一化消除了非均匀缩放，但如果 shape_invariant 配对也存在顶点 shuffle，同样会失败。

**计划**：

- 沿用 `_nn_procrustes_in_normalized_space()` 作为 helper（它已经在归一化空间里做 NN + Procrustes，与 shape_invariant 语义一致）。
- 在 `compute_V_shape_invariant()` 的同顶点数分支后面加一个类似的门控：先算 baseline bbox_delta，如果超阈值且 NN 结果满足 Tier1/Tier2 条件，则采纳 NN 的 V。
- **阈值需要单独标定**（shape_invariant 的误差分布与 topo 不同），所以放在 Phase C 单独做 spike。

当前 bottle 只有 14 条 `bbox_precheck_failed_shape_invariant`，影响面小，优先级低于 topo 的 297 条。

## 四、分阶段实施计划

### Phase A：离线模拟新门控（只用 CSV，不改代码）

**目标**：在 spike CSV 上模拟不同 Tier2 阈值组合，选出最佳配置；验证 V 在多对一 NN 下的安全性。

**输入数据**：

- `check_reports/test0_rebuilt_dedup/topo_precheck_recovery_spike.csv`（297 条，已有全部指标）
- 对推荐组合内的配对，需额外计算 vertex_rmse（需读取 USD 顶点数据）

**需要做的**：

1. **参数扫描**（纯 CSV 分析）：写分析脚本 `scripts/analyze_tier2_gate_thresholds.py`，对 CSV 做参数扫描：
  - `nn_bbox` 阈值候选：[0.01, 0.05, 0.10, 0.15, 0.50]
  - `nn_unique_ratio` 阈值候选：[0.3, 0.5, 0.7, 0.9]
  - `nn_mean_dist_norm` 阈值候选：[0.01, 0.02, 0.05]
  - `nn_close_pct` 下限候选：[0, 10, 15, 20]
  - 对每个组合输出：救回条数、救回对的 nn_bbox 分布（p50/p95/max）
2. **按 baseline_bbox 分桶分析**：将推荐组合内的配对按 baseline_bbox 分桶（<1, 1-5, 5-10, >10），检查每个桶内 nn_bbox 的分布。如果高 baseline_bbox 桶（>10）的配对表现系统性地差，考虑对 baseline_bbox 设上限。
3. **Per-canonical 分组分析**：按 canonical asset 分组，检查是否有某些 canonical 系统性地贡献了 Tier2 救回或系统性地有高 nn_bbox。如有异常 canonical，考虑 per-canonical 排除机制。
4. **Vertex RMSE 验证**（需读 USD，关键安全验证）：对推荐组合内的 ~77 条配对，用 NN 的 V 计算 `vertex_rmse = sqrt(mean(||pts_old - pts_canon @ V||^2))`。验证 rmse 分布是否与 nn_bbox 一致——如果存在 nn_bbox 很小但 rmse 很大的配对，说明 bbox 作为安全网不够，需要收紧阈值或增加 rmse 作为额外门控条件。
5. **低 nn_close_pct 配对专项分析**：对 close_pct < 20% 的 ~9 条配对，单独检查 vertex_rmse，决定是否需要设 close_pct 下限。

**成功标准**：

- 产出一份带数据支撑的阈值推荐表
- 推荐配置下所有配对的 vertex_rmse p95 ≤ 0.05（USD 世界坐标单位。对于 GRScenes-test0 这类 `metersPerUnit=0.01` 的场景，相当于约 0.5 mm 量级；在假设 1.0=1 m 的场景中则约为 5 cm，仅作为量级参考。）
- Per-canonical 分析无系统性异常

**Phase A 失败路径决策矩阵**：

- 若 vertex_rmse 与 nn_bbox 一致且 per-canonical 无异常 → 进入 Phase B
- 若 vertex_rmse 显著高于 nn_bbox（如 p95 > 0.15）但个别配对可解释 → 收紧阈值后进入 Phase B
- 若 vertex_rmse 系统性高于 nn_bbox → 回退到策略 2（shadow label），不上线 Tier2
- 若 per-canonical 分析发现少数 canonical 贡献了大部分异常 → 考虑 per-canonical 排除后进入 Phase B

**可回滚性**：纯离线分析，不改代码、不改数据。

**涉及脚本**：

- 新建 `scripts/analyze_tier2_gate_thresholds.py`（参数扫描 + 分桶 + per-canonical）
- 新建或扩展 `scripts/debug_topo_precheck_recovery_spike.py`（vertex_rmse 验证，需读 USD）

### Phase B：Bottle + 额外类目上线验证 + Pairwise 验证

**目标**：把 Phase A 选定的 Tier2 门控写入 `_topo_same_vtx_with_nn_fallback()`，在 bottle 及 1-2 个额外类目上跑完整 cert → apply → pairwise 流程。

> **为什么需要额外类目**：bottle 的 297 条配对来自 37 个 canonical，几何类型单一。其他类目的顶点拓扑、多对一 NN 模式可能有类目特异性。应选择 1-2 个几何复杂度和顶点数与 bottle 差异大的类目（如 chair/lamp）做交叉验证。

**输入数据**：

- `GRScenes-test0-rebuilt-normalize-prededup/GRScenes_assets/bottle/`（及额外类目）
- `GRScenes-test0-rebuilt-normalized_bak/`（pairwise left-root）
- Phase A 确定的阈值

**需要改动的函数/脚本**：

1. `scripts/compute_vertex_transform.py`：
  - `_topo_same_vtx_with_nn_fallback()` — Gate 1 失败后新增 Gate 1B 分支
  - 新增常量：`_NN_TIER2_BBOX_THRESHOLD`、`_NN_TIER2_UNIQUE_THRESHOLD`、`_NN_TIER2_MEAN_DIST_THRESHOLD`、`_NN_TIER2_CLOSE_PCT_FLOOR`
  - 将 `_nn_procrustes_in_normalized_space()` 拆分为 core + wrapper：
    - 新增内部 core 函数（示例命名）：`_nn_procrustes_core(...) -> (V, nn_close_pct, nn_unique_ratio, nn_mean_dist_norm)`
    - 保留现有 `_nn_procrustes_in_normalized_space(...)` 的 3 返回值接口，内部调用 core 丢弃 `nn_mean_dist_norm`，供 Tier1 shuffle 路径使用（`V, close, uniq, _ = _nn_procrustes_core(...); return V, close, uniq`）
    - 新增 `_nn_procrustes_with_stats(...) -> (V, nn_close_pct, nn_unique_ratio, nn_mean_dist_norm)`，供 Tier2 / 未来 shape_invariant NN 门控使用
      > **回归风险**：调用方 `_topo_same_vtx_with_nn_fallback:439` 当前解包 3 个值（`V_nn, nn_close_pct, nn_unique_ratio`），需同步更新为 4 个。这是 Tier1 路径也会走的代码，**必须回归测试**（现有 17 个单元测试 + 合成数据 shuffle 测试）。
2. `scripts/c1_build_bulk_mapping_from_dedup_report.py` 中的 `build_pair_certificate()` — cert 产出增加溯源字段：
  - `v_source`：值为 `"baseline"` / `"tier1_shuffle"` / `"tier2_nn"`
  - Tier2 相关指标：`tier2_nn_bbox`、`tier2_nn_unique_ratio`、`tier2_nn_mean_dist_norm`
  - 用于事后审计和问题定位
3. `scripts/c1_autorun_categories.py` — 无需改动，只是重跑探针
4. `scripts/placement_pairwise_compare.py` — 无需改动，只是验证

**执行步骤**：

1. 实现 Gate 1B + cert v_source 标签，保留 `DEDUP_DISABLE_NN_FALLBACK=1` kill-switch（已有，覆盖整个 NN 路径）
2. 新增 `DEDUP_DISABLE_NN_TIER2=1` kill-switch（只禁用 Tier2，保留 Tier1 shuffle fix）
3. 支持环境变量阈值覆盖：`DEDUP_NN_TIER2_BBOX=X` 可动态调整 Tier2 bbox 阈值（方便回退到更保守配置而不完全关闭）
4. 跑 bottle + 额外类目 dry_run cert（`c1_autorun_categories.py --step6-mode dry_run --include-regex "^(bottle|chair|lamp)$"`，额外类目待定）
5. 检查 cert summary：eligible 应增加约 Phase A 预测的条数
6. 跑 apply + pairwise compare
7. 判定：`displaced > 0.01 = 0` → PASS，否则 FAIL 并回滚

**Shape_Invariant spike（并行任务，只做数据分析）**：

- 在 Phase B 验证的同时，对 14 条 `bbox_precheck_failed_shape_invariant` 做 spike 分析（类似 topo spike，不改代码）
- 产出 CSV + 分析报告，为 Phase C 的 shape_invariant 代码改动提供数据支撑

**成功/失败判定**：

- **PASS**：所有验证类目 displaced > 0.01 = 0 且 vertex_rmse > 0.01 不新增
- **FAIL**：任何 displaced > 0.01 新增 → 设 `DEDUP_DISABLE_NN_TIER2=1` 回滚，或用 `DEDUP_NN_TIER2_BBOX=0.01` 收紧到只救 B-1 层

**可回滚性**：

- 代码层面：
  - `DEDUP_DISABLE_NN_TIER2=1`：完全禁用 Tier2（保留 Tier1）
  - `DEDUP_NN_TIER2_BBOX=0.01`：收紧到只救最安全的 B-1 层
  - `DEDUP_DISABLE_NN_FALLBACK=1`：禁用全部 NN（回到纯 Procrustes）
- 数据层面：
  - apply 会 in-place 修改 `GRScenes-test0-rebuilt-normalize-prededup/` 中的 layout USD 文件（重写资产引用路径和补偿 xform）
  - `GRScenes-test0-rebuilt-normalized_bak/` 保存了 apply 前的完整 layout 副本
  - 回滚操作：从 `_bak` 目录恢复被修改的 layout 文件（`cp _bak/scenes/*/start_result_*.usd → prededup/scenes/*/`）
  - 逐类目回滚：只恢复该类目涉及的 scene layout 文件，不影响其他已通过的类目

### Phase C：推广到全类目 + Shape_Invariant 代码改动

**前置条件（已满足）**：
- Phase A+B 完成，bottle 探针 GO（eligible 1294→1371, +77, displaced=0）
- Audit observe 策略修复已实施：observe + tier2 模式下 bbox 检查降级为 soft warning（`placement_pairwise_compare.py`），
  verdict.passed 不再被 tier2 bbox 差异阻断。详见 `docs/operations/tier2_audit_40cat_failure_investigation_20260407.md`

**目标**：推广到 83 类目全量；基于 Phase B 并行产出的 shape_invariant spike 数据，实现 shape_invariant 的 NN 门控。

**输入数据**：

- Phase B 验证通过的代码
- `v8_prededup/shape_invariant/` 报告（已就位）

**需要改动的函数/脚本**：

1. **全量 rollout**（无额外代码改动）：
  - `c1_autorun_categories.py` 不加 `--include-regex`，跑全部类目
  - 全量 pairwise compare 签核
2. **Shape_Invariant NN 门控代码改动**（基于 Phase B 并行 spike 数据）：
  - 如果 spike 数据支持，在 `compute_V_shape_invariant()` 的同顶点数分支加入 NN fallback（复用 `_nn_procrustes_with_stats()` 或 core 函数，可能需要不同阈值）
  - 新增独立 kill-switch `DEDUP_DISABLE_SHAPE_INVARIANT_NN_FALLBACK=1`

**成功/失败判定**：

- 全量：全部 83 类目 displaced > 0.01 = 0
- Shape_invariant spike：先出数据再定阈值

**可回滚性**：

- 全量 rollout 可逐类目回滚（ledger 支持 `--skip-done`）
- Shape_invariant 改动独立于 topo Tier2，用单独的 kill-switch（如 `DEDUP_DISABLE_SHAPE_INVARIANT_NN_FALLBACK=1`）

### 环境变量控制一览

- `DEDUP_DISABLE_NN_FALLBACK=1`：关闭 topo + shape 的全部 NN 路径（回到纯 Procrustes）。
- `DEDUP_DISABLE_NN_TIER2=1`：仅关闭 topo Tier2 第二档门控，保留 Tier1 shuffle 修复。
- `DEDUP_NN_TIER2_BBOX=<float>`：覆盖 Tier2 的 `nn_bbox` 阈值，解析为 `float(...)`，解析失败时回退到代码内默认值。
- `DEDUP_DISABLE_SHAPE_INVARIANT_NN_FALLBACK=1`：关闭未来 shape_invariant NN 路径（若实现）。

## 五、风险与缓解


| 风险                                                    | 缓解措施                                                       |
| ----------------------------------------------------- | ---------------------------------------------------------- |
| 多对一 NN 下 Procrustes V 不稳定（bbox 小但内部顶点位移大）             | Phase A 强制验证 vertex_rmse；如 rmse >> bbox 则不上线               |
| Tier2 门控放宽后引入位移回归                                     | pairwise compare 兜底，displaced>0.01 即 FAIL                  |
| nn_unique_ratio 低导致 V 不稳定                             | Tier2 要求 ur ≥ 阈值 + close_pct 下限；Phase A 模拟确认安全范围           |
| 阈值选择过于激进                                              | Phase A 先模拟，Phase B 在 bottle + 额外类目验证，逐步放开                 |
| 少数 canonical 贡献了大部分 Tier2 救回                          | Phase A per-canonical 分组分析，异常 canonical 可 per-canonical 排除 |
| 高 baseline_bbox（>10）的配对 V 不可靠                         | Phase A 按 baseline_bbox 分桶验证，必要时设上限                        |
| shape_invariant 阈值与 topo 差异大                          | Phase B 并行 spike + Phase C 独立实现，不混用 topo 阈值                |
| transitive 累积误差放大                                     | 每步 V 改进自动传导；transitive 不需要额外改动                             |
| `_nn_procrustes_in_normalized_space` 返回值变更引入 Tier1 回归 | 更新调用方解包 + 现有 17 个单测 + 合成 shuffle 测试回归                      |
| 事后溯源困难（不知道哪条 V 来自 Tier2）                              | cert 产出增加 v_source + tier2 指标字段                            |
| 全量 rollout 性能影响                                       | Phase A 估算全量类目 Tier2 NN 总配对数和最大顶点数的 wall_time 上界           |


## 六、相关文档

- `docs/changes/2026-04-03_topo_nn_shuffle_gated_fallback.md` — Tier1 shuffle fix 实现记录
- `docs/operations/topo_vertex_shuffle_correspondence_fix_plan.md` — 原 NN 对应修复计划
- `docs/operations/v8_shuffle_fix_probe_status_20260403.md` — v8 shuffle fix 探针结果
- `docs/operations/topo_precheck_recovery_investigation_20260404.md` — 297 条 precheck reject 调研
- `docs/operations/grscenes_test0_bbox_gated_multimode_verification_runbook.md` — v8 runbook
- `check_reports/test0_rebuilt_dedup/topo_precheck_recovery_spike.csv` — spike 原始数据

## 七、审核记录

### 2026-04-05 Plan Agent 审核（已整合）

审核发现 15 条建议，其中 2 条 P0、8 条 P1、5 条 P2。**P0 和 P1 已全部整合入本文档**：


| ID  | 优先级    | 摘要                                           | 处置                            |
| --- | ------ | -------------------------------------------- | ----------------------------- |
| 1.1 | **P0** | 多对一 NN 下 V 稳定性风险 + Phase A 增加 vertex_rmse 验证 | ✅ 新增"稳定性风险"章节 + Phase A 第 4 步 |
| 2.1 | **P0** | `_nn_procrustes_in_normalized_space` 返回值变更方案 | ✅ Phase B 中明确回归风险和测试要求        |
| 1.2 | P1     | 推荐配置下三条件独立筛选力说明                              | ✅ 新增"条件独立性说明" blockquote      |
| 2.2 | P1     | Phase A 增加按 baseline_bbox 分桶分析               | ✅ Phase A 第 2 步               |
| 2.3 | P1     | 三条件冗余关系说明                                    | ✅ 合并入 1.2                     |
| 3.1 | P1     | Phase B 增加额外类目验证                             | ✅ Phase B 标题和内容更新             |
| 5.1 | P1     | 数据侧回滚具体步骤                                    | ✅ Phase B 回滚方案细化              |
| 6.1 | P1     | Tier2 增加 nn_close_pct 下限                     | ✅ Gate 1B 新增前置条件              |
| 6.3 | P1     | cert 增加 v_source 溯源字段                        | ✅ Phase B 改动清单增加              |
| 6.4 | P1     | Per-canonical 分组分析                           | ✅ Phase A 第 3 步               |
| 3.2 | P2     | Phase A 失败路径决策矩阵                             | ✅ Phase A 新增决策矩阵              |
| 4.1 | P2     | shape_invariant spike 提前到 Phase B 并行         | ✅ Phase B 新增并行任务              |
| 5.2 | P2     | 阈值级别环境变量覆盖                                   | ✅ Phase B 第 3 步               |
| 6.2 | P2     | 全量 wall_time 估算                              | ✅ 风险表新增一行                     |
| 6.5 | P2     | pairwise displaced 已有覆盖                      | 已有（Tier1 验证中 displaced=0 已确认） |

