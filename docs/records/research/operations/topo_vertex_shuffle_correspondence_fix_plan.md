---
title: "Topo Vertex Shuffle — NN Correspondence Recovery Plan"
code_reference: scripts/compute_vertex_transform.py
created_at: 2026-04-03
updated_at: 2026-04-05
maintainer: zhuzihou
status: implemented
---

# Topo Vertex Shuffle — NN Correspondence Recovery Plan

## Phase 0: 目标冻结与成功标准

### 目标

在 **不破坏现有 geom_only 及已通过 cert 的路径** 的前提下，评估「先 NN
correspondence recovery 再相似变换」能否降低 topo_filesize + bbox-gated
管线下的误杀（false reject）。

### 根因（2026-04-03 确认）

`procrustes_full()` 假设 vertex[i] ↔ vertex[i]（索引对齐），但
topo_filesize 去重配对的资产之间存在 **mesh 内顶点重排序 + 非均匀缩放**：

| 现象 | 证据 |
|------|------|
| 去重匹配本身正确 | topo-invariant sig（mesh 数 + 每 mesh 顶点数/面数/材质 multiset）+ GLB 文件大小 ≤2% |
| 几何完全一致 | bbox 归一化后 NN close_pct = 100%，extent_ratio ≈ [1.02, 0.95, 1.02] |
| Procrustes 失败 | 索引对齐 RMSE = 86.07，bbox_delta = 61.5 |
| NN 修正后 Procrustes 成功 | NN-reorder RMSE = 0.0005，bbox_delta ≈ 0 |

bottle 品类抽样（15 对/桶）：

| 区间 | 对数 | 纯 shuffle | shuffle + scale | 总同几何 |
|------|------|-----------|-----------------|---------|
| good (≤0.15) | 510 | 100% | — | 100% |
| gap (0.15–0.5) | 322 | ~13% | ~87% | 100% |
| rejected (>0.5) | 595 | ~27% | ~66% | ~93% |

### 成功标准

与 Phase 2 抽样方案及判定规则一致（单一来源，避免冲突）：

1. **覆盖率提升**：在 100 对 poor（bbox>0.5）+ 100 对 gap（0.15<bbox≤0.5）样本上，
   新流程 bbox_delta.max_abs 降到可接受范围的比例 ≥ 80%。
   可接受阈值从 spike 数据的双峰分布间隙中选取，不预设。
2. **零回归**：随机抽 50 对 geom_only + 100 对已知 good topo（bbox≤0.01），
   新流程 RMSE 和 bbox_delta 不允许比 baseline 差超过 1e-6。
   注：若 baseline 已接近机器精度（如 ~1e-10）或量纲很大导致 1e-6 过严，
   由 Analyst 在报告中说明并改用相对阈值（如 relative delta ≤ 1e-4）。
3. **NN 质量**：nn_unique_ratio（unique NN indices / total）应 ≥ 0.95，
   否则标记为退化样本，单独分析。
4. **性能**：单对处理耗时 < 2s（含 KDTree 构建 + NN 查询 + Procrustes），
   避免 O(n²) 匈牙利在大品类上 OOM。

---

## Phase 1: 只读调用链分析（不改 compute_vertex_transform.py）

### 调用链

```
c1_autorun_categories.py
  → c1_build_bulk_mapping_from_dedup_report.py    # 生成 mapping
    → build_pair_certificate()                      # 每对发证书
      → compute_V_for_pair(old, canon, mode)
        ├─ geom_only       → return Identity
        ├─ topo_filesize
        │   ├─ len(canon)==len(old) → procrustes_full()        ← 索引对齐 SVD
        │   └─ len(canon)!=len(old) → compute_V_shape_invariant()
        ├─ shape_invariant → compute_V_shape_invariant()
        │   ├─ same vtx count  → bbox normalize + Procrustes (索引对齐)
        │   └─ diff vtx count  → bbox normalize + ICP (NN 迭代)
        └─ transitive      → find_transitive_V()
```

### 当前分支对照表

| 条件 | topo_filesize 走哪条 | shape_invariant 走哪条 | 已有 NN? |
|------|---------------------|----------------------|---------|
| 顶点数相同 + 无 shuffle | `procrustes_full` 索引对齐 | bbox 归一 + 索引对齐 Procrustes | 无 |
| 顶点数相同 + 有 shuffle | `procrustes_full` 索引对齐 **← 失败** | bbox 归一 + 索引对齐 Procrustes **← 也会失败** | 无 |
| 顶点数不同 | `compute_V_shape_invariant` → ICP | `compute_V_shape_invariant` → ICP | ICP 内有 NN |

### 关键观察

- `compute_V_shape_invariant` 里的 ICP 路径（`icp_in_normalized_space`）已经
  用 KDTree NN 做迭代匹配 — 但 **仅在顶点数不同时才走 ICP**。
- topo_filesize 的同顶点数路径直接走 `procrustes_full`，无 NN correspondence，
  这是失败的根源。
- shape_invariant 的同顶点数路径同样走索引对齐 Procrustes，也会受 shuffle 影响，
  但 shape_invariant 先做了 **bbox 归一化**（`_normalize_to_unit_bbox`），
  能把非均匀缩放消除，所以 shuffle + scale 的情况在 shape_invariant 下
  Procrustes 对齐更好。

### 不重复造轮子

- 归一化函数 `_normalize_to_unit_bbox()` 已存在（L346-357）
- ICP 函数 `icp_in_normalized_space()` 已存在（L360-439）
- 修复思路：对 topo_filesize 同顶点数的情况，先归一化 + 单轮 NN reorder，
  再走 Procrustes。不需要写新算法，只需要组合现有函数。

---

## Phase 2: 独立 spike（新脚本，禁止改核心库）

### 脚本

`scripts/debug_topo_nn_correspondence_spike.py`

### 输入

从现有 cert JSONL 读取 (old_usd, canon_usd) 路径：
```
check_reports/test0_rebuilt_dedup/c1_bulk_v8_multimode/
  bottle_bbox_primary_rmse_observe_v1/01_cert/pair_certificates.jsonl
```

### 实验设计

对每个样本对比两条路径：

| 路径 | 方法 | 说明 |
|------|------|------|
| **Baseline** | 当前 `procrustes_full(pts_canon, pts_old)` | 索引对齐 SVD |
| **Candidate A** | bbox 归一化 → KDTree NN reorder → Procrustes → 反归一化得 V | 与 `compute_V_shape_invariant` 归一化方式对齐 |

Candidate A 伪代码：
```python
# 1. bbox 归一化（复用 _normalize_to_unit_bbox）
c_norm, c_min, c_ext = _normalize_to_unit_bbox(pts_canon)
o_norm, o_min, o_ext = _normalize_to_unit_bbox(pts_old)

# 2. NN correspondence recovery
tree = cKDTree(o_norm)
dists, indices = tree.query(c_norm)
o_norm_reordered = o_norm[indices]

# 3. 在归一化空间做 Procrustes（现在索引对齐了）
# ... 复用 _try_single_procrustes 逻辑 ...

# 4. 反归一化构建 V（复用 compute_V_shape_invariant 的公式）
```

### 抽样方案

| 桶 | 抽样数 | 来源 |
|----|--------|------|
| topo poor (bbox > 0.5) | 100 | cert rejected pairs |
| topo gap (0.15 < bbox ≤ 0.5) | 100 | cert eligible pairs |
| topo good (bbox ≤ 0.01) | 100 | cert eligible pairs |
| geom_only (回归守卫) | 50 | cert eligible pairs |

### 输出

CSV：`check_reports/test0_rebuilt_dedup/topo_nn_spike_results.csv`

列：`old_usd, canon_usd, mode, bucket, n_verts,
baseline_rmse, baseline_bbox_max_abs,
candidate_rmse, candidate_bbox_max_abs,
nn_mean_dist, nn_close_pct, nn_unique_ratio, is_shuffle_heuristic, wall_time_s`

### 判定规则

- Candidate A 有效 = poor/gap 桶中 ≥80% 的对 candidate_bbox_max_abs 降到可接受范围
  （具体阈值从 spike 数据的双峰分布中选取，不预设）
- 零回归 = good/geom_only 桶中 0 对的 candidate 指标比 baseline 差超过 1e-6
  （若 baseline 量级导致 1e-6 过严，Analyst 可改用相对阈值并在报告中说明）
- 性能 = 单对 wall time < 2s（在 CSV 中记录）
- NN 质量 = nn_unique_ratio（unique NN indices / total）应 ≥ 0.95，否则标记退化

---

## Phase 3: 合入策略设计（spike 验证后执行）

**Status**: IMPLEMENTED (2026-04-03). Spike verdict: CONDITIONAL GO — shuffle-gated only.

### 实际实现（与原 plan 的差异）

原 plan 设想的是「baseline bbox_delta > 阈值就走 NN fallback」。Spike 数据表明
**无条件 NN 会误伤 good/geom_only 桶**（31/100 回归，最差 1000x），因此实际实现
采用 **三重门控**：

```python
# compute_vertex_transform.py :: _topo_same_vtx_with_nn_fallback()
def _topo_same_vtx_with_nn_fallback(pts_canon, pts_old):
    V_baseline = procrustes_full(pts_canon, pts_old)

    # Kill-switch: DEDUP_DISABLE_NN_FALLBACK=1
    if os.environ.get("DEDUP_DISABLE_NN_FALLBACK") == "1":
        return V_baseline

    # Gate 0: baseline must be bad enough to try
    if _bbox_delta_max_abs(V_baseline, ...) <= 0.15:
        return V_baseline

    # Gate 1: NN must confirm genuine shuffle (nn_close_pct > 95%)
    V_nn, nn_close_pct, _ = _nn_procrustes_in_normalized_space(...)
    if nn_close_pct <= 95.0:
        return V_baseline

    # Gate 2: candidate must actually be good (bbox_delta <= 0.01)
    if _bbox_delta_max_abs(V_nn, ...) > 0.01:
        return V_baseline

    return V_nn
```

### 阈值来源（spike 数据支撑）

| 常量 | 值 | 来源 |
|------|-----|------|
| `_NN_FALLBACK_BASELINE_THRESHOLD` | 0.15 | spike: shuffle 对 baseline bbox 均 > 0.15 |
| `_NN_FALLBACK_CLOSE_PCT_THRESHOLD` | 95.0% | spike: 所有 shuffle 对 nn_close_pct = 100% |
| `_NN_FALLBACK_ACCEPT_THRESHOLD` | 0.01 | spike: 所有 shuffle 对 candidate < 0.004；非 shuffle 起始 ~0.02；5x 安全余量 |

详细分析见 `check_reports/test0_rebuilt_dedup/topo_nn_spike_analysis.md`。

### 共享 helper

`_topo_same_vtx_with_nn_fallback()` 被两处调用：
- `compute_V_for_pair()` — 直接 V 计算
- `_accumulate_V_along_path()` — transitive BFS 逐步 V

两处原来的 `procrustes_full(pts_canon, pts_old)` 均替换为
`_topo_same_vtx_with_nn_fallback(pts_canon, pts_old)`。

### 运行时 Kill-switch

环境变量 `DEDUP_DISABLE_NN_FALLBACK=1` 可在不改代码的情况下禁用 NN
fallback，回退到纯 `procrustes_full`。用于生产紧急回滚。

### 本变更范围

- **修复**：72/200 poor+gap 对（全部是 shuffle 根因），全部降到 bbox < 0.004
- **不涉及**：128/200 poor+gap 非 shuffle 对（可能是别的误差源：拓扑差异、
  阈值语义、真几何差等），需要下一轮用同样 spike 方法拆开看
- **零回归**：good/geom_only 桶无任何回归（shuffle 门控从不在这些桶触发）

### 算法选择（不变）

- **KDTree NN + 单次 Procrustes**（不用 ICP 多轮迭代）：
  topo 对顶点数相同且几何一致，单轮 NN 已经能恢复 100% 对应（实验证据）。
  ICP 多轮只在顶点数不同时有价值。
- **不用匈牙利**：O(n³) 时间 + O(n²) 内存，n=2815 时约 22G 矩阵，不可行。
  KDTree NN 是 O(n log n) 构建 + O(n log n) 查询，单对 < 100ms。
- **归一化空间**：先 `_normalize_to_unit_bbox` 消除非均匀缩放，再 NN，
  与 `compute_V_shape_invariant` 语义一致，避免两套归一化。

### 非均匀缩放

spike 数据（extent_ratio ≈ [1.02, 0.95, 1.02]）显示 topo 对的缩放差异 < 5%，
bbox 归一化后 NN close_pct = 100%。128 对非 shuffle 的 poor/gap 需要下一轮
单独调查，**不与 shuffle 修复混在同一个 PR**。

---

## Agent Teams 执行方案

### 总体设计

一个 Team Lead + 三个并行 Agent，team_name = `topo-nn-spike`。

Phase 1（只读分析）已在本对话中完成。下面的 agent 分配覆盖 Phase 2 spike
执行和 Phase 3 设计文档输出。

### Team Lead（当前主会话）

**职责**

- 创建 team 和 task list（`TeamCreate team_name="topo-nn-spike"`）
- 拆分 task 并分配给各 agent
- 监控进度，整合结论
- 在所有 agent 完成后，汇总 spike CSV 数据，做出阈值选择和 go/no-go 决策
- 最终撰写 Phase 3 合入设计 doc 或将其委派给 docs-writer

**启动命令**

```python
TeamCreate(team_name="topo-nn-spike", description="Topo vertex shuffle NN correspondence spike")
```

然后创建 task list 并按下文分配。

### Agent A: spike-implementer（feature-implementer）

**subagent_type**: `feature-implementer`（需要写文件，用 worktree 隔离）

**Task**: 实现 `scripts/debug_topo_nn_correspondence_spike.py`

**输入**

- 本 plan 的 Phase 2 伪代码和 CSV 列定义
- Cert JSONL 路径
- `compute_vertex_transform.py` 中 `procrustes_full`、`_normalize_to_unit_bbox`、
  `_try_single_procrustes` 的实现（只读引用，不可修改）

**交付物**

- `scripts/debug_topo_nn_correspondence_spike.py`，可独立运行
- 脚本功能：读取 cert JSONL → 按桶抽样 → 对每对跑 baseline + candidate A →
  输出 CSV 到 `check_reports/test0_rebuilt_dedup/topo_nn_spike_results.csv`

**约束**

- 允许只读 import：`from scripts.compute_vertex_transform import procrustes_full,
  _normalize_to_unit_bbox, _try_single_procrustes`（只读引用，禁止修改源文件）
- 若 import 路径不通（如 pxr 依赖），则 copy 一份到 spike 脚本内部，
  并加验收：在 3 个固定 fixture 对上与 import 路径数值对齐到 rtol=1e-8
- 脚本必须包含 `--cert-jsonl`、`--dataset-root`、`--out-csv` 参数
- 每对记录 `wall_time_s`（`time.perf_counter` 计时）

**停止条件**

- 脚本能在 bottle 品类上跑通，输出 CSV 列完整

**Prompt 关键段**

```
你的任务是实现 Phase 2 spike 脚本。阅读以下 plan：
docs/operations/topo_vertex_shuffle_correspondence_fix_plan.md

重点看 Phase 2 部分的伪代码、CSV 列定义、抽样方案。
脚本放在 scripts/debug_topo_nn_correspondence_spike.py。
优先 import compute_vertex_transform.py 中的函数（只读，禁止修改源文件）；
若因 pxr 依赖 import 不通，则 copy 到 spike 脚本内部并加验收对齐。
```

### Agent B: spike-runner（general-purpose）

**subagent_type**: `general-purpose`（需要 Bash 执行脚本）

**Task**: 执行 spike 脚本，收集 CSV 数据

**依赖**: Agent A 完成后才能开始（task blockedBy）

**输入**

- Agent A 交付的 spike 脚本
- 数据集路径 `GRScenes-test0-rebuilt-normalize-prededup/`

**执行命令**

```bash
python3 scripts/debug_topo_nn_correspondence_spike.py \
  --cert-jsonl check_reports/test0_rebuilt_dedup/c1_bulk_v8_multimode/\
bottle_bbox_primary_rmse_observe_v1/01_cert/pair_certificates.jsonl \
  --dataset-root GRScenes-test0-rebuilt-normalize-prededup \
  --out-csv check_reports/test0_rebuilt_dedup/topo_nn_spike_results.csv
```

**交付物**

- 完整 CSV 文件
- 运行日志（含报错处理）
- 简要统计：每桶样本数、baseline vs candidate 的 bbox_delta 分布摘要

**停止条件**

- CSV 行数 ≥ 350（100 poor + 100 gap + 100 good + 50 geom_only）
- 无 crash

**Prompt 关键段**

```
Agent A 已完成 spike 脚本。你的任务是执行它并收集数据。
执行命令见 plan。如果脚本报错，诊断并修复（仅限 spike 脚本，
不可改 compute_vertex_transform.py）。
把 CSV 和运行摘要报回。
```

### Agent C: spike-analyst（general-purpose）

**subagent_type**: `general-purpose`（需要 Bash 跑 Python 分析 + 读写文件）

**Task**: 分析 spike CSV，输出判定报告

**依赖**: Agent B 完成后才能开始（task blockedBy）

**输入**

- Agent B 交付的 CSV 文件

**分析内容**

1. 按桶统计 baseline_bbox_max_abs 和 candidate_bbox_max_abs 的分布
   （p5/p25/p50/p75/p95/max）
2. 用分位数表（p5/p25/p50/p75/p95/max）和文本直方图描述 candidate_bbox_max_abs
   的分布，从分布间隙中推荐 fallback 触发阈值 `TBD_FROM_SPIKE`。
   如环境支持 matplotlib 可输出 PNG，但不强求出图。
3. 验证成功标准：
   - 覆盖率：poor/gap 桶中 candidate 降到推荐阈值以下的比例
   - 零回归：good/geom_only 桶中是否有退化
   - NN 质量：nn_unique_ratio 分布
   - 性能：wall_time_s 分布
4. 输出判定：go / no-go / 需要进一步调查

**交付物**

- 分析报告写入
  `check_reports/test0_rebuilt_dedup/topo_nn_spike_analysis.md`
- 推荐的 fallback 触发阈值（带数据支撑）
- go/no-go 判定

**停止条件**

- 报告覆盖全部 4 条成功标准，每条有数字支撑

**Prompt 关键段**

```
Agent B 已跑完 spike，CSV 在：
check_reports/test0_rebuilt_dedup/topo_nn_spike_results.csv

你的任务是分析这份 CSV 并输出判定报告。
参照 plan Phase 0 的 4 条成功标准逐条验证。
报告写到 check_reports/test0_rebuilt_dedup/topo_nn_spike_analysis.md。
```

### Task List 与依赖关系

```
Task 1: [spike-implementer] 实现 spike 脚本          → owner: Agent A
Task 2: [spike-runner]      执行 spike 收集 CSV       → owner: Agent B, blockedBy: [1]
Task 3: [spike-analyst]     分析 CSV 输出判定报告      → owner: Agent C, blockedBy: [2]
Task 4: [team-lead]         汇总结论 + Phase 3 决策    → owner: Lead,   blockedBy: [3]
```

### 启动顺序

1. Team Lead 创建 team + 4 个 task
2. 立即 spawn Agent A（`feature-implementer`, `isolation: worktree`）
3. Agent A 完成 → Task 1 marked completed → spawn Agent B
4. Agent B 完成 → Task 2 marked completed → spawn Agent C
5. Agent C 完成 → Task 3 marked completed → Lead 执行 Task 4

Agent B 和 C 是串行依赖，不能并行。但如果未来扩展到多品类（不只 bottle），
可以对每个品类各 spawn 一个 Agent B 并行跑。

### 文档要求（注入每个 Agent prompt）

```
【Documentation Requirement】
You MUST document your work before finishing. This is mandatory.
- What to document: research findings, code changes, test commands & results,
  decisions, errors & resolutions.
- Where to write:
  • Results/progress → docs/ (with YAML frontmatter)
  • Task logs → project memory
- Timing: document as you go, not just at the end.
```

### 预估时间线

| 步骤 | 预计耗时 | 说明 |
|------|---------|------|
| Agent A: spike 实现 | 10-15 min | 组合已有函数，逻辑简单 |
| Agent B: spike 执行 | 5-15 min | 350 对，每对 < 2s；以实测 wall_time_s 为准 |
| Agent C: 分析报告 | 5-10 min | 纯数值分析 |
| Lead: 汇总决策 | 5 min | 读报告做判定 |

---

## 硬性约束

1. **Phase 2 完成前禁止修改 `compute_vertex_transform.py` 的默认行为**；
   只允许新增 `scripts/debug_*.py` spike 脚本或 `tests/` 里的实验函数。
2. 任何合入必须 **feature flag / 明确 fallback 分支**，默认行为与现网一致。
3. 必须先有 **Phase 2 数字报告**（样本数、前后对比 CSV），再开 PR 改库。

---

## 附录：关键路径

| 项 | 路径 |
|----|------|
| 核心库（已合入 NN fallback） | `scripts/compute_vertex_transform.py` |
| Cert JSONL | `check_reports/test0_rebuilt_dedup/c1_bulk_v8_multimode/bottle_bbox_primary_rmse_observe_v1/01_cert/pair_certificates.jsonl` |
| Spike 脚本 | `scripts/debug_topo_nn_correspondence_spike.py` （已实现，见 `docs/changes/2026-04-03_topo_nn_shuffle_gated_fallback.md`） |
| Spike CSV | `check_reports/test0_rebuilt_dedup/topo_nn_spike_results.csv` （已生成，350 对实验数据） |
| Spike 分析 | `check_reports/test0_rebuilt_dedup/topo_nn_spike_analysis.md` （已完成，verdict: CONDITIONAL GO） |
| 实现记录 | `docs/changes/2026-04-03_topo_nn_shuffle_gated_fallback.md` |
| 数据集 | `GRScenes-test0-rebuilt-normalize-prededup/` |
