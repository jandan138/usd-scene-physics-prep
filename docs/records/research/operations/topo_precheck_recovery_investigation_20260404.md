---
title: "Topo BBox Precheck Rejects — Recovery Investigation (bottle)"
code_reference:
  - scripts/debug_topo_precheck_recovery_spike.py
  - scripts/compute_vertex_transform.py
created_at: 2026-04-04
updated_at: 2026-04-04
maintainer: zhuzihou
status: draft
---

# Topo `bbox_precheck_failed_topo_filesize` — 能否合理救回？

## 背景

- **数据集**：`GRScenes-test0-rebuilt-normalize-prededup`，类目 **bottle**。
- **证书**：`c1_bulk_v8_shuffle_fix/.../pair_certificates.jsonl`。
- **对象**：`reject_reason == bbox_precheck_failed_topo_filesize` 的 **297** 条（`pair_certificate_summary.json` 与 jsonl 一致）。
- **假设**：topo_filesize 判重下「多数应为同一几何」，但证书 **粗预检 `bbox_delta.max_abs > 0.5`** 拒绝。

## 方法（spike）

- **分支**：`feat/topo_precheck_recovery_v1`。
- **脚本**：`scripts/debug_topo_precheck_recovery_spike.py`（只读证书 + 重算几何，**不改**证书主逻辑）。
- 对每条 reject：
  1. 读 `canonical` / `old` 的实例空间顶点（与 cert 相同提取方式）。
  2. `baseline_bbox` = `procrustes_full` + `_bbox_delta_max_abs`。
  3. `nn_bbox` / `nn_close_pct` / `nn_unique_ratio` = `_nn_procrustes_in_normalized_space`（与生产 NN fallback 同实现）。
  4. `nn_mean_dist_norm` = 双方 bbox 归一化后，每个 canon 点到 old 最近邻距离的 **均值**。

**产物**：`check_reports/test0_rebuilt_dedup/topo_precheck_recovery_spike.csv`

## 结果摘要（297 条全部成功加载，无顶点数不一致）

| 指标 | 数量 | 说明 |
|------|------|------|
| `nn_close_pct > 95` | **1** | 当前 shuffle 门控里「>95% 点在归一化空间 <0.01」几乎从不成立 |
| `nn_close_pct ≤ 95` | **296** | **绝大多数失败边被 Gate1（close_pct）挡在 NN 结果之外**，生产仍用 **很差 baseline** |
| `nn_bbox ≤ 0.01` | **41** | 若 **强制采用 NN 的 V**，世界空间 bbox 已极好 |
| `nn_bbox ≤ 0.15` | **136** | 采用 NN 的 V 后 bbox 较紧 |
| `nn_bbox ≤ 0.5`（可过当前证书粗预检） | **223** | 采用 NN 的 V 则 **不再触发 0.5 precheck** |
| `nn_close ≤ 95` 且 `nn_bbox ≤ 0.01` | **41** | **NN 几何已经极好，但因 close_pct 未过线未被采纳** |
| `nn_close ≤ 95` 且 `nn_bbox ≤ 0.5` | **222** | 不依赖 close_pct，只看 NN 后的世界 bbox，**可过 0.5 门** |
| `cur_code_should_accept`（close>95 且 nn_bbox≤0.01） | **0** | 与「仅 1 条 close>95」一致；**生产逻辑与「41 条本可极好」不矛盾**——是 **门控定义过严** |

### 与证书 `cert_max_abs` 分桶（采用 NN 后能否 ≤0.5）

| cert_max_abs 区间 | 条数 | 其中 `nn_bbox ≤ 0.5` |
|-------------------|------|----------------------|
| (0.5, 1] | 101 | 84 |
| (1, 2] | 57 | 43 |
| (2, 5] | 29 | 21 |
| (5, 10] | 84 | 66 |
| (10, +] | 26 | 9 |

**解读**：即使证书上 bbox 很差，**仍有一大半**在「采用 NN 的 V」后可压到 0.5 以下；**最差的 26 条（>10）**里只有 9 条能被 NN 救到 ≤0.5，**更像真不一致或相似变换不够**。

### 为何 `nn_close_pct` 低但 `nn_bbox` 可以很小？

抽样：`nn_close_pct ≈ 51%` 时 `nn_mean_dist_norm ≈ 0.012`，略高于计数阈值 **0.01**，导致 **close 比例暴跌**，但 **整体 NN + Procrustes 在世界空间已经把框对齐得很好**。

另有一些 `nn_unique_ratio ≈ 0.72`：**多对一 NN**，当前若放宽门控需 **单独处理**（不可照搬 shuffle 双射假设）。

## 结论（是否合理救回）

1. **不是「297 条都该救」**：约 **74 条**（297−223）在 spike 下 **NN 的 V 仍无法把 bbox 压到 0.5 以下**，更可能 **非同一刚性几何 + 相似变换** 或 **严重非刚性差异**，与「topo 判重偶发不准」一致时应 **保持拒绝**。
2. **强烈信号：门控与预检不匹配**  
   - 生产：**close_pct > 95** 才肯用 NN 的 V，且 **nn_bbox ≤ 0.01** 才采纳。  
   - 数据：**296/297** 不满足 close_pct，但其中 **222** 条若改用 NN 的 V 已能过 **0.5 粗预检**，**41** 条甚至已达 **≤0.01**。  
   - **主要矛盾**是 **「shuffle 判别」过严（0.01 逐点比例）**，不是 Procrustes 完全算不出。
3. **与「整个模型都不一致就算了」对齐**：优先只对 **NN 后 `nn_bbox ≤ 0.5` 且 `nn_unique_ratio` 高（如 ≥0.95）** 的边考虑放宽；**>10 且 NN 仍救不回** 的优先 **放弃**。

## 建议的下一步（设计级，本分支尚未改证书）

**方向 A（推荐先做 spike 级模拟）**：在 **不重写主逻辑** 的前提下，对 mapping 流水线做一次 **离线 shadow**：若 `baseline precheck 失败` 且 `nn_bbox ≤ 0.5` 且 `nn_unique_ratio ≥ 0.95`，打标签 `recoverable_candidate`，统计与 placement audit 的相关性。

**方向 B（代码级，需单独 PR）**：为 topo 增加 **第二档门控**（示例，非最终值）：
- 当 `baseline_bbox > 0.15` 且 **原 Gate1 失败** 时：若 `nn_unique_ratio ≥ 0.95` 且 `nn_mean_dist_norm < T`（如 0.02）且 `nn_bbox ≤ 0.15`（或 ≤0.5），则采纳 NN 的 V；  
- 或 **分层**：先救 `nn_bbox ≤ 0.01` 且 unique 高的一批，再评估是否放宽到 0.15。

**方向 C**：**不要用单一 close_pct**；改为 **mean / p99 NN 距离** + **unique_ratio** + **nn_bbox** 联合判定。

## 复现实验

```bash
python3 scripts/debug_topo_precheck_recovery_spike.py \
  --cert-jsonl check_reports/test0_rebuilt_dedup/c1_bulk_v8_shuffle_fix/bottle_bbox_primary_rmse_observe_v1/01_cert/pair_certificates.jsonl \
  --dataset-root GRScenes-test0-rebuilt-normalize-prededup \
  --out-csv check_reports/test0_rebuilt_dedup/topo_precheck_recovery_spike.csv
```

---

*本调研在 Cursor 于分支 `feat/topo_precheck_recovery_v1` 上完成；未修改 `compute_vertex_transform.py` 的默认证书行为。*
