---
title: "BBox-Gated Multi-Mode Verification Runbook (v8)"
created_at: "2026-04-01"
updated_at: "2026-04-02"
maintainer: "Claude Code (reviewed)"
status: "v8-ready"
code_reference:
  - "scripts/report_asset_mesh_dedup.py"
  - "scripts/compute_vertex_transform.py"
  - "scripts/rewrite_layout_asset_refs_with_compensation.py"
  - "scripts/c1_autorun_categories.py"
  - "scripts/c1_build_bulk_mapping_from_dedup_report.py"
  - "scripts/placement_pairwise_compare.py"
  - "scripts/union_dedup_reports.py"
  - "scripts/merge_dedup_reports.py"
---

# 目标

验证 6-change multi-mode 实现（commit `0c51924`）的正确性，确保：
1. geom_only 报告不再被 tolerance_merge 污染
2. 三种模式（geom_only、topo_filesize、shape_invariant）均可通过 bbox-gated 管线
3. 场景中物体位置无回归（pairwise compare 通过）
4. 去重覆盖率相比 v5/v6 有提升

# 前置条件

| 项目 | 路径 | 说明 |
|------|------|------|
| 去重前快照（**活跃**） | `GRScenes-test0-rebuilt-normalize-prededup/` | 所有操作在此执行 |
| 旧已去重集（**废弃**） | `GRScenes-test0-rebuilt-normalized_v3deduped_20260322_DEPRECATED/` | 已被 v3 去重，勿用 |
| 备份 | `GRScenes-test0-rebuilt-normalized_bak/` | pairwise compare 的 left-root |
| v8 dedup 报告（**活跃**） | `check_reports/test0_rebuilt_dedup/v8_prededup/{geom_only,shape_invariant,topo_filesize}/` | 在 prededup 上重新生成 |
| v8 联合报告 | `check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/all_categories_union_merged.json` | Step 1 生成 |
| v5 基线 | `check_reports/test0_rebuilt_dedup/c1_bulk_v5_translation_fix/` | 覆盖率对比基线 |

> **重要**：
> - `GRScenes-test0-rebuilt-normalized/` 已重命名为 `..._v3deduped_20260322_DEPRECATED/`，不要使用。
> - 旧报告目录 `check_reports/test0_rebuilt_dedup/{geom_only,topo_filesize,shape_invariant}/` 基于已去重数据集生成，数据不完整，已被 v8 替代。
> - 所有命令中 `--mode-reports-dir` 必须指向 `check_reports/test0_rebuilt_dedup/v8_prededup`。

# Step 0: 在 prededup 上重新生成全部 114 类目的三种模式报告 (v8) [DONE]

## 为什么

旧报告是在 `GRScenes-test0-rebuilt-normalized/`（已被 v3 去重的数据集）上生成的：
- `geom_only/`: 所有类目 duplicate_group_count=0（重复资产已被 v3 物理删除）
- `topo_filesize/` 和 `shape_invariant/`: 不完整——漏掉了 v3 去重时删除的资产参与的配对

## 命令

```bash
# 30 DLC jobs = 3 modes x 10 chunks
bash scripts/dlc/submit_v8_prededup_geom_only_dedup.sh         # 10 jobs -> v8_prededup/geom_only/
bash scripts/dlc/submit_v8_prededup_shape_invariant_dedup.sh    # 10 jobs -> v8_prededup/shape_invariant/
bash scripts/dlc/submit_v8_prededup_topo_filesize_dedup.sh      # 10 jobs -> v8_prededup/topo_filesize/
```

## 执行记录

- **2026-04-02**: 30 DLC jobs 提交并全部成功（job names: `v8p_geom_dedup_{0-9}`, `v8p_shape_dedup_{0-9}`, `v8p_topo_dedup_{0-9}`）
- 输出: `check_reports/test0_rebuilt_dedup/v8_prededup/{geom_only,shape_invariant,topo_filesize}/`，各 114 类目
- 旧目录 `GRScenes-test0-rebuilt-normalized/` 已重命名为 `..._v3deduped_20260322_DEPRECATED/`

# Step 1: 重建 v8 union_3way 联合报告

## 为什么

`c1_autorun_categories.py --report` 需要一个合并了三种模式的联合报告 JSON。v8 的三种模式报告已分别就位，需要：
1. 对每个类目合并三种模式的重复组（union-find 传递闭包）
2. 把 114 个类目的合并结果拼接为一个全局报告

## 命令

```bash
# Step 1a: 按类目合并三种模式（union-find）
python scripts/union_dedup_reports.py \
  --batch \
  --geom-dir check_reports/test0_rebuilt_dedup/v8_prededup/geom_only \
  --shape-dir check_reports/test0_rebuilt_dedup/v8_prededup/shape_invariant \
  --topo-dir check_reports/test0_rebuilt_dedup/v8_prededup/topo_filesize \
  --output-dir check_reports/test0_rebuilt_dedup/v8_prededup/union_3way \
  --summary check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/summary.json

# Step 1b: 拼接所有类目为全局报告
python scripts/merge_dedup_reports.py \
  --reports-dir check_reports/test0_rebuilt_dedup/v8_prededup/union_3way \
  --output check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/all_categories_union_merged.json
```

## 验证标准

```bash
# 检查 114 类目的 union 报告都存在
ls check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/ | wc -l
# 期望: >= 114 个子目录 + summary.json + all_categories_union_merged.json

# 检查全局报告的类目数和重复组数
python3 -c "
import json
d = json.load(open('check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/all_categories_union_merged.json'))
print('categories:', d['meta'].get('categories', 'N/A'))
print('duplicate_groups:', len(d.get('duplicates', [])))
"
# 期望: categories=114, duplicate_groups > 7022（旧 union 基于不完整数据只有 7022 组）

# 检查 v8 geom_only 的 bottle 不再有 tolerance_merge 污染
python3 -c "
import json
d = json.load(open('check_reports/test0_rebuilt_dedup/v8_prededup/geom_only/bottle/bottle_asset_mesh_dedup_geom_only.json'))
print('tolerance_merged_groups:', d['meta']['tolerance_merged_groups'])
print('duplicate_group_count:', d['meta']['duplicate_group_count'])
"
# 期望: tolerance_merged_groups=0, duplicate_group_count > 0（v7 探针为 74）
```

# Step 2: 探针测试 — 对 bottle 和 other 跑 bbox-gated 管线（dry_run）

## 为什么

验证 Changes 2-6 协同工作：新模式能通过证书阶段、白名单阶段，且使用正确的补偿公式。

## 命令

```bash
python3 scripts/c1_autorun_categories.py \
  --dataset-root GRScenes-test0-rebuilt-normalize-prededup \
  --bak-root GRScenes-test0-rebuilt-normalized_bak \
  --report check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/all_categories_union_merged.json \
  --c1-bulk-dir check_reports/test0_rebuilt_dedup/c1_bulk_v8_multimode \
  --include-regex "^(bottle|other)$" \
  --group-label c1_v8_multimode_probe \
  --step6-mode dry_run \
  --bbox-gated \
  --bbox-policy bbox_primary_rmse_observe \
  --v-matrix-mode auto \
  --mode-reports-dir check_reports/test0_rebuilt_dedup/v8_prededup
```

## 验证标准

```bash
# 从 ledger 检查无 category_fail 事件
grep '"event": "category_fail"' \
  check_reports/test0_rebuilt_dedup/c1_bulk_v8_multimode/_autorun/*/ledger.jsonl
# 期望: 无输出

# 从证书统计检查 mode_not_enabled 拒绝数为 0
python3 -c "
import json
for cat in ['bottle', 'other']:
    d = json.load(open(f'check_reports/test0_rebuilt_dedup/c1_bulk_v8_multimode/{cat}_bulk_batch_v1/pair_certificate_summary.json'))
    print(f'{cat}: eligible={d[\"eligible_count\"]}, reject_reasons={d.get(\"reject_reason_counts\", {})}')
"
# 期望: mode_not_enabled_* 拒绝为 0

# 检查 eligible 对数 > v5 同类别
python3 -c "
import json
for cat in ['bottle', 'other']:
    d = json.load(open(f'check_reports/test0_rebuilt_dedup/c1_bulk_v8_multimode/{cat}_bulk_batch_v1/pair_certificate_summary.json'))
    print(f'{cat}: eligible_count={d[\"eligible_count\"]}')
"
```

# Step 3: Apply + Pairwise Compare 审计

## 为什么

确认去重替换后场景中每个物体的世界空间位置不变（bbox、质心、顶点 RMSE 均在阈值内）。

> **注意**：Step 2 使用 `dry_run` 模式不会实际修改文件。需要先以 `apply` 模式在 prededup 上
> 执行一次，然后对比 prededup 修改前后。
> `--bak-root` 提供的备份目录保留了修改前的 layout，作为 pairwise compare 的 left-root。

## 命令

```bash
# Step 3a: 以 apply 模式执行（Step 2 dry_run 确认无误后）
python3 scripts/c1_autorun_categories.py \
  --dataset-root GRScenes-test0-rebuilt-normalize-prededup \
  --bak-root GRScenes-test0-rebuilt-normalized_bak \
  --report check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/all_categories_union_merged.json \
  --c1-bulk-dir check_reports/test0_rebuilt_dedup/c1_bulk_v8_multimode \
  --include-regex "^(bottle|other)$" \
  --group-label c1_v8_multimode_probe \
  --step6-mode apply \
  --bbox-gated \
  --bbox-policy bbox_primary_rmse_observe \
  --v-matrix-mode auto \
  --mode-reports-dir check_reports/test0_rebuilt_dedup/v8_prededup

# Step 3b: pairwise compare — left=备份(修改前) right=prededup(修改后)
./scripts/isaac_python.sh scripts/placement_pairwise_compare.py \
  --left-root GRScenes-test0-rebuilt-normalized_bak \
  --right-root GRScenes-test0-rebuilt-normalize-prededup \
  --label "v8_multimode_probe_bottle_other" \
  --out check_reports/test0_rebuilt_dedup/v8_verification/pairwise_compare.json \
  --verdict-out check_reports/test0_rebuilt_dedup/v8_verification/verdict.json \
  --workers 8 \
  --bbox-policy bbox_primary_rmse_observe \
  --eps-bbox 0.01 \
  --eps-pos 0.01 \
  --eps-angle 1.0 \
  --eps-geom 0.01 \
  --allow-no-mesh
```

## 验证标准

```bash
# 审计判定必须通过
python3 -c "
import json
v = json.load(open('check_reports/test0_rebuilt_dedup/v8_verification/verdict.json'))
print('passed:', v['verdict']['passed'])
"
# 期望: True

# 无位移超阈值的 prim
python3 -c "
import json
d = json.load(open('check_reports/test0_rebuilt_dedup/v8_verification/pairwise_compare.json'))
print('displaced_breakdown:', d['aggregate'].get('displaced_breakdown', {}))
print('vertex_rmse_breakdown:', d['aggregate'].get('vertex_rmse_breakdown', {}))
"
# 期望: 全部为 0
```

# Step 4: 覆盖率对比（v5 -> v8）

## 为什么

量化多模式启用后的去重覆盖率提升。

## 命令

```bash
python3 -c "
import json

# v5 ground-only 基线
v5 = json.load(open('check_reports/test0_rebuilt_dedup/c1_bulk_v5_translation_fix/ground_geom_only_mapping.json'))
print(f'v5 ground-only mapping pairs: {len(v5)}')

# v8 探针
for cat in ['bottle', 'other']:
    path = f'check_reports/test0_rebuilt_dedup/c1_bulk_v8_multimode/{cat}_bulk_batch_v1/filtered_mapping.json'
    m = json.load(open(path))
    print(f'v8 {cat} mapping pairs: {len(m)}')

# 证书中各模式的分布
for cat in ['bottle', 'other']:
    path = f'check_reports/test0_rebuilt_dedup/c1_bulk_v8_multimode/{cat}_bulk_batch_v1/pair_certificate_summary.json'
    d = json.load(open(path))
    print(f'v8 {cat}: eligible={d[\"eligible_count\"]}, reject_reasons={d.get(\"reject_reason_counts\", {})}')
"
```

## 期望结果

- eligible 对数 > v5 同类别（因为新增了 topo_filesize 和 shape_invariant 模式的对）
- `mode_not_enabled_*` 拒绝原因应该消失（对三种模式而言）
- 仍可能存在 `aspect_ratio_rejected`、`mesh_probe_failed` 等正常拒绝

# Step 5: 全量 83 类目滚动发布

探针通过后，对全部类目执行：

```bash
python3 scripts/c1_autorun_categories.py \
  --dataset-root GRScenes-test0-rebuilt-normalize-prededup \
  --bak-root GRScenes-test0-rebuilt-normalized_bak \
  --report check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/all_categories_union_merged.json \
  --c1-bulk-dir check_reports/test0_rebuilt_dedup/c1_bulk_v8_multimode \
  --step6-mode apply \
  --bbox-gated \
  --bbox-policy bbox_primary_rmse_observe \
  --v-matrix-mode auto \
  --mode-reports-dir check_reports/test0_rebuilt_dedup/v8_prededup
```

随后全量 pairwise compare 作为最终签核：

```bash
./scripts/isaac_python.sh scripts/placement_pairwise_compare.py \
  --left-root GRScenes-test0-rebuilt-normalized_bak \
  --right-root GRScenes-test0-rebuilt-normalize-prededup \
  --label "v8_multimode_full_rollout" \
  --out check_reports/test0_rebuilt_dedup/v8_verification/full_pairwise_compare.json \
  --verdict-out check_reports/test0_rebuilt_dedup/v8_verification/full_verdict.json \
  --workers 8 \
  --bbox-policy bbox_primary_rmse_observe \
  --eps-bbox 0.01 \
  --eps-pos 0.01 \
  --eps-angle 1.0 \
  --eps-geom 0.01 \
  --allow-no-mesh
```

# 风险与回退

| 风险 | 应对 |
|------|------|
| topo_filesize/shape_invariant V 矩阵补偿不准 | bbox gate 会拦截；pairwise compare 兜底 |
| 报告重新生成与旧缓存冲突 | 每个 step 输出到独立版本目录（v8_prededup、c1_bulk_v8_multimode） |
| 全量发布出现个别失败 | `--skip-done`（默认开启）+ ledger 支持断点续跑 |
| 需要回退 | `_bak` 备份完整，可从备份恢复原始 layout |

# Agent Teams 执行指南

本 runbook 的各步骤可通过 Claude Code Agent Teams 协同执行。以下是推荐的团队组织方式。

## 团队结构

```
Team Lead（你自己）
  |
  +-- worker-union      # Step 1: 重建 union 报告
  +-- worker-pipeline   # Step 2-3: 管线探针 + apply + pairwise compare
  +-- worker-audit      # Step 4: 覆盖率统计
```

## 执行策略

### Step 1: 重建 union（单 agent）

```python
# Team Lead 操作
TeamCreate(team_name="v8-multimode-verify", description="v8 multi-mode dedup verification")

# 创建任务
TaskCreate(subject="Rebuild v8 union_3way", description="Run union_dedup_reports.py --batch + merge_dedup_reports.py, verify 114 categories")

# 启动 worker
Agent(
    name="worker-union",
    team_name="v8-multimode-verify",
    subagent_type="general-purpose",
    prompt="""
    Execute Step 1 of the v8 verification runbook.
    1) Run: python scripts/union_dedup_reports.py --batch --geom-dir ... --shape-dir ... --topo-dir ... --output-dir ... --summary ...
    2) Run: python scripts/merge_dedup_reports.py --reports-dir ... --output ...
    3) Verify 114 category union files + merged report
    4) Report results via SendMessage to team lead

    【Documentation Requirement】
    Document results in docs/ with YAML frontmatter.
    """,
    mode="bypassPermissions"
)
```

### Step 2-3: 管线探针 + Apply（顺序执行，单 agent）

Step 2 (dry_run) 和 Step 3 (apply + pairwise) 必须顺序执行，因为：
- Step 3 的 apply 依赖 Step 2 dry_run 确认无误
- pairwise compare 依赖 apply 完成

```python
Agent(
    name="worker-pipeline",
    team_name="v8-multimode-verify",
    subagent_type="general-purpose",
    prompt="""
    Execute Step 2 (dry_run probe) then Step 3 (apply + pairwise compare).
    Step 2: c1_autorun_categories.py --step6-mode dry_run --include-regex "^(bottle|other)$" ...
    Step 3: c1_autorun_categories.py --step6-mode apply ... then placement_pairwise_compare.py ...
    Report ledger/certificate/verdict results.
    """,
    mode="bypassPermissions"
)
```

### Step 4: 覆盖率统计（可与 Step 2-3 同时准备，但需等 Step 2 产出）

```python
# 在 Step 2 完成后启动
Agent(
    name="worker-audit",
    team_name="v8-multimode-verify",
    subagent_type="general-purpose",
    prompt="Execute Step 4: coverage comparison v5 vs v8. Report eligible counts and mode distribution.",
    mode="bypassPermissions"
)
```

### Step 5: 全量发布（Team Lead 确认后手动触发）

全量发布影响较大，建议 Team Lead 审查 Step 2-4 结果后手动执行或委派单独 agent。

## 并行度说明

| Step | 依赖 | 可并行 |
|------|------|--------|
| Step 1 (union rebuild) | Step 0 完成 | 独立执行 |
| Step 2 (dry_run probe) | Step 1 完成 | 等 Step 1 |
| Step 3 (apply + pairwise) | Step 2 无 fail | 等 Step 2 |
| Step 4 (coverage stats) | Step 2 产出 | 可与 Step 3 并行 |
| Step 5 (full rollout) | Step 3 verdict passed | 最后执行 |

## 文档要求

每个 agent 必须在完成时记录工作：
- 结果/进展写入 `docs/` 目录（带 YAML frontmatter）
- 任务日志写入 project memory
- 只读 agent（如 Explore）通过 SendMessage 发送发现给 team lead

# 相关文档

- `docs/operations/grscenes_test0_bbox_gated_multimode_plan.md` — 6-change 实现计划（status: implemented）
- `docs/operations/grscenes_test0_bbox_gated_v6_rmse_bbox_investigation.md` — v6 RMSE/bbox 调查
- `docs/operations/v_compensation_translation_scaling_bug.md` — V 补偿缩放 bug 记录
- `docs/changes/2026-04-01_*.md` — 本次代码变更文档
