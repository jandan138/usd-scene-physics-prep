---
title: "Phase 2: Normalized 全量去重扫描与 C1 执行"
code_reference: scripts/c1_autorun_categories.py
created_at: 2026-03-09
updated_at: 2026-03-09
maintainer: claude
status: active
---

# Phase 2: Normalized 全量去重扫描与 C1 执行

## 背景

### Phase 1 结论（未 normalize 数据）

对 GRScenes-test1（未 normalize）做全量去重扫描，整体仅 **0.7%** 重复率（208 个可节省）。
原因：未 normalize 的资产顶点带着世界坐标偏移，差异远超 0.005 容差。

### Phase 2 目标

在 `GRScenes-test1-normalized` 上重新扫描，预期重复率大幅提高。

---

## Step 1: DLC 扫描提交

10 个 DLC worker，每个处理 ~8 个类别（79 类 / 10 chunks）：

```bash
CHUNK_TOTAL=10
for i in $(seq 0 $((CHUNK_TOTAL - 1))); do
  bash scripts/dlc/launch_job.sh grscenes-dedup-norm $i $CHUNK_TOTAL "" \
    "dedup --chunk-id $i --chunk-total $CHUNK_TOTAL \
     --assets-root GRScenes-test1-normalized/GRScenes_assets \
     --out-dir check_reports/normalized_dedup_tolerance \
     --merge-tolerance 0.005 --float-quantize-eps 1e-2"
done
```

**Job IDs**: `dlc1mbruh5cmqnak` ~ `dlc1pxmkg95eyrdx`
**Status**: 全部 Succeeded (2026-03-09)

---

## Step 2: 汇总结果

```bash
python scripts/summarize_dedup_reports.py \
    --reports-dir check_reports/normalized_dedup_tolerance
```

### 结果对比

| 指标 | Phase 1 (未normalize) | Phase 2 (normalized) |
|------|----------------------|---------------------|
| 整体重复率 | 0.7% (208) | **20.1% (8,091)** |
| bottle | ~4.5% | **55.1% (761)** |

### Top 10 收益类别

| 类别 | 总数 | 可节省 | 重复率 |
|------|------|--------|--------|
| other | 12,209 | 2,795 | 29.0% |
| wall | 15,961 | 2,729 | 22.1% |
| ground | 10,107 | 1,138 | 16.6% |
| bottle | 1,698 | 761 | 55.1% |
| plate | 426 | 203 | 54.0% |
| book | 1,595 | 168 | 15.6% |
| column | 401 | 76 | 29.2% |
| cabinet | 1,277 | 49 | 6.3% |
| ceiling | 1,610 | 48 | 4.5% |
| cup | 549 | 37 | 10.2% |

26 个类别有重复，53 个类别无重复。

---

## Step 3: 代码修改（支持 normalized 数据集）

### 问题

C1 脚本链中有 6 处硬编码 `GRScenes-test1`，无法处理 `GRScenes-test1-normalized`。

### 修改

#### 3a. `c1_build_bulk_mapping_from_dedup_report.py`

- `_norm_rel_asset_path(p)` → `_norm_rel_asset_path(p, dataset_name)`
- `_abs_to_report_style(abs_path)` → `_abs_to_report_style(abs_path, dataset_name)`
- `_count_layout_asset_usage(dataset_root)` → `_count_layout_asset_usage(dataset_root, dataset_name)`
- `main()` 中 `dataset_name = Path(args.dataset_root).name`
- category filter 改为 `f"{dataset_name}/GRScenes_assets/{want}/"`

#### 3b. `c1_bulk_step6_category_promote_scan_soft_delete.py`

- `_abs_to_report_style(abs_path)` → `_abs_to_report_style(abs_path, dataset_name)`
- `_scan_stage_for_old_assets(stage_path, old_set)` → `_scan_stage_for_old_assets(stage_path, old_set, dataset_name)`
- `_scan_tree_pxr(root, ...)` 增加 `dataset_name` 参数
- `main()` 中 `dataset_name = dataset_root.name`
- 所有 4 处 `_scan_stage_for_old_assets` 调用和 1 处 `_scan_tree_pxr` 调用均传入 `dataset_name`

#### 3c. 新建 `scripts/merge_dedup_reports.py`

合并 per-category dedup 报告为单一 JSON（`c1_autorun_categories.py` 需要）：

```bash
python scripts/merge_dedup_reports.py \
    --reports-dir check_reports/normalized_dedup_tolerance \
    --output check_reports/normalized_dedup_tolerance/merged_geom_only.json
```

结果：79 categories → 2,547 dup groups, 52,904 assets

---

## Step 4: C1 自动执行

### 命令

```bash
C1_BULK_DIR=check_reports/normalized_dedup_tolerance/c1_bulk

./scripts/isaac_python.sh scripts/c1_autorun_categories.py \
  --dataset-root GRScenes-test1-normalized \
  --bak-root GRScenes-test1-normalized_bak \
  --report check_reports/normalized_dedup_tolerance/merged_geom_only.json \
  --c1-bulk-dir "$C1_BULK_DIR"
```

### autorun 流程（per category）

1. **build_mapping** — 从合并报告中提取该类别的 old→canonical mapping
2. **bulk_apply** — 批量改写 layout + start_result_* USD，输出去重版本
3. **step6** — promote（备份+覆盖）→ pxr 扫描 gate（hit=0）→ soft delete 到 `_bak`

### 安全特性

- `--skip-done` 默认开启，已完成类别自动跳过（可断点续跑）
- step6 有 scan gate：post-promote 和 post-soft-delete 都要求 hit=0
- 备份在 `GRScenes-test1-normalized_bak/_dedup_assets/`
- 建议通过 `--c1-bulk-dir` 使用 namespaced C1 工作目录，避免不同数据集或不同 dedup run 复用同一组 mapping / step6 状态
- ledger 记录在 `<c1-bulk-dir>/_autorun/`

---

## 产出物清单

| 路径 | 说明 |
|------|------|
| `check_reports/normalized_dedup_tolerance/<cat>/*_geom_only.json` | 79 个 per-category dedup 报告 |
| `check_reports/normalized_dedup_tolerance/summary.csv` | 汇总 CSV |
| `check_reports/normalized_dedup_tolerance/summary.json` | 汇总 JSON |
| `check_reports/normalized_dedup_tolerance/merged_geom_only.json` | 合并报告 |
| `check_reports/normalized_dedup_tolerance/c1_bulk/` | 示例 autorun 产出根目录（mapping, batch reports, step6 reports） |
| `GRScenes-test1-normalized_bak/` | soft-deleted 资产备份 |

---

## 回滚

与 Phase 1 相同，见 [asset_dedup_c1_scaling_workflow.md](asset_dedup_c1_scaling_workflow.md#5-回滚策略每个组都能回滚)。
数据集路径改为 `GRScenes-test1-normalized`，备份根改为 `GRScenes-test1-normalized_bak`。
