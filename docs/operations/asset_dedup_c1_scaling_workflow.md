---
title: C1 规模化执行与审核流程（可控批处理版）
code_reference:
- scripts/c1_build_bulk_mapping_from_dedup_report.py
- scripts/c1_bulk_apply_layout_dedup.py
- scripts/c1_bulk_step6_category_promote_scan_soft_delete.py
created_at: '2026-01-31'
updated_at: '2026-01-31'
maintainer: Codex
status: Active
---

# C1 规模化执行与审核流程（可控批处理版）

> 目标：把“重复几何”按组（sig）批量落地到场景侧，同时做到：
> - 每一步**可回滚**、**可验证**、**可审计**（有证据链文件）
> - 长耗时步骤**有进度**，避免误以为卡住
> - 每个组都有**已处理记录**（ledger），避免下次重复处理
>
> 适用：你已经决定 C1 默认走 **Step 3B（改引用 + 补偿）**。

---

## 0. 你需要记住的两个“安全阀”

- **安全阀 A：只要还没通过验收，就不要 promote（覆盖 layout.usd），更不要删资产。**
- **安全阀 B：只要 post-promote 扫描 hit != 0，就必须停下，不做 soft delete。**

这两个阀门保证了：即使批处理规模很大，也不会“越改越乱”。

---

## 1. 输入/输出约定（每个组都一样）

### 1.1 输入

- dedup 报告（建议用 geom_only）：
  - `check_reports/test1_asset_mesh_dedup_geom_only.json`
- 数据集根：
  - `GRScenes-test1/`
- 备份根（软删除目的地）：
  - `GRScenes-test1_bak/`

### 1.2 每个组（sig）必须产出的“证据链”

建议每个组都固定放在一个目录下：

- `check_reports/c1_pilot/<group_label>_batch/`

其中 `<group_label>` 通常就是 `sig` 的简写（例如 `sig6da9` / `sigb526`）。

一个完整闭环的组，至少应包含：

- mapping（old→canonical）：
  - `check_reports/c1_pilot/pilot_mapping_<category>_<group_label>_full.json`
- Step 3B 扫描/改写相关：
  - `check_reports/c1_pilot/<category>_<group_label>_layout_hits.json`
  - `check_reports/c1_pilot/<group_label>_batch/*_layout_dryrun.json`
  - `check_reports/c1_pilot/<group_label>_batch/*_layout_apply.json`
  - 输出的 `layout.c1_<group_label>_dedup_v1.usd`（在每个 scene 目录里，与 `layout.usd` 同级）
  - 抽查清单（可选但推荐）：
    - `check_reports/c1_pilot/<group_label>_spotcheck_list.md`
- Step 6（闭环）相关：
  - promote 清单：`check_reports/c1_pilot/<group_label>_batch/promote_to_layout_usd_report.json`
  - promote 后 layout 扫描：`.../post_promote_layout_scan.json`
  - promote 后全库扫描（排除备份）：`.../post_promote_full_usd_scan_excluding_backups.json`
  - soft delete 清单：`.../soft_delete_old_assets_report.json`
  - soft delete 后 layout 扫描：`.../post_soft_delete_layout_scan.json`

### 1.3 已处理组的 ledger（避免重复处理）

- 追加式 ledger：
  - `check_reports/c1_pilot/c1_dedup_ledger.jsonl`

规则：
- 每完成一个组的 Step 6（promote+scan+soft delete）就追加一行。
- 下次运行前先查 ledger：存在则跳过该组。

---

## 2. “可控规模化”是什么意思（核心思想）

你可以把全量去重当成一个队列（queue），但每次只处理一个**小批次**，并且每个批次都要经过同一套关卡。

- **批次粒度**：建议“按组（sig）”作为最小单位。
- **每个批次的关卡**：
  1) Step 3B 批量 apply 输出新 layout（不覆盖原文件）
  2) 人工抽查通过（少量代表场景即可）
  3) Step 6 promote + 扫描 hit=0
  4) Step 6 soft delete（可回滚）

这套关卡保证：
- 你可以持续推进全量，但风险只在“当前这一组”。
- 出问题也只需要回滚这一组，而不是回滚整个数据集。

---

## 3. 每次执行（一个组）的标准流程

下面是你每次“处理一个组”的标准 SOP。你可以把它当成审核 checklist。

### 3.1 Phase A：准备（生成 mapping）

**目标**：确定该组的 canonical，并生成 old→canonical mapping。

审核点：
- canonical 是否合理（被引用多/更标准/不是空壳）。
- mapping 条目数是否等于“组大小 - 1”。

产物：
- `pilot_mapping_<category>_<group_label>_full.json`

### 3.2 Phase B：Step 3B 批处理（只输出，不覆盖）

**目标**：
- 在全库 `layout.usd` 中找到命中该 mapping 的场景
- 对命中场景输出 `layout.c1_<group_label>_dedup_v1.usd`

关键原则：
- **先 dry-run 全量扫描**（99 个 layout 也可以接受）得到 hit 清单。
- **只对 hit_layouts 做 apply**。
- apply 输出必须写在原 layout 同目录（避免相对路径解析变化）。

审核点：
- `hit_layouts` 数量合理（不是 0，也不是离谱地大）。
- apply 报告里 `refs_changed/xform_compensated` 与预期一致。

产物：
- hit 清单 + per-layout 报告 + 输出 layout 文件。

### 3.3 Phase C：人工抽查（少量代表场景）

**目标**：确认外观/摆放不变。

推荐抽查策略（用最少人工覆盖最多风险）：
- 优先抽查“changed prim 数最多”的场景。
- 至少抽查 3~8 个 scene（规模越大抽查越多）。

抽查要点：
- 位置/旋转/缩放
- 材质是否变灰/丢失
- 是否出现 missing reference

产物：
- `*_spotcheck_list.md`（推荐）

### 3.4 Phase D：Step 6 闭环（promote → 扫描 → soft delete）

**目标**：
- 把输出 layout 提升为正式 `layout.usd`（并生成备份）
- 扫描确认“活跃数据”不再引用 old 资产
- soft delete old 资产目录到 bak

推荐使用一键脚本（包含进度 & ledger）：
- [scripts/c1_step6_promote_scan_soft_delete.py](../../scripts/c1_step6_promote_scan_soft_delete.py)

先 dry-run 看计划：
- `python scripts/c1_step6_promote_scan_soft_delete.py --dataset-root GRScenes-test1 --bak-root GRScenes-test1_bak --group-label <group_label> --category <category> --mapping-json <mapping_json> --dry-run`

正式执行：
- `python scripts/c1_step6_promote_scan_soft_delete.py --dataset-root GRScenes-test1 --bak-root GRScenes-test1_bak --group-label <group_label> --category <category> --mapping-json <mapping_json>`

进度显示：
- 脚本会在终端打印 `processed/total/rate/eta`。
- 也会在 batch 目录写进度 JSON/JSONL（用于 watch）。

审核点：
- promote 后扫描 `hit=0`
- soft delete 后扫描 `hit=0`
- 目录移动清单与数量正确
- ledger 已追加一条记录

---

## 4. 长耗时步骤怎么“看进度”（你审核用）

### 4.1 全库扫描的进度

如果你想单独跑扫描（不走 Step6 一键脚本），用：
- [scripts/scan_usd_for_asset_paths.py](../../scripts/scan_usd_for_asset_paths.py)

它支持：
- `--progress-json`（快照，适合 watch）
- `--progress-jsonl`（历史）

示例：
- `python scripts/scan_usd_for_asset_paths.py --root GRScenes-test1 --mapping-json <mapping_json> --exclude-name-contains pre_c1_<group_label> --out-json <out_report> --progress-json <progress.json> --progress-jsonl <progress.jsonl>`

另一个终端看：
- `watch -n 2 cat <progress.json>`

### 4.2 为什么会慢（审核口径）

- 这是 I/O 密集型：会读大量 `.usd` 文件字节；网络/并行文件系统吞吐决定速度。
- 只要进度在涨、`cur` 在变化，就不是卡住。

---

## 5. 回滚策略（每个组都能回滚）

- 如果你还停在 Step 3B 输出阶段：
  - 直接丢弃 `layout.c1_<group_label>_dedup_v1.usd` 即可（不影响原始 layout）。

- 如果你已 promote：
  - 用备份文件 `layout.pre_c1_<group_label>.<stamp>.usd` 恢复 `layout.usd`。

- 如果你已 soft delete：
  - 把 `GRScenes-test1_bak/_dedup_assets/<group_label>_<stamp>/...` 下的目录移回 `GRScenes-test1/GRScenes_assets/...`。

---

## 6. 如何把“所有重复资产”都处理完（队列推进建议）

强烈建议分阶段推进：

### 6.1 阶段 1：按类别推进（最稳）

例如：
- 先 `plant` 全做完
- 再 `book`
- 再 `other`

好处：
- 风险集中（材质/物理差异的坑往往类别相关）
- 审核更容易（你对该类资产的“正常样子”更熟）

### 6.2 阶段 2：按收益推进（最快见效）

- 每次从 dedup 报告里挑 `count` 最大的组开始做。

### 6.3 每轮推进的推荐节奏（可控）

- 每轮处理 1~5 个组（取决于你抽查成本）
- 每轮结束时你审核：
  - 该轮每个组是否都有完整证据链
  - ledger 是否新增对应记录

---

## 7. 参考样例（已完成的组）

- `sig6da9`：见 runbook 中对应 Step 6 记录（promote+scan+soft delete）。
- `sigb526`：同上。

相关证据链都在：
- `check_reports/c1_pilot/sig6da9_batch/`
- `check_reports/c1_pilot/sigb526_batch/`
