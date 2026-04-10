---
title: 为只有 USD 的资产目录生成 GLB（按目录标准输出）
code_reference: scripts/convert_subset_usd_to_glb.py
created_at: '2026-01-20'
updated_at: '2026-01-20'
maintainer: Codex
status: Active
---

# 为只有 USD 的资产目录生成 GLB（按目录标准输出）

> Last Updated: 2026-01-20

当你拿到一个“只有 USD”的资产子集包（例如 `sandbox/subset_20_gt10mb/`），并希望为其中每个资产生成标准位置的 GLB：

- 输入：`GRScenes_assets/<category>/<uid>/usd/<uid>.usd`
- 输出：`GRScenes_assets/<category>/<uid>/glb/<uid>.glb`

可以使用本仓库提供的批量转换脚本来完成，并产出 JSON report 方便审计/重跑。

---

## 1. 目录规范（必须遵守）

### 1.1 输入目录（normalized subset）

子集根目录 `<subset_root>` 下应包含：

- `GRScenes_assets/<category>/<uid>/usd/<uid>.usd`

脚本会遍历 `GRScenes_assets/**`，只对存在 `usd/<uid>.usd` 的资产做转换。

### 1.2 输出目录（normalized, optional）

每个资产的 GLB 固定写入：

- `GRScenes_assets/<category>/<uid>/glb/<uid>.glb`

如果目录 `glb/` 不存在，脚本会自动创建。

---

## 2. 转换工具链（ConvertAsset）

本仓库的批量脚本不会直接用 `pxr.Usd` 来导出 GLB，而是驱动 ConvertAsset：

- ConvertAsset 命令等价于：

```bash
<ConvertAsset>/scripts/isaac_python.sh <ConvertAsset>/main.py \
  usd-to-glb <input_usd> --out <output_glb>
```

默认 ConvertAsset 路径为：

- `/cpfs/shared/simulation/zhuzihou/dev/ConvertAsset`

如果你本机路径不同，使用 `--convertasset-root` 指定。

---

## 3. 批量生成 GLB（推荐流程）

实现脚本：

- [scripts/convert_subset_usd_to_glb.py](../../scripts/convert_subset_usd_to_glb.py)

### 3.1 最常用：对整个 subset 批量生成

```bash
python3 scripts/convert_subset_usd_to_glb.py \
  --subset-root sandbox/subset_20_gt10mb \
  --jobs 4 \
  --report check_reports/subset_20_gt10mb_glb_report.json
```

说明：

- `--jobs`：并行度（线程池，通常 2~8 合理；过大可能导致 ConvertAsset 资源竞争）。
- `--report`：输出 JSON 报告，包含每个 uid 的状态、耗时、失败时的 stdout_tail（最后 8KB）。

### 3.2 增量重跑 / 强制重跑

默认是“增量模式”：

- 若 `glb/<uid>.glb` 已存在且时间戳新于 `usd/<uid>.usd`，脚本会 `skipped_existing`。

如需强制重新生成：

```bash
python3 scripts/convert_subset_usd_to_glb.py \
  --subset-root sandbox/subset_20_gt10mb \
  --jobs 4 \
  --force \
  --report check_reports/subset_20_gt10mb_glb_report_force.json
```

### 3.3 只转换指定 UID

```bash
python3 scripts/convert_subset_usd_to_glb.py \
  --subset-root sandbox/subset_20_gt10mb \
  --uid 00ca2676bbed26d6a39a968d99d61176 \
  --uid 46078b032b5f00e665294e43999d653e \
  --report check_reports/subset_20_gt10mb_glb_report_selected.json
```

或从文件读取：

```bash
python3 scripts/convert_subset_usd_to_glb.py \
  --subset-root sandbox/subset_20_gt10mb \
  --uid-file sandbox/uids_20_gt10mb.txt \
  --report check_reports/subset_20_gt10mb_glb_report_uidfile.json
```

### 3.4 Dry-run（只打印将执行的命令，不实际生成）

```bash
python3 scripts/convert_subset_usd_to_glb.py \
  --subset-root sandbox/subset_20_gt10mb \
  --dry-run \
  --report check_reports/subset_20_gt10mb_glb_report_dryrun.json
```

---

## 4. 产物验收（建议）

### 4.1 检查 report 统计

脚本结束会打印：

- `ok/failed/skipped_existing/missing_input/dry_run` 的计数

返回码：

- `0`：全部成功（或被跳过）
- `4`：存在失败
- `5`：存在缺失输入 USD

### 4.2 检查输出文件是否存在

例如抽查某个 uid：

- `GRScenes_assets/<category>/<uid>/glb/<uid>.glb`

---

## 5. 常见问题

### 5.1 为什么我需要 ConvertAsset？

GLB 导出不是纯“文件拷贝”，需要一个稳定可复现的转换链。

我们当前使用 ConvertAsset 的 `usd-to-glb` 子命令来做规范化导出，并通过 report 记录结果。

### 5.2 报告文件是否需要提交到 git？

通常不建议提交运行产物 report（它更像一次运行的日志）。

如果你需要长期基准/对账，可以把 report 存到专门的 artifacts 目录或外部存储。
