# GRScenes 资产 Mesh 去重报告（仅分析，不修改数据）

> 最后更新：2026-01-26

## 1. 这份报告解决什么问题

目标：回答“资产库里是否存在几何冗余（同一个模型被复制多份、transform 不同）”。

本流程只生成报告，不做任何删除/合并操作。

## 2. 报告由哪个脚本产生

主脚本：
- `scripts/report_asset_mesh_dedup.py`

用于把 300MB 大报告压缩成易读摘要的脚本：
- `scripts/summarize_asset_mesh_dedup_report.py`

## 3. 输入范围（本次 test1）

资产根目录（用户指定）：
- `GRScenes-test1/GRScenes_assets`

脚本默认按 GRScenes 资产布局扫描：
- `<assets_root>/<category>/<uid>/usd/<uid>.usd`（如果不符合，会退回到遍历所有 `usd/` 目录的兜底逻辑）

## 4. 如何运行（推荐 Isaac Sim python）

生成三份去重报告（三个角度）：

```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep \
  && ./scripts/isaac_python.sh scripts/report_asset_mesh_dedup.py \
      --assets-root GRScenes-test1/GRScenes_assets \
      --out-dir check_reports \
      --dataset test1
```

输出文件：
- `check_reports/test1_asset_mesh_dedup_geom_only.json`
- `check_reports/test1_asset_mesh_dedup_scale_only.json`
- `check_reports/test1_asset_mesh_dedup_full_matrix.json`

## 5. 三份报告分别代表什么

三份报告的明细 (`assets[]`) 是同一批资产，只是 `duplicates[]` 的分组口径不同：

1) `geom_only`
- 仅比较 Mesh 几何（points + 拓扑 + normals/UV 等），忽略 transform
- 用途：找“同几何但摆放/朝向/缩放不同”的重复资产候选

2) `scale_only`
- 比较“几何 + scale”，忽略平移/旋转
- 本项目约定：**scale 不同视为不同资产**，所以这份通常是首选

3) `full_matrix`
- 比较“几何 + 完整 4×4 world matrix（平移/旋转/缩放都敏感）”
- 用途：更像“完全拷贝/完全重复”的检查

## 6. 如何解读 300MB 的大 JSON

每个报告都是一个 JSON，对应结构：
- `meta`：统计信息（asset 数量、重复组数量、耗时等）
- `duplicates[]`：核心结果（每个重复组：`sig`、`count`、`usd_paths[]`）
- `assets[]`：全量明细（每个资产的签名、mesh 列表、每个 mesh 的点面数等）——这部分最大
- `errors[]`：打开 USD 或提取 mesh 失败的错误列表

通常只需要先看：
- `meta.duplicate_group_count`
- `duplicates` 的 top group（最大 `count` 的那些）

## 7. 生成“摘要报告”（强烈推荐）

把 300MB 压缩到几百 KB 的 summary：

```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep \
  && python3 scripts/summarize_asset_mesh_dedup_report.py \
      --input check_reports/test1_asset_mesh_dedup_scale_only.json \
      --top 50
```

会输出：
- `check_reports/test1_asset_mesh_dedup_scale_only.json.summary.json`

其中包含：
- `duplicates_summary.group_size_histogram`：重复组 size 的分布
- `duplicates_summary.top_groups`：最大的 N 个重复组（默认 50）

## 8. 后台运行与进度查看

后台运行示例（把 stdout/stderr 写入 log）：

```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep \
  && nohup ./scripts/isaac_python.sh scripts/report_asset_mesh_dedup.py \
      --assets-root GRScenes-test1/GRScenes_assets \
      --out-dir check_reports \
      --dataset test1 \
      --progress-every 200 \
      > check_reports/test1_asset_mesh_dedup_run.log 2>&1 < /dev/null &
```

进度文件：
- `check_reports/test1_asset_mesh_dedup_progress.json`（最新快照，适合 `watch`）
- `check_reports/test1_asset_mesh_dedup_progress.jsonl`（历史记录，适合追溯）

查看进度：

```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep \
  && watch -n 5 'cat check_reports/test1_asset_mesh_dedup_progress.json'
```

查看日志：

```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep \
  && watch -n 5 'tail -n 30 check_reports/test1_asset_mesh_dedup_run.log'
```
