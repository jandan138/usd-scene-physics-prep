---
title: Rebuild GRScenes-test0 From Legacy GRScenes
code_reference:
  - scripts/rebuild_test0_from_legacy.py
  - scripts/dlc/run_task.sh
  - specs_normalizer/normalize.py
created_at: 2026-03-14
updated_at: 2026-03-14
maintainer: Project Team
status: Active
---

# Rebuild `GRScenes-test0`

本流程用于从外部旧数据源：

- `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes`
- `/cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/commercial_scenes`

重新生成一份干净的 `GRScenes-test0`。

目标是恢复旧流程产物，不进入 normalize。

## 方案概览

恢复流程固定为三步：

1. `submit-dlc`
   - 提交两个 DLC 任务
   - `home_scenes` 和 `commercial_scenes` 分别导出到独立临时根
2. `merge`
   - 将两个临时导出目录合并成最终 `GRScenes-test0-rebuilt`
   - 对重叠文件做内容一致性检查
3. `validate`
   - 校验 scene 数量
   - 校验资产目录布局
   - 可选用 `pxr.Usd` 打开所有 `layout.usd`

## 1. 提交 DLC 任务

```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep

python3 scripts/rebuild_test0_from_legacy.py submit-dlc \
  --legacy-root /cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100 \
  --out-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt
```

默认行为：

- 最终目录：`GRScenes-test0-rebuilt`
- 临时目录：
  - `GRScenes-test0-rebuilt_home_tmp`
  - `GRScenes-test0-rebuilt_commercial_tmp`
- manifest：
  - `check_reports/rebuild_test0/<RUN_ID>/run_manifest.json`

只看命令而不提交：

```bash
python3 scripts/rebuild_test0_from_legacy.py submit-dlc \
  --dry-run \
  --run-id dryrun_check
```

## 2. 合并 DLC 结果

确认两个 DLC 任务都成功并且两个临时导出根都已生成后，执行：

```bash
python3 scripts/rebuild_test0_from_legacy.py merge \
  --manifest /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/rebuild_test0/<RUN_ID>/run_manifest.json
```

合并规则：

- `Material/mdl/**`：按文件路径合并；同路径文件必须内容一致
- `GRScenes_assets/**`：按文件路径合并；同路径文件必须内容一致
- `GRScenes100/home/**` 与 `GRScenes100/commercial/**` 分别来自各自临时根
- `GRScenes_assets/Asset_annotation.json` 不直接复用临时产物，合并后重新生成

若发现冲突，脚本会停止并写：

- `check_reports/rebuild_test0/<RUN_ID>/merge_summary.json`

## 3. 校验最终数据集

```bash
python3 scripts/rebuild_test0_from_legacy.py validate \
  --manifest /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/rebuild_test0/<RUN_ID>/run_manifest.json
```

默认校验项：

- 源数据 scene 数量必须是：
  - `home = 69`
  - `commercial = 30`
- 输出 `layout.usd` 数量必须和源一致
- 每个资产目录必须满足：
  - `GRScenes_assets/<category>/<uid>/usd/<uid>.usd`
- 如果环境里可用 `pxr.Usd`，则会尝试打开所有 `layout.usd`

若只想限制打开检查的数量：

```bash
python3 scripts/rebuild_test0_from_legacy.py validate \
  --manifest /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/rebuild_test0/<RUN_ID>/run_manifest.json \
  --open-check-limit 10
```

校验报告输出到：

- `check_reports/rebuild_test0/<RUN_ID>/validation_summary.json`

## 产物结构

成功后，最终目录应包含：

```text
GRScenes-test0-rebuilt/
├── Material/mdl/
├── GRScenes_assets/<category>/<uid>/usd/<uid>.usd
└── GRScenes100/
    ├── home/<scene_id>/layout.usd
    └── commercial/<scene_id>/layout.usd
```

## 注意事项

- 本流程是“恢复旧 `GRScenes-test0`”，不是 normalize。
- 不要把最终输出直接写回当前坏掉的 `GRScenes-test0`。
- 先生成 `GRScenes-test0-rebuilt`，确认无误后再手动切换。
