---
title: 2026-01-23 — GRScenes-test1 类别别名合并（coffeemaker/sofachair/tvstand）
code_reference: scripts/merge_asset_categories_test1.py
created_at: '2026-01-23'
updated_at: '2026-01-23'
maintainer: Codex
status: completed
---

```markdown
# 2026-01-23 — GRScenes-test1 类别别名合并（coffeemaker/sofachair/tvstand）

## 背景与目标

在 `GRScenes-test1/GRScenes_assets/` 下同时存在“无下划线”和“下划线”两套类别目录，导致：

- 资产目录风格不一致（`tvstand` vs `tv_stand` 等）
- 场景 USD 内部引用路径会指向不同风格目录，维护成本高

本次目标是**统一下划线风格**，并保证：

- 既执行合并操作，也提供可审计的校验机制
- 合并后 `GRScenes100` 的场景 USD 仍能正常打开（至少：引用路径不因目录移动而断裂）

> 重要决策：`door_*` 变体**暂不合并**。
> 原因：不同 `door_*` 目录之间存在 UID 碰撞风险；若强行合并会发生覆盖（需要重新分配 UID 或引入额外命名空间策略）。

---

## 合并规则（本次范围）

仅处理以下 3 组别名：

- `coffeemaker` → `coffee_maker`
- `sofachair` → `sofa_chair`
- `tvstand` → `tv_stand`

不处理：

- `door_*` → `door`（明确不做）

---

## 实施方式（工具与机制）

新增并使用脚本：

- [scripts/merge_asset_categories_test1.py](../../scripts/merge_asset_categories_test1.py)

脚本包含三段闭环机制：

1. **Dry-run**：计算将要移动的 UID 目录列表，并检测 UID collision
2. **Apply**：执行移动 + 更新 annotation + 更新索引 + 重写 USD 引用路径
3. **Validate**：遍历 `GRScenes100/**/layout.usd`，检查引用/属性路径解析到的文件是否存在（best-effort）

---

## 执行前安全措施

在落盘修改前，已对整个子集做完整备份：

- 原始：`GRScenes-test1/`
- 备份：`GRScenes-test1_bak/`

---

## 实际执行命令

Dry-run（预览与碰撞检查）：

```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root GRScenes-test1 \
  --dry-run \
  --report check_reports/test1_category_merge_dryrun.json
```

Apply（执行合并与 USD 重写）：

```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root GRScenes-test1 \
  --apply \
  --report check_reports/test1_category_merge_apply.json
```

Validate（场景依赖存在性校验）：

```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root GRScenes-test1 \
  --validate \
  --report check_reports/test1_category_merge_validate.json
```

Post-check（合并后再次 dry-run，确保无需进一步移动/重写）：

```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root GRScenes-test1 \
  --dry-run \
  --max-usd-files 2000 \
  --report check_reports/test1_category_merge_postcheck.json
```

---

## 结果摘要

Apply 结果（见报告）：

- `uid_collisions_total: 0`
- `apply moved_count: 74`
- `usd_rewritten_count: 165`

Validate 结果（见报告）：

- `validated scenes: 99 / 99`
- `missing: 89`

Post-check 结果（见报告）：

- `dry_run move_count: 0`
- `usd_rewrite_preview_count: 0`

---

## 校验结果解读（重要）

本次 validate 的 `missing` 主要为 **MDL 引用文件缺失**，形如：

- `../../../Material/mdl/vMaterials_2/.../*.mdl`

报告中同时给出了 `resolved` 绝对路径（例如解析到 `GRScenes-test1/Material/...`），其中部分文件在磁盘上确实不存在。

这类缺失：

- 与本次类别合并（`tvstand/sofachair/coffeemaker`）**无直接关系**
- 即使不做合并，打开 stage 时也会触发类似缺失检查

同时，Post-check 显示对三组类别别名的 USD 路径重写已经完全收敛（无需再次重写）。

---

## 产出物（审计与回滚）

报告文件：

- Dry-run: [check_reports/test1_category_merge_dryrun.json](../../check_reports/test1_category_merge_dryrun.json)
- Apply: [check_reports/test1_category_merge_apply.json](../../check_reports/test1_category_merge_apply.json)
- Validate: [check_reports/test1_category_merge_validate.json](../../check_reports/test1_category_merge_validate.json)
- Post-check: [check_reports/test1_category_merge_postcheck.json](../../check_reports/test1_category_merge_postcheck.json)

回滚方式：

- 若需要完全回退本次操作：直接用 `GRScenes-test1_bak/` 覆盖恢复 `GRScenes-test1/`。

```