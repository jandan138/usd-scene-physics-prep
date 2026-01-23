```markdown
# GRScenes-test1 类别别名合并操作指南（维护向）

> Last Updated: 2026-01-23

本指南面向仓库维护者，用于在 `GRScenes-test1` 这类导出包内，合并 `GRScenes_assets` 下的“类别别名目录”，并同步修复所有相关 USD 引用路径。

对应变更记录：

- [docs/changes/2026-01-23_test1_category_alias_merge.md](../changes/2026-01-23_test1_category_alias_merge.md)

实现脚本：

- [scripts/merge_asset_categories_test1.py](../../scripts/merge_asset_categories_test1.py)

---

## 1. 脚本做了什么

脚本将以下几类内容保持一致：

- 资产目录：将旧类别目录下的 UID 资产目录移动到新类别目录
- 资产 annotation：更新每个 `<uid>_annotation.json` 的 `category` 字段
- 资产索引：更新 `GRScenes_assets/Asset_annotation.json` 中受影响类别的 UID 列表
- USD 引用：重写 `GRScenes_assets/**` 与 `GRScenes100/**` 中对旧类别目录的引用路径

---

## 2. 本次默认合并规则（硬编码）

脚本内 `CATEGORY_MERGES`（下划线风格作为规范）：

- `coffeemaker` → `coffee_maker`
- `sofachair` → `sofa_chair`
- `tvstand` → `tv_stand`

不在范围内：

- `door_*` 变体（明确不合并）

---

## 3. 运行前准备（强烈建议）

1) 先做完整备份（目录级）：

```bash
rsync -a --info=progress2 GRScenes-test1/ GRScenes-test1_bak/
```

2) 确保使用带 USD/pxr 的运行环境：

- 推荐：使用仓库自带的 `./scripts/isaac_python.sh`

---

## 4. 推荐执行顺序（闭环）

### 4.1 Dry-run（只计算、不落盘）

```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root GRScenes-test1 \
  --dry-run \
  --report check_reports/test1_category_merge_dryrun.json
```

重点关注：

- `uid_collisions_total` 必须为 `0` 才建议继续 apply
- `dry_run move_count` 估算移动规模
- `usd_rewrite_preview_count` 估算需要重写的 USD 数量

### 4.2 Apply（落盘修改：移动 + 更新 JSON + 重写 USD）

```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root GRScenes-test1 \
  --apply \
  --report check_reports/test1_category_merge_apply.json
```

### 4.3 Validate（场景引用存在性检查）

```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root GRScenes-test1 \
  --validate \
  --report check_reports/test1_category_merge_validate.json
```

### 4.4 Post-check（确认“移动/重写已收敛”）

```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root GRScenes-test1 \
  --dry-run \
  --max-usd-files 2000 \
  --report check_reports/test1_category_merge_postcheck.json
```

---

## 5. 参数说明

- `--subset-root <DIR>`：指向子集根目录（例如 `GRScenes-test1`）
- `--dry-run`：只计算 move/rewrite 计划，不保存 USD、不移动目录
- `--apply`：执行移动目录 + 写回 JSON + 保存 USD root layer
- `--validate`：遍历 `GRScenes100/**/layout.usd`，检查引用/属性解析到的文件是否存在
- `--report <FILE>`：输出 JSON 报告（建议每次运行都写，便于审计）
- `--max-usd-files N`：限制扫描/处理的 USD 文件数量（用于快速测试）
- `--max-scenes N`：限制校验场景数量（用于快速测试）

---

## 6. Validate 报告如何解读

validate 的 `missing` 代表“USD 里引用/属性解析到的目标文件在磁盘上不存在”。

常见类型：

- 材质 MDL 缺失（例如 `../../../Material/mdl/.../*.mdl`）
- 其他相对路径引用缺失

注意：

- validate 的 missing 不一定由本次类别合并造成
- 若怀疑是合并造成的缺失，建议重点排查是否出现旧路径片段：
  - `GRScenes_assets/tvstand/`
  - `GRScenes_assets/sofachair/`
  - `GRScenes_assets/coffeemaker/`

如果 Post-check 显示 `usd_rewrite_preview_count: 0`，通常意味着“类别别名导致的 USD 路径问题已修复完成”。

---

## 7. 回滚

如需彻底回滚本次操作：

- 用备份目录覆盖恢复：`GRScenes-test1_bak/` → `GRScenes-test1/`

---

## 8. 扩展到更多类别（维护者须知）

若未来需要合并更多类别别名：

1) 在脚本中扩展 `CATEGORY_MERGES`
2) 必须先 dry-run 确认 `uid_collisions_total == 0`
3) 再 apply + validate + post-check

若出现 UID collision：

- 不建议强行合并（会覆盖/丢失资产）
- 需要制定 UID 重映射策略或引入新的目录分层策略

```