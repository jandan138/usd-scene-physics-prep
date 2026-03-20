---
title: layout.usd 资产引用归一 + transform 补偿（Step 3B 工具）
code_reference: scripts/rewrite_layout_asset_refs_with_compensation.py
created_at: '2026-01-28'
updated_at: '2026-01-28'
maintainer: Codex
status: Active
---

# layout.usd 资产引用归一 + transform 补偿（Step 3B 工具）

最后更新：2026-01-27

本页说明脚本：`scripts/rewrite_layout_asset_refs_with_compensation.py`

它用于 C1 方案的 Step 3B：
- 把 layout.usd 中引用的“重复资产 USD”统一改成 canonical；
- 同时把资产 USD 内部的对齐/缩放差异迁移到 layout 实例 prim 上（写补偿矩阵），确保场景外观不变；
- 便于你后续删除“已完全无引用”的多余资产目录。

## 1. 依赖

- 需要 `pxr`（通常在 Isaac Sim 的 python 环境里）

## 2. 输入：mapping JSON

支持两种格式：

1) Dict 格式（最简单）：

```json
{
  "GRScenes-test1/GRScenes_assets/plant/AAA/usd/AAA.usd": "GRScenes-test1/GRScenes_assets/plant/BBB/usd/BBB.usd",
  "GRScenes-test1/GRScenes_assets/plant/CCC/usd/CCC.usd": "GRScenes-test1/GRScenes_assets/plant/BBB/usd/BBB.usd"
}
```

2) List-of-objects 格式：

```json
[
  {"old": ".../AAA.usd", "canonical": ".../BBB.usd"},
  {"old": ".../CCC.usd", "canonical": ".../BBB.usd"}
]
```

路径可以是：
- subset-root 相对路径（推荐，用 `GRScenes-test1/...` 这种）；或
- 绝对路径。

## 3. Dry-run（强烈建议先做）

```bash
python scripts/rewrite_layout_asset_refs_with_compensation.py \
  --layout-usd GRScenes-test1/GRScenes100/home/<scene>_usd/layout.usd \
  --mapping-json /path/to/mapping.json \
  --dry-run \
  --report-out /tmp/layout_rewrite_report.json
```

输出里你重点看：
- `counts.refs_changed / payloads_changed / asset_attrs_changed`
- `counts.xform_compensated`（是否对实例 prim 写了补偿矩阵）

## 4. 写出到新文件（推荐）

先不要原地改：

```bash
python scripts/rewrite_layout_asset_refs_with_compensation.py \
  --layout-usd GRScenes-test1/GRScenes100/home/<scene>_usd/layout.usd \
  --mapping-json /path/to/mapping.json \
  --out-usd /tmp/layout.dedup.usd \
  --report-out /tmp/layout_rewrite_report.json
```

验证没问题后，再决定是否替换回原文件。

## 5. 只改引用、不做补偿（不推荐）

如果你已经明确验证组内资产内部 transform 完全一致，可以用：

```bash
python scripts/rewrite_layout_asset_refs_with_compensation.py \
  --layout-usd ... \
  --mapping-json ... \
  --no-compensation
```

## 6. 已知限制

- “资产内部 transform”的定义：当前取资产 USD 的 `defaultPrim` 的 local transform。
  - 适合常见的“根 prim 对齐/缩放”场景。
  - 如果你的数据把对齐写在更深层级（不是 defaultPrim），需要扩展脚本的 internal transform 计算策略。

- 补偿 transform 的写入方式：脚本会把实例 prim 写成单一的 `xformOp:transform`（矩阵）。
  - 优点：不会引入 TRS 分解误差，能表达剪切等复杂矩阵。
  - 代价：会覆盖原有的 TRS op 栈（但保持最终矩阵等价）。
