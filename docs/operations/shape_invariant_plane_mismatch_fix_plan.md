---
title: "Shape-Invariant Plane Orientation Mismatch Fix Plan"
code_reference:
  - scripts/compute_vertex_transform.py
  - scripts/rewrite_layout_asset_refs_with_compensation.py
  - docs/operations/ground_prim_agent_team_investigation_plan.md
created_at: 2026-03-23
updated_at: 2026-03-23
maintainer: team-lead
status: active
---

# Shape-Invariant Plane Orientation Mismatch — 完整修复方案

## 1. 通俗解释：到底是什么问题

### 背景：dedup 为什么需要补偿矩阵 V

数据集里有大量几何相似的资产（比如多个地板砖、天花板块），dedup 把它们合并为一个 canonical 资产。
合并时，场景里对原来资产的引用被替换为 canonical，同时需要一个补偿矩阵 V 来修正"两者几何差异"。

V 由 `compute_V_shape_invariant` 计算：把两边顶点归一化到单位包围盒，然后用 Procrustes SVD 找最优对齐，从 scale + rotation + translation 构成 V。

### 问题根因：坐标约定不一致

Ground（地面）资产在不同场景里被导出时，坐标轴约定不统一：

| 资产来源 | 顶点所在平面 | 平面法向 |
|---------|------------|---------|
| **某些 old 资产**（normalize 前） | XY 平面（Z 恒为常数） | Z 轴 |
| **canonical 资产**（normalize 后）| XZ 平面（Y ≈ 0） | Y 轴 |

**这两者外形完全相同**（都是长方形地板砖），只是"躺"的方向不同。
Procrustes SVD 发现两者需要 90° 旋转才能对齐，就把这个旋转编进了 V。

**V 本来的语义**：补偿两个"相同外形"资产之间的 scale 差异，不应该包含旋转。
**实际效果**：V 带着错误的 90° 旋转被写进场景，导致地面 footprint 长短轴对调。

### 图示

```
修复前（错误）:
  old_local = ((1,0,0),(0,0,-1),(0,1,0)) @ (192,581,80)   ← 正确的世界朝向
  V         = scale(1.863) * Rot90Y                        ← 错误：混入了 90° 旋转
  new_local = V * old_local                                → footprint 转了 90°

修复后（正确）:
  V         = scale(1.863) * Identity                      ← 只有 scale，无旋转
  new_local = V * old_local                                → footprint 保持原方向
```

---

## 2. 受影响范围扫描结果

扫描工具：检查每个 shape_invariant group 的 canonical 资产（normalized dataset）和 non-canonical 成员（rebuilt raw dataset）的顶点平面法向是否一致。

| 类别 | 总 Groups | 总 Pairs | 已扫描 | 法向不一致 Groups | 法向不一致 Pairs | Pairs 比例 |
|------|-----------|---------|--------|-----------------|----------------|-----------|
| **ground** | 981 | 6053 | 852 | 846 (99.3%) | **3923** | **64.8%** |
| **wall** | 1129 | 8074 | 728 | 359 (49.3%) | **1396** | **17.3%** |
| **ceiling** | 146 | 255 | 38 | 35 (92.1%) | **70** | **27.5%** |
| **合计** | 2256 | 14382 | — | — | **~5389** | — |

**结论**：ground 类几乎全部受影响，wall 约 1/5 受影响，ceiling 约 1/4 受影响。
其他类别（book, bottle, chair 等非扁平几何）无此问题（非扁平资产 Procrustes 不会引入平面旋转）。

---

## 3. 修复方案

### 3.1 算法修复（一次性改代码）

**文件**：`scripts/compute_vertex_transform.py`
**函数**：`compute_V_shape_invariant`（line 403）

**修改点**：在 Procrustes / ICP 之前，检测两边顶点的平面法向是否一致。
若不一致（两者都是平面资产但平面不同），则跳过旋转，只保留 scale。

```python
# 在 line 424-425（normalize 之后）插入以下检查：

def _flat_normal_axis(pts_norm, var_threshold=0.01):
    """归一化顶点：若是扁平资产，返回最小方差轴（法向轴）。否则返回 None。"""
    var = np.var(pts_norm, axis=0)
    min_ax = int(np.argmin(var))
    return min_ax if var[min_ax] < var_threshold else None

c_ax = _flat_normal_axis(c_norm)
o_ax = _flat_normal_axis(o_norm)

if c_ax is not None and o_ax is not None and c_ax != o_ax:
    # 两者都是平面资产但法向轴不同 → 跳过旋转
    logger.warning(
        "compute_V_shape_invariant: plane normal mismatch "
        f"(canon_axis={c_ax}, old_axis={o_ax}). "
        "Setting R_norm=Identity, t_norm=0 (scale-only V)."
    )
    R_norm = np.eye(3)
    t_norm = np.zeros(3)
else:
    # 原有 Procrustes / ICP 逻辑（保持不变）
    if len(pts_canon) == len(pts_old):
        # ... Procrustes ...
    else:
        # ... ICP ...
```

**效果**：V 变为 `scale(o_ext/c_ext) * Identity`，消除错误旋转。

### 3.2 数据修复（批量重跑）

算法修复后，需要对已经被错误 V 处理过的场景进行重跑。

**策略**：从 pre-normalize 快照（`layout.pre_c1_normalize_only.*.usd`）出发，重新 apply dedup。

**步骤**：

```
Step 1: 修改 compute_vertex_transform.py（算法修复）
Step 2: 生成受影响 pair 列表
        - ground:  3923 pairs（~65% of total 6053）
        - wall:    1396 pairs（~17% of total 8074）
        - ceiling: 70 pairs（~27% of total 255）
Step 3: 重跑 rewrite_layout_asset_refs_with_compensation.py
        --categories ground wall ceiling
        --input-snapshot pre_c1_normalize_only
        --affected-pairs <list from step 2>
Step 4: 验证
        - placement_pairwise_compare.py 对比修复前后
        - 重点抽查 ground（影响最大）
        - centroid delta 应 < 5，vertex RMSE 应 < 0.01
Step 5: 更新 ledger，生成新的 layout.usd
```

### 3.3 验证方案

**单 pair 验证**（已完成，作为基准）：
- `layout.phase2_route_b_test.usd`：correct_new_local → X=178.858, Y=37.262 ✅

**批量验证指标**：
| 指标 | 目标 |
|------|------|
| centroid delta | < 5（中心点不大幅移动） |
| vertex RMSE | < 0.01（修复后 vs 原始 rebuilt） |
| 受影响 pairs 中 footprint 方向正确比例 | > 95% |

---

## 4. 修复边界与风险

### 可能的副作用

1. **少数 wall 资产**：wall 类的 mismatch 约 49.3%，其余 ~50% 没有 plane mismatch。
   修改只影响"都是平面且法向不同"的 pair，不影响非平面或法向相同的 pair。
   **风险低**。

2. **部分 pair 可能确实需要旋转**：极少数情况下，两个形状相似但真实摆放方向不同的资产被归入同一 shape_invariant group，此时 R_norm != Identity 是正确的。
   - 这类情况对 ground/ceiling（几乎全是水平面）概率极低。
   - wall 需要更谨慎评估（wall 可以有不同朝向）。
   - **建议**：先处理 ground + ceiling，wall 单独评估后再处理。

3. **ceiling 高 skip 率**（108/146 groups canonical 未找到）：需确认 ceiling canonical 资产是否已被其他 dedup 模式进一步合并。如 canonical 本身也被替换，需追踪 transitive 链路。

### 不做的事

- 不修改全量 shape_invariant 逻辑（只针对 flat-plane mismatch 情形）
- 不修改 geom_only / topo_filesize 模式（无此问题）
- 不修改 normalize_asset_transforms.py（normalize 阶段本身无问题）

---

## 5. 执行优先级

```
Priority 1（立即做）: 算法修复 compute_V_shape_invariant
Priority 2（算法修复后）: 重跑 ground 类（受影响最大，3923 pairs）
Priority 3（ground 验证通过后）: 重跑 ceiling 类（70 pairs，小）
Priority 4（评估后决定）: 重跑 wall 类（需先确认 wall 的 mismatch 不是真实旋转差异）
```

---

## 6. 相关证据文档

| 文档 | 路径 |
|------|------|
| Phase 2 落锤 | `.codex/worklogs/main/2026-03-23/ground_phase2_final_conclusion.md` |
| Phase 1 深化调查 | `.codex/worklogs/main/2026-03-23/ground_phase1_deep_investigation.md` |
| Route B 验证结果 | `.codex/worklogs/subagents/2026-03-23/phase2_route_b_result.md` |
| Route A precheck | `.codex/worklogs/subagents/2026-03-23/phase2_route_a_precheck.md` |
| 扫描脚本 | `/tmp/scan_plane_v2.py`（临时，需整理为正式脚本） |
