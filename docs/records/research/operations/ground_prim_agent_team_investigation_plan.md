---
title: "Ground Prim Agent Team Investigation Plan"
code_reference:
  - docs/operations/agent-team-playbook.md
  - scripts/oneoff_inspect_layout_prim.py
  - scripts/debug_scene_prim_geometry_delta.py
  - scripts/rewrite_layout_asset_refs_with_compensation.py
  - scripts/normalize_asset_transforms.py
  - scripts/compute_vertex_transform.py
created_at: 2026-03-23
updated_at: 2026-03-23
maintainer: Codex
status: complete
---

# Ground Prim Agent Team Investigation Plan

## 通俗版判断

先用大白话说清楚这次问题：

- 现在看起来，不像是那块小地面 mesh 自己“跑错位置”了。
- 更像是它上面的整个 `ground model root` 先在 normalize 阶段发生了变化，后面 dedup 又把它引用的 ground 资产换成了另一个 canonical 资产，并且系统对这个替换做了补偿。
- 这个补偿没有把最终效果完全补对。
- 所以画面上看到的是：**中心点大致还在原来的地方，但地面的长宽方向像被转了，footprint 不对了。**

换句话说：

- 不要先盯着叶子 prim
  - `/Root/Meshes/Base/ground/model_5c427c87c906fdb4c0decedb9ba5b2b2_0/Instance/MUN6F7QKTJS66AABAAAAADY8_level_MeshWithData_130_0`
- 要先盯：
  - 它的祖先 model root
    - `/Root/Meshes/Base/ground/model_5c427c87c906fdb4c0decedb9ba5b2b2_0`
  - old asset `5c427c87c906fdb4c0decedb9ba5b2b2`
  - canonical asset `117169286fd95fdeda8a3fa1fc4abe30`
  - 以及 old -> canonical 替换时用到的补偿矩阵

## 本次 Agent Teams 必须回答的 3 个问题

1. 最早是从哪个 snapshot 开始偏掉的？
2. old -> canonical 这对 ground 资产里，到底是哪一个矩阵不对？
3. 这是公式错、输入矩阵错，还是公式本身没错但对这类 ground 资产不够用？

## 调查边界

- 第一阶段只做只读调查，不改代码，不改 USD，不覆盖别人的结果。
- 所有 agent 都只产出证据、结论和下一步建议。
- 只有在第一阶段把责任点缩到很小之后，才进入第二阶段的修复。
- 同一份源码或同一个输出结论，只能有一个 owner。

## 目标场景与对象

**场景**

- 原始参考场景：
  - `GRScenes-test0-rebuilt/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd`
- 当前新场景：
  - `GRScenes-test0-rebuilt-normalized/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd`

**重点 prim**

- model root：
  - `/Root/Meshes/Base/ground/model_5c427c87c906fdb4c0decedb9ba5b2b2_0`
- 用户观察到不对的叶子 prim：
  - `/Root/Meshes/Base/ground/model_5c427c87c906fdb4c0decedb9ba5b2b2_0/Instance/MUN6F7QKTJS66AABAAAAADY8_level_MeshWithData_130_0`

**重点资产对**

- old:
  - `GRScenes_assets/ground/5c427c87c906fdb4c0decedb9ba5b2b2/usd/5c427c87c906fdb4c0decedb9ba5b2b2.usd`
- canonical:
  - `GRScenes_assets/ground/117169286fd95fdeda8a3fa1fc4abe30/usd/117169286fd95fdeda8a3fa1fc4abe30.usd`

## Team 设计

### Team Lead

**Owner**

- 当前主会话

**职责**

- 保持范围收敛
- 防止和其他同事冲突
- 管理 worklog
- 整合结论
- 决定是否进入修复阶段

### Agent A: 场景快照对比

**目标**

- 证明问题是不是写在叶子 prim 上
- 找到“最早开始不对”的 snapshot

**输入**

- 原始 `layout.usd`
- `layout.pre_c1_normalize_only.*`
- 当前 `layout.usd`
- `layout.c1_v3_vmatrix_fix_dedup_v1.usd`

**建议命令**

```bash
./scripts/isaac_python.sh scripts/oneoff_inspect_layout_prim.py \
  GRScenes-test0-rebuilt/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd \
  /Root/Meshes/Base/ground/model_5c427c87c906fdb4c0decedb9ba5b2b2_0

./scripts/isaac_python.sh scripts/oneoff_inspect_layout_prim.py \
  GRScenes-test0-rebuilt-normalized/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.pre_c1_normalize_only.20260315_chain_fix_v1.usd \
  /Root/Meshes/Base/ground/model_5c427c87c906fdb4c0decedb9ba5b2b2_0

./scripts/isaac_python.sh scripts/oneoff_inspect_layout_prim.py \
  GRScenes-test0-rebuilt-normalized/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd \
  /Root/Meshes/Base/ground/model_5c427c87c906fdb4c0decedb9ba5b2b2_0

./scripts/isaac_python.sh scripts/debug_scene_prim_geometry_delta.py \
  --left-stage GRScenes-test0-rebuilt/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd \
  --right-stage GRScenes-test0-rebuilt-normalized/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd \
  --prim-path /Root/Meshes/Base/ground/model_5c427c87c906fdb4c0decedb9ba5b2b2_0
```

**交付物**

- 哪个 prim 才是第一个 authored divergence 点
- 最早异常 snapshot
- world bbox / centroid / footprint 差异

**停止条件**

- 能明确回答“错写在 leaf、Instance、还是 model root”

### Agent B: 矩阵链路核对

**目标**

- 把这一个 pair 的关键矩阵全部对出来

**必须拿到的 5 个矩阵**

- `old_local`
- `old_internal`
- `canonical_internal`
- `V`
- `new_local`

**重点代码**

- `scripts/rewrite_layout_asset_refs_with_compensation.py`
- `scripts/compute_vertex_transform.py`

**要验证的核心公式**

```text
new_local = canonical_internal^-1 * V * old_internal * old_local
```

**交付物**

- 这 5 个矩阵的实数值
- 用这 5 个矩阵能否解释当前看到的 rotated footprint
- 结论到底是：
  - 公式错
  - 输入矩阵错
  - 公式没错但对这类资产不够用

**停止条件**

- 能把责任缩到“某个具体矩阵”或“某个具体公式假设”

### Agent C: 资产几何与基底对比

**目标**

- 直接比较 old ground 资产和 canonical ground 资产本身

**要看什么**

- `/Root` 到 `Instance` 再到 mesh parent 的层级
- 两边 local bbox、footprint、主轴方向
- canonical 是否天然就比 old 转了 90 度或换了基底

**交付物**

- old 和 canonical 的“本体差异”说明
- 是否存在“视觉上很像同一个地面，但坐标基底不同”的情况

**停止条件**

- 能说明问题更像“资产本体差异未被完全补偿”还是“场景层写错”

### Agent D: 报告与 provenance 核对

**目标**

- 把这条 prim 在报告里的时间线串起来

**要看什么**

- union / mapping / layout apply report
- placement investigation report
- 是否存在 descendant remap 记录

**交付物**

- old -> canonical 替换证据
- `references` / `xform_compensation` / `set_instanceable` 证据
- descendant remap 是否涉及这棵子树

**停止条件**

- 能明确说出“这是 dedup 补偿问题”还是“这是 descendant remap 问题”

### Reviewer

**目标**

- 检查最后的整合结论有没有说过头

**重点检查**

- 不能把“normalize 阶段首次偏离”说成“normalize 单独造成最终问题”
- 不能在没有矩阵证据前，就断言是公式错还是矩阵输入错

## 推荐启动顺序

### Phase 1: 全只读调查

1. Team Lead 建立 worklog 根目录
2. 并行启动 Agent A / B / C / D
3. 等所有 agent 返回
4. Team Lead 做一次整合
5. Reviewer 只检查“结论是否过度”

### Phase 2: 只在证据足够时启动

只有满足下面条件，才进入修复阶段：

- 已经锁定到一个很小的责任面
- 不会和其他同事的排查冲突
- 能用一个很小的 patch 验证假设

## 本轮最重要的判断规则

- 如果 leaf prim 本身没有 authored xform 变化，就不要从 leaf 修。
- 如果问题最早出现在 normalize-only，就记录为“最早偏离点”，但不要直接下结论说 normalize 就是最终根因。
- 如果 dedup apply report 明确记录了 `xform_compensation`，就必须把矩阵链路对出来，再决定是算法问题还是输入数据问题。
- 如果 apply report 没有 descendant-path remap 记录，就不要优先怀疑 descendant override 路线。

## 推荐的 Team Lead 启动口令

下面这段可以直接作为主会话的启动说明：

```text
启动 agent teams，对 scene MV7J6NIKTKJZ2AABAAAAADA8_usd 的 ground placement 问题做只读排查。

目标 prim:
/Root/Meshes/Base/ground/model_5c427c87c906fdb4c0decedb9ba5b2b2_0/Instance/MUN6F7QKTJS66AABAAAAADY8_level_MeshWithData_130_0

重点 model root:
/Root/Meshes/Base/ground/model_5c427c87c906fdb4c0decedb9ba5b2b2_0

重点 asset pair:
5c427c87c906fdb4c0decedb9ba5b2b2 -> 117169286fd95fdeda8a3fa1fc4abe30

要求:
1. 第一阶段全部只读，不改代码，不改 USD。
2. 先判断最早偏离点，再判断 dedup 补偿链路。
3. 不要把 normalize-only 的首次偏离，直接说成最终根因。
4. 最终必须给出 old_local / old_internal / canonical_internal / V / new_local 的核对建议。
```

## 预期结论格式

最终结论最好强制写成下面这种格式：

1. **最早偏离点**
2. **最终问题出现在哪个阶段被放大**
3. **当前最可疑的矩阵或公式**
4. **最小下一步验证动作**

## 这份计划的目的

这份计划不是为了立刻修 bug，而是为了把问题压缩到一个足够小、足够清楚的责任面，确保下一次真正动代码时：

- 不会和同事撞车
- 不会一边修一边猜
- 不会把”最早偏离点”和”最终根因”混成一件事

---

## Phase 2 修正版计划（2026-03-23 更新）

### Phase 1 已确认的事实

Phase 1 + 补充深化调研已经完整打出了矩阵链路，见
`.codex/worklogs/main/2026-03-23/ground_phase1_deep_investigation.md`。
以下为 Phase 2 的直接前提：

| 项目 | 结论 |
|------|------|
| dedup mode | `shape_invariant` |
| formula | `new_local = V * old_local`（两边 internal 0.1*I 抵消） |
| V scale | 1.8631005289，**正确** |
| V translation | 零，**正确** |
| **V rotation** | ~90° 绕 Y 轴，**错误** |
| 正确 V | `scale(1.8631005289) * Identity` |
| 正确 new_local | `scale(1.8631005289) * old_local` |
| old_local 数值 | `((1,0,0,0),(0,0,-1,0),(0,1,0,0),(192.456,581.0,80.0,1))` |
| correct_new_local 数值 | `((1.8631,0,0,0),(0,0,-1.8631,0),(0,1.8631,0,0),(192.456,581.0,80.0,1))` |

Phase 2 **不再验证** formula、V 的施用方式、leaf prim、descendant-remap。

---

### 关键约束：old normalized asset 已删除，不能直接用 rebuilt 原始资产复算 V

V 是在 **normalized** old asset 上算的：
```
extract_instance_space_vertices(
    GRScenes-test0-rebuilt-normalized/GRScenes_assets/ground/5c427c87.../usd/5c427c87....usd
)
```

该文件已被 soft-delete，找不到。

`GRScenes-test0-rebuilt` 里的**原始 raw 资产**与 normalized 版本有本质差异：
- 内部 scale 很可能是 1（不是 0.1）
- 顶点坐标数量级、坐标轴约定均不同
- 直接拿来输入 `compute_V_shape_invariant` 会算出完全不同的 V，**不能还原当时的真实 V**

因此：
- **路线 B（直接修复验证）** 不依赖 old asset，是主路线
- **路线 A（V 根因复算）** 需要先对原始资产补跑 normalize，条件严格，降级为可选深挖

---

### 路线 B（主路线）：直接验证 V_correct 能否修复 footprint

**前提**：不需要 old asset，不需要复算 V。

**目标**：把这一个 prim 的 `xformOp:transform` 替换为 `correct_new_local`，
验证 footprint 是否从错误的 `37.3×178.9` 恢复为正确的 `178.9×37.0`。

**correct_new_local 数值**（直接可用）：
```
((1.8631005289, 0,            0,             0),
 (0,            0,            -1.8631005289, 0),
 (0,            1.8631005289, 0,             0),
 (192.456,      581.0,        80.0,          1))
```
即 `scale(1.8631005289) * old_local`，scale 保持不变，去掉 V 里的 90° 旋转。

**验证步骤**：
1. 在当前 `layout.usd` 的副本里，把
   `/Root/Meshes/Base/ground/model_5c427c87c906fdb4c0decedb9ba5b2b2_0`
   的 `xformOp:transform` 改为 `correct_new_local`
2. 用 pxr 计算 canonical 资产顶点经 `correct_new_local` 后的世界坐标 bbox
3. 量化验证：
   - world X-extent ∈ [170, 190]（目标 178.9）
   - world Y-extent ∈ [32, 42]（目标 37.0）
   - centroid 与修复前 delta < 5（中心点基本不动）
4. （可选）在 Isaac Sim / USD Composer 目视确认 ground 与桌子/墙壁的相对关系恢复合理

**成功标准**：量化指标全部通过，且不引入新的明显平移偏差。

**路线 B 结论**：
- 成功 → 确认”纯 scale V 能修复该 pair”，进入”是否推广”决策
- 失败 → 说明 correct_new_local 假设本身有误，需要重新分析

---

### 路线 A（可选深挖）：重建 normalized old asset 再复算 V

**前提**：需要先完成以下步骤，才能走路线 A。

1. 确认 `GRScenes-test0-rebuilt/GRScenes_assets/ground/5c427c87.../` 存在且可正常打开
2. 对该原始资产单独跑一遍 `normalize_asset_transforms.py`，
   得到临时的 normalized 版本（不写回 dataset，只用于本次复算）
3. 用临时 normalized 版本作为 `pts_old` 输入 `compute_V_shape_invariant`

**风险**：补跑的 normalize 可能与当初批量运行的结果有微小差异（环境、随机seed等），
复算出的 V 不保证和当初完全一致。因此路线 A 的结论只能是”大概率根因”，不是”确定还原”。

**路线 A 必须打印的中间量**：
- `len(pts_old)` vs `len(pts_canon)`（决定走 Procrustes 还是 ICP）
- `pts_old` bbox center / max_extent
- `pts_canon` bbox center / max_extent
- 归一化后的 `c_norm` / `o_norm` 形状（长轴方向）
- `R_norm` / `s_norm` / `t_norm`
- 最终 V

**路线 A 结论格式**：
- R_norm ≈ Identity → V 应为纯 scale，当初运行时可能有环境差异或 ICP 局部最优
- R_norm ≈ 90° 稳定复现 → 确认是 `compute_V_shape_invariant` 对此类 ground pair 的系统性问题

---

### 执行优先级

```
Step 0: 运行路线 B 量化验证（不依赖任何外部资产，直接可执行）
Step 1: 如果路线 B 成功，决策是否推广到 ground 类 shape_invariant pairs
Step 2: （可选）如果需要搞清楚根因，走路线 A 的前置检查
```

---

### Phase 2 落锤格式

Phase 2 结束时必须二选一：

**结论 A**（路线 A 完成后）：
> `compute_V_shape_invariant` 对这类 ground pair 会因 [ICP局部最优 / 轴向歧义 / ...]
> 错误引入 90° 旋转。后续应修算法或为此类 pair 加保护逻辑。

**结论 B**（仅路线 B 完成）：
> 无法稳定复算 V 根因，但”纯 scale V”已证明能修复该 pair 的 footprint。
> 后续先做定向 patch，再评估是否推广到 ground 类 shape_invariant pairs。

---

### 回归边界（本阶段不做）

- 不修改全量 ground 类别
- 不重跑全量 dedup apply
- 不修改 `compute_V_shape_invariant` 代码
- 不把修复推广到其他 shape_invariant pair
