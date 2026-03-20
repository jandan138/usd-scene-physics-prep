---
title: test0 smoke normalize descendant override research
code_reference:
  - scripts/normalize_asset_transforms.py
  - scripts/audit_normalize_phase2.py
  - scripts/check_normalize_gate_from_reports.py
created_at: 2026-03-12
updated_at: 2026-03-12
maintainer: codex
status: completed
---

# 问题背景

这轮 `S1 smoke` 的原始目标，是验证 `GRScenes-test0` 从 `normalize` 起跑是否安全，
也就是先做 `normalize-only`，只有这一关通过后才允许进入 `dedup`。

这次实际运行在 `normalize-only gate` 处停止，最终结论是：

- `fail_normalize_s1`
- `phase2 audit` 表面干净
- 但 `pairwise` 仍然发现 1 个真实 outlier

失败对象是：

- scene: `home/MV7J6NIKTKJZ2AABAAAAADA8_usd`
- prim:
  `/Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0`

最初表象看起来有些矛盾：

- `pairwise` 说 refrigerator 漂了
- `phase2 audit` 又说 `matrix_mismatch = 0`

本轮调研的核心任务，就是把这个矛盾解释清楚，并判断下一步应该继续修
`normalize`，还是可以直接进入 `dedup`。

# 已确认根因

目前已经确认，问题不在 `dedup`，也不在“资产文件被 normalize 写坏了”。

真正的根因是：

- 原始 scene 在 refrigerator 的**引用子树内部**额外 author 了一条
  descendant xform override
- 具体路径是：
  `/Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0/Instance`
- 这条 override 包含：
  `xformOp:scale = (0.098, 0.1012024, 0.1012024)`

在原始 scene 中，这条 scene-layer override 是生效的，因为原始 asset
内部仍然暴露了对应的 descendant xform 结构。

在 normalize 之后，资产内部 xform 结构被改写了。结果是：

- 顶层 reference root prim 的补偿矩阵仍然能按当前实现写对
- 但 scene 里原本写在 descendant `/Instance` 上的 `xformOp:scale`
  不再参与新的 composed xform 计算
- 这条 override 变成了 inert opinion
- refrigerator 因此发生了真实的 world-space 几何漂移

所以这不是一个“顶层矩阵公式算错”的问题，而是一个
“scene-authored descendant override 在 normalize 后失效”的问题。

# 证据文件

以下文件共同支持上面的结论。

## 1. 原始 gate 失败与 outlier 现象

- [final_verdict.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/summary/final_verdict.json)
- [summary.md](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/summary/summary.md)
- [s1_test0_vs_normalized_pre_dedup.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/s1_test0_vs_normalized_pre_dedup.json)

这些文件证明：

- `normalize-only` 没有过关
- refrigerator 的 `displacement = 0.061537`
- `max_per_mesh_displacement = 1.244992`

## 2. `phase1` 并没有把资产写坏

- [refrigerator_expected_vs_actual_asset.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/investigation/refrigerator_expected_vs_actual_asset.json)
- [refrigerator_expected_vs_actual_normalized_asset.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/investigation/refrigerator_expected_vs_actual_normalized_asset.json)

这些文件说明：

- refrigerator 的 normalized asset 与当前 `phase1` 数学基本一致
- 聚合点位误差只有 `~1e-5` 到 `2e-5`

因此，这不是一个“phase1 写盘错误”的简单 bug。

## 3. refrigerator 的漂移是真实几何漂移，不是 prim 表示假象

- [refrigerator_geometry_delta.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/investigation/refrigerator_geometry_delta.json)
- [refrigerator_scene_geometry_delta.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/investigation/refrigerator_scene_geometry_delta.json)
- [refrigerator_geometry_delta_v2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/refrigerator_geometry_delta_v2.json)

这些文件说明：

- refrigerator 的 world-space 几何确实漂了
- 不只是 prim origin 变了
- 很多 mesh 的偏差都集中在 world-space 某个轴向上

## 4. control 资产正常

- [nightstand_control_geometry_delta.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/investigation/nightstand_control_geometry_delta.json)
- [nightstand_scene_geometry_delta.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/investigation/nightstand_scene_geometry_delta.json)
- [nightstand_geometry_delta_v2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/nightstand_geometry_delta_v2.json)

这些文件说明：

- 这不是 normalize 链路对所有资产都会触发的普遍漂移
- 至少 `nightstand` control 仍保持在噪声级误差

## 5. descendant override 失效的直接证据

- [orig_scene_ref_descendant_xform_overrides.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/investigation/orig_scene_ref_descendant_xform_overrides.json)
- [normalized_scene_ref_descendant_xform_overrides.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/investigation/normalized_scene_ref_descendant_xform_overrides.json)
- [s1_normalize_phase2_audit_v2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/s1_normalize_phase2_audit_v2.json)
- [normalize_gate_verdict_v2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/summary/normalize_gate_verdict_v2.json)

这些文件说明：

- 原始 scene 中确实存在 descendant xform override
- normalize 后这条 override 在 composed xform 中变成 inert
- 新版 gate 已经能把这件事直接作为失败原因输出

## 6. 当前调查与修复计划文档

- [summary.md](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/investigation/summary.md)
- [descendant_override_remap_plan.md](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/docs/test0_smoke/20260312_074731/descendant_override_remap_plan.md)

# 结论边界

当前已经能确定的事情：

- refrigerator 的失败和 `dedup` 无关
- refrigerator 的失败不是 `placement_pairwise_compare.py` 误报
- refrigerator 的失败不是 `_get_chain_transform()` 顺序错误导致的简单 compose bug
- refrigerator 的 normalized asset 与当前 `phase1` 写盘实现一致
- 当前 `phase2` 顶层矩阵 audit 不足以覆盖 descendant override 失效
- 新 gate 已经能把这类失败显式报出来

当前**还没有**完成的事情：

- 还没有完成正式的 descendant override remap / compensation 修复
- 还没有证明修复后的 `normalize-only` 能让 refrigerator 清零
- 还没有重跑一轮通过新版 gate 的 `S1 normalize-only`
- 还没有任何证据支持“可以绕过这个问题直接进 dedup”

因此，当前结论的边界是：

我们已经知道“为什么失败”，也已经让系统能“准确报这个失败”，
但还没有完成“把这个失败修掉”。

# 为什么下一步是 remap，而不是 dedup

下一步必须先做 descendant override remap / compensation，而不是进入 `dedup`，
原因有四个。

## 1. 失败发生在 dedup 之前

这轮 run 是在 `normalize-only gate` 就停住的。

也就是说：

- 当前数据在进入 dedup 之前就已经不安全
- 如果此时继续跑 dedup，只会把后续问题和现有 normalize 问题混在一起

## 2. 根因已经定位在 normalize 语义缺口

现在不是“怀疑 normalize 可能有问题”，而是已经确认：

- scene-authored descendant xform override 在 normalize 后失效

这说明需要修的是 normalize 的建模边界，而不是 dedup。

## 3. 新 gate 已经明确把它定义为 normalize fail

新版 verdict 已经把失败原因从单纯的 pairwise outlier 扩展为：

- `pairwise_displaced_gt_0.01 = 1`
- `pre_c1_inert_descendant_xform_override_count = 1`
- `became_inert_descendant_xform_override_count = 1`

既然 gate 已经正式认定这是 normalize fail，就不应该在 gate fail 的情况下继续推进 dedup。

## 4. remap 是最小闭环修复路径

当前最小且正确的修复方向是：

- 在 `phase2` 识别 scene-authored descendant xform override
- 把它们 remap 成 normalize 后仍然有效的 scene-layer变换
- 再重跑 `S1 normalize-only`

只有当 refrigerator 在这条路径下清零，并且 gate 全绿，才有资格讨论 dedup。

# 当前建议

建议按下面顺序推进，不要跳步：

1. 保持新版 normalize gate 作为后续判定口径。
2. 实现 descendant xform override 的 remap / compensation 原型。
3. 只重跑 `S1 normalize-only`，验证 refrigerator 与 control。
4. 通过后再决定是否放行 dedup。

一句话总结：

这次不是“dedup 前有个小异常”，而是“normalize 还没有正确保留 scene 里一条真实生效的局部变换语义”。
所以，下一步必须是 remap 修复，而不是 dedup。

# 交接说明

本阶段调研已经完成，并且结论已经移交到后续文档：

- 规划：
  [descendant_override_remap_plan.md](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/docs/test0_smoke/20260312_074731/descendant_override_remap_plan.md)
- 执行：
  [execution.md](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/docs/test0_smoke/20260312_074731/execution.md)
- 测试：
  [test.md](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/docs/test0_smoke/20260312_074731/test.md)
- 总结：
  [summary.md](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/docs/test0_smoke/20260312_074731/summary.md)
