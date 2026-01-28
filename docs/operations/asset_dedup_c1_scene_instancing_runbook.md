# 资产去重落地方案 C1 执行手册（Scene 侧引用归一 + Instancing）

> 目标读者：想把“重复几何”省掉，但又不想把场景搞坏的人
>
> 最后更新：2026-01-28

## 0. 你要解决的矛盾（先讲人话）

- `geom_only` 报告告诉你：“很多资产 USD 的几何其实一样”。
- 但你不能直接把这些资产 USD 合并/删除，因为：
  - 场景（`layout.usd`）引用的是“资产 USD 路径”；
  - 很多资产 USD 虽然几何一样，但它们的 **transform / 层级 / 材质绑定 / variant / 物理属性** 可能不同；
  - 你一合并，场景引用就会断或者摆放变了。

**方案 C1 的核心思想：不动场景里“每个实例的 transform”，只让它们共享同一个 prototype（同一份几何/资产定义）。**

落地效果：
- 场景打开外观不变（同位置同大小同朝向）。
- 重复资产带来的“重复加载/重复几何”显著减少。
- 你可以在验证通过后，再考虑删除“被场景完全不再引用”的重复资产。

如果你的最终目标是“把 transform/scale 都写回 layout.usd，并删除多余资产”：
- 这份手册的 C1 仍然适用，但 **Step 3 默认应走 3B（改引用 + 补偿）**。
- 直观理解：3B 的补偿把 old 资产内部的对齐/缩放差异，迁移成 scene 侧实例 prim 的 transform，从而让“同几何”真正收敛为“一份 canonical 资产”。
- canonical 资产本身如果还残留内部对齐 transform，也可以放到后续单独做“canonical 归一化（可选）”；不影响你先把重复资产删掉。

---

## 1. 术语对齐（避免讨论跑偏）

- **资产 USD**：`GRScenes-test1/GRScenes_assets/<category>/<uid>/usd/<uid>.usd`
- **场景 / layout.usd**：`GRScenes100/**/layout.usd`（或你们具体数据集里的 layout）
- **实例（instance）**：场景里摆放的一个物体（一个 prim + transform）
- **prototype / canonical**：某个重复组里你选择保留的“代表资产 USD”，其它实例都改为引用它
- **引用归一**：把场景里多个不同资产 USD 的引用路径，改成同一个 canonical USD 路径

---

## 2. 前置要求（必须满足）

1) 你需要先有 dedup 报告：

- **如果你的最终目标是“几何一致就视为同一个资产（一个 prototype）”，并且希望把 transform/scale 逐步迁移到 `layout.usd`**：通常应该 **以 `geom_only` 作为分组依据**：
  - `check_reports/test1_asset_mesh_dedup_geom_only.json`
  - 关键点：哪怕当前很多 transform/scale 写在 asset 里，你也可以通过 Step 3B（补偿）把这些差异迁到 scene 侧，从而让“同几何”真正收敛成“一份资产”。

- **`scale_only` 什么时候还有用**：
  - 你想先把风险压到最低，只合并那些“几何相同且 scale 也相同”的候选（减少补偿带来的复杂度）；
  - 或者你计划的落地策略并不打算做 Step 3B 补偿（不推荐），那就必须用 `scale_only` 避免错位。

2) 你需要决定本次试点范围：
- 强烈建议从 **单个类别、单个重复组** 开始（例如 plant 的一个组）。

3) 本手册默认你遵循一个最重要的安全原则：
- **先改场景引用并验证；验证通过之前，绝不删除资产。**

---

## 3. 总体流程（每一步都要“能回滚、能验证”）

建议按下面 6 步走。

### Step 1：挑一个“试点重复组”（只选 1 组）

目标：选一个你愿意手动检查的组，成员数量别太大（比如 5~20 个）。

怎么挑：
- 从 `check_reports/*geom_only*.summary.json` 的 `top_groups` 里选一个 plant/xxx 类别的组；
- 或者用你自己关心的资产路径，用查询脚本找它属于哪个组。

验收：
- 你能列出该组的 `usd_paths[]`（至少 2 个），并确认它们都在同一组。

回滚：
- 无需回滚（只是选定目标）。

【已完成：pilot 记录】

- 分组口径：`geom_only`
- dedup 组签名（sig）：`6da971de7b943008782a1f0fd6e5406fb526f4f1878e42be95ddcad9aa5f2bff`
- 组大小：`count=19`（全部为 `plant` 类）
- 试点命中的场景：`GRScenes-test1/GRScenes100/home/MVUCSQAKTKJ5EAABAAAAABI8_usd/layout.usd`

### Step 2：选 canonical（prototype）

目标：在这个重复组里选一个 canonical 资产 USD。

选择建议（按优先级）：
- 优先选：被引用次数最多的那个资产（通常更“标准”）；
- 如果组内存在 `mesh_count==0` 的空壳资产：不要选它；
- 如果组内存在材质明显不同的：先不要合到一起（说明其实不是“同一资产”）。

验收：
- 你明确记录：canonical 的 `usd_path` 是哪一个。

回滚：
- 无需回滚（只是记录）。

【已完成：pilot 记录】

- canonical 资产（prototype）：
  - `GRScenes-test1/GRScenes_assets/plant/00d53f94d6d634583187d4b3070f6d04/usd/00d53f94d6d634583187d4b3070f6d04.usd`
- 本次试点被替换的 old 资产：
  - `GRScenes-test1/GRScenes_assets/plant/39131e7f7bf511eb2e71baa304cf06ca/usd/39131e7f7bf511eb2e71baa304cf06ca.usd`
- mapping JSON（old → canonical）：
  - `check_reports/c1_pilot/pilot_mapping_plant_sig6da9.json`

### Step 3：在场景中做“引用归一”（最关键的一步）

目标：把场景里引用组内其它成员的地方，都改成引用 canonical，并确保摆放不变。

先讲一个结论：
- **“只改引用路径”并不总是安全。**
- 它只有在一个前提成立时才安全：
  - 该物体的最终摆放完全由 scene 侧（`layout.usd`）实例 prim 的 transform 决定；
  - 或者说：组内各资产 USD 自己内部的“额外对齐 transform”是相同/可忽略的。

为什么会不安全（用公式解释一次）：
- 物体的世界变换通常可以理解成：
  - `M_world = M_layout * M_assetInternal`
- 如果你把引用从 old 资产换成 canonical，但 `M_assetInternal` 不一样，就会错位/缩放不对。

因此 Step 3 分成两条路径：

推荐策略（你这条反馈我完全同意）：
- **默认优先走 3B（改引用 + transform 补偿）**：成本更高，但对“资产内部自带对齐 transform”的情况更稳。
- **只有在你已经明确验证组内 `M_assetInternal` 一致/不存在时，才走 3A（仅改引用路径）**：它是提速路径，不是默认路径。

#### Step 3A：仅改引用路径（可选提速：当且仅当满足前提）

你需要满足的前提：
- 该组内资产 USD 的“内部对齐 transform”一致（或都没有）。

你需要改动的对象（通常在 layout）：
- `references` / `payload` 里指向资产 USD 的路径

验收（强烈建议做两条）：
- 文本级验收：确认 old paths 被替换为 canonical；
- 运行级验收：打开场景外观不变（位置/旋转/缩放一致），且没有大量 missing reference。

回滚：
- 最简单：用 git（或你们的备份目录）回退 `layout.usd` 的改动。

#### Step 3B：改引用 + transform 补偿（默认推荐：当资产 USD 内部可能有不同的对齐 transform）

适用条件：
- 该组内资产 USD 虽然几何一样，但它们的摆放差异部分写在“资产 USD 内部”（例如资产根 prim 自带缩放/旋转/平移）。

操作思路（保持世界结果不变）：
1) 你仍然把引用从 old 换成 canonical。
2) 同时在 scene 侧把实例 prim 的 transform 乘上一个补偿矩阵，让最终 `M_world` 不变。

补偿矩阵的直觉公式：
- 目标是让：
  - `M_layout_new * M_canonicalInternal == M_layout_old * M_oldInternal`
- 所以可以设：
  - `M_layout_new = M_layout_old * M_oldInternal * inverse(M_canonicalInternal)`

验收：
- 运行级验收优先：打开场景，确认该物体外观与原来完全一致。

推荐工具化（先 dry-run）：
- `scripts/rewrite_layout_asset_refs_with_compensation.py`
  - 输入：一个 `layout.usd` + 一个 `{old_asset_usd: canonical_asset_usd}` 的 mapping JSON
  - 输出：改引用 +（默认）写补偿矩阵到实例 prim 的 `xformOp:transform`

【已完成：pilot 记录】

- dry-run（仅报告，不落盘）：
  - 命令：
    - `./scripts/isaac_python.sh scripts/rewrite_layout_asset_refs_with_compensation.py \
      --layout-usd GRScenes-test1/GRScenes100/home/MVUCSQAKTKJ5EAABAAAAABI8_usd/layout.usd \
      --mapping-json check_reports/c1_pilot/pilot_mapping_plant_sig6da9.json \
      --dry-run \
      --report-out check_reports/c1_pilot/pilot_layout_rewrite_dryrun_report.json \
      --preview 50`
  - 结果摘要（见报告 JSON `summary.counts`）：
    - `refs_changed=1`、`xform_compensated=1`
  - dry-run 报告：`check_reports/c1_pilot/pilot_layout_rewrite_dryrun_report.json`

- apply（输出到新文件，不改原文件）：
  - 重要建议：`--out-usd` 最好写到原 `layout.usd` 同目录，避免相对引用解析变化。
  - 命令：
    - `./scripts/isaac_python.sh scripts/rewrite_layout_asset_refs_with_compensation.py \
      --layout-usd GRScenes-test1/GRScenes100/home/MVUCSQAKTKJ5EAABAAAAABI8_usd/layout.usd \
      --mapping-json check_reports/c1_pilot/pilot_mapping_plant_sig6da9.json \
      --out-usd GRScenes-test1/GRScenes100/home/MVUCSQAKTKJ5EAABAAAAABI8_usd/layout.c1_pilot_dedup_v2.usd \
      --report-out check_reports/c1_pilot/pilot_layout_rewrite_apply_report_inplace_dir_v2.json \
      --preview 50`
  - 结果摘要（见报告 JSON `summary.counts`）：
    - `refs_changed=1`、`xform_compensated=1`
  - apply 报告：`check_reports/c1_pilot/pilot_layout_rewrite_apply_report_inplace_dir_v2.json`

- 关键验证（文本级）：确认 layout 中引用从 old 变为 canonical（相对路径保持 `../../../GRScenes_assets/...` 风格）：
  - old：`.../plant/39131e7f.../usd/39131e7f....usd`
  - new：`.../plant/00d53f94.../usd/00d53f94....usd`

- 关键验证（USD 级）：目标 prim 上有 `xformOp:transform`（补偿后的矩阵），并且可选设置了 `instanceable=true`。

实现备注（踩坑记录）：
- 旧版 USD Python binding 下，`Sdf.ReferenceListOp` 的 `explicitItems/prependedItems` 不能用 `.append()` 原地修改（会静默不生效）。
  - 因此工具脚本采用“构造新 list → 重新赋值给 bucket”的方式写回，才能保证引用真正被改写。

回滚：
- 回滚引用修改 + 回滚你加的补偿 transform。

【已完成：sig=6da9 扩展到更多 layout（batch 记录）】

- full mapping（组内 18→1，除 canonical 外其余成员全部映射到 canonical）：
  - `check_reports/c1_pilot/pilot_mapping_plant_sig6da9_full.json`
- 全库 layout 命中扫描（扫描 `GRScenes-test1/GRScenes100/**/layout.usd`）：
  - 扫描总数：`scanned_layouts=99`
  - 命中 layout 数：`hit_layouts=12`
  - 命中清单：`check_reports/c1_pilot/plant_sig6da9_layout_hits.json`
- batch dry-run / apply 产物：
  - 每个命中 scene 生成一份报告（12 份）：`check_reports/c1_pilot/sig6da9_batch/*_dryrun.json`、`check_reports/c1_pilot/sig6da9_batch/*_apply.json`
  - 每个命中 scene 生成一个输出 layout（不覆盖原文件，写同目录）：`layout.c1_sig6da9_dedup_v1.usd`
- 关键一致性验证（自动）：
  - 对 12 个输出 layout 中所有被补偿的 prim 做了“effective world matrix”一致性检查：`checked_prims=18`，全部通过（diff ≤ `1e-6`）

#### Step 3 前置小检查（强烈建议加在任何改动之前）

为了决定走 3A 还是 3B，建议你对该组抽样 2~3 个资产 USD 做一次检查：
- 在“只打开资产 USD 自己的 stage”时，检查它的资产根 prim（或顶层 xform）的 transform 是否一致。
- 如果不一致：**必须走 3B**（改引用 + 补偿）。
- 如果一致：你可以选择走 3A（更省事），但我仍建议你在第一个场景里先按 3B 走通一次，确认工具链/验收没坑，再考虑用 3A 提速。

补充说明：
- 本手册先把“怎么做补偿”讲清楚，后续我们真正执行时，可以先从一个重复组、一个场景开始，逐步验证。

### Step 4：开启 instancing（可选但推荐）

目标：让这些“重复实例”在 USD 层面真正共享 prototype。

做法：
- 对实例 prim 设置 `instanceable=true`（注意是实例 prim，不是 prototype 里的 prim）。

为什么可选：
- 即使不设置 instanceable，你“引用归一”也能减少资产数量；
- 但设置后通常能进一步减少内存/加载开销（取决于加载器/渲染器）。

验收：
- 场景外观不变。
- 如果你们有工具能查看 instancing 状态，确认这些 prim 进入 instancing。

回滚：
- 只回滚 `instanceable` 这条属性即可。

【已完成：pilot 记录】

- 本次试点直接在 Step 3B apply 时带了 `--set-instanceable`：
  - 报告摘要：`instanceable_set=1`
  - 作用 prim：`/Root/Meshes/Furnitures/plant/model_39131e7f7bf511eb2e71baa304cf06ca_0`

### Step 5：打开场景验证（必须）

目标：确认“看起来一样、跑起来一样”。

建议的检查清单：
- 外观：位置/旋转/缩放是否一致；是否有物体消失或材质变灰
- 层级：脚本是否还按 prim path 找得到（如果你们有这样的逻辑）
- 物理：碰撞体是否还在；质量/摩擦是否还正常（如果场景要跑物理）

验收：
- 你能明确说：这个组在至少 1~3 个典型 scene 里验证通过。

【已完成：pilot 记录】

- Isaac Sim 打开验证：
  - 打开文件：`GRScenes-test1/GRScenes100/home/MVUCSQAKTKJ5EAABAAAAABI8_usd/layout.c1_pilot_dedup_v2.usd`
  - 命令行：无明显报错（用户观察）
  - 人工检查 prim：`/Root/Meshes/Furnitures/plant/model_39131e7f7bf511eb2e71baa304cf06ca_0`
  - 结果：摆放/外观未见异常（用户确认）

- 自动一致性校验（矩阵级，避免“prim 自己 transform 变了但几何没对齐”的误判）：
  - 校验目标：比较“几何最终世界变换”是否一致：
    - `M_world_geom = M_prim_world * M_asset_internal`
  - 结果：对该 prim，`max_abs_diff(effective_world) = 0.0`，判定 OK。

回滚：
- 如果出现问题：先回滚 Step 3（引用归一），再回滚 Step 4（instanceable）。

### Step 6：删除“已完全无引用”的重复资产（最后做）

目标：在确认所有 scene 都不再引用那些 old 资产 USD 后，才删除对应资产目录。

关键风险：
- 不仅是 scene 会引用资产；其它资产也可能引用其它资产（嵌套引用）。

推荐做法：
1) 先跑“引用检查”确认没有任何 USD 还在引用这些 old paths
2) 再删除（或先移动到 `*_bak/` 做软删除）

验收：
- 扫描全库后没有 missing reference
- 重新打开典型场景没问题

回滚：
- 如果你做的是“移动到 bak”，回滚就是移回来
- 如果你做了永久删除，回滚就只能从备份/版本库恢复

【已完成：sig=6da9 的 Step 6（soft delete）记录】

1) 把输出文件“落地”为正式文件（并保留可回滚备份）：
- `layout.usd`：已从 `layout.c1_sig6da9_dedup_v1.usd` 提升为正式 `layout.usd`，原文件备份为 `layout.pre_c1_sig6da9.<stamp>.usd`
  - 操作清单：`check_reports/c1_pilot/sig6da9_batch/promote_to_layout_usd_report.json`
- `start_result_navigation.usd` / `start_result_interaction.usd`：同样执行了改引用 + 补偿并就地替换，原文件备份为 `*.pre_c1_sig6da9.<stamp>.usd`
  - 操作清单：`check_reports/c1_pilot/sig6da9_batch/promote_start_results_report.json`

2) 引用检查（确保“活跃数据”不再引用 old 资产）：
- layout 扫描：`check_reports/c1_pilot/sig6da9_batch/post_promote_layout_scan.json`（hit=0）
- 全库 USD 扫描（排除 `*pre_c1_sig6da9*` 备份文件）：`check_reports/c1_pilot/sig6da9_batch/post_promote_full_usd_scan_excluding_backups.json`（hit=0）

3) soft delete（移动旧资产目录，而非永久删除）：
- 已将 sig=6da9 的 18 个 old plant 资产目录移动到：
  - `GRScenes-test1_bak/_dedup_assets/sig6da9_<stamp>/GRScenes_assets/plant/`
  - 详情报告：`check_reports/c1_pilot/sig6da9_batch/soft_delete_old_assets_report.json`
- 移动后 layout 再扫描：`check_reports/c1_pilot/sig6da9_batch/post_soft_delete_layout_scan.json`（hit=0）

---

## 4. 推荐的执行节奏（你要的“一步一步来”）

建议我们按下面节奏配合：
- 你执行 Step 1 后告诉我你选了哪个组（或把其中 2~3 个 usd path 发我），我帮你确认组的合理性。
- 你执行 Step 3 改完一批 layout 引用后，我们先做一次“引用检查”，再去 Isaac Sim 打开验证。
- 验证通过后再进入 Step 6。

---

## 5. 常见坑（提前说，省你时间）

- **空壳 USD（mesh_count==0）**：会在 `geom_only` 下形成很大的重复组，但它们不一定能被当作“可实例化的几何重复”。建议先剔除。
- **材质绑定不一致**：几何一样不代表材质一样；强行归一会导致材质变化。
- **物理/碰撞不一致**：某些资产 USD 里带 collider，归一到 canonical 后可能丢失或改变。
- **内部自带 transform**：如果资产 USD 内部有额外 transform，归一后需要确保 layout 上的 transform 仍然等价。

---

## 6. 下一步我们从哪里开始

当你准备开始 Step 1 时，把下面信息发我即可：
- 你想试点的 `layout.usd` 路径（1~3 个）
- 你挑的重复组里任意 2~5 个资产 `usd_path`
- 你倾向的口径（如果 transform 都在 layout：推荐 `geom_only`；否则 `scale_only` 更稳）
