# 交互预处理排错（SimBench/GRSceneUSD task10）

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../set_physics/simready.py
> - ../../set_physics/preprocess_for_interaction.py
> - ../../scripts/isaac_python.sh
> - ../../scripts/prep_interaction_root_scene.py
> - ../../scripts/list_draggable_prims.py
> - ../../scripts/oneoff_make_static_collider_only.py
>
> 总索引：../overview/docs_index.md

## 索引
- [背景](#背景)
- [补充：实战踩坑汇总（task9/task10）](#补充实战踩坑汇总task9task10)
- [问题 1：`simready.py` 报 `Materials/` 缺失](#问题-1simreadypy-报-materials-缺失)
- [问题 2：场景结构不是 `/Root/Meshes`](#问题-2场景结构不是-rootmeshes)
- [问题 3：Isaac 环境缺少 `pxr.PhysxSchema`](#问题-3isaac-环境缺少-pxrphysxschema)
- [问题 4：启用物理后报 `Mesh ... does not have points`](#问题-4启用物理后报-mesh--does-not-have-points)
- [问题 5：PhysX cooking failure（door 子树）](#问题-5physx-cooking-failuredoor-子树)
- [问题 6：负质量/无效惯性（negative mass / invalid inertia）](#问题-6负质量无效惯性negative-mass--invalid-inertia)
- [问题 7：引用层已经启用 physics flag，简单 RemoveAPI 不生效](#问题-7引用层已经启用-physics-flag简单-removeapi-不生效)
- [问题 8：GLB 导入对象（`__xx` / `_`）拖不动](#问题-8glb-导入对象__xx---_拖不动)
- [问题 8.1：GLB payload 多 Mesh（`geometry_01` 等）未绑定 collider 导致穿透](#问题-81glb-payload-多-meshgeometry_01-等未绑定-collider-导致穿透)
- [问题 9：新增可交互对象后“无限下落”】【静态环境 cooking 不稳定】](#问题-9新增可交互对象后无限下落静态环境-cooking-不稳定)
- [问题 10：仅此一次把某个对象设为静态 collider-only（不受重力）](#问题-10仅此一次把某个对象设为静态-collider-only不受重力)
- [问题 11：对象启用刚体后“消失”（飞走/掉下去）](#问题-11对象启用刚体后消失飞走掉下去)
- [推荐处理流程（task10 已验证）](#推荐处理流程task10-已验证)
- [验证与自检](#验证与自检)

## 背景
在对外部数据集（例如 SimBench 的 GRSceneUSD/task10）做交互物理预处理时，会遇到与本仓库默认 pipeline（主要面向 GRScenes 规范化输出）不同的数据组织方式：
- 根节点是 `/root`（小写），而不是 `/Root/Meshes`。
- 输入目录可能没有 `Materials/`，材质多为 `UsdPreviewSurface`，并且存在指向 `../../Materials/*.mdl` 的残留 asset 引用。
- 场景里存在大量“空 Mesh”（无 points/topology），以及某些子树（如 door 动画）在 PhysX cooking 时失败。

为适配这类数据，本仓库新增了脚本 `scripts/prep_interaction_root_scene.py`，用于直接对 `/root` 结构绑定 collider/rigid。

## 补充：实战踩坑汇总（task9/task10）
如果你希望以“案例汇总”的形式查看本次实战过程中遇到的典型问题、最终稳定策略、以及 one-off 工具使用方式，见：
- `docs/operations/simbench_interaction_preprocess_field_notes.md`

## 问题 1：`simready.py` 报 `Materials/` 缺失
**现象**
- 运行 `set_physics/simready.py` 时，报 `FileNotFoundError: <input_dir>/Materials`。

**原因**
- `simready.py` 的第一阶段会调用数据清洗/拆分逻辑，默认假设输入目录包含 `Materials/`（且其中包含 `.mdl`）。

**解决方式**
- 对这类“没有 Materials/、且不满足 target 目录结构”的场景：
  - 不走 `simready.py` 的 normalize/parse 流程。
  - 改用 `scripts/prep_interaction_root_scene.py` 直接在原始 stage 上绑定物理（只生成一个输出 USD）。

## 问题 2：场景结构不是 `/Root/Meshes`
**现象**
- `simready.py`、`preprocess_for_interaction.py` 等脚本的默认路径假设（如 `/Root/Meshes`）找不到对应 prim。

**原因**
- 外部数据集的 layer 结构与本仓库默认数据结构不同。

**解决方式**
- 统一以外部场景真实的根节点为入口（task10 为 `/root`），并按命名规则（如 `/root/obj_*`）做刚体绑定。

## 问题 3：Isaac 环境缺少 `pxr.PhysxSchema`
**现象**
- 在 Isaac Sim python 环境内运行脚本时报错：
  - `ImportError: cannot import name 'PhysxSchema' from 'pxr'`

**原因**
- 不同 Isaac/Kit 发行版本暴露的 USD Python bindings 不完全一致：有些环境只提供 `Usd/UsdGeom/UsdPhysics`，不提供 `PhysxSchema`。

**解决方式**
- 在 `scripts/prep_interaction_root_scene.py` 中仅使用 `UsdPhysics` API：
  - `UsdPhysics.CollisionAPI` + `UsdPhysics.MeshCollisionAPI(approximation=convexDecomposition)`
  - `UsdPhysics.RigidBodyAPI`
- 不依赖 `PhysxSchema` 的 mesh-merge collision / convex decomposition 参数扩展。

## 问题 4：启用物理后报 `Mesh ... does not have points`
**现象**
启用仿真后出现类似报错：
- `Provided mesh geom with a PhysicsCollisionAPI does not have points, collision will not be created.`

**原因**
- 场景中存在大量 Mesh prim 没有 `points`（或 points 为空）/拓扑不一致。
- 如果对这些 Mesh 绑定了 `CollisionAPI`，PhysX 会尝试创建 collider，进而报错。

**解决方式（已在脚本中实现）**
- 仅对“Mesh 且 points 非空 且 faceVertexCounts/faceVertexIndices 一致”的 Mesh 绑定 collider。
- 对无效 Mesh 直接跳过，并在脚本输出中统计 `skipped_meshes`。

## 问题 5：PhysX cooking failure（door 子树）
**现象**
- `UjitsoMeshCookingContext: cooking failure for .../door/...`

**原因**
- door 动画子树通常包含拓扑退化/极端复杂/不适合所选近似方式的几何，导致 PhysX cooking 失败。

**解决方式（默认策略）**
- 默认排除 `/root/room/Meshes/BaseAnimation/door` 子树，不对其绑定 collider（避免 cooking failure）。
- 如确实需要 door 碰撞：建议单独做“简化碰撞”（例如只给门框/门板加极简 primitive collider），不要直接对整棵子树做复杂近似。

## 问题 6：负质量/无效惯性（negative mass / invalid inertia）
**现象**
- `negative mass` / `invalid inertia tensor` 警告。

**原因**
- 数据集可能在引用层里写了不合法的质量属性（例如负质量）。

**解决方式（已在脚本中实现）**
- 对刚体对象：
  - 清理已有 mass 属性。
  - 在输出层强制写入 `physics:mass = 1.0`（作为保守默认），避免负质量触发 PhysX 警告。

## 问题 7：引用层已经启用 physics flag，简单 RemoveAPI 不生效
**现象**
- 你在输出层里移除了 `RigidBodyAPI`/`CollisionAPI`，但打开 USD 后 Isaac 里仍显示某些对象是刚体/有碰撞。

**原因**
- 这些属性可能来自更强的引用层（reference layer / sublayer）。
- 仅靠 `RemoveAPI(...)` 可能无法“覆盖掉引用层里已经写死的 `physics:*Enabled=True`”。

**解决方式（本仓库脚本采用的做法）**
- 在输出层显式写 override：
  - `physics:rigidBodyEnabled = False`
  - `physics:collisionEnabled = False`
- `scripts/prep_interaction_root_scene.py` 的策略是：先对 `/root` 子树递归强制关闭所有 physics flag，再对“明确要启用物理”的对象重新绑定 collider/rigid。

## 问题 8：GLB 导入对象（`__xx` / `_`）拖不动
**现象**
- 在 task10 里，类似这些路径的对象无法 Shift+左键拖拽（持续施力也不动）：
  - `/root/__06/...`
  - `/root/_/...`

**原因**
- 外部数据集的命名不遵循本仓库默认的 `obj_*` 规则。
- 预处理脚本最初只会把 `obj_*` 视为“动态刚体候选”，所以 `__xx`、`_` 没有被设置为刚体。

**解决方式（已在脚本中实现）**
- 在 `scripts/prep_interaction_root_scene.py` 中扩展动态匹配规则：
  - 动态前缀：默认包含 `obj_` + `__`（覆盖 `/root/__01..__10` 这类）
  - 动态精确名字：默认包含 `_`（覆盖 `/root/_`）

## 问题 8.1：GLB payload 多 Mesh（`geometry_01` 等）未绑定 collider 导致穿透
**现象**
- 场景里同一 payload 对象下可能出现多个 sibling Mesh（例如两本书）：
  - `/root/__08/geometry_0/geometry_0`
  - `/root/__08/geometry_0/geometry_01`
- 如果只给其中一个 Mesh author 了 collider，动态物体下落时会穿过另一个没有 collider 的 Mesh。

**处理方式**
- 详见独立页面：`docs/operations/troubleshooting_glb_payload_multimesh.md`

## 问题 9：新增可交互对象后“无限下落”】【静态环境 cooking 不稳定】
**现象**
- 你新增了一批动态对象 collider 后，这些对象在仿真中直接“穿过地面一直下落”。

**原因（常见）**
- 静态环境（例如 `/root/room`）使用了不稳定的近似方式（例如对大场景做 `convexDecomposition`），导致部分区域没有可靠的碰撞体。

**解决方式（task10 已验证）**
- 对静态环境（room）使用三角网格碰撞：`approximation = none`。
- 对动态对象使用更快更稳定的方式（通常 `convexHull` 就够用）。

## 问题 10：仅此一次把某个对象设为静态 collider-only（不受重力）
**目标**
- 某个对象保持碰撞，但不再是刚体（不受重力、不再被施力推动）。

**做法（已提供 one-off 脚本）**
- 使用 `scripts/oneoff_make_static_collider_only.py`：
  - 复制输入 USD 到输出
  - 对目标 prim 子树移除 `RigidBodyAPI`/质量相关属性
  - 在输出层显式写 `physics:rigidBodyEnabled = False` 覆盖引用层

示例：将 `/root/__04` 静态化（task10 已验证）
```bash
./scripts/isaac_python.sh scripts/oneoff_make_static_collider_only.py \
  --input  /shared/smartbot/jiamingda/data_code/simbench/MesaTask-USD/simbench_shared/GRSceneUSD/task10/scene_interaction_dynamic_v6.usd \
  --output /shared/smartbot/jiamingda/data_code/simbench/MesaTask-USD/simbench_shared/GRSceneUSD/task10/scene_interaction_dynamic_v6_oneoff_static__04.usd \
  --prim /root/__04
```

## 问题 11：对象启用刚体后“消失”（飞走/掉下去）
**现象**
- 薄物体（例如眼镜）启用刚体后，看起来“突然不见了”。

**常见原因**
- **弹飞**：初始姿态与环境/自身穿插，求解器在第一帧做 penetration correction，物体被强力推出视野。
- **穿透掉落**：碰撞体 cooking 失败或退化，导致实际没有有效 collider，一路穿过地面下落。

**在本仓库当前约束下的处理策略**
- 由于部分 Isaac 环境无法导入 `pxr.PhysxSchema`（无法稳定启用 CCD/接触偏移等 PhysX 扩展设置），优先用“更稳的近似 + 更干净的 collider”来规避：
  - 保证每个对象只 author 一个 collider（避免叠加导致异常接触）。
  - 对该对象切换 `physics:approximation`：优先试 `convexHull`；若仍异常可尝试 `sdf`。
  - 若确认是弹飞：检查初始 pose 是否与静态环境穿插；必要时在仿真前把物体抬高一点。

**快速 one-off（推荐）**
- 使用 `scripts/oneoff_force_draggable.py` 对单个对象切换近似（示例见 `docs/operations/simbench_interaction_preprocess_field_notes.md`）。

## 推荐处理流程（task10 已验证）
**目标规则**
- `/root/room`：静态 collider-only
- `/root/obj_table`：静态 collider-only（table 不动）
- `/root/obj_*`：刚体 + collider
- `/root/__*`：刚体 + collider（GLB 导入常见命名）
- `/root/_`：刚体 + collider（GLB 导入常见命名）

**生成命令（推荐：静态 room 用 `none`，动态用 `convexHull`）**
```bash
cd /shared/smartbot/zzh/my_dev/usd-scene-physics-prep

./scripts/isaac_python.sh scripts/prep_interaction_root_scene.py \
  --input  /shared/smartbot/jiamingda/data_code/simbench/MesaTask-USD/simbench_shared/GRSceneUSD/task10/scene.usd \
  --output /shared/smartbot/jiamingda/data_code/simbench/MesaTask-USD/simbench_shared/GRSceneUSD/task10/scene_interaction_dynamic_v6.usd \
  --root /root \
  --static /root/room \
  --static /root/obj_table \
  --exclude-prefix /root/room/Meshes/BaseAnimation/door \
  --approx-static none \
  --approx-dynamic convexHull
```

## 验证与自检
建议在 Isaac Sim 打开输出 USD 后：
- 启用物理仿真，观察 Console 是否还有大量 `no points` 或 cooking failure。
- 重点确认：`/root/obj_table` 不应成为刚体；`/root/obj_*` 应为刚体。

可用脚本快速自检（无需 GUI）：
```bash
./scripts/isaac_python.sh - <<'PY'
from pxr import Usd
p='/shared/smartbot/jiamingda/data_code/simbench/MesaTask-USD/simbench_shared/GRSceneUSD/task10/scene_interaction_dynamic_v3.usd'
stage=Usd.Stage.Open(p)

def get_bool_attr(prim, name):
    a=prim.GetAttribute(name)
    return None if not a else a.Get()

print('obj_table rigidBodyEnabled:', get_bool_attr(stage.GetPrimAtPath('/root/obj_table'), 'physics:rigidBodyEnabled'))
for name in ['obj_c86f38f1','obj_7d793037','obj_bcff950f','obj_b77be5e1']:
    prim=stage.GetPrimAtPath('/root/'+name)
    print(name,'rigidBodyEnabled:', get_bool_attr(prim,'physics:rigidBodyEnabled'), 'mass:', get_bool_attr(prim,'physics:mass'))
PY
```

**列出“可 Shift+左键拖拽”的刚体候选**

判定经验：在 Isaac 中“可持续施力拖动”通常至少满足：
- 目标 prim（或其祖先）`physics:rigidBodyEnabled=True`
- 其子树中至少存在一个 `physics:collisionEnabled=True` 的 collider

可以直接跑本仓库脚本（无需 GUI）：
```bash
./scripts/isaac_python.sh scripts/list_draggable_prims.py \
  --input  /shared/smartbot/jiamingda/data_code/simbench/MesaTask-USD/simbench_shared/GRSceneUSD/task10/scene_interaction_dynamic_v6.usd \
  --root /root
```

