# SimBench（GRSceneUSD task9/task10）交互预处理：实战踩坑汇总

> 最后更新：2025-12-23
>
> 相关代码：
> - ../../scripts/isaac_python.sh
> - ../../scripts/prep_interaction_root_scene.py
> - ../../scripts/list_draggable_prims.py
> - ../../scripts/oneoff_force_draggable.py
> - ../../scripts/oneoff_make_static_collider_only.py
> - ../../scripts/inspect_usd_physics_props.py
> - ../../scripts/check_usd_external_assets.py
> - ../../scripts/oneoff_fix_mass_invalid_values.py
> - ../../scripts/oneoff_bind_physics_material.py
> - ../../scripts/oneoff_add_proxy_box_collider.py
> - ../../scripts/oneoff_stabilize_contact_ccd_damping.py
> - ../../scripts/oneoff_add_spoon_multi_box_proxy.py
> - ../../set_physics/simready.py
> - ../../set_physics/preprocess_for_interaction.py
>
> 总索引：../overview/docs_index.md

## 索引
- [适用范围](#适用范围)
- [核心结论（先看这一段）](#核心结论先看这一段)
- [典型问题与处理策略](#典型问题与处理策略)
- [推荐命令模板](#推荐命令模板)
- [自检方法（无需 GUI）](#自检方法无需-gui)

## 适用范围
本页记录的是一次针对外部数据集（SimBench 的 GRSceneUSD，task9/task10）做“交互物理预处理（可拖拽/可仿真）”时的实战踩坑与最终稳定策略。

这类输入场景的典型特征：
- 根节点常为 `/root`（小写），而不是本仓库默认管线使用的 `/Root/Meshes`。
- 可能缺失 `Materials/` 目录，且存在残留 `.mdl` 引用。
- 子树里可能包含大量“空 Mesh”（无 points/topology），以及某些子树（例如 door 动画）导致 PhysX cooking failure。
- 一部分对象通过 `.glb` payload 引入：在某些无头/离线 USD 环境中无法加载 GLB，导致 prim 变成 untyped/over，遍历也不可枚举。

## 核心结论（先看这一段）
- 对 `/root` 结构的外部场景，不建议硬走 `set_physics/simready.py`：目录结构/Materials 假设很容易不匹配。
- 直接使用 `scripts/prep_interaction_root_scene.py` 在“原始 stage”上 author 物理属性通常更稳。
- 对“静态环境”（例如 `/root/room`）用三角网格碰撞：`approx-static none`，可明显减少“穿透→无限下落”。
- 对“动态对象”优先用 `convexHull`（更稳更快）；`convexDecomposition` 更贴合但更容易 cooking 不稳定，且对薄物体更可能出现异常。
- GLB payload 在无头环境不可加载时：仍可在固定 prim path 上 author `RigidBodyAPI/CollisionAPI`，让 Isaac 运行时 payload 能加载时生效。

## 典型问题与处理策略

### 1) `simready.py` 报 `Materials/` 缺失
- **现象**：`FileNotFoundError: <input_dir>/Materials`。
- **原因**：`simready.py` 的前置阶段默认输入目录有 `Materials/`（且包含 `.mdl`）。
- **策略**：对这类外部输入，直接走 `scripts/prep_interaction_root_scene.py`。

### 2) 根结构不是 `/Root/Meshes`
- **现象**：默认脚本找不到预期 prim path。
- **策略**：以实际根（如 `/root`）为入口；静态/动态由命名规则或显式参数决定。

### 3) Isaac 环境缺少 `pxr.PhysxSchema`
- **现象**：`ImportError: cannot import name 'PhysxSchema' from 'pxr'`。
- **影响**：无法通过 PhysX 扩展 schema 设置 CCD/接触偏移等更细的物理参数。
- **策略**：
  - 仍以 `UsdPhysics.*`（CollisionAPI/MeshCollisionAPI/RigidBodyAPI/MassAPI）为主。
  - 若需要 CCD / contactOffset / depenetration clamp 等 PhysX 扩展字段：即使没有 `pxr.PhysxSchema` Python 绑定，也可以在 USD 中直接 author 对应的命名空间属性（例如 `physxRigidBody:enableCCD`、`physxCollision:contactOffset`）。
    - Isaac/PhysX 在运行时通常会读取这些属性（前提是对应扩展启用）。
    - 相关 one-off：`scripts/oneoff_stabilize_contact_ccd_damping.py`。

### 4) PhysX 报 `Mesh ... does not have points`
- **原因**：场景中存在无 points 或拓扑不一致的 Mesh；给它绑定 collider 会触发 PhysX 报错。
- **策略**：预处理时只对“points 非空且拓扑一致”的 Mesh 绑定 collider；其余跳过。

### 5) cooking failure（door 子树）
- **现象**：类似 `.../door/...` cooking failure。
- **策略**：默认排除 door 子树；若确实需要 door 碰撞，建议单独做简化碰撞而不是对整棵复杂子树做近似。

### 6) 引用层已启用 physics flag，RemoveAPI 不生效
- **现象**：输出层 RemoveAPI 后仍显示刚体/碰撞启用。
- **原因**：更强的引用层已写死 `physics:*Enabled=True`。
- **策略**：在输出层显式写 override：
  - `physics:rigidBodyEnabled=False`
  - `physics:collisionEnabled=False`
  再对需要启用的对象重新绑定。

### 7) GLB payload 无法加载（无头环境）导致“拖不动”
- **现象**：`Cannot determine file format for *.glb`，并且遍历不到真实 Mesh，导致脚本无法基于 points 绑定 collider。
- **策略**：采用“固定路径探测 + 强制写属性”的降级方案：
  - 刚体写在对象根 prim（例如 `/root/__05`）
  - collider 写在常见 mesh 路径（例如 `/root/__05/geometry_0/geometry_0`）
  - 即使 prim 是 untyped/over，也照样 Apply API 并 Set attribute，等 Isaac 运行时能加载 payload 时生效。

### 8) 物体“浮空”
- **常见原因**：碰撞近似（尤其 convexHull）外扩；以及同一对象意外写了多个 collider，叠加后会更明显。
- **策略**：
  - 保证每个对象只 author 一个 collider（优先最深 mesh 路径）。
  - 必要时对个别资产切换近似（例如 convexDecomposition 或 sdf）。

### 9) 物体“开物理后不见”（飞走/掉下去）
- **现象**：启用刚体后，薄物体（如眼镜）可能瞬间消失。
- **两类根因**：
  - **弹飞**：初始与环境/自身强穿插导致求解器强力纠正。
  - **穿透掉落**：collider cooking 失败/退化，实际没有有效碰撞体。
- **策略**（在无法使用 PhysxSchema/CCD 的前提下）：
  - 先尝试把该对象的 `physics:approximation` 从 `convexDecomposition` 切换到 `convexHull` 或 `sdf`。
  - 若是弹飞：优先检查初始位置是否与环境穿插；必要时手动把对象抬高一点再仿真。
  - 若是穿透：优先换更稳近似（convexHull）或简化碰撞。

> 相关 one-off：`scripts/oneoff_force_draggable.py` 可对单个对象快速切换 approximation（见下方命令模板）。

### 10) 案例：勺子接触桌面后“乱弹/弹起特别大”（flex_task1_cookie）

> 目标：记录一次“多轮 one-off 仍无法稳定”的排查全过程，便于后续复用与定位。

#### 场景与症状
- 场景：`flex_task1_cookie`（Collected_scene 目录下的 simready USD）。
- 症状：勺子与桌面接触后出现明显“穿透→被顶飞/弹起特别大/持续抖动”，即使把 `restitution=0` 也不明显收敛。

#### 核心结论（本次案例）
- **“弹很大”不一定是 restitution**：常见是**接触穿透较深**后，求解器使用 depenetration（分离修正）把物体推出去，视觉上像“被弹飞”。
- **双边材质必须绑在“真正参与碰撞的 collider”上**：如果替换了桌面 collider（proxy box），但忘记绑定 physics material，会回退到默认材质/默认 combine 行为，表现依然可能“很弹”。
- **凸分解并不总是更稳**：对薄表面+小物体接触，复杂/碎片化 collider 更容易抖；少量 primitive proxy（box/capsule）往往更稳定。
- **仅靠 USD author 仍可能不够**：如果依然“乱弹”，下一步通常需要 Isaac/PhysX 运行时参数（substeps/solver iterations/contact offsets/CCD 配置）配合。

#### 过程记录：每轮修改、输出与仍存问题

1) 物理属性检查（定位对象与关键 prim path）
- 工具：`scripts/inspect_usd_physics_props.py`
- 目的：确认刚体 prim、实际 collider prim、`approximation`、质量属性、physics material 绑定是否生效。
- 发现：勺子刚体与桌面 collider 的实际“参与碰撞的 prim”需要逐层确认（mesh vs 父级 xform vs proxy prim）。

2) 修复勺子质量属性异常（MassAPI 强制 override）
- 工具：`scripts/oneoff_fix_mass_invalid_values.py`
- 动机：发现勺子存在无效 MassAPI 值（例如 COM / principalAxes / inertia 不合法），可能导致求解不稳定。
- 输出：生成 `*_massfixed.usd` 变体。
- 仍存问题：接触时仍可能出现明显弹起（说明并非仅质量属性导致）。

3) 绑定 Physics Material（restitution=0）但不影响渲染材质
- 工具：`scripts/oneoff_bind_physics_material.py`
- 做法：创建/复用 `/_PhysicsMaterials/pmat_no_bounce`，通过 `material:binding:physics` 绑定到目标 collider prim。
- 输出：生成 `*_physmat.usd` 变体。
- 仍存问题：弹起仍较大（说明“顶飞/穿透修正”占比可能更高）。

4) 桌面替换为 proxy box collider（禁用原三角网格碰撞）
- 工具：`scripts/oneoff_add_proxy_box_collider.py`
- 做法：根据桌面 world AABB 建 proxy Cube collider，并将原桌面 mesh `physics:collisionEnabled=False`，避免双 collider。
- 关键修复：父级存在缩放时，需把 world AABB 转回 parent-local（通过 world AABB 角点做 world→local 变换）确保 proxy 尺寸正确。
- 输出：生成 `*_box.usd` 变体。
- 仍存问题：弹起仍大。

5) 发现并修复：桌面 proxy collider 未绑定 physics material
- 发现方式：用 `scripts/inspect_usd_physics_props.py` 检查 `material:binding:physics`。
- 修复：把同一 `pmat_no_bounce` 绑定到桌面 proxy collider。
- 输出：生成 `*_box_physmat.usd` 变体。
- 仍存问题：仍有“乱弹”。

6) CCD + 阻尼 + depenetration clamp + contactOffset（不依赖 PhysxSchema 绑定）
- 工具：`scripts/oneoff_stabilize_contact_ccd_damping.py`
- 做法（示例）：
  - 在刚体 prim 上 author：
    - `physxRigidBody:enableCCD=True`
    - `physics:linearDamping/physics:angularDamping`
    - `physxRigidBody:maxDepenetrationVelocity=<较小值>`
  - 在 collider prim 上 author：
    - `physxCollision:contactOffset` / `physxCollision:restOffset`
- 输出：生成 `*_stable.usd`、`*_strongdamp.usd` 变体。
- 仍存问题：用户侧反馈仍“乱弹”（说明仅靠这些 USD author 仍不足，或运行时参数/初始姿态/接触几何仍在触发强 depenetration）。

7) 勺子替换为多 Box proxy collider（柄/头两段）
- 工具：`scripts/oneoff_add_spoon_multi_box_proxy.py`
- 做法：
  - 基于 mesh 在刚体局部空间的 relative AABB，沿最长轴切成两段，创建 2 个 Cube collider。
  - 关闭原 mesh 碰撞（避免 mesh 的 convexDecomposition 继续参与）。
  - 把 `pmat_no_bounce` 绑定到新 proxy colliders。
- 输出：生成 `*_spoonproxy2box.usd` 变体。
- 仍存问题：用户侧反馈该 box 方案仍不满足预期（可能需要更贴合的 capsule/多段 convexHull，或直接调运行时 substeps/solver）。

#### 建议的后续动作（超出纯 USD author 的部分）
- 在 Isaac/PhysX 运行时提高 substeps / solver iterations（比继续叠加碰撞体更有效）。
- 检查初始姿态是否与桌面存在穿插（穿插越大，第一帧 depenetration 越“顶飞”）。
- 若仍需 proxy：优先考虑 `Capsule`（接触最平滑）或 3 段以上 primitive 组合；同时控制 contactOffset/restOffset 和 depenetration clamp。

## 推荐命令模板

### 对 `/root` 场景做交互预处理（推荐默认）
```bash
cd /shared/smartbot/zzh/my_dev/usd-scene-physics-prep

./scripts/isaac_python.sh scripts/prep_interaction_root_scene.py \
  --input  /path/to/taskX/scene.usd \
  --output /path/to/taskX/scene_interaction_dynamic.usd \
  --root /root \
  --static /root/room \
  --static /root/obj_table \
  --exclude-prefix /root/room/Meshes/BaseAnimation/door \
  --approx-static none \
  --approx-dynamic convexHull
```

### One-off：对单个 GLB 对象强制可拖拽 + 切换近似
```bash
./scripts/isaac_python.sh scripts/oneoff_force_draggable.py \
  --input  /path/to/scene_interaction_dynamic.usd \
  --output /path/to/scene_interaction_dynamic_oneoff.usd \
  --rigid-prim /root/__09 \
  --collider-mesh /root/__09/geometry_0/geometry_0 \
  --approx convexHull
```

### One-off：将某对象设为静态 collider-only（不受重力）
```bash
./scripts/isaac_python.sh scripts/oneoff_make_static_collider_only.py \
  --input  /path/to/scene_interaction_dynamic.usd \
  --output /path/to/scene_interaction_dynamic_oneoff_static.usd \
  --prim /root/__04
```

## 自检方法（无需 GUI）

### 列出“可拖拽候选”
```bash
./scripts/isaac_python.sh scripts/list_draggable_prims.py \
  --input /path/to/scene_interaction_dynamic.usd \
  --root /root
```

### 快速检查某个对象是否 authored 刚体/碰撞
```bash
./scripts/isaac_python.sh - <<'PY'
from pxr import Usd
p='/path/to/scene_interaction_dynamic.usd'
stage=Usd.Stage.Open(p)

def v(path, attr):
    prim=stage.GetPrimAtPath(path)
    a=prim.GetAttribute(attr) if prim else None
    return None if not a else a.Get()

print('rb:', v('/root/__09','physics:rigidBodyEnabled'))
print('mass:', v('/root/__09','physics:mass'))
print('col:', v('/root/__09/geometry_0/geometry_0','physics:collisionEnabled'))
print('approx:', v('/root/__09/geometry_0/geometry_0','physics:approximation'))
PY
```

