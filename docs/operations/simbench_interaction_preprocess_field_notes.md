# SimBench（GRSceneUSD task9/task10）交互预处理：实战踩坑汇总

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../scripts/isaac_python.sh
> - ../../scripts/prep_interaction_root_scene.py
> - ../../scripts/list_draggable_prims.py
> - ../../scripts/oneoff_force_draggable.py
> - ../../scripts/oneoff_make_static_collider_only.py
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
- **策略**：脚本仅使用 `UsdPhysics.*`（CollisionAPI/MeshCollisionAPI/RigidBodyAPI/MassAPI），并通过更稳的近似与过滤规则规避 cooking 风险。

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

