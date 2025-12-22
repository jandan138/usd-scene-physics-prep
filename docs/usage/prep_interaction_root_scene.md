# `/root` 场景交互 sim-ready：`prep_interaction_root_scene.py`（用法）

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../scripts/prep_interaction_root_scene.py
> - ../../scripts/isaac_python.sh
> - ../../scripts/list_draggable_prims.py
> - ../../scripts/oneoff_force_draggable.py
> - ../../scripts/oneoff_make_static_collider_only.py
>
> 总索引：../overview/docs_index.md

## 索引
- [适用场景](#适用场景)
- [快速开始](#快速开始)
- [参数说明](#参数说明)
- [推荐参数与经验默认](#推荐参数与经验默认)
- [输出与验证](#输出与验证)
- [与 `simready.py` 的关系](#与-simreadypy-的关系)

## 适用场景
当输入 USD 来自外部数据集，常见特征如下：
- 根节点是 `/root`（小写），而不是本仓库规范化后的 `/Root/Meshes`。
- 输入目录可能缺失 `Materials/`，或材质引用不满足 `simready.py` 的默认假设。
- 场景中可能存在大量空 Mesh（无 points/topology）或复杂子树（例如 door 动画）导致 PhysX cooking failure。
- 场景中可能有 `.glb` payload（无头 USD 环境可能无法解析 GLB），但 Isaac 运行时能解析。

此时推荐使用：`scripts/prep_interaction_root_scene.py`。
它会直接在原始 stage 的 prim path 上 author `UsdPhysics` 属性，生成一个“交互 sim-ready”输出 USD。

## 快速开始

### 1) 最小命令（只做动态刚体与碰撞）
```bash
cd /shared/smartbot/zzh/my_dev/usd-scene-physics-prep

./scripts/isaac_python.sh scripts/prep_interaction_root_scene.py \
  --input  /abs/path/to/scene.usd \
  --output /abs/path/to/scene_simready.usd \
  --root /root \
  --approx-dynamic convexHull
```

### 2) 推荐命令（静态环境用 triangle mesh，更稳）
```bash
./scripts/isaac_python.sh scripts/prep_interaction_root_scene.py \
  --input  /abs/path/to/scene.usd \
  --output /abs/path/to/scene_simready.usd \
  --root /root \
  --static /root/room \
  --static /root/obj_table \
  --exclude-prefix /root/room/Meshes/BaseAnimation/door \
  --approx-static none \
  --approx-dynamic convexHull
```

## 参数说明
- `--input`：输入 USD。
- `--output`：输出 USD（默认会在输入同目录生成 `*_interaction_dynamic.usd`）。
- `--root`：根 prim path（外部数据集通常是 `/root`）。
- `--static`：静态 collider-only 的 prim path（可重复）。
- `--exclude-prefix`：排除某些子树不绑定 collider（可重复），常用于避开 door cooking failure。
- `--dynamic-prefix`：把根下一级、名字匹配某些前缀的 prim 当作动态刚体候选（可重复）。默认包含 `obj_` 与 `__`。
- `--dynamic-name`：把根下一级、名字精确匹配的 prim 当作动态刚体候选（可重复）。默认包含 `_`。
- `--approx-static`：静态碰撞近似 token。
- `--approx-dynamic`：动态碰撞近似 token。

支持的近似 token：
- `convexHull`
- `convexDecomposition`
- `sdf`
- `meshSimplification`
- `none`（三角网格）

## 推荐参数与经验默认
- 静态环境（room/地面/大场景）：优先 `--approx-static none`，可显著减少“穿透→无限下落”。
- 动态对象：优先 `--approx-dynamic convexHull`（快且相对稳）；`convexDecomposition` 更贴合但更容易 cooking 不稳定，薄物体更可能异常。
- GLB payload 多 Mesh（如 `geometry_01`）：脚本会探测 `geometry_0/geometry_0` 与 `geometry_0/geometry_01..geometry_09` 并为它们 author collider。

## 输出与验证

### 输出是什么
输出 USD 是对输入 USD 的拷贝，并在输出层写入：
- 对静态对象：`CollisionAPI` + `MeshCollisionAPI(approximation=...)` 且不启用刚体。
- 对动态对象：`RigidBodyAPI` + `MassAPI(mass=1.0)`，并对子树 Mesh 绑定 collider。

### 验证 1：列出可拖拽候选（无需 GUI）
```bash
./scripts/isaac_python.sh scripts/list_draggable_prims.py \
  --input /abs/path/to/scene_simready.usd \
  --root /root
```

### 验证 2：典型排错入口
- `docs/operations/troubleshooting_interaction_preprocess.md`
- `docs/operations/troubleshooting_glb_payload_multimesh.md`

## 与 `simready.py` 的关系
- `set_physics/simready.py`：面向“规范化输出结构（/Root/Meshes + Materials/models/scenes）”，适合一键跑 clean+physics。
- `scripts/prep_interaction_root_scene.py`：面向“外部 `/root` 结构”，跳过 normalize/parse，直接在原始 stage 上做 physics author。
