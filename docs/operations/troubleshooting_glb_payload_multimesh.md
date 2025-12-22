# 排错：GLB payload 多 Mesh（`geometry_01` 等）未绑定 collider 导致穿透

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../scripts/prep_interaction_root_scene.py
> - ../../scripts/list_draggable_prims.py
> - ../../scripts/oneoff_force_draggable.py
>
> 总索引：../overview/docs_index.md

## 索引
- [现象](#现象)
- [根因](#根因)
- [如何确认（无头检查）](#如何确认无头检查)
- [修复策略](#修复策略)
- [推荐做法](#推荐做法)

## 现象
在 Isaac Sim 里看起来是“同一个 GLB（例如两本书）拖进场景两次”，但仿真时出现：
- 动态物体（例如眼镜）下落过程中会穿过某一本书。

对应到 prim path，常见形态是同一个对象子树里同时存在多个 sibling Mesh：
- `/root/__08/geometry_0/geometry_0`
- `/root/__08/geometry_0/geometry_01`

其中 `geometry_0` 可能有 collider，而 `geometry_01` 没有 collider，导致“看得到、但物理穿透”。

## 根因
在一些 SimBench/GRSceneUSD 资产里，GLB payload 解析后会产生多个 Mesh sibling（`geometry_01`, `geometry_02`, ...）。

如果预处理脚本的 GLB fallback 只对一个固定路径（例如 `geometry_0/geometry_0`）author 了 `CollisionAPI`：
- 那么其它 sibling Mesh（例如 `geometry_01`）就不会有 `physics:collisionEnabled=True`
- PhysX 不会为它生成碰撞体
- 结果就是：物体会穿过它

## 如何确认（无头检查）
你可以在输出 USD 上检查某个 Mesh 是否真的 authored 了 collision：

```bash
./scripts/isaac_python.sh - <<'PY'
from pxr import Usd
p='/path/to/scene_interaction_dynamic.usd'
stage=Usd.Stage.Open(p)

def show(path):
    prim=stage.GetPrimAtPath(path)
    print('\n==', path, '==')
    if not prim or not prim.IsValid():
        print('missing')
        return
    for n in ('physics:collisionEnabled','physics:approximation'):
        a=prim.GetAttribute(n)
        print(' ', n, ':', None if not a else a.Get(), '(authored)' if (a and a.HasAuthoredValueOpinion()) else '')

show('/root/__08/geometry_0/geometry_0')
show('/root/__08/geometry_0/geometry_01')
PY
```

如果 `geometry_01` 的 `physics:collisionEnabled` 是 `<no-attr>` 或 `None`，则它没有碰撞。

## 修复策略
推荐的策略是：
- **优先**对 payload 对象的“leaf mesh”逐个 author collider（`geometry_0/geometry_0` + `geometry_0/geometry_01..`）
- 只有当 leaf mesh 都不存在时，才退回给父级 `/geometry_0` author collider

这样可以：
- 覆盖“同一 payload 多 Mesh”的场景（避免漏碰撞）
- 同时避免对 parent+child 都 author collider 造成重复碰撞体（会让物体明显“浮空”）

本仓库的实现位置：
- `scripts/prep_interaction_root_scene.py`：GLB fallback 的 leaf mesh 探测与 collider 绑定
- `scripts/list_draggable_prims.py`：统计 collider 时的 leaf mesh probe（避免误报 colliders=1）

## 推荐做法
- 最推荐：重新运行 `scripts/prep_interaction_root_scene.py` 生成新的输出 USD。
- 如果你只想临时修某一个对象：可以用 `scripts/oneoff_force_draggable.py` 给目标 mesh path 单独补 collider/approximation（但这种做法更适合小范围验证）。
