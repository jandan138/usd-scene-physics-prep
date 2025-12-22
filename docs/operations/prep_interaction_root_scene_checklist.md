# `/root` 交互预处理：自检与排错清单（`prep_interaction_root_scene.py`）

> 最后更新：2025-12-22
>
> 相关代码：
> - ../../scripts/prep_interaction_root_scene.py
> - ../../scripts/list_draggable_prims.py
> - ../../scripts/isaac_python.sh
>
> 总索引：../overview/docs_index.md

## 索引
- [跑完后应该看到什么](#跑完后应该看到什么)
- [常见现象与定位](#常见现象与定位)
- [推荐进一步阅读](#推荐进一步阅读)

## 跑完后应该看到什么
- 脚本 stdout 会打印：
  - `static_bound` / `dynamic_bound`
  - `skipped_meshes`（有无 points/topology 的 Mesh 会被跳过）
  - `forced_untyped_colliders`（GLB payload 降级时写入 collider 的 prim path）

建议最少做一次无头自检：
```bash
./scripts/isaac_python.sh scripts/list_draggable_prims.py \
  --input /abs/path/to/scene_simready.usd \
  --root /root
```

## 常见现象与定位

### 1) 动态对象拖不动
- 先用 `list_draggable_prims.py` 看该 prim 的 `colliders_in_subtree` 是否为 0。
- 若为 0：
  - 可能是 GLB payload 未被脚本探测到正确 mesh path。
  - 或者该对象其实没有被匹配为 dynamic（名字不符合 `obj_/__/ _`）。

### 2) 物体穿透某个可见 Mesh（例如 `geometry_01`）
- 通常是该 Mesh 没有 `physics:collisionEnabled=True`。
- 见专门排错页：`docs/operations/troubleshooting_glb_payload_multimesh.md`

### 3) 物体明显浮空
- 常见原因：碰撞近似外扩（convexHull）或重复 collider（同一对象多个 collider）。
- 确认同一 payload 对象是否对 parent+child 都写了 collider；推荐只给 leaf mesh 写 collider。

### 4) 启用刚体后“消失”（弹飞/掉落）
- 弹飞：多是初始穿插导致求解器强力纠正。
- 掉落：多是 collider cooking 失败/退化。
- 见：`docs/operations/troubleshooting_interaction_preprocess.md`（问题 11）。

## 推荐进一步阅读
- `docs/usage/prep_interaction_root_scene.md`
- `docs/modules/prep_interaction_root_scene.md`
- `docs/operations/troubleshooting_interaction_preprocess.md`
- `docs/operations/troubleshooting_glb_payload_multimesh.md`
