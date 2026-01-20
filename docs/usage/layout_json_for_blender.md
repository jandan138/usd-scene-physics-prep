# 从 layout.usd 生成 layout.json，并在 Blender 中加载（基于 GLB 资产）

> 适用场景：你有一个“资产库/子集包”（例如 scene subset），其中 **所有物体资产**（`GRScenes_assets/**`）已经具备 `usd/<uid>.usd` 和 `glb/<uid>.glb`，但 **scene 侧**只有 `layout.usd`（例如 `GRScenes100/**/<scene_uid>_usd/layout.usd`）。
>
> 目标：以 `layout.usd` 为权威布局，生成一个 `layout.json`，让 Blender 通过导入 GLB 并应用位姿，得到“物体摆放布局一致”的场景。

如果你需要更深入的“流程原理 + 代码解析”，请看：
- [docs/usage/layout_usd_to_layout_json_deep_dive.md](layout_usd_to_layout_json_deep_dive.md)

---

## 你会得到什么

### ✅ 已覆盖（做到的）

- 从 `layout.usd` 中识别每个“物体实例 prim”引用到的资产 UID（仅 `GRScenes_assets/<category>/<uid>`）。
- 计算每个 prim 的 **世界变换**（world transform），并写入 JSON。
- JSON 里的每个实例都会指向对应的 GLB：
  - `GRScenes_assets/<category>/<uid>/glb/<uid>.glb`
- 支持严格校验：如果某个实例对应的 GLB 不存在可直接失败（`--strict`）。

### ❌ 未覆盖（没做到的 / 本方案刻意不做）

- 不补齐 **scene-only** 内容：场景 USD 中若有不通过 `GRScenes_assets` 引用的几何、灯光、相机、环境等，本方案会忽略。
- 不做坐标轴转换（例如 USD Y-up → Blender Z-up）。后续需要时可在 Blender importer 层统一加。
- 不展开 `PointInstancer`（按约定暂不需要）。
- 不处理 USD 的 layer/variant 语义（本方案只输出“最终摆放的实例列表”）。

---

## 前置条件

1. 目录结构（以 subset root 为例）

- `subset_root/GRScenes_assets/<category>/<uid>/usd/<uid>.usd`
- `subset_root/GRScenes_assets/<category>/<uid>/glb/<uid>.glb`
- `subset_root/GRScenes100/<scene_category>/<scene_uid>_usd/layout.usd`

2. 运行环境

- 生成 `layout.json` 需要 `pxr`（USD Python bindings）。建议直接用本仓库的 Isaac wrapper 运行：
  - `./scripts/isaac_python.sh ...`

---

## Step 0：确认物体 GLB 已存在（可选）

如果你的资产包里物体还只有 USD，没有 GLB，可以先批量生成：

- 参考文档：[docs/usage/usd_to_glb_in_subset.md](usd_to_glb_in_subset.md)
- 示例命令：

```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
python3 scripts/convert_subset_usd_to_glb.py \
  --subset-root sandbox/scene_subset_<SCENE_UID> \
  --jobs 8 \
  --report check_reports/scene_subset_<SCENE_UID>_glb_report.json
```

---

## Step 1：从 layout.usd 生成 layout.json

使用脚本：[scripts/generate_layout_json_from_usd.py](../../scripts/generate_layout_json_from_usd.py)

### 最常用

```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
./scripts/isaac_python.sh scripts/generate_layout_json_from_usd.py \
  --layout-usd sandbox/scene_subset_<SCENE_UID>/GRScenes100/<scene_category>/<SCENE_UID>_usd/layout.usd \
  --subset-root sandbox/scene_subset_<SCENE_UID> \
  --strict
```

默认输出：与 `layout.usd` 同目录的 `layout.json`。

### 常用参数说明

- `--layout-usd`：场景布局 USD（权威输入）。
- `--subset-root`：资产库根目录（包含 `GRScenes_assets` 和 `GRScenes100`）。
  - 省略时脚本会尝试从 `--layout-usd` 向上自动推断。
- `--out`：自定义输出路径（默认 `layout.usd` 同级 `layout.json`）。
- `--strict`：任一实例缺 GLB 则立即失败。
- `--skip-missing-glb`：缺 GLB 的实例直接跳过（不建议默认用，除非你能接受缺物体）。

### 输出统计

脚本会打印：
- `instances_total`：识别到的实例总数
- `instances_with_glb`：能在本地找到 GLB 的实例数
- `instances_missing_glb`：缺 GLB 的实例数

---

## Step 2：在 Blender 中加载 layout.json（参考脚本）

Blender 不能“直接打开 JSON 变成场景”，你需要一个很薄的 importer：
- 读 `layout.json`
- 对每条 instance：导入 GLB，然后把导入的对象整体放到 instance 的 world matrix 上

下面给一个可直接用的最小示例（Blender → Scripting 面板运行）。

> 注意：本示例不做坐标轴转换；是否需要单位缩放请看下方「单位」说明。

```python
import json
import os
import bpy
from mathutils import Matrix

# 1) 填路径
subset_root = "/abs/path/to/scene_subset_<SCENE_UID>"
layout_json = os.path.join(subset_root, "GRScenes100", "home", "<SCENE_UID>_usd", "layout.json")

with open(layout_json, "r", encoding="utf-8") as f:
    data = json.load(f)

# 可选：单位缩放（USD metersPerUnit 可能不是 1.0）
meters_per_unit = (data.get("units") or {}).get("meters_per_unit")
scale_to_meters = 1.0
# 如果你发现 Blender 里尺寸/位移偏大或偏小，可以手动启用：
# scale_to_meters = float(meters_per_unit) if meters_per_unit else 1.0

# 放到一个 collection 里方便管理
coll_name = f"layout_{data.get('scene_uid') or 'scene'}"
if coll_name in bpy.data.collections:
    coll = bpy.data.collections[coll_name]
else:
    coll = bpy.data.collections.new(coll_name)
    bpy.context.scene.collection.children.link(coll)

for inst in data.get("instances", []):
    glb_rel = inst["glb"]
    glb_abs = os.path.join(subset_root, glb_rel)
    if not os.path.isfile(glb_abs):
        print("missing glb:", glb_abs)
        continue

    m = inst["transform"]["matrix_row_major"]
    mw = Matrix((
        (m[0],  m[1],  m[2],  m[3]),
        (m[4],  m[5],  m[6],  m[7]),
        (m[8],  m[9],  m[10], m[11]),
        (m[12]*scale_to_meters, m[13]*scale_to_meters, m[14]*scale_to_meters, m[15]),
    ))

    # 导入前记录对象集合，用差分拿到新导入的对象
    before = set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=glb_abs)
    after = set(bpy.data.objects)
    new_objs = list(after - before)

    # 用一个 empty 作为根，把本次导入的所有对象挂到它下面
    empty = bpy.data.objects.new(inst.get("name") or inst.get("uid") or "inst", None)
    coll.objects.link(empty)
    empty.matrix_world = mw

    for obj in new_objs:
        # 把新导入对象移入 collection，并 parent 到 empty
        if obj.name not in coll.objects:
            coll.objects.link(obj)
        obj.parent = empty

print("done")
```

---

## layout.json 格式（v1）

- `schema_version`: 当前为 1
- `subset_root`: 绝对路径（生成时写入，方便追溯）
- `scene_uid`, `scene_category`
- `source_layout_usd`: 相对 subset_root 的路径
- `units.meters_per_unit`, `up_axis`
- `instances[]`：每条实例包含：
  - `prim_path`, `name`
  - `category`, `uid`
  - `glb`：相对 subset_root 的 GLB 路径
  - `transform.matrix_row_major`：长度 16 的 row-major 4×4 world matrix

---

## 常见问题排查

- `instances_total = 0`
  - 说明 `layout.usd` 没有发现指向 `GRScenes_assets/<category>/<uid>` 的引用路径。
  - 先检查 `layout.usd` 是否是你要的那个布局 USD；以及它是否引用到了 `GRScenes_assets`。

- `--strict` 报错缺 GLB
  - 先用 [docs/usage/usd_to_glb_in_subset.md](usd_to_glb_in_subset.md) 补齐 GLB。

- Blender 导入后位置不对/大小不对
  - 先看 `layout.json` 里的 `units.meters_per_unit`，必要时在 Blender importer 里把平移量乘以该系数。
  - 坐标轴转换暂不做；若未来遇到 Y-up，可在 importer 层统一加转换矩阵。
