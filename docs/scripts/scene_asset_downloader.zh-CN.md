# 场景 UID 下载脚本（场景 + 所有引用的资产 + 材质）

> 最后更新：2026-01-19
>
> 脚本：`scripts/build_scene_uid_subset_package.py`

## 这个脚本做什么

给定一个**场景 UID**（位于 `GRScenes100/**/<scene_uid>_usd/`），该脚本会构建一个可分发/可下载的 subset 目录，包含：

- 完整的场景文件夹：`GRScenes100/<category>/<scene_uid>_usd/**`
- 场景中引用到的**所有物体资产**（包含传递依赖）：`GRScenes_assets/<asset_category>/<asset_uid>/**`
- 让这些 USD 能正确渲染所需的最小材质库：
  - `Material/mdl/*.mdl`
  - `Material/mdl/textures/**`

输出 subset 会保持与原始全量包相同的顶层目录结构（`GRScenes100/`、`GRScenes_assets/`、`Material/`）。

## 输入与输出

- `--src`：全量 GRScenes 包根目录（例如 `GRScenes-test1/`）
- `--dst`：输出 subset 根目录
- `--scene-uid`：场景 UID，例如 `MV7J6NIKTKJZ2AABAAAAADA8`
- `--scene-category`（可选）：`home` 或 `commercial`（不填时自动探测）

## 资产 UID 是如何从场景里提取出来的

脚本会用 `pxr.Usd` 打开场景 USD，并从以下信息中提取引用到的 USD 路径：

- prim 的 `references`
- prim 的 `payloads`
- asset-valued attributes（指向 `.usd/.usda/.usdc` 的属性）
- used layers（兜底）

凡是匹配 `GRScenes_assets/<category>/<uid>/...` 的路径，都会被解析为一个资产 UID。

随后会做一次“传递闭包”：对每个资产的 USD 也继续扫描引用，从而把嵌套引用的资产也一起包含进 subset。

## 快速开始

```bash
./scripts/isaac_python.sh scripts/build_scene_uid_subset_package.py \
  --src /abs/path/to/GRScenes-test1 \
  --dst /abs/path/to/subset_scene \
  --scene-uid MV7J6NIKTKJZ2AABAAAAADA8 \
  --write-manifest
```

可选校验（检查 scene + assets 的 USD 中是否存在缺失的 MDL/贴图引用）：

```bash
./scripts/isaac_python.sh scripts/build_scene_uid_subset_package.py \
  --src /abs/path/to/GRScenes-test1 \
  --dst /abs/path/to/subset_scene \
  --scene-uid MV7J6NIKTKJZ2AABAAAAADA8 \
  --verify
```

## 备注

- 该脚本依赖 `pxr`（USD 的 Python bindings）。如果系统 Python 没有 `pxr`，请使用 `./scripts/isaac_python.sh`。
- `--verify` 如果报缺失文件，有可能是源包本身就缺（数据质量问题），不是 subset 构建逻辑的问题。
