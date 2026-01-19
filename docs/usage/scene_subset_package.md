# Scene 子集导出（按场景 UID 打包场景 + 所有引用资产）

> Last Updated: 2026-01-19

这个文档说明如何从一个完整的 GRScenes 导出包（例如 `GRScenes-test1/`）中，按给定 **scene uid** 导出一个“可分发/可下载”的子集包。

核心目标：

- `GRScenes100/<scene_category>/<scene_uid>_usd/**`：复制完整场景目录
- `GRScenes_assets/<asset_category>/<asset_uid>/**`：复制场景引用到的所有资产（包含传递依赖）
- `Material/`：只携带这些场景/资产 USD 真实引用到的 **MDL + 纹理**（最小闭包）
- 输出目录结构与原始包一致（便于直接替换/分发）

实现脚本：

- [scripts/build_scene_uid_subset_package.py](../../scripts/build_scene_uid_subset_package.py)

---

## 1. 基本用法

### 1.1 最常用（推荐）

```bash
./scripts/isaac_python.sh scripts/build_scene_uid_subset_package.py \
  --src GRScenes-test1 \
  --dst sandbox/subset_scene_MV7J6NIKTKJZ2AABAAAAADA8 \
  --scene-uid MV7J6NIKTKJZ2AABAAAAADA8 \
  --include-scene-usds layout-only \
  --write-manifest
```

说明：

- `--include-scene-usds layout-only`：只用 `layout.usd` 作为“解析引用资产”的入口，通常足够且更安静。
- `--write-manifest`：会生成 `subset_manifest.json`，便于追溯“这个 subset 里包含哪些资产”。

### 1.2 可选：校验 MDL/贴图引用是否缺失

```bash
./scripts/isaac_python.sh scripts/build_scene_uid_subset_package.py \
  --src GRScenes-test1 \
  --dst sandbox/subset_scene_MV7J6NIKTKJZ2AABAAAAADA8 \
  --scene-uid MV7J6NIKTKJZ2AABAAAAADA8 \
  --verify
```

注意：如果 `--verify` 报缺失，有可能是源包本身缺文件（数据质量问题），不一定是 subset 构建逻辑的问题。

### 1.3 Dry-run（不落盘）

```bash
./scripts/isaac_python.sh scripts/build_scene_uid_subset_package.py \
  --src GRScenes-test1 \
  --dst sandbox/subset_scene_MV7J6NIKTKJZ2AABAAAAADA8 \
  --scene-uid MV7J6NIKTKJZ2AABAAAAADA8 \
  --dry-run
```

---

## 2. 输出目录结构

导出的子集包目录结构类似：

- `GRScenes100/<scene_category>/<scene_uid>_usd/**`
- `GRScenes_assets/<asset_category>/<asset_uid>/**`
- `Material/mdl/*.mdl`
- `Material/mdl/textures/**`
- `subset_manifest.json`（如果启用 `--write-manifest`）

---

## 3. 依赖解析逻辑（它做了什么）

脚本会：

1. 在源包中定位 scene 目录：`GRScenes100/<category>/<scene_uid>_usd/`
2. 扫描场景 USD，提取引用到的 `GRScenes_assets/<category>/<uid>/...`
   - 覆盖 `references` / `payloads` / asset-valued attributes / used layers
3. 对每个 asset UID 的 USD 继续扫描，形成“传递闭包”（防止资产嵌套引用）
4. 复制所有资产的整目录到 subset
5. 扫描 subset 需要的 MDL/贴图，并复制最小 `Material/` 依赖闭包

---

## 4. 环境与依赖

脚本依赖 `pxr.Usd` / `pxr.Sdf`。

- 如果系统 python 能 `from pxr import Usd, Sdf`：可以直接用 `python3` 跑。
- 否则推荐使用 Isaac 环境：`./scripts/isaac_python.sh ...`
