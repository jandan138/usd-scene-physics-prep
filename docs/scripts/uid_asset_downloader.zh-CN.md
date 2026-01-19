# UID 资产下载脚本（按 UID 构建子集包）

> 最后更新：2026-01-19
>
> 脚本：`scripts/build_uid_subset_package.py`

## 0. 这个脚本做什么

该脚本会从“全量 GRScenes 导出包”中，按给定的 `uid` 列表构建一个**可分发/可下载的子集包（subset package）**。

我们经常把它称为“下载脚本”，因为它的输入输出非常像下载行为：

- 输入：一组资产 UID
- 输出：一个目录，只包含这些 UID 对应的资产文件夹，以及让它们能正确渲染所必需的最小 `Material`（MDL + 贴图）

示例：

- `sandbox/subset_20_gt10mb/` 是一个子集目录，包含 **20 个资产**，且**不包含 Scenes**。

## 1. 什么时候该用它

当你满足以下条件时使用该脚本：

- 你已经在本地有一份**全量** GRScenes 包（例如 `GRScenes-test1/`），并且
- 你希望只拿其中一部分 UID 对应的资产，生成一个更小的 subset，且
- 你希望 subset 本身对渲染是自洽的（包含所需 MDL 与贴图）

注意：该脚本**不负责**从 HTTP/S3 等远程服务真正下载数据；它是“从全量包按 UID 抽取子集”的本地构建工具。

## 2. 输入与输出

### 2.1 输入：全量包根目录

`--src` 指向全量包根目录，至少需要包含：

- `GRScenes_assets/`
- `Material/mdl/` 与 `Material/mdl/textures/`

示例：

- `/abs/path/to/GRScenes-test1`

### 2.2 输入：UID 列表

提供 UID 的方式二选一或混用：

- 多次使用 `--uid <uid>`
- 使用 `--uid-file uids.txt`（一行一个 UID；支持 `#` 注释行）

### 2.3 输出：subset 根目录

`--dst` 指向输出目录（脚本会创建/更新）。输出 subset 的核心布局：

- `GRScenes_assets/<category>/<uid>/...`：**复制整个 UID 文件夹**（为未来 `glb/`、`urdf/` 等预留）
- `Material/mdl/*.mdl` 与 `Material/mdl/textures/**`：仅复制该子集实际需要的最小集合
- `subset_manifest.json`：可选（使用 `--write-manifest` 生成）

设计上：

- **不会复制 Scenes。**
- 可用 `--create-empty-grscenes100` 创建空的 `GRScenes100/` 目录占位（无 scenes 内容）。

## 3. 关键设计选择（给外部使用者的重要说明）

### 3.1 为什么是“整 UID 文件夹复制”？

即使今天你只关心 `usd/<uid>.usd`，资产目录未来可能包含：

- `glb/<uid>.glb`
- `urdf/<uid>.urdf` 及附属网格/材质
- 缩略图、额外元数据等

因此脚本选择复制整个目录：

- `GRScenes_assets/<category>/<uid>/`（目录下所有内容）

### 3.2 为什么 subset 需要包含 `Material/`？

USD 资产会引用 MDL 材质与贴图。为了让 subset 在离线环境下仍能正确渲染，脚本会构建最小 `Material/mdl` 依赖闭包：

1. 扫描资产 USD，收集 `.mdl` 与贴图引用
2. 对每个被引用的 `.mdl`，递归解析其 `import` / `using`，收集依赖的 MDL 模块
3. 复制该闭包所需的贴图文件（`Material/mdl/textures/**` 子树中的最小集合）

### 3.3 可选：为每个资产创建 `usd/textures` 软链接

某些工具/流程更喜欢在每个资产 USD 旁边有一个 `textures` 目录入口，因此可选创建：

- `GRScenes_assets/<category>/<uid>/usd/textures -> <dst>/Material/mdl/textures`

但由于软链接在 Windows / 某些拷贝工具下可能被“展开复制”，所以默认不强制。

启用：

- `--create-usd-textures-symlink`

如果已存在但指向错误位置，强制修复：

- `--force-usd-textures-symlink`

## 4. 工作流程（概览）

1. 对每个 UID，在 `GRScenes_assets/*/<uid>/` 下找到唯一匹配
2. 复制源包中的整个 UID 目录到目标 subset
3. 扫描源包中这些 UID 的 USD，收集 `.mdl`/贴图依赖
4. 将最小 `Material/mdl` 与 `Material/mdl/textures/**` 复制到 subset
5. （可选）`--verify`：对 subset 重新扫描，检查 MDL/贴图是否缺失
6. （可选）创建每个资产的 `usd/textures` 软链接
7. （可选）写出 `subset_manifest.json`

## 5. 常用命令

### 5.1 构建 subset（推荐：同时写 manifest + verify）

```bash
./scripts/isaac_python.sh scripts/build_uid_subset_package.py \
  --src /abs/path/to/GRScenes-test1 \
  --dst /abs/path/to/my_subset \
  --uid-file uids.txt \
  --write-manifest \
  --verify \
  --create-empty-grscenes100
```

### 5.2 构建 subset + 创建 `usd/textures` 软链接

```bash
./scripts/isaac_python.sh scripts/build_uid_subset_package.py \
  --src /abs/path/to/GRScenes-test1 \
  --dst /abs/path/to/my_subset \
  --uid-file uids.txt \
  --create-usd-textures-symlink
```

### 5.3 Dry-run（不写入文件，仅估算/检查依赖）

```bash
./scripts/isaac_python.sh scripts/build_uid_subset_package.py \
  --src /abs/path/to/GRScenes-test1 \
  --dst /abs/path/to/my_subset \
  --uid-file uids.txt \
  --dry-run
```

## 6. 运行环境与依赖

### 6.1 需要 pxr（USD Python bindings）

脚本依赖 `pxr.Usd` / `pxr.Sdf` 扫描 USD 引用。

如果你的系统 Python 中 `from pxr import Usd, Sdf` 不可用，请使用 Isaac Sim Python：

```bash
./scripts/isaac_python.sh scripts/build_uid_subset_package.py --help
```

### 6.2 文件系统注意事项

- subset 是通过文件复制产生的；请确保磁盘空间充足。
- 大规模 subset 建议在与 `--src` 同一文件系统上运行以减少拷贝耗时。

## 7. 常见问题

### 7.1 “UID not found / multiple matches”

脚本要求每个 UID 在 `GRScenes_assets/*/<uid>/` 下**唯一**匹配。

如果出现重复 UID，需要先修复源包或换用不同 UID。

### 7.2 缺 MDL / 贴图

当依赖闭包发现缺失文件时，脚本会返回非 0 退出码，方便自动化/CI 捕获。

建议加 `--verify` 获取更具体的缺失列表。

### 7.3 Windows 下载导致软链接被展开

某些工具（包括 VS Code Remote 的 Download）会把软链接“解引用”后复制真实目录，从而导致体积膨胀。

如果需要面向 Windows 分发，建议：

- 不启用 `--create-usd-textures-symlink`，或
- 在 Linux 端用 `tar.gz` 打包分发（以尽量保留软链接语义；Windows 端是否保留取决于解压工具与权限）。

## 8. 退出码（便于脚本化）

脚本会用退出码表达常见错误：

- `2`：参数/环境错误（例如缺 UID、缺 `pxr`）
- `3`：UID 未找到或不唯一
- `4`：`--verify` 发现缺失的 MDL/贴图引用
- `5`：依赖闭包扫描发现缺失的 MDL/贴图文件
