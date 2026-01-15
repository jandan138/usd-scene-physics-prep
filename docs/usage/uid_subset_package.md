# UID 子集导出（只打包指定资产）

> Last Updated: 2026-01-15

这个文档说明如何从一个完整的 GRScenes 导出包（例如 `GRScenes-test1/`）中，按给定 `uid` 列表导出一个“可分发/可下载”的子集包。

核心目标：

- `GRScenes_assets/<category>/<uid>/`：**整目录复制**（未来可能包含 `.glb` 等非 USD 文件，不能只拷 USD）。
- `Material/`：只携带这些资产 USD 真实引用到的 **MDL + 纹理**（最小闭包）。
- 不包含 Scenes（`GRScenes100` 不拷贝场景文件；可选择创建空目录占位）。
- 不创建 symlink。

实现脚本：

- [scripts/build_uid_subset_package.py](../../scripts/build_uid_subset_package.py)

---

## 1. 基本用法

### 1.1 直接给 uid（重复传参）

```bash
python3 scripts/build_uid_subset_package.py \
  --src GRScenes-test1 \
  --dst sandbox/my_subset \
  --uid 00ca2676bbed26d6a39a968d99d61176 \
  --uid 46078b032b5f00e665294e43999d653e \
  --write-manifest \
  --verify \
  --create-empty-grscenes100
```

### 1.2 从文件读取 uid（推荐）

文件 `uids.txt` 每行一个 uid，支持 `#` 注释：

```text
# beds
00ca2676bbed26d6a39a968d99d61176
46078b032b5f00e665294e43999d653e
```

执行：

```bash
python3 scripts/build_uid_subset_package.py \
  --src GRScenes-test1 \
  --dst sandbox/my_subset \
  --uid-file uids.txt \
  --write-manifest \
  --verify \
  --create-empty-grscenes100
```

### 1.3 Dry-run（不落盘，只看依赖规模）

```bash
python3 scripts/build_uid_subset_package.py \
  --src GRScenes-test1 \
  --dst sandbox/my_subset \
  --uid-file uids.txt \
  --dry-run \
  --write-manifest
```

注意：`--dry-run` 不会写文件（包含 manifest），也不会执行 `--verify`。

---

## 2. 输出内容与目录结构

导出的子集包目录结构类似：

- `GRScenes_assets/<category>/<uid>/...`：来自源包的整目录复制
- `Material/mdl/*.mdl`：仅复制被引用的 MDL + 递归 MDL 依赖
- `Material/mdl/textures/**`：仅复制被引用纹理（保留 hash 分桶结构）
- `GRScenes100/`：如果加了 `--create-empty-grscenes100`，只创建空目录
- `subset_manifest.json`：如果加了 `--write-manifest`，会写出完整清单

---

## 3. 依赖解析逻辑（做了什么）

脚本会：

1. 在源包中定位每个 uid 的目录：`GRScenes_assets/<category>/<uid>/`
2. 扫描该 uid 目录下的所有 USD（`.usd/.usda/.usdc`），收集 asset-valued attribute 引用
3. 从 USD 引用中提取 `.mdl` 与纹理引用，并复制到子集包
4. 对复制到子集包的 MDL 做递归依赖闭包：解析 `import/using` 语句，把依赖模块对应的 `*.mdl` 一并复制

如果最终仍存在缺失依赖，脚本会返回非 0 退出码（用于自动化/CI）。

---

## 4. 常见问题

### 4.1 我应该用 `python3` 还是 `./scripts/isaac_python.sh`？

只要当前 Python 环境能 import `pxr.Usd` / `pxr.Sdf` 就可以运行。

- 如果 `python3 -c "from pxr import Usd, Sdf"` 失败：请用 Isaac 的 python：

```bash
./scripts/isaac_python.sh scripts/build_uid_subset_package.py --help
```

### 4.2 我只想复制 USD，不想复制 Material？

当前脚本的定位是“可用子集包”，默认会构建最小 `Material/`。
如果要做“仅资产目录镜像”，建议用 rsync/cp 直接复制 `GRScenes_assets` 子树。

---

## 5. 示例：挑选 20 个 USD > 10MB 的资产再导出子集

下面示例会从源包里扫描 `usd/<uid>.usd` 大小 >= 10MB 的资产，选 20 个 uid 写入文件，再打包。

```bash
python3 - <<'PY'
import os
SRC='GRScenes-test1/GRScenes_assets'
MIN=10*1024*1024
picked=[]
for cat in sorted(os.listdir(SRC)):
    cat_dir=os.path.join(SRC,cat)
    if not os.path.isdir(cat_dir):
        continue
    for uid in sorted(os.listdir(cat_dir)):
        usd=os.path.join(cat_dir,uid,'usd',f'{uid}.usd')
        if os.path.isfile(usd) and os.path.getsize(usd) >= MIN:
            picked.append(uid)
        if len(picked) >= 20:
            break
    if len(picked) >= 20:
        break

os.makedirs('sandbox', exist_ok=True)
with open('sandbox/uids_20_gt10mb.txt','w',encoding='utf-8') as f:
    for u in picked:
        f.write(u+'\n')
print('picked', len(picked))
PY

python3 scripts/build_uid_subset_package.py \
  --src GRScenes-test1 \
  --dst sandbox/subset_20_gt10mb \
  --uid-file sandbox/uids_20_gt10mb.txt \
  --write-manifest \
  --verify \
  --create-empty-grscenes100
```
