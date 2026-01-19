# UID Asset Downloader (build subset package)

> Last Updated: 2026-01-19
>
> Script: `scripts/build_uid_subset_package.py`

## 0. What the script does

This script creates a **downloadable subset package** from a full GRScenes export package, by selecting assets using their `uid`.

In practice, we often call this a “download script” because it behaves like:

- input: a list of asset UIDs
- output: a directory that contains **only those assets** (plus the minimal material library required to render them)

Example output subset (no scenes):

- `sandbox/subset_20_gt10mb/` is a subset that contains **20 assets** and **no scenes**.

## 1. When you should use it

Use this script when you:

- already have a **full** GRScenes package on disk (e.g. `GRScenes-test1/`), and
- want to build a smaller package containing only specific assets, and
- want the subset to be self-contained for rendering (i.e. include required MDL + textures)

If you need “real network download from a server”, this script is still useful as a reference for:

- required directory layout
- how to compute the minimal material dependency closure

…but it does not perform HTTP/S3 downloads by itself.

## 2. Inputs and outputs

### 2.1 Input: full package root

`--src` points to a full package root that contains at least:

- `GRScenes_assets/`
- `Material/mdl/` and `Material/mdl/textures/`

Example:

- `/abs/path/to/GRScenes-test1`

### 2.2 Input: asset UID list

Provide UIDs in either way:

- `--uid <uid>` repeated many times
- `--uid-file uids.txt` where `uids.txt` contains one UID per line (supports `#` comments)

### 2.3 Output: subset root

`--dst` points to a destination directory that will be created/updated.

The subset output layout is:

- `GRScenes_assets/<category>/<uid>/...`  (copied **entire UID folder**, future-proof for `glb/`, `urdf/`, etc.)
- `Material/mdl/*.mdl` and `Material/mdl/textures/**` (minimal set for the selected assets)
- `subset_manifest.json` (optional, if `--write-manifest`)

By design:

- **Scenes are not copied.**
- You may create an empty placeholder `GRScenes100/` directory with `--create-empty-grscenes100`.

## 3. Key design choices (important for external users)

### 3.1 Why “copy entire UID folder”?

Even if today you only care about `usd/<uid>.usd`, the asset folder may later include:

- `glb/<uid>.glb`
- `urdf/<uid>.urdf` and extra mesh files
- thumbnails, metadata, etc.

So the script copies the whole folder:

- `GRScenes_assets/<category>/<uid>/` (everything inside)

### 3.2 Why include `Material/`?

The USD assets reference MDL materials and textures.

To ensure the subset package can render correctly, the script builds a minimal `Material/mdl` library:

1. scan asset USD files and find referenced `.mdl` and texture paths
2. for each referenced `.mdl`, recursively parse its `import` / `using` statements to collect dependent MDL modules
3. copy only the referenced textures under `Material/mdl/textures/**`

### 3.3 Optional: per-USD `textures` symlink

Some workflows prefer creating a symlink next to each asset USD:

- `GRScenes_assets/<category>/<uid>/usd/textures -> ../../../../Material/mdl/textures`

This can make it easier to open a USD inside its folder.

It is optional because symlinks can be problematic when copying to Windows or when using tools that “dereference” symlinks.

Enable it with:

- `--create-usd-textures-symlink`

If an existing symlink points elsewhere and you want to fix it:

- `--force-usd-textures-symlink`

## 4. How it works (step-by-step)

1. For each requested UID, locate a unique match under:
   - `GRScenes_assets/<category>/<uid>/`
2. Copy the entire UID directory to the destination subset:
   - keeps any extra files that may exist (glb/urdf/etc.)
3. Scan USD files under each selected UID folder in the **source** package:
   - collects `.mdl` references and texture references
4. Copy the minimal material library into the subset:
   - `Material/mdl/*.mdl`
   - `Material/mdl/textures/**`
5. (Optional) Verify: rescan subset USD files and check that MDL/textures resolve.
6. (Optional) Create per-USD `textures` symlink.
7. (Optional) Write `subset_manifest.json` for traceability.

## 5. Quickstart commands

### 5.1 Build subset (recommended, with verification)

```bash
./scripts/isaac_python.sh scripts/build_uid_subset_package.py \
  --src /abs/path/to/GRScenes-test1 \
  --dst /abs/path/to/my_subset \
  --uid-file uids.txt \
  --write-manifest \
  --verify \
  --create-empty-grscenes100
```

### 5.2 Build subset + create USD textures symlinks

```bash
./scripts/isaac_python.sh scripts/build_uid_subset_package.py \
  --src /abs/path/to/GRScenes-test1 \
  --dst /abs/path/to/my_subset \
  --uid-file uids.txt \
  --create-usd-textures-symlink
```

### 5.3 Dry-run (estimate dependency size; no output written)

```bash
./scripts/isaac_python.sh scripts/build_uid_subset_package.py \
  --src /abs/path/to/GRScenes-test1 \
  --dst /abs/path/to/my_subset \
  --uid-file uids.txt \
  --dry-run
```

## 6. Requirements and environment

### 6.1 pxr (USD) requirement

The script needs `pxr.Usd` / `pxr.Sdf` to scan USD references.

If `python3 -c "from pxr import Usd, Sdf"` fails, use Isaac Sim Python wrapper:

```bash
./scripts/isaac_python.sh scripts/build_uid_subset_package.py --help
```

### 6.2 File system notes

- The subset is created via file copy; ensure you have enough space.
- For large subsets, prefer running on the same filesystem as `--src` to reduce copy time.

## 7. Common issues and fixes

### 7.1 “UID not found” or “multiple matches”

The script requires each UID to resolve to exactly one directory under `GRScenes_assets/*/<uid>/`.

If there are duplicates, fix the source package or pick a different UID.

### 7.2 Missing textures / MDL

The script exits non-zero if it detects missing MDL or texture files during dependency closure.

This usually indicates:

- incomplete source package
- broken USD references
- missing texture files on disk

Run with `--verify` to get a concrete list of missing references.

### 7.3 Windows copy expands symlinks

Some tools (including VS Code Remote “Download”) may dereference symlinks and copy the whole target directory.

If you plan to distribute to Windows users, consider:

- not enabling `--create-usd-textures-symlink`, or
- packaging with `tar.gz` on Linux to preserve symlinks.

## 8. Exit codes (for automation)

The script returns non-zero for common failure modes:

- `2`: missing args, missing source root, or missing `pxr` environment
- `3`: UID not found uniquely
- `4`: `--verify` found missing MDL/texture references
- `5`: dependency closure detected missing MDL/textures
