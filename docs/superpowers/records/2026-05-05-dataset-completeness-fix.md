---
title: Dataset Completeness Fix - Material Directory Copy
code_reference:
  - scripts/c1_phase2_merge_scan_delete.py
  - scripts/orchestrate_c1_parallel.py
created_at: 2026-05-05
updated_at: 2026-05-06
maintainer: OpenCode
status: design
---

# Dataset Completeness Fix - Material Directory Copy

## Problem

The parallel C1 pipeline output dataset at `/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/` was missing the `Material/` directory (29G) that exists in the serial workspace. 

**Root cause**: Phase 2 (merge + scan + delete) only processed scene files and asset references. It did not copy non-asset data directories like `Material/`.

**Impact**: Scene files reference materials via paths like `Material/mdl/MI_*.mdl`. Without the Material directory, these references would be broken if the serial workspace is ever moved or deleted.

## Solution

### Step 1: Remove Temporary Symlink

A symlink was temporarily created as a quick fix. Remove it before doing the real copy:

```bash
rm /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/Material
```

### Step 2: Copy Material Directory

Use `rsync` with archive mode and progress display:

```bash
rsync -avP \
  /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/dataset/Material/ \
  /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/Material/
```

**Why rsync**:
- `-a`: archive mode (preserves permissions, timestamps, symlinks)
- `-v`: verbose output
- `-P`: show progress and enable partial transfer (resumable)

### Step 3: Verify

Check that:
1. The directory exists and is not a symlink
2. Contains `mdl/` subdirectory
3. File count matches source

```bash
# Verify it's a real directory, not a symlink
ls -ld /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/Material

# Check contents
ls /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/Material/mdl/ | wc -l
```

## Prevention

For future parallel pipeline runs, Phase 2 should either:
1. Copy non-asset directories (Material, textures, etc.) as part of the workspace setup, OR
2. Add a post-processing step that verifies and copies missing directories

## Execution Results

**Date**: 2026-05-05
**Executor**: OpenCode

### Verification
- Source file count: 22,204
- Destination file count: 22,204
- Directory type: Real directory (`drwxr-xr-x`, not symlink)
- `mdl/` subdirectory: Present
- Total size: 29G

### Status
- [x] Symlink removed
- [x] Material copied via rsync (30,136,223,096 bytes sent, ~41MB/s)
- [x] File count verified (22,204 = 22,204)
- [x] Design document updated (`2026-04-29-c1-parallel-pipeline-design.md`)
- [x] Record document created

### Notes
Serial workspace remains untouched. Parallel workspace is now fully self-contained.
All material references in scene files will resolve correctly.

## Step 4: Cleanup Layout Intermediate Files

After Phase 2, each scene directory contains dozens of intermediate `layout.*.usd` files (backups, per-category iterations, etc.). These are not needed in the final dataset.

### Script

`scripts/cleanup_layout_intermediates.sh` — safely removes all `layout.*.usd` while keeping only `layout.usd`.

```bash
# Preview (dry run)
./scripts/cleanup_layout_intermediates.sh --dry-run

# Execute
./scripts/cleanup_layout_intermediates.sh
```

**Safety features**:
- Verifies `layout.usd` exists in each scene directory before deleting anything
- Reports space to be freed
- Shows progress every 500 files

### Execution Results

- **Files deleted**: 9,138 intermediate layout files
- **Space freed**: 14GB → 162MB per scene directory
- **Final state**: Each scene directory contains only `layout.usd`

## Step 5: MDL Import Fix

The Material directory copied from serial workspace contains MDL files with
absolute imports (`import ::KooPbr::*`) which cause Isaac Sim to render
materials as solid red. These need to be converted to relative imports.

### Script

`scripts/fix_mdl_absolute_imports.py` — batch-fixes MDL absolute imports.

```bash
# Dry run
python scripts/fix_mdl_absolute_imports.py \
  --mdl-dir /cpfs/user/.../dataset/Material/mdl \
  --dry-run

# Apply
python scripts/fix_mdl_absolute_imports.py \
  --mdl-dir /cpfs/user/.../dataset/Material/mdl
```

### Execution Results

- **Files scanned**: 1,566
- **Files modified**: 1,566
- **Replacements**: 3,683
- **Residual absolute imports**: 0
- **Errors**: 0

**11 import patterns replaced**:
- `import ::KooPbr::KooMtl;` → `using .::KooPbr import KooMtl;`
- `import ::KooPbr_maps::KooPbr_bitmap;` → `using .::KooPbr_maps import KooPbr_bitmap;`
- ... (and 9 more patterns)

## Documentation Updates

1. Update `docs/superpowers/specs/2026-04-29-c1-parallel-pipeline-design.md` to mention:
   - Material directory handling
   - Layout intermediate cleanup
   - MDL import fix
2. Create this record document for traceability
3. Add cleanup script to repo: `scripts/cleanup_layout_intermediates.sh`
