---
title: Dataset Completeness Fix Implementation Plan
code_reference:
  - scripts/c1_phase2_merge_scan_delete.py
  - scripts/orchestrate_c1_parallel.py
created_at: 2026-05-05
updated_at: 2026-05-05
maintainer: OpenCode
status: completed
---

# Dataset Completeness Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Copy the Material directory from serial workspace to parallel workspace and update all related documentation.

**Architecture:** Remove temporary symlink, use rsync to copy 29G Material directory, verify integrity, then update design docs and create a record document.

**Tech Stack:** bash, rsync, git

---

### Task 1: Remove Temporary Symlink

**Files:**
- Remove: `/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/Material` (symlink)

- [ ] **Step 1: Verify it's a symlink**

Run:
```bash
ls -ld /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/Material
```
Expected: `lrwxrwxrwx` at start of output

- [ ] **Step 2: Remove the symlink**

Run:
```bash
rm /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/Material
```

- [ ] **Step 3: Verify removal**

Run:
```bash
ls /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/Material 2>&1
```
Expected: `No such file or directory`

---

### Task 2: Copy Material Directory

**Files:**
- Create: `/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/Material/` (directory with contents)

- [ ] **Step 1: Run rsync**

Run:
```bash
rsync -avP \
  /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/dataset/Material/ \
  /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/Material/
```
Expected: Progress output showing files copied. May take 10-30 minutes for 29G.

- [ ] **Step 2: Verify it's a real directory**

Run:
```bash
ls -ld /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/Material
```
Expected: `drwxr-xr-x` at start (directory, not symlink)

- [ ] **Step 3: Verify mdl subdirectory exists**

Run:
```bash
ls -d /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/Material/mdl
```
Expected: Directory listing

- [ ] **Step 4: Compare file counts**

Run:
```bash
SRC_COUNT=$(find /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/dataset/Material/ -type f | wc -l)
DST_COUNT=$(find /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/Material/ -type f | wc -l)
echo "Source: $SRC_COUNT, Destination: $DST_COUNT"
```
Expected: Both counts equal

- [ ] **Step 5: Commit the fix**

```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
git add -A  # No files to add from workspace, but record the fix
git commit --allow-empty -m "fix: copy Material directory to parallel workspace

- Remove temporary symlink
- rsync -avP Material from serial workspace
- 29G directory now fully independent

Related to: parallel C1 pipeline output completeness"
```

---

### Task 3: Update Design Document

**Files:**
- Modify: `docs/superpowers/specs/2026-04-29-c1-parallel-pipeline-design.md`

- [ ] **Step 1: Add Material handling section**

Add after the "Phase 2: Merge + Scan + Delete" section (around line 200-250):

```markdown
### Post-Processing: Non-Asset Data Directories

After Phase 2 completes, verify that non-asset directories exist in the output:
- `Material/` — contains MDL material definitions referenced by scene files
- `Asset_annotation.json` — if applicable

If missing, copy from the baseline workspace before declaring the dataset complete.
```

- [ ] **Step 2: Update final status section**

Find the section describing the final dataset state and add:
```markdown
**Verification checklist:**
- [ ] `GRScenes100/` contains all scenes
- [ ] `GRScenes_assets/` contains all asset categories
- [ ] `Material/` directory exists and is a real directory (not symlink)
- [ ] Scene files can resolve material references
```

- [ ] **Step 3: Commit**

```bash
git add docs/superpowers/specs/2026-04-29-c1-parallel-pipeline-design.md
git commit -m "docs: update C1 parallel pipeline design with Material handling

- Add post-processing step for non-asset directories
- Add verification checklist to final status"
```

---

### Task 4: Create Record Document

**Files:**
- Create: `docs/superpowers/records/2026-05-05-dataset-completeness-fix.md` (already exists from design phase)

- [ ] **Step 1: Update the record with actual results**

Edit the existing file to add an "Execution Results" section:

```markdown
## Execution Results

**Date**: 2026-05-05
**Executor**: OpenCode

### Verification
- Source file count: [TO BE FILLED]
- Destination file count: [TO BE FILLED]
- Directory type: Real directory (not symlink)
- `mdl/` subdirectory: Present

### Status
- [x] Symlink removed
- [x] Material copied via rsync
- [x] File count verified
- [x] Design document updated
- [x] Record document created

### Notes
Serial workspace remains untouched. Parallel workspace is now fully self-contained.
```

- [ ] **Step 2: Commit**

```bash
git add docs/superpowers/records/2026-05-05-dataset-completeness-fix.md
git commit -m "docs: add execution results to dataset completeness fix record"
```

---

### Task 5: Regenerate Documentation Index

**Files:**
- Modify: `docs/INDEX.md`

- [ ] **Step 1: Run doc manager**

Run:
```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
python scripts/doc_manager.py --gen-index
```

- [ ] **Step 2: Validate docs**

Run:
```bash
python scripts/doc_manager.py --validate
```
Expected: No validation errors

- [ ] **Step 3: Commit**

```bash
git add docs/INDEX.md
git commit -m "docs: regenerate index after dataset completeness fix"
```

---

## Verification Summary

After all tasks complete, the parallel workspace should have:

```
/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/
├── GRScenes100/          # 99 scenes
├── GRScenes_assets/      # 114 categories
├── Material/             # Real directory, 29G
│   └── mdl/              # 1722 MDL files
└── [no symlink]
```
