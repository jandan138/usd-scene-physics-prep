---
title: Cleanup Empty door_UUID Directories Implementation Plan
created_at: 2026-05-13
updated_at: 2026-05-13
maintainer: Codex
status: planned
---

# Cleanup Empty door_UUID Directories Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove 31 empty `door_*` placeholder category directories from the final parallel workspace dataset.

**Architecture:** A Bash one-liner backed by pre/post verification. No Python script needed — the operation is a straightforward `rm -rf` of directories whose UIDs all resolve to the empty-MD5 hash (`d41d8cd98f00b204e9800998ecf8427e`), which we confirmed has zero scene references.

**Tech Stack:** Bash, `ls`, `rm`, `diff`

**Pre-flight facts:**
- 31 directories under `GRScenes_assets/door_{UUID}/`
- Each contains only `d41d8cd98f00b204e9800998ecf8427e/` (empty-MD5 annotation)
- Zero `layout.usd` files reference any `door_` path (verified above)
- Backup exists at `bak_full_20260506_052808/`

---

### Task 1: Pre-cleanup Snapshot

**Files:** N/A (verification only)

- [ ] **Step 1: Save directory listing before deletion**

```bash
ls -d /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/GRScenes_assets/door_* > /tmp/door_uuids_pre_cleanup.txt
wc -l /tmp/door_uuids_pre_cleanup.txt
```

Expected: `31 /tmp/door_uuids_pre_cleanup.txt`

- [ ] **Step 2: Run the deletion**

```bash
rm -rf /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/GRScenes_assets/door_*
```

- [ ] **Step 3: Verify deletion — directories gone**

```bash
ls -d /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/GRScenes_assets/door_* 2>&1
```

Expected: `ls: cannot access ... No such file or directory`

- [ ] **Step 4: Verify no side effects — non-door categories intact**

```bash
ls /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/GRScenes_assets/ | grep -v door_ | wc -l
```

Expected: `79` (110 total categories - 31 door = 79)

- [ ] **Step 5: Verify scene layouts still load cleanly (spot-check 3 scenes)**

```bash
python3 -c "
from pxr import Usd
from pathlib import Path
scene_dir = Path('/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset/GRScenes100')
scenes = sorted([d for d in scene_dir.iterdir() if d.is_dir()])[:3]
for s in scenes:
    layout = s / 'layout.usd'
    if layout.exists():
        stage = Usd.Stage.Open(str(layout))
        print(f'{s.name}: OK (prims={len(stage.GetPrims())})')
    else:
        print(f'{s.name}: NO layout.usd')
"
```

- [ ] **Step 6: Record result and commit**

```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep

# Add audit snapshot to records
cp /tmp/door_uuids_pre_cleanup.txt docs/records/

git add docs/records/door_uuids_pre_cleanup.txt docs/superpowers/plans/2026-05-13-cleanup-empty-door-uuids.md
git commit -m "cleanup: remove 31 empty door_UUID placeholder directories from parallel workspace"
```
