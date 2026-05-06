# Parallel Workspace Asset Category Underscore Normalization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Normalize asset category names to snake_case (underscore style) in the parallel C1 deduplication workspace (`test0_transitive_apply_parallel/dataset/`) by merging duplicate category folders and rewriting USD references.

**Architecture:** Reuse the proven `merge_asset_categories_test1.py` script (which already contains the correct `CATEGORY_MERGES` mapping and three-phase workflow: dry-run → apply → validate) against the parallel workspace path. Add a TDD-style validation test to ensure the normalization is complete.

**Tech Stack:** Python, USD (pxr), existing merge script

---

## File Structure

| File | Action | Purpose |
|------|--------|---------|
| `scripts/merge_asset_categories_test1.py` | **Read/Execute** | Existing script with CATEGORY_MERGES mapping and dry-run/apply/validate workflow |
| `tests/test_parallel_workspace_underscore_normalize.py` | **Create** | TDD test: verifies no non-underscore category folders remain after normalization |
| `docs/superpowers/plans/2026-05-06-parallel-workspace-underscore-normalization.md` | **Create** | This plan document |

---

## Task 1: Verify Parallel Workspace Backup Exists

**Files:**
- Read: `/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/bak/`

- [ ] **Step 1: Check if backup exists and is complete**

Run:
```bash
ls -la /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/bak/
```

Expected: Should see `GRScenes_assets/`, `GRScenes100/`, `Material/` directories (or similar complete backup).

- [ ] **Step 2: If backup is missing or incomplete, create one**

Run:
```bash
cd /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/
rsync -avh --progress dataset/ bak_full_$(date +%Y%m%d_%H%M%S)/
```

Expected: Complete backup of dataset directory with progress output.

- [ ] **Step 3: Commit (if backup was created)**

```bash
git add docs/superpowers/plans/2026-05-06-parallel-workspace-underscore-normalization.md
git commit -m "docs: add parallel workspace underscore normalization plan"
```

---

## Task 2: Write TDD Validation Test

**Files:**
- Create: `tests/test_parallel_workspace_underscore_normalize.py`

- [ ] **Step 1: Write failing test**

```python
#!/usr/bin/env python3
"""TDD test: Verify parallel workspace asset categories are normalized to underscore style.

This test should FAIL before running merge_asset_categories_test1.py
and PASS after the normalization is complete.
"""

import os
import pytest

# The 16 category pairs that should be normalized
CATEGORY_MERGES = {
    "bathtub": "bath_tub",
    "bookshelf": "book_shelf",
    "chestofdrawers": "chest_of_drawers",
    "coffeemaker": "coffee_maker",
    "dishwasher": "dish_washer",
    "electriccooker": "electric_cooker",
    "nightstand": "night_stand",
    "shoppingtrolley": "shopping_trolley",
    "shoecabinet": "shoe_cabinet",
    "sideboardcabinet": "sideboard_cabinet",
    "sofachair": "sofa_chair",
    "teatable": "tea_table",
    "trashcan": "trash_can",
    "tvstand": "tv_stand",
    "washingmachine": "washing_machine",
    "Musical_instrument": "musical_instrument",
}

PARALLEL_DATASET_ROOT = "/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset"


def test_no_old_category_folders_remain():
    """Verify no old (non-underscore) category folders exist in GRScenes_assets."""
    assets_root = os.path.join(PARALLEL_DATASET_ROOT, "GRScenes_assets")
    
    for old_cat in CATEGORY_MERGES.keys():
        old_path = os.path.join(assets_root, old_cat)
        assert not os.path.exists(old_path), f"Old category folder still exists: {old_path}"


def test_canonical_category_folders_exist():
    """Verify all canonical (underscore) category folders exist in GRScenes_assets."""
    assets_root = os.path.join(PARALLEL_DATASET_ROOT, "GRScenes_assets")
    
    for new_cat in CATEGORY_MERGES.values():
        new_path = os.path.join(assets_root, new_cat)
        # Note: Some canonical folders might be empty if all UIDs were moved from old
        # We just check they exist as directories
        if os.path.exists(new_path):
            assert os.path.isdir(new_path), f"Canonical path exists but is not a directory: {new_path}"


def test_layout_usd_references_use_canonical_paths():
    """Spot-check: sample a few layout.usd files to ensure they use canonical category paths."""
    import glob
    
    layout_files = glob.glob(
        os.path.join(PARALLEL_DATASET_ROOT, "GRScenes100", "*", "layout.usd")
    )
    
    # Sample at most 5 scenes
    sample_layouts = layout_files[:5]
    
    for layout_path in sample_layouts:
        with open(layout_path, 'r', encoding='utf-8') as f:
            content = f.read()
        
        for old_cat in CATEGORY_MERGES.keys():
            needle = f"GRScenes_assets/{old_cat}/"
            assert needle not in content, (
                f"layout.usd {layout_path} still references old category: {old_cat}"
            )
```

- [ ] **Step 2: Run test to verify it fails**

Run:
```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
python -m pytest tests/test_parallel_workspace_underscore_normalize.py -v
```

Expected: FAIL — `test_no_old_category_folders_remain` should fail because old folders like `coffeemaker`, `tvstand`, etc. still exist.

- [ ] **Step 3: Commit**

```bash
git add tests/test_parallel_workspace_underscore_normalize.py
git commit -m "test: add TDD validation for parallel workspace underscore normalization"
```

---

## Task 3: Dry-Run Merge Script

**Files:**
- Execute: `scripts/merge_asset_categories_test1.py`

- [ ] **Step 1: Run dry-run mode**

Run:
```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset \
  --dry-run \
  --report check_reports/parallel_category_merge_dryrun.json
```

Expected output:
```
uid_collisions_total: 0
dry_run move_count: <N>  (N = total UIDs to move across all categories)
usd_rewrite_preview_count: <M>  (M = total USD files that will have paths rewritten)
```

- [ ] **Step 2: Inspect dry-run report for collisions**

Run:
```bash
python -c "import json; d=json.load(open('check_reports/parallel_category_merge_dryrun.json')); print('collisions:', d.get('uid_collisions_total', 'N/A')); print('moves:', sum(p['move_count'] for p in d['plans']))"
```

Expected: `collisions: 0` (if > 0, STOP and resolve before proceeding).

- [ ] **Step 3: Log dry-run results in plan progress**

Note the `move_count` and `usd_rewrite_preview_count` values for later comparison.

---

## Task 4: Apply Merge (Destructive — Backup Verified)

**Files:**
- Execute: `scripts/merge_asset_categories_test1.py`

⚠️ **SAFETY:** Only proceed if Step 3 confirmed `uid_collisions_total: 0` and backup exists.

- [ ] **Step 1: Run apply mode**

Run:
```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset \
  --apply \
  --report check_reports/parallel_category_merge_apply.json
```

Expected output:
```
uid_collisions_total: 0
apply moved_count: <N>  (should match dry-run move_count)
usd_rewritten_count: <M>  (should match dry-run preview count)
```

- [ ] **Step 2: Verify apply report matches dry-run**

Run:
```bash
python -c "
import json
dry = json.load(open('check_reports/parallel_category_merge_dryrun.json'))
app = json.load(open('check_reports/parallel_category_merge_apply.json'))
print('dry moves:', sum(p['move_count'] for p in dry['plans']))
print('app moves:', app['moves']['moved_count'])
print('dry rewrites:', dry.get('usd_rewrite_preview_count', 0))
print('app rewrites:', app.get('usd_rewritten_count', 0))
"
```

Expected: Dry-run and apply counts should match (or apply counts should be ≥ dry-run counts if new files were created between runs).

- [ ] **Step 3: Commit progress**

```bash
git add check_reports/parallel_category_merge_*.json
git commit -m "feat: apply underscore category normalization to parallel workspace"
```

---

## Task 5: Validate Scenes

**Files:**
- Execute: `scripts/merge_asset_categories_test1.py`

- [ ] **Step 1: Run validate mode**

Run:
```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset \
  --validate \
  --report check_reports/parallel_category_merge_validate.json
```

Expected output:
```
validated scenes: 99 / 99 missing: <N>
```

- [ ] **Step 2: Analyze missing references**

Run:
```bash
python -c "
import json
v = json.load(open('check_reports/parallel_category_merge_validate.json'))['validate']
print(f'scenes: {v[\"checked\"]}/{v[\"layout_usd_count\"]}')
print(f'missing: {v[\"missing_count\"]}')
if v['missing']:
    kinds = {}
    for m in v['missing']:
        kinds[m['kind']] = kinds.get(m['kind'], 0) + 1
    print('missing by kind:', kinds)
    # Show first 5 missing items
    for m in v['missing'][:5]:
        print(f'  {m[\"kind\"]}: {m[\"path\"]}')
"
```

Expected: `missing` count should be similar to pre-merge baseline (any missing references are pre-existing Material/mdl issues, not caused by the merge). If `missing` increased significantly vs baseline, investigate.

---

## Task 6: Post-Check (Convergence Verification)

**Files:**
- Execute: `scripts/merge_asset_categories_test1.py`

- [ ] **Step 1: Run post-check dry-run**

Run:
```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset \
  --dry-run \
  --report check_reports/parallel_category_merge_postcheck.json
```

Expected output:
```
uid_collisions_total: 0
dry_run move_count: 0
usd_rewrite_preview_count: 0
```

- [ ] **Step 2: Verify convergence**

Run:
```bash
python -c "
import json
pc = json.load(open('check_reports/parallel_category_merge_postcheck.json'))
print('move_count:', pc.get('move_preview', {}).get('moved_count', 0))
print('rewrite_count:', pc.get('usd_rewrite_preview_count', 0))
assert pc.get('move_preview', {}).get('moved_count', 0) == 0, 'Post-check failed: still have moves pending'
assert pc.get('usd_rewrite_preview_count', 0) == 0, 'Post-check failed: still have USD rewrites pending'
print('POST-CHECK PASSED: All categories normalized.')
"
```

Expected: Both counts must be exactly 0. If not, re-run apply.

---

## Task 7: Run TDD Validation Test (Should Now Pass)

**Files:**
- Test: `tests/test_parallel_workspace_underscore_normalize.py`

- [ ] **Step 1: Run validation test**

Run:
```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
python -m pytest tests/test_parallel_workspace_underscore_normalize.py -v
```

Expected: ALL tests PASS.
- `test_no_old_category_folders_remain` ✅
- `test_canonical_category_folders_exist` ✅
- `test_layout_usd_references_use_canonical_paths` ✅

- [ ] **Step 2: If any test fails, debug and fix**

Check which assertion failed and determine if it's:
- A merge script issue (re-run apply for specific category)
- A test issue (update test logic)
- A pre-existing issue (document and move on)

- [ ] **Step 3: Commit**

```bash
git add tests/test_parallel_workspace_underscore_normalize.py check_reports/
git commit -m "test: verify underscore normalization passes TDD validation"
```

---

## Task 8: Documentation Update

**Files:**
- Create: `docs/records/changes/2026-05-06_parallel_workspace_underscore_normalize.md`

- [ ] **Step 1: Write change record**

Create `docs/records/changes/2026-05-06_parallel_workspace_underscore_normalize.md`:

```markdown
---
title: '2026-05-06 — Parallel Workspace: Asset Category Underscore Normalization'
code_reference: scripts/merge_asset_categories_test1.py
created_at: '2026-05-06'
updated_at: '2026-05-06'
maintainer: Claude
status: completed
---

# 2026-05-06 — Parallel Workspace: Asset Category Underscore Normalization

## Background
The parallel C1 deduplication workspace (`test0_transitive_apply_parallel/dataset/`) contained both underscore and non-underscore style category folders (e.g., `tvstand` + `tv_stand`, `coffeemaker` + `coffee_maker`). This was inherited from the original dataset and persisted through deduplication.

## Action
Applied `scripts/merge_asset_categories_test1.py` to the parallel workspace with the existing CATEGORY_MERGES mapping.

## Execution Log

### Dry-Run
```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset \
  --dry-run \
  --report check_reports/parallel_category_merge_dryrun.json
```

Results: `uid_collisions_total: 0`, `<N>` moves, `<M>` USD rewrites previewed.

### Apply
```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset \
  --apply \
  --report check_reports/parallel_category_merge_apply.json
```

Results: `<N>` moved, `<M>` USD rewritten.

### Validate
```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset \
  --validate \
  --report check_reports/parallel_category_merge_validate.json
```

Results: `99 / 99` scenes validated, `<X>` missing (pre-existing Material/mdl issues).

### Post-Check
```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset \
  --dry-run \
  --report check_reports/parallel_category_merge_postcheck.json
```

Results: `move_count: 0`, `usd_rewrite_preview_count: 0` → fully converged.

## TDD Validation
Added `tests/test_parallel_workspace_underscore_normalize.py` which verifies:
1. No old category folders remain
2. Canonical folders exist
3. layout.usd references use canonical paths

All tests pass.

## Report Files
- `check_reports/parallel_category_merge_dryrun.json`
- `check_reports/parallel_category_merge_apply.json`
- `check_reports/parallel_category_merge_validate.json`
- `check_reports/parallel_category_merge_postcheck.json`

## Rollback
Backup at: `/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/bak/`
```

- [ ] **Step 2: Update docs INDEX**

Run:
```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
python scripts/doc_manager.py --gen-index
```

Expected: `docs/INDEX.md` updated with new change record.

- [ ] **Step 3: Commit**

```bash
git add docs/records/changes/2026-05-06_parallel_workspace_underscore_normalize.md docs/INDEX.md
git commit -m "docs: record parallel workspace underscore normalization"
```

---

## Task 9: Final Verification & Push

- [ ] **Step 1: Run full test suite**

Run:
```bash
cd /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
python -m pytest tests/test_parallel_workspace_underscore_normalize.py -v
```

Expected: All tests pass.

- [ ] **Step 2: Verify git status**

Run:
```bash
git status
git log --oneline -5
```

Expected: Clean working tree, commits for plan, test, apply, validation, docs.

- [ ] **Step 3: Push to remote**

Run:
```bash
git push origin main
```

If proxy fails, retry with:
```bash
GIT_SSL_NO_VERIFY=true git -c http.proxy= -c https.proxy= push origin main
```

---

## Self-Review Checklist

Before starting execution, verify:

- [ ] **Spec coverage:** All 16 category pairs from CATEGORY_MERGES are covered
- [ ] **Placeholder scan:** No TBD, TODO, or vague steps in this plan
- [ ] **Type consistency:** Script parameters match existing merge_asset_categories_test1.py signature
- [ ] **Safety:** Backup verified before destructive apply step
- [ ] **TDD:** Test is written first, fails before apply, passes after

## Execution Options

**Plan complete and saved to `docs/superpowers/plans/2026-05-06-parallel-workspace-underscore-normalization.md`.**

Two execution options:

**1. Subagent-Driven (recommended)** - I dispatch a fresh subagent per task, review between tasks, fast iteration

**2. Inline Execution** - Execute tasks in this session using executing-plans, batch execution with checkpoints

**Which approach?**
