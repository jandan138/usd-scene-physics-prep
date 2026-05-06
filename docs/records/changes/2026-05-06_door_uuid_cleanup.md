---
title: '2026-05-06 — Parallel Workspace: door_UUID Missing Reference Cleanup'
code_reference: scripts/oneoff_clear_missing_door_references.py
created_at: '2026-05-06'
updated_at: '2026-05-06'
maintainer: Claude
status: completed
---

# 2026-05-06 — Parallel Workspace: door_UUID Missing Reference Cleanup

## Background
After underscore category normalization, the parallel workspace still contained 31 `door_UUID` placeholder categories with empty assets (no USD geometry). These caused 31 `reference` missing entries in scene validation reports.

## Action
Applied `scripts/oneoff_clear_missing_door_references.py` (Scheme A - minimal removal) to clean broken door references from layout.usd files.

## Execution Log

### Dry-Run
```bash
./scripts/isaac_python.sh scripts/oneoff_clear_missing_door_references.py \
  --validate-report /cpfs/user/zhuzihou/parallel_category_merge_validate.json \
  --dry-run \
  --report /cpfs/user/zhuzihou/door_ref_clear_dryrun.json
```

Results: `31` door references to remove across `25` scenes.

### Apply
```bash
./scripts/isaac_python.sh scripts/oneoff_clear_missing_door_references.py \
  --validate-report /cpfs/user/zhuzihou/parallel_category_merge_validate.json \
  --apply \
  --report /cpfs/user/zhuzihou/door_ref_clear_apply.json
```

Results: `31` references removed from `25` scenes.

### Validate (After Cleanup)
```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset \
  --validate \
  --report /cpfs/user/zhuzihou/parallel_category_merge_validate_after_door_cleanup.json
```

Results: `99/99` scenes validated, `73` missing (was `104` before cleanup).

## Impact

| Metric | Before | After | Delta |
|--------|--------|-------|-------|
| Total missing | 104 | 73 | -31 |
| Reference missing | 46 | 15 | -31 |
| door_UUID missing | 31 | 0 | -31 |

**Note**: The remaining 73 missing references are pre-existing issues:
- `attr`: 58 (Material/mdl path issues)
- `reference`: 15 (cabinet, other, person placeholder assets)

## Method

**Scheme A (Minimal Removal)**:
- Only removes `reference` items pointing to `GRScenes_assets/door_*`
- Does NOT delete prim nodes
- Does NOT touch payloads or asset-valued attributes
- Does NOT affect non-door references

## Report Files
- `/cpfs/user/zhuzihou/door_ref_clear_dryrun.json`
- `/cpfs/user/zhuzihou/door_ref_clear_apply.json`
- `/cpfs/user/zhuzihou/parallel_category_merge_validate_after_door_cleanup.json`

## Related
- Previous test1 cleanup: `docs/records/changes/2026-02-09_test1_missing_cleanup.md`
- Allowlist approach: `docs/records/runs/test0-full/grscenes_test0_rebuilt_phase1_allowlist_verdict_20260315.md`
