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

Results: `uid_collisions_total: 0`, `403` moves previewed, `97` USD rewrites previewed.

### Apply
```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset \
  --apply \
  --report check_reports/parallel_category_merge_apply.json
```

Results: `192` moved, `97` USD rewritten.

### Validate
```bash
./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py \
  --subset-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset \
  --validate \
  --report check_reports/parallel_category_merge_validate.json
```

Results: `99 / 99` scenes validated, `104` missing (pre-existing Material/mdl and placeholder issues).

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
Backup at: `/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/bak_full_20260506_052808/`