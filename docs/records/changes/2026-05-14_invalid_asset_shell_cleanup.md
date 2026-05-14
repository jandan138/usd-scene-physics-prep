---
title: Test0 Parallel Dataset Invalid Asset Shell Cleanup
doc_class: record
code_reference:
  - scripts/oneoff_clear_invalid_asset_shells.py
  - tests/test_oneoff_clear_invalid_asset_shells.py
  - docs/records/research/2026-05-14_test0_parallel_dataset_integrity_investigation.md
created_at: 2026-05-14
updated_at: 2026-05-14
maintainer: Codex
status: completed
---

# Test0 Parallel Dataset Invalid Asset Shell Cleanup

## Summary

Resolved the four annotation-only asset shells in the current test0 parallel
dataset by removing their authored scene references and moving the shell
directories out of the final `GRScenes_assets` tree.

Dataset root:

`/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/dataset`

## Targets

```text
cabinet/b98d6ccbeb75dfdeb60e27649a5b055a
other/d41d8cd98f00b204e9800998ecf8427e
person/351316cbb083f9f4df0cccd60cbfa848
person/d41d8cd98f00b204e9800998ecf8427e
```

## Changes

- Added `scripts/oneoff_clear_invalid_asset_shells.py`.
- Added `tests/test_oneoff_clear_invalid_asset_shells.py`.
- Removed 15 authored references to missing target USD files from 4
  `layout.usd` files.
- Moved the 4 annotation-only asset shell directories to quarantine.
- Updated the dataset integrity investigation record and generated docs index.

## Runtime Records

Reports:

```text
check_reports/test0_parallel_invalid_asset_shells_dryrun_20260514.json
check_reports/test0_parallel_invalid_asset_shells_apply_20260514.json
check_reports/test0_parallel_invalid_asset_shells_postcheck_20260514.json
```

Backups:

```text
/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/bak_invalid_asset_shell_cleanup_20260514/layouts
/cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_parallel/bak_invalid_asset_shells_20260514/GRScenes_assets
```

Apply result:

| metric | value |
| --- | ---: |
| layouts scanned | 99 |
| touched layout stages | 4 |
| references removed | 15 |
| asset shells quarantined | 4 |

Post-check result:

| metric | value |
| --- | ---: |
| references planned | 0 |
| references removed | 0 |
| touched stages | 0 |
| asset shells found in final dataset | 0 |

## Verification

Commands run:

```bash
python -m pytest -q tests/test_oneoff_clear_invalid_asset_shells.py
python -m py_compile scripts/oneoff_clear_invalid_asset_shells.py
python scripts/doc_manager.py --validate
git diff --check
```

Additional dataset checks:

- Exact USD API scan over all 99 layouts found 0 authored references to the four
  invalid asset USD paths.
- Opening all 99 layouts emitted 0 stderr lines containing the target UIDs.
- Full `GRScenes_assets/<category>/<uid>` scan found 0 asset dirs missing
  `usd/<uid>.usd`.
- Full four-view PNG scan found 0 asset dirs missing `front.png`, `left.png`,
  `back.png`, or `right.png`.
- Annotation-only direct asset shell scan found 0 remaining entries.

## Commit

Implemented in:

`aeb83ed fix: clear invalid asset shell references`

## OSS Refresh

After the dataset cleanup, the existing OSS release prefix was refreshed in
place:

```text
aliyun-beijing-internal:pjlab-bjpai-zhuzihou-assets/GRScenes-test1-parallel-dedup-20260506
```

This was not a full re-copy of the 108GB package. The upload used a targeted
incremental update:

- Uploaded the 4 changed `layout.usd` files with `rclone copyto`.
- Deleted the 4 stale remote annotation JSON objects that used to represent the
  invalid asset shells.

Remote stale objects removed:

```text
GRScenes_assets/cabinet/b98d6ccbeb75dfdeb60e27649a5b055a/b98d6ccbeb75dfdeb60e27649a5b055a_annotation.json
GRScenes_assets/other/d41d8cd98f00b204e9800998ecf8427e/d41d8cd98f00b204e9800998ecf8427e_annotation.json
GRScenes_assets/person/351316cbb083f9f4df0cccd60cbfa848/351316cbb083f9f4df0cccd60cbfa848_annotation.json
GRScenes_assets/person/d41d8cd98f00b204e9800998ecf8427e/d41d8cd98f00b204e9800998ecf8427e_annotation.json
```

Upload verification:

```text
rclone check: 0 differences found
rclone check: 341307 matching files
```

The previous OSS upload had 341311 matching files. The new count is 4 lower
because the four invalid annotation-only shell JSON objects were intentionally
removed from the final dataset and from OSS.

This cleanup does not address the separate Material/MDL missing dependency
issue, which remains tracked in the investigation record.
