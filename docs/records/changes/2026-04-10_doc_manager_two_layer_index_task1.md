---
title: "Doc Manager Task 1: two-layer index hardening"
code_reference:
  - scripts/doc_manager.py
  - tests/test_doc_manager.py
created_at: 2026-04-10
updated_at: 2026-04-10
maintainer: OpenCode
status: active
doc_class: record
---

# Doc Manager Task 1: two-layer index hardening

## Summary

Implemented Task 1 from the docs reorganization plan in the dedicated worktree only.
The change hardens `scripts/doc_manager.py` for two-layer indexing and adds focused tests
for the new behavior.

## Red-Green Cycle

Command:
```bash
python -m pytest tests/test_doc_manager.py -q
```

Red state before production change:
- `test_generate_index_separates_primary_docs_and_records`
- `test_validate_docs_reports_invalid_doc_class`
- `test_generate_index_skips_index_and_readme_files` already passed

Green state after production change:
- `3 passed in 0.03s`

Follow-up red state from code quality review:
- `test_generate_index_uses_stable_group_for_root_level_docs`
- `test_generate_index_handles_archive_docs_explicitly`

Follow-up green state after the review fixes:
- `5 passed in 0.03s`

## Code Changes

- Added `SKIP_BASENAMES = {"INDEX.md", "README.md", "index.md"}`.
- Added `VALID_DOC_CLASSES = {"primary", "record", "archive"}` validation in `validate_docs()`.
- Added path-based `doc_class` inference when metadata omits it.
- Replaced flat category buckets in `generate_index()` with top-level `Primary Docs` and `Records` sections.
- Added path-derived `###` group headings, including `Getting Started` for `docs/usage/*` and `Research` for record-style research paths such as `docs/test0_smoke/*`.
- Added an explicit `doc_class` to section mapping so `archive` is intentionally routed into the two-section index model.
- Added a stable `General` group for root-level docs under `docs/` so headings do not derive from filenames.
- Added an explicit `Archive` group heading for archived docs inside the `Records` section.

## Notes

- Kept changes localized to `scripts/doc_manager.py` and the new targeted test file.
- Did not attempt to fix the pre-existing repository-wide metadata issues mentioned in the task handoff.
- Did not start Task 2 and did not move any existing docs.
