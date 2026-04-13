---
title: "Task 4 Transitive BBox Gating"
code_reference:
  - scripts/c1_autorun_categories.py
  - tests/test_c1_autorun_categories.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Task 4 Transitive BBox Gating

## Summary

Task 4 wires per-category `pair_certificates.jsonl` into the bbox autorun audit step
through a dedicated command builder helper.

## Code Changes

- Added `_build_bbox_audit_cmd(...)` in `scripts/c1_autorun_categories.py` so the
  bbox audit argv is constructed in one unit-testable place.
- Preserved the existing audit flags and path handling, while adding
  `--certificate-jsonl <plan.certificate_jsonl>`.
- Replaced the inline audit command assembly in `_run_bbox_gated(...)` with the new
  helper.
- Added `tests/test_c1_autorun_categories.py` to verify the built audit command
  includes both `--certificate-jsonl` and `--mode-reports-dir`.
- Expanded the same test to lock down the preserved pre-existing audit arguments:
  `--left-root`, `--right-root`, `--left-mode`, `--right-mode`,
  `--right-layout-name`, `--label`, `--out`, `--verdict-out`,
  `--scene-list-json`, `--bbox-policy`, `--eps-bbox`, `--eps-pos`,
  `--eps-angle`, `--eps-geom`, and `--allow-no-mesh`.

## TDD Notes

Red phase command:

```bash
python -m pytest tests/test_c1_autorun_categories.py -q
```

Initial red result:

- test collection failed because the new test loader did not register the imported
  script module in `sys.modules`, which dataclass processing requires

Harness-fix red result:

- `1 failed in 0.05s`
- failure reason: `c1_autorun_categories` had no `_build_bbox_audit_cmd`

Green phase command:

```bash
python -m pytest tests/test_c1_autorun_categories.py -q
```

Green result:

- `1 passed in 0.05s`

Quality follow-up command:

```bash
python -m pytest tests/test_c1_autorun_categories.py -q
```

Quality follow-up result:

- `1 passed in 0.04s`
- no production change was necessary because `_build_bbox_audit_cmd(...)` already
  preserved the pre-existing audit arguments; the gap was test coverage only

## Constraints

- Did not modify mapping-builder logic.
- Did not modify bulk-apply behavior.
- Did not modify the audit script itself.
- Did not create a commit.
