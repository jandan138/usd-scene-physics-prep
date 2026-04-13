---
title: "BBox-Gated Transitive Mapping Admission"
code_reference: scripts/c1_build_bulk_mapping_from_dedup_report.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
---

# BBox-Gated Transitive Mapping Admission

## Summary

Task 2 removes the temporary hard reject for `pair_mode == "transitive"` in the
bbox-gated mapping builder.

Eligible transitive pairs now flow through
`build_transitive_pair_certificate(...)` and can participate in the certified graph
and `filtered_mapping` the same way as other certified edges.

## Code Changes

- `scripts/c1_build_bulk_mapping_from_dedup_report.py`
  now builds an absolute `group_members` list once per duplicate group.
- The `pair_mode == "transitive"` branch now calls
  `_cvt.build_transitive_pair_certificate(...)` with:
  - `old_usd`
  - `canonical_usd`
  - `group_members`
  - `mode_index`
  - `policy`
- Certified-graph rebuild, canonical selection, and filtered-mapping emission were left
  unchanged.
- Rejected transitive pairs now contribute their real certificate reject reason to the
  summary counters instead of the placeholder `transitive_not_supported` bucket.
- Follow-up review fix: per-pair mode resolution now depends on whether
  `--mode-reports-dir` was supplied, not whether the loaded `mode_index` is non-empty.
  An empty loaded index therefore still routes unresolved pairs to `transitive`.

## Tests

Added targeted coverage in `tests/test_bbox_gated_mapping.py` for:
- empty loaded mode-index fallback to transitive resolution
- real transitive-builder integration via `group_members` and `find_transitive_V(...)`
- rejected transitive certification preserving the real reject reason in summary output
- self-contained test collection by prepending the repo-vendored
  `third_party/runtime_deps/isaac_py310` path before importing the mapping script

## TDD Notes

Red phase command:

```bash
PYTHONPATH="/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.worktrees/feat-transitive-bbox-gating/third_party/runtime_deps/isaac_py310${PYTHONPATH:+:$PYTHONPATH}" python -m pytest tests/test_bbox_gated_mapping.py -k transitive
```

Red result:
- `2 failed, 2 deselected`
- failure reason: `_run_bbox_gated()` still forced transitive pairs to
  `transitive_not_supported`, so no eligible edge reached `filtered_mapping`

Green phase command:

```bash
PYTHONPATH="/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.worktrees/feat-transitive-bbox-gating/third_party/runtime_deps/isaac_py310${PYTHONPATH:+:$PYTHONPATH}" python -m pytest tests/test_bbox_gated_mapping.py -k transitive
```

Green result:
- `2 passed, 2 deselected`

Broader verification:

```bash
PYTHONPATH="/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.worktrees/feat-transitive-bbox-gating/third_party/runtime_deps/isaac_py310${PYTHONPATH:+:$PYTHONPATH}" python -m pytest tests/test_bbox_gated_mapping.py
```

Verification result:
- `4 passed`

Review follow-up red/green command:

```bash
PYTHONPATH="/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.worktrees/feat-transitive-bbox-gating/third_party/runtime_deps/isaac_py310${PYTHONPATH:+:$PYTHONPATH}" python -m pytest tests/test_bbox_gated_mapping.py -k 'empty_mode_index or rejected_transitive_pair_uses_real_reject_reason'
```

Follow-up red result:
- `2 failed, 2 deselected`
- failure reason: empty loaded `mode_index` still fell back to `report_mode`, so
  `_run_bbox_gated()` called `build_pair_certificate(...)` instead of the transitive
  certificate path

Follow-up green result:
- `2 passed, 2 deselected`

Final self-contained test bootstrap verification:

```bash
python -m pytest tests/test_bbox_gated_mapping.py -q
```

Result:
- `4 passed`
- the test file now imports `c1_build_bulk_mapping_from_dedup_report.py` without any
  shell-level `PYTHONPATH` injection

## Concerns

- This task intentionally did not modify audit or autorun paths, so any mirrored
  transitive logic outside this script remains unchanged.
