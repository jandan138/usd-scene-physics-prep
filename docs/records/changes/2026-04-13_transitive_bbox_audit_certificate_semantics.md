---
title: "Transitive BBox Audit Certificate Semantics"
code_reference:
  - scripts/placement_pairwise_compare.py
  - tests/test_placement_pairwise_compare_bbox_gate.py
created_at: 2026-04-13
updated_at: 2026-04-13
maintainer: OpenCode
status: active
doc_class: record
---

# Transitive BBox Audit Certificate Semantics

## Summary

Task 3 makes `scripts/placement_pairwise_compare.py` certificate-aware for certified
transitive pairs.

The audit path now prefers certificate semantics over raw `mode_index` fallback, so a
certified transitive pair can inherit its effective tier from certificate metadata.
Under `bbox_primary_rmse_observe`, this lets certified transitive pairs behave like
tier2 soft warnings when their certificate resolves to `topo_filesize` or
`shape_invariant`, while preserving the old strict behavior when no certificate JSONL
is provided.

## Code Changes

- Added `_certificate_mode_from_row(...)` to derive the effective audit mode from a
  certificate row.
- Added `_load_certificate_lookup(...)` to optionally ingest `--certificate-jsonl`
  into path and UID lookup tables.
- Tightened certificate ingestion so only certified rows contribute to lookup
  semantics: `eligible` must be truthy and `reject_reason` must be empty.
- Added `_lookup_certificate_mode(...)` so certificate semantics are resolved
  symmetrically even when `_lookup_prim_dedup_mode(...)` receives reversed left/right
  input ordering.
- Updated `_lookup_prim_dedup_mode(...)` to prefer certificate-derived semantics
  before falling back to `mode_index`.
- Threaded `certificate_lookup` through `compare_scene(...)` and the CLI execution
  path.
- Added `--certificate-jsonl` to the pairwise compare CLI.

## Effective Mode Rules

- If `transitive_effective_mode` exists, use it.
- Otherwise, direct non-transitive certificates keep their own `mode`.
- For transitive certificates without `transitive_effective_mode`, derive the mode from
  `transitive_witness_modes` with this precedence:
  - `shape_invariant`
  - `topo_filesize`
  - `geom_only`
- If no witness mode metadata is present, stay at strict `transitive`.

## Tests

Added targeted coverage in `tests/test_placement_pairwise_compare_bbox_gate.py` for:

- certificate lookup overriding a contradictory `mode_index` fallback
- certified transitive observe-policy behavior downgrading tier2 bbox drift to a
  soft failure
- legacy strict transitive behavior remaining unchanged when no certificate lookup is
  supplied
- rejected transitive certificate rows being ignored by both lookup resolution and
  scene-level audit behavior
- reversed-order certificate lookup still resolving certified semantics
- certified direct-mode rows still resolving to their direct mode without regressing to
  fallback behavior

## TDD Notes

Red phase command:

```bash
./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py -q
```

Red result:

- collection failed with `ImportError: cannot import name '_load_certificate_lookup'`
- this confirmed the tests were exercising missing certificate-aware audit support,
  not an already-implemented path

Green phase command:

```bash
./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py -q
```

Green result:

- `7 passed in 0.36s`

Spec-gap follow-up command:

```bash
./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py -q
```

Spec-gap red result:

- `2 failed, 7 passed`
- failure reason: `_load_certificate_lookup(...)` still admitted rejected rows, so a
  rejected certificate could override a certified row or soften a strict transitive
  audit case

Spec-gap green result:

- `9 passed in 0.40s`

Quality-fix command:

```bash
./scripts/isaac_python.sh -m pytest tests/test_placement_pairwise_compare_bbox_gate.py -q
```

Quality-fix red result:

- `1 failed, 10 passed`
- failure reason: certificate lookup was directional, so reversing the left/right
  inputs fell through to legacy `mode_index` behavior instead of using certified
  certificate semantics

Quality-fix green result:

- `11 passed in 0.44s`

## Decisions

- Kept the implementation local to the audit script and test file only.
- Did not modify mapping-builder or autorun flows.
- Used UID lookup in addition to path lookup so certificate rows remain usable even if
  stored asset paths are relative while composed stage references are absolute.
- Chose a strict certificate admission rule for audit semantics: only certified rows
  may influence effective mode resolution.
- Matched certificate precedence to legacy `mode_index` symmetry so argument order
  alone does not decide whether certified semantics apply.

## Concerns

- Certificate lookup currently trusts UID uniqueness across the compared asset set when
  a path match is unavailable. That matches the existing `mode_index` assumption and is
  sufficient for this task, but a future cross-dataset collision case would need a more
  explicit key.
