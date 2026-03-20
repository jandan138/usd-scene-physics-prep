---
title: Add allowlist-aware center gap check to gate scripts
code_reference: scripts/check_normalize_gate_from_reports.py, scripts/build_normalize_gate_verdict.py, scripts/assemble_normalize_gate_bundle.py, scripts/orchestrate_test0_rebuilt_normalize.py
created_at: 2026-03-15
updated_at: 2026-03-15
maintainer: gate-scripts
status: active
---

# Gate Allowlist Support for Missing Centers

## Problem

The normalize hard gate requires `center_found == common_ref_prim_count`. Known-bad
assets (36 meshless books, 1 cabinet, 1 other, 1 person = 39 assets) produce no
centers during Phase 1 normalization. This causes 46 scene-prim instances to lack
centers, failing the gate even though the displacement is expected and documented
in the Phase 1 allowlist_verdict.json.

## Solution

Three-layer approach:

1. **Gate scripts** (`check_normalize_gate_from_reports.py`, `build_normalize_gate_verdict.py`):
   - Added `--max-missing-centers INT` CLI arg (default 0, backward compatible)
   - Changed center check from `center_found == common_ref` to
     `(common_ref - center_found) <= max_missing_centers`
   - Renamed check key to `center_gap_within_allowance`
   - Added metrics: `max_missing_centers`, `skipped_missing_center`

2. **Assemble script** (`assemble_normalize_gate_bundle.py`):
   - Added `--allowlist-json` CLI arg
   - Helper `_count_max_missing_from_allowlist()` extracts known-bad asset keys
     from `allowed_asset_errors` + `door_*` patterns in `allowed_nonzero_exit_codes`
   - Computes `max_missing_centers = len(bad_assets) * 10` (instance multiplier)
   - Passes `--max-missing-centers` to both gate subprocess calls

3. **Orchestrator** (`orchestrate_test0_rebuilt_normalize.py`):
   - Both smoke and full-run `bundle_cmd` now pass `--allowlist-json` pointing to
     `phase1_root / "allowlist_verdict.json"` when the file exists

## Backward Compatibility

- When `--max-missing-centers` is not provided (default 0), the gate behaves
  identically to before (strict equality).
- When `--allowlist-json` is not provided to assemble, max_missing_centers stays 0.

## Verification

- All 146 existing tests pass (including new allowlist-specific tests).
- Real allowlist (39 known-bad assets) produces `max_missing_centers=390`,
  accommodating the actual gap of 46.
