---
title: "Remove tolerance_merge contamination from geom_only dedup report"
code_reference: scripts/report_asset_mesh_dedup.py
created_at: 2026-04-01
updated_at: 2026-04-01
maintainer: TraeAI
status: complete
---

# Remove tolerance_merge from geom_only mode

## Problem

The `_write_report` function in `scripts/report_asset_mesh_dedup.py` (lines 1048-1056)
ran `_tolerance_merge` specifically when `mode == "geom_only"`. This contaminated the
geom_only report with tolerance-merged groups whose members have **different** geometry
hashes — they are not true geom_only (hash-identical) duplicates.

Investigation showed bottle had 66% contamination, other had 16%.

## Fix

Removed the entire `_tolerance_merge` call block from geom_only mode. The geom_only
report now contains **only** hash-identical pairs from `_make_duplicates_map`.

- `tolerance_merged_count` is still initialized to `0` so the `meta` dict field
  `"tolerance_merged_groups"` remains valid (always 0 for geom_only).
- The `_tolerance_merge` function itself is preserved — it may be used by other modes
  or future code.
- No other mode's behavior is affected.

## Changed lines

`scripts/report_asset_mesh_dedup.py` ~line 1048: replaced the `if merge_tolerance > 0
and mode == "geom_only" ...` block with a comment and `tolerance_merged_count = 0`.
