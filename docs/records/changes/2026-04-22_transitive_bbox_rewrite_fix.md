---
title: "Transitive BBox-Gated Rewrite Fix"
created_at: "2026-04-22"
updated_at: "2026-04-22"
maintainer: "OpenCode"
status: "active"
code_reference:
  - "scripts/rewrite_layout_asset_refs_with_compensation.py"
  - "scripts/c1_bulk_apply_layout_dedup.py"
  - "tests/test_bbox_gated_rewrite.py"
  - "/shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/c1_bulk/bed_bbox_primary_rmse_observe_v1/02_apply/rewrite_reject_ledger.jsonl"
  - "/shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/c1_bulk/bed_bbox_primary_rmse_observe_v1/04_step6/post_promote_full_usd_scan_excluding_backups_pxr.json"
doc_class: record
---

# Summary

Fixed a bbox-gated rewrite gap where `01_cert` and mapping already admitted a
`transitive` pair, but `02_apply` still rejected it as
`mode_not_enabled_transitive`.

The bug surfaced in the promoted-clone run at:

- `/shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046`

Observed failure chain:

- `bed` passed `01_cert`
- `bed` passed `03_audit`
- `bed` failed `04_step6` post-promote scan because one old bed asset remained
  referenced
- root cause in `02_apply`: the rewrite layer still rejected `dedup_mode ==
  "transitive"`

# Code Changes

## `scripts/rewrite_layout_asset_refs_with_compensation.py`

Added certificate-aware transitive rewrite support:

- `rewrite_layout()` now accepts `certificate_jsonl`
- loads a per-pair certificate lookup from `pair_certificates.jsonl`
- filters that lookup to certified rows only (`eligible=true` and no
  `reject_reason`)
- reconstructs transitive witness inputs from:
  - `transitive_witness_uids`
  - `transitive_witness_modes`
- computes a real transitive `V` instead of falling back to an implicit identity
- allows `dedup_mode == "transitive"` through the bbox-gated compensation path
- keeps aspect-ratio prefilter semantics narrow so only explicit
  `aspect_ratio_rejected` pairs are dropped there

## `scripts/c1_bulk_apply_layout_dedup.py`

- now forwards `--certificate-jsonl` into `rewrite_layout()` so the rewrite
  layer can actually use transitive witness metadata during apply

# Tests

Added focused regression coverage in `tests/test_bbox_gated_rewrite.py`:

- `test_bbox_gated_transitive_ref_rewrite_uses_certificate_witness`

This test proves that a bbox-gated rewrite with:

- a transitive pair
- witness metadata in `certificate_jsonl`
- `v_matrix_mode="auto"`

now rewrites the reference and records no reject.

Verification commands:

```bash
./scripts/isaac_python.sh -m pytest tests/test_bbox_gated_rewrite.py::test_bbox_gated_transitive_ref_rewrite_uses_certificate_witness -q
./scripts/isaac_python.sh -m pytest tests/test_bbox_gated_rewrite.py::test_bbox_gated_transitive_ref_rewrite_prefers_certified_row tests/test_bbox_gated_rewrite.py::test_bbox_gated_transitive_ref_rewrite_rejects_noncertified_row -q
./scripts/isaac_python.sh -m pytest tests/test_bbox_gated_rewrite.py -q
```

Observed results:

- focused red: failed before the fix because `rewrite_layout()` did not accept
  `certificate_jsonl`
- focused green: `1 passed`
- certificate-row filtering regression: `2 passed`
- file-level regression: `5 passed`

# Runtime Verification

Before rerunning the full category, a targeted scene probe was executed against
the exact bed-failing scene:

- scene root:
  `/shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/dataset/GRScenes100/home/MVUHLWYKTKJ5EAABAAAAACQ8_usd`

Probe result (`02_apply_fix_probe/batch_summary.json`):

- `refs_changed = 1`
- `xform_compensated = 1`
- `reject_records = 0`

This directly verified that the residual old bed reference which previously
caused `bed` Step 6 to abort is now rewritten successfully.

# Follow-Up Runtime Action

Started a bed-only bbox autorun rerun on the promoted-clone workspace using the
fixed rewrite path:

```bash
./scripts/isaac_python.sh scripts/c1_autorun_categories.py \
  --dataset-root /shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/dataset \
  --bak-root /shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/bak \
  --report /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/all_categories_union_merged.json \
  --c1-bulk-dir /shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/c1_bulk \
  --group-label test0_transitive_apply_seeded \
  --bbox-gated --bbox-policy bbox_primary_rmse_observe \
  --step6-mode apply --dedup-mode geom_only --v-matrix-mode auto \
  --mode-reports-dir /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_rebuilt_dedup/v8_prededup \
  --scene-files layout.usd --include-regex '^bed$' --no-skip-done
```

Tracking:

- PID: `701294`
- log:
  `/shared/smartbot/zhuzihou/assets/.opencode_logs/test0_transitive_apply_20260421_103046_bed_rerun.log`
- autorun run dir:
  `/shared/smartbot/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/c1_bulk/_autorun/test0_transitive_apply_seeded_bbox_primary_rmse_observe_20260422_044747`

Fresh verification after the rerun completed:

- ledger shows:
  - `category_done` for `bed`
  - `run_done` with `failed_categories = []`
- `03_audit/audit_verdict.json` remains `passed: true`
- `04_step6/post_promote_full_usd_scan_excluding_backups_pxr.json` now reports
  `hit_files = 0`
- `04_step6/post_soft_delete_layout_scan_pxr.json` now reports
  `hit_layouts = 0`
- `04_step6/soft_delete_old_assets_report.json` shows the expected old bed
  assets, including `ceca271d76d866045caf8f8b277f9457`, moved into
  `bak/_dedup_assets/...`

This verifies that the transitive rewrite fix resolved the original `bed`
Step 6 failure rather than merely masking it.
