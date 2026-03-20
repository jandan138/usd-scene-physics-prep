---
title: test0 smoke s1 descendant override remap summary draft
created_at: 2026-03-12
updated_at: 2026-03-12
maintainer: codex
status: completed
---

# Summary

- Run / cycle id:
  - `20260312_074731 / remap_fix_phase2_v3`
- Scope:
  - `test0 smoke / S1 / normalize-only`
- Repair target:
  - descendant override remap / compensation
- Final status:
  - isolated `phase2-only` validation passed

# What Changed

- Code changes:
  - [normalize_asset_transforms.py](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/normalize_asset_transforms.py)
- Reporting / gate changes:
  - reused existing gate helpers and wrote new artifacts under
    [remap_fix_phase2_v3](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3)
- Explicitly unchanged:
  - dedup logic
  - unrelated scene processing
  - `phase1` centers and canonical normalized asset tree

# Inputs

- Source subset:
  - `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1`
- Output subset:
  - `/tmp/test0_desc_override_fix_phase2_v3/out`
- Pairwise report:
  - [pairwise_compare.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/pairwise_compare.json)
- Audit report:
  - [audit_phase2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/audit_phase2.json)
- Gate verdict:
  - [normalize_gate_verdict.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/normalize_gate_verdict.json)

# Key Findings

## Before Repair

- Refrigerator failing prim:
  - `/Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0`
- Observed geometry drift:
  - aggregate centroid delta `0.061537`
  - aggregate point delta max `1.293842`
- Descendant override finding:
  - source scene authored `/Instance.xformOp:scale`
  - normalized scene let that override become inert under the old path

## After Repair

- Refrigerator result:
  - aggregate centroid delta `0.0`
  - aggregate point delta max `0.000002`
- Control result:
  - nightstand remained stable at aggregate centroid delta `0.000064`
  - no new outlier introduced
- Descendant override result:
  - descendant override was remapped to scene-layer `xformOp:transform`
  - fixed scene still has one descendant xform finding, but inert list is empty
  - phase2 audit reports `pre_c1_inert_descendant_xform_override_count = 0`
    and `became_inert_descendant_xform_override_count = 0`

# Gate Outcome

- Pairwise gate:
  - pass
  - `displaced_breakdown.gt_0.01 = 0`
  - `ref_same_breakdown.gt_0.01 = 0`
- Phase2 audit gate:
  - pass
  - `center_found = 1105`
  - `common_ref_prim_count = 1105`
  - `matrix_mismatch = 0`
- Unified normalize gate:
  - pass
  - `status = pass_normalize_gate`
  - `reason = ok`

# Risks / Caveats

- This validation was intentionally run in an isolated `phase2-only` output root.
  It proves the remap logic closes the refrigerator failure, but it is not yet
  the canonical rerun of the official `S1` normalized output tree.
- Current remap handling is validated for the known root-layer descendant xform
  override case. Sublayer-authored descendant overrides and unusual
  multi-reference prims remain residual edge cases that should stay on the
  watch list.

# Next Step

- If normalize-only passed:
  - rerun the official `S1 normalize-only` output with the same code path
  - regenerate canonical pairwise / audit / verdict artifacts under the normal
    report directory
  - only after that official rerun stays green should dedup be reopened
- If normalize-only failed:
  - not applicable for this isolated validation cycle

# Artifact Index

- Execution log / note:
  - [execution.md](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/docs/test0_smoke/20260312_074731/execution.md)
- Test report:
  - [test.md](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/docs/test0_smoke/20260312_074731/test.md)
- Summary report:
  - [summary.md](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/docs/test0_smoke/20260312_074731/summary.md)
- Additional diagnostics:
  - [refrigerator_geometry_delta_fixed.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/refrigerator_geometry_delta_fixed.json)
  - [nightstand_geometry_delta_fixed.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/nightstand_geometry_delta_fixed.json)
  - [source_descendant_override_audit.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/source_descendant_override_audit.json)
  - [fixed_descendant_override_audit.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/fixed_descendant_override_audit.json)
