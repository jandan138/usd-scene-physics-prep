---
title: test0 smoke s1 descendant override remap test draft
created_at: 2026-03-12
updated_at: 2026-03-12
maintainer: codex
status: completed
---

# Purpose

This document records the actual test results for the descendant override
remap repair.

It records:

- what to test
- why each test exists
- what must pass before this cycle can move past `normalize-only`

# Test Matrix And Results

## A. Static Checks

### A1. Python syntax / import check

- Target:
  - [normalize_asset_transforms.py](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/normalize_asset_transforms.py)
  - [audit_normalize_phase2.py](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/audit_normalize_phase2.py)
  - [check_normalize_gate_from_reports.py](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/check_normalize_gate_from_reports.py)
  - [build_normalize_gate_verdict.py](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/build_normalize_gate_verdict.py)
- Command:
  - `python3 -m py_compile scripts/normalize_asset_transforms.py scripts/audit_normalize_phase2.py scripts/check_normalize_gate_from_reports.py scripts/build_normalize_gate_verdict.py`
- Pass criteria:
  - command exits `0`
  - no syntax errors
  - no import errors
- Result:
  - pass

## B. Focused Functional Checks

### B1. Refrigerator failing prim geometry check

- Target scene:
  - `home/MV7J6NIKTKJZ2AABAAAAADA8_usd`
- Target prim:
  - `/Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0`
- Input reports:
  - before:
    [refrigerator_geometry_delta_v2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/refrigerator_geometry_delta_v2.json)
  - after:
    [refrigerator_geometry_delta_fixed.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/refrigerator_geometry_delta_fixed.json)
- Pass criteria:
  - aggregate centroid delta is within target tolerance
  - max per-mesh displacement is within target tolerance
  - max point delta is within target tolerance
- Before:
  - aggregate centroid delta: `0.061537`
  - aggregate point delta max: `1.293842`
- After:
  - aggregate centroid delta: `0.0`
  - aggregate bbox-mid delta: `0.0`
  - aggregate point delta max: `0.000002`
- Result:
  - pass

### B2. Descendant override survival check

- Target prim:
  - `/Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0/Instance`
- Input audit:
  - source:
    [source_descendant_override_audit.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/source_descendant_override_audit.json)
  - fixed:
    [fixed_descendant_override_audit.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/fixed_descendant_override_audit.json)
  - gate audit:
    [audit_phase2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/audit_phase2.json)
- Pass criteria:
  - `pre_c1_inert_descendant_xform_override_count = 0`
  - `became_inert_descendant_xform_override_count = 0`
  - no unexpected new descendant override regression
- Source scene:
  - `finding_count = 1`
  - scene-layer props: `['xformOp:scale']`
  - inert scene xform props: `[]`
- Fixed scene:
  - `finding_count = 1`
  - scene-layer props: `['xformOp:transform', 'xformOpOrder']`
  - inert scene xform props: `[]`
- Phase2 audit:
  - `source_descendant_xform_override_count = 1`
  - `pre_c1_inert_descendant_xform_override_count = 0`
  - `became_inert_descendant_xform_override_count = 0`
- Result:
  - pass

### B3. Control asset non-regression check

- Control scene / prim:
  - `home/MV7J6NIKTKJZ2AABAAAAADA8_usd`
  - `/Root/Meshes/Animation/nightstand/model_627f5d31c8996749a1f33bfef27562f3_0`
- Input reports:
  - before:
    [nightstand_geometry_delta_v2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/nightstand_geometry_delta_v2.json)
  - after:
    [nightstand_geometry_delta_fixed.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/nightstand_geometry_delta_fixed.json)
- Pass criteria:
  - control remains within prior small-error range
  - no new outlier introduced by the repair
- Before:
  - aggregate centroid delta: `0.000064`
  - aggregate point delta max: `0.000065`
- After:
  - aggregate centroid delta: `0.000064`
  - aggregate point delta max: `0.000065`
- Result:
  - pass

## C. Normalize Gate Checks

### C1. Pairwise gate

- Input:
  - [pairwise_compare.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/pairwise_compare.json)
- Required fields:
  - `aggregate.displaced_breakdown.gt_0.01`
  - `aggregate.ref_same_breakdown.gt_0.01`
- Pass criteria:
  - `displaced_breakdown.gt_0.01 = 0`
  - `ref_same_breakdown.gt_0.01 = 0`
- Actual result:
  - `displaced_breakdown.gt_0.01 = 0`
  - `ref_changed_breakdown.gt_0.01 = 0`
  - `ref_same_breakdown.gt_0.01 = 0`
- Result:
  - pass

### C2. Phase2 audit gate

- Input:
  - [audit_phase2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/audit_phase2.json)
- Required fields:
  - `aggregate.totals.common_ref_prim_count`
  - `aggregate.totals.center_found`
  - `aggregate.totals.matrix_mismatch`
  - `aggregate.totals.pre_c1_inert_descendant_xform_override_count`
  - `aggregate.totals.became_inert_descendant_xform_override_count`
- Pass criteria:
  - `center_found == common_ref_prim_count`
  - `matrix_mismatch = 0`
  - `pre_c1_inert_descendant_xform_override_count = 0`
  - `became_inert_descendant_xform_override_count = 0`
- Actual result:
  - `common_ref_prim_count = 1105`
  - `center_found = 1105`
  - `matrix_mismatch = 0`
  - `pre_c1_inert_descendant_xform_override_count = 0`
  - `became_inert_descendant_xform_override_count = 0`
- Result:
  - pass

### C3. Unified gate verdict

- Input:
  - [normalize_gate_verdict.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/normalize_gate_verdict.json)
- Pass criteria:
  - verdict status is `pass_normalize_gate`
  - verdict reason does not contain pairwise failure
  - verdict reason does not contain descendant override failure
- Actual result:
  - `status = pass_normalize_gate`
  - `reason = ok`
- Result:
  - pass

# Commands Used

- Pairwise command:
  - `./scripts/isaac_python.sh scripts/placement_pairwise_compare.py --left-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1 --right-root /tmp/test0_desc_override_fix_phase2_v3/out --left-mode current --right-mode current --label test0_desc_override_fix_phase2_v3 --scene-filter MV7J6NIKTKJZ2AABAAAAADA8_usd --workers 1 --out /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/pairwise_compare.json`
- Phase2 audit command:
  - `./scripts/isaac_python.sh scripts/audit_normalize_phase2.py --source-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1 --normalized-root /tmp/test0_desc_override_fix_phase2_v3/out --centers-dir /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/phase1/centers_merged --scene-filter MV7J6NIKTKJZ2AABAAAAADA8_usd --workers 1 --out /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/audit_phase2.json`
- Gate verdict command:
  - `python3 scripts/check_normalize_gate_from_reports.py --pairwise-report /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/pairwise_compare.json --audit-report /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/audit_phase2.json --out /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/normalize_gate_verdict.json`
- Focused geometry diagnostic command:
  - `./scripts/isaac_python.sh scripts/debug_scene_prim_geometry_delta.py --left-stage /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd --right-stage /tmp/test0_desc_override_fix_phase2_v3/out/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd --prim-path /Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0 --out /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/refrigerator_geometry_delta_fixed.json`

# Exit Criteria

- All static checks pass
- Refrigerator focused geometry check passes
- Control asset stays clean
- Normalize gate passes
- Results are written to the cycle report directory
- Final result:
  - all exit criteria satisfied in the isolated `phase2-only` validation

# Deferred / Out Of Scope

- Dedup validation
- S3 validation
- Full-run validation
- Performance benchmarking unless the repair materially changes runtime
