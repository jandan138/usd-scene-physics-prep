# Test0 Smoke S1 Summary

- Run ID: `20260312_074731`
- Verdict: `fail_normalize_s1`
- Stop reason: normalize-only gate failed before dedup
- Pairwise normalize-only: displaced `> 0.01` = 1
- Audit phase2: center_found = 1105 / common_ref_prim_count = 1105
- Audit phase2: matrix_mismatch = 0

## Key outlier

- Prim: `/Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0`
- Category: `refrigerator`
- Displacement: `0.061537`
- Max per-mesh displacement: `1.244992`

## Reports

- normalize pairwise: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/s1_test0_vs_normalized_pre_dedup.json`
- normalize audit: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/s1_normalize_phase2_audit.json`
- phase2 report: `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/phase2/normalize_report.json`

## Decision

Do not continue to dedup on this run. Investigate the refrigerator placement outlier first.
