---
title: Smoke Test Gap Analysis - Matrix Multiply Order Bug Detection Failure (2026-03-15)
code_reference:
- scripts/test0_rebuilt_normalize.py
- scripts/test0_full_normalize.py
- scripts/placement_pairwise_compare.py
created_at: '2026-03-15'
updated_at: '2026-03-15'
maintainer: team-lead
status: completed
---

# Smoke Test Gap Analysis - Matrix Multiply Order Bug Detection Failure

## Executive Summary

The smoke test for the V2 asset normalization pipeline **failed to detect** a critical matrix multiply order bug that caused 487 prims across 37 scenes to be displaced (up to 854.5 meters). The bug was discovered only during the full pipeline run.

**Root cause**: The selected smoke scene happened to have minimal affected assets (only 1 displaced prim), making the issue invisible in the test results. The pairwise comparison **was running**, but no validation checked whether displacement should be zero.

## Smoke Test Configuration

### Default Smoke Scene

**Scene UID**: `MV7J6NIKTKJZ2AABAAAAADA8` (home category)

**Location in source**:
```
GRScenes-test0-rebuilt/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd
```

**Smoke output**:
```
GRScenes-test0-rebuilt-smoke-S1/
GRScenes-test0-rebuilt-smoke-S1-normalized/
check_reports/test0_rebuilt_smoke/*/normalize/test0_vs_normalized_pre_dedup.json
```

### Scene Selection

The smoke scene was selected with no explicit criteria other than being a single valid scene from the dataset. The default is hardcoded in `scripts/test0_rebuilt_normalize.py:1256`:

```python
smoke.add_argument(
    "--scene-uid",
    default="MV7J6NIKTKJZ2AABAAAAADA8",
    help="Scene UID to package for smoke testing",
)
```

## Detection Gap Details

### What the Smoke Test DID Detect

The smoke test **did run pairwise comparison** and **did identify displacement**:

**Smoke Scene Pairwise Results**:
```json
{
  "scene_id": "home/MV7J6NIKTKJZ2AABAAAAADA8_usd",
  "total_common": 1105,
  "total_compared": 1105,
  "displaced_breakdown": {
    "gt_0.01": 1,
    "gt_0.1": 1,
    "gt_1.0": 1,
    "gt_10.0": 0
  },
  "category_breakdown": {
    "other": {
      "count": 149,
      "displaced_breakdown": {
        "gt_0.01": 1
      }
    }
    // ... 51 other categories all show 0 displacement
  }
}
```

**Key metrics**:
- Displaced prims (>0.01m): **1 of 1,105** (0.09%)
- Max displacement: **2.13 meters**
- Affected category: **other** (1 of 149 prims)
- Test outcome: **Passed** (no validation checked for zero displacement)

### What the Smoke Test Did NOT Do

1. **No validation**: The pairwise results were not checked for displacement violations
2. **No failure condition**: Reporting displacement > 0 did not cause test failure
3. **No comprehensive coverage**: One scene cannot be expected to catch sparse, inconsistent bugs

### What the Full Run Revealed

**Full Run Pairwise Results** (test0_rebuilt_full, 99 scenes):
```
Total displaced prims (>0.01m): 487 of 101,919 (0.48%)
Affected scenes: 37 of 99
Affected categories: 21
Max displacement: 854.5 meters
```

**Category breakdown** (affected >0 prims):
- other: 93 displaced (10.9% of 854 total)
- wall: 87 displaced (4.2% of 2067 total)
- ground: 72 displaced (2.0% of 3560 total)
- refrigerator: 31 displaced (31.6% of 98 total)
- table: 28 displaced (9.6% of 291 total)
- nightstand: 21 displaced (24.1% of 87 total)
- desk: 19 displaced (27.9% of 68 total)
- ... (13 more categories)

## Why the Gap Occurred

### 1. Inconsistent Bug Manifestation

The `_get_chain_transform` matrix multiply order bug only manifests when:

**Trigger conditions** (all must be true):
1. Asset has intermediate Transform/Group prim(s) between `/Root/Instance` and mesh
2. The asset's specific transformation chain activates the bug (depends on matrix elements)
3. The asset is normalized (not all assets in a category may be affected)

**Asset structure triggering the bug**:
```usda
def Xform "Instance"
{
    def Xform "Group_default_00"  # <-- INTERMEDIATE PRIM
    {
        def Mesh "SM_00_obj10_0" { ... }
        def Mesh "SM_01_obj9_0" { ... }
    }
}
```

### 2. Incomplete Coverage in Smoke Scene

The smoke scene `MV7J6NIKTKJZ2AABAAAAADA8` contains all 52 categories with 1,105 total prims, **but**:

- **other category**: 149 prims, **1 displaced** (0.7%)
- **wall category**: 452 prims, **0 displaced** (0.0%)
- **ground category**: 254 prims, **0 displaced** (0.0%)

The full run shows:
- **other category**: 854 prims, **93 displaced** (10.9%)
- **wall category**: 2,067 prims, **87 displaced** (4.2%)
- **ground category**: 3,560 prims, **72 displaced** (2.0%)

The smoke scene sampled only ~17% of `other` assets and ~22% of `wall` assets, and happened to select the "lucky" subset without displacement.

### 3. No Validation of Displacement

The smoke test checks for:
- ✓ Source hash preservation (assets not modified)
- ✓ Main USD files are not symlinks in output
- ✓ Assets directory was created
- ✗ **Pairwise results are zero displacement** (NOT CHECKED)

The pairwise comparison runs and reports displacement, but the validation does not fail if displacement > 0 is found.

## Comparison with Full Run

| Aspect | Smoke Test | Full Run | Gap |
|--------|-----------|----------|-----|
| Scenes processed | 1 | 99 | 98x smaller |
| Prims processed | 1,105 | 101,919 | 92x smaller |
| Displaced prims found | 1 | 487 | 487x smaller |
| Test failed | No | No | (but hard gate failed on different metric) |
| Pairwise comparison | Yes | Yes | None |
| Displacement validated | No | No | **ROOT CAUSE** |

## Why This Matters

### Discovery Timing

1. **Pre-fix**: Smoke test passed (only 1 displaced prim, no validation)
2. **Post-fix**: Smoke test passed (0 displaced prims, no validation)
3. **Full run**: Hard gate FAILED (487 displaced prims detected by audit)

The bug was **hidden in plain sight** in the smoke test results. The only reason it was discovered was through the full run's additional audit step (`test0_normalize_phase2_audit.json`).

### Risk Implications

- A developer could have deployed the fix claiming "smoke test passed" without realizing the widespread impact
- The full run took hours/days to discover; smoke test took minutes
- Future bugs with similar sparse manifestation patterns could evade smoke testing

## Recommendations

### Immediate (PR Review)

1. **Validate displacement in smoke test**: Fail if any prim has displacement > 0.01m
2. **Document known affected scenes**: For future reference, list which scenes have intermediate-prim assets

### Short-term (1-2 weeks)

1. **Improve smoke scene selection**:
   - Option A: Use a different scene with higher density of edge-case assets
   - Option B: Select the worst scene from the full run's pairwise report
   - Option C: Select 3 diverse scenes instead of 1

2. **Add regression test** (already implemented in Task #3):
   - Select 5 highest-displacement assets from full run
   - Create minimal test scene with only these assets
   - Run normalization and verify zero displacement
   - Run as part of CI/CD pipeline

### Long-term (Architecture)

1. **Formalize smoke test contract**:
   - Define what checks must pass (currently implicit)
   - Specify minimum coverage requirements
   - Document how to add new validation checks

2. **Add smoke test metrics tracking**:
   - Report displacement percentiles (min/max/p95)
   - Compare against baseline thresholds
   - Alert if metrics regress

3. **Integrate pairwise validation**:
   - Add option to fail pipeline if displacement > threshold
   - Make threshold configurable (default: 0.01m)
   - Report violations in structured format

## Related Documentation

- **Matrix multiply order bug**: `docs/changes/2026-03-15_matrix_multiply_order_fix.md`
- **Hard gate failure analysis**: `docs/test0_smoke/20260312_074731/summary.md`
- **Pairwise comparison design**: `scripts/placement_pairwise_compare.py` (header comments)
- **Test orchestration**: `scripts/test0_rebuilt_normalize.py` (smoke command section)

## Test Evidence

### Smoke Test Report
```
check_reports/test0_rebuilt_smoke/20260314_rebuilt_smoke_s1/
├── normalize/
│   └── phase1/  (only phase 1 ran, no pairwise or audit)
└── preflight/
    └── baseline_verification.json
```

### Full Run Report
```
check_reports/test0_rebuilt_full/20260314_rebuilt_full_dlc_v1/
├── normalize/
│   ├── phase1/  (per-category)
│   ├── phase2/  (per-scene)
│   ├── test0_vs_normalized_pre_dedup.json  (pairwise: 487 displaced)
│   └── test0_normalize_phase2_audit.json  (caught displacement in hard gate)
└── summary/
    └── final_verdict.json  (FAIL: pairwise_displaced_gt_0.01 = 487)
```

## Conclusion

The smoke test gap was a **systematic coverage limitation**, not a tool failure. The pairwise comparison algorithm works correctly; the issue was:

1. **No validation** of pairwise results in smoke test
2. **Sparse manifestation** of the bug in the selected scene
3. **No feedback loop** from full run back to smoke test configuration

The recommended solution is to add explicit displacement validation to the smoke test with a zero-tolerance policy for any detected displacement > 0.01m.
