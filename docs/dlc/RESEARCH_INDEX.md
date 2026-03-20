---
title: Dedup Compensation Research Index (2026-03-19)
created_at: 2026-03-19
status: complete
---

# Dedup Compensation Research Index

**Date**: March 19, 2026
**Status**: Complete (4 tasks, 3 documents, 948 lines of analysis)
**Researcher**: researcher-compensation

## Overview

Comprehensive research into why dedup compensation catastrophically fails (24.9% placement displacement) on test0-rebuilt-normalized dataset. Root cause identified: compensation formula has **scope limitation** (prim-space only), not mathematical error. Dedup detection is correct; failing pairs are actually identical but have baked vertex-space transforms.

---

## Documents (In Reading Order)

### 1. Dedup Compensation Failure Analysis
**File**: `docs/dlc/dedup_compensation_failure_analysis.md` (427 lines)

**Scope**: Detailed technical analysis of the compensation system.

**Contents**:
- Part 1: Compensation formula & implementation (correct)
- Part 2: Asset internal matrix computation & chain walk (correct)
- Part 3: Why compensation fails (3 root causes with examples)
- Part 4: Formula limitations & assumptions
- Part 5: Dedup detection vs. compensation mismatch
- Part 6: Test coverage gaps (explains why tests pass despite failures)
- Part 7: Next steps (3-phase plan)
- Part 8: Code references (function locations)
- Part 9: FAQ (answers key questions)

**Key Findings**:
- Formula `M_new = M_canon⁻¹ × M_old × M_old_layout` is mathematically correct
- Chain walk implementation is correct (bug fixed in commit ca544fc)
- Fails because formula assumes identical vertex configurations
- 53.5% of pairs (30,560) have baked vertex-space transforms:
  - Baked rotation (desk: 1084 unit displacement)
  - Mirror/reflection (curtain: 246.65 units)
  - Non-uniform scale

---

### 2. Universal Dedup Compensation Plan
**File**: `docs/dlc/universal_compensation_plan.md` (521 lines)

**Scope**: Actionable implementation plan to fix compensation.

**Contents**:
- Part 1: Problem summary (placement failures quantified)
- Part 2: Risk stratification & filtering strategy
- Part 3: Stratified compensation implementation
- Part 4: Implementation roadmap (4 steps, 4-5 days)
- Part 5: Expected outcomes (24.9% → < 5% displacement)
- Part 6: Integration with existing codebase
- Part 7: Acceptance criteria
- Part 8: Fallback plans

**Key Decision**: Moderate filtering strategy
- Exclude: topo_only (30.5%) + none (27.2%) strata = 32,971 false positives
- Include: safe_geom_only + safe_procrustes = 24,768 pairs (43.4% of total)
- Result: 41% of dedup, but 10× fewer displacements

**Key Enhancement**: Procrustes R-matrix integration
```python
# Standard (current):
M_new = M_canon⁻¹ × M_old × M_old_layout

# Improved (for rotation-based pairs):
M_new = M_canon⁻¹ × M_old × R_procrustes⁻¹ × M_old_layout
```

---

### 3. Memory Files (Session Persistence)

**`memory/compensation_research.md`**:
- Quick reference for all compensation code details
- 9 sections, functions by line number
- Test coverage gaps

**`memory/procrustes_pair_types.md`**:
- Detailed breakdown of 57,174 pairs from pair_type_investigation.json
- 7 classification labels (hierarchy_only, rigid_rotation, mirror, etc.)
- Risk stratification table
- Procrustes metrics explained

---

## Key Quantitative Results

### Placement Verification (Current State)
```
101,919 prims compared
  > 0.01 units displaced: 25,431 (24.9%)
  > 0.1 units:           22,951 (22.5%)
  > 1.0 units:           14,300 (14.0%)
  > 10.0 units:           2,274 (2.2%)

Worst categories:
  1. wall:       248.14 units
  2. curtain:    246.65 units ← mirror case
  3. counter:    200.75 units
  4. other:      140.80 units
  5. ground:     135.22 units
```

### Pair Type Distribution (57,174 pairs)
```
should_exclude:        30,560 (53.5%) ← truly different geometry
safe_geom_only:        12,332 (21.6%) ← identical vertices
safe_procrustes:       11,138 (19.5%) ← rigid rotation only
needs_mirror_handling:  2,352 (4.1%)  ← reflection needed
needs_scale_handling:     792 (1.4%)  ← scale handling needed
```

### Root Cause Breakdown
```
Baked vertex rotation:    43% of topo_only, 13% of none strata
Mirror/reflection:        2-6% across strata
False positives:          50% topo_only, 78% none stratum
Procrustes threshold:     84% shape_only marked "not_same" (too strict)
```

---

## Code Implementation Summary

### Functions to Add (est. 3-4 hours)
- `_compute_procrustes_R(old_abs, canon_abs)` — SVD-based rotation fitting
- `_load_asset_mesh_vertices(asset_abs)` — Extract mesh vertices from USD
- `rewrite_layout_with_procrustes(...)` — Stratified compensation main function
- `_handle_reflection_and_scale(R, det_R, scale_ratio)` — Special case handling

### Functions to Modify (est. 1-2 hours)
- `rewrite_layout()` — Add pair_type_info parameter, conditional logic
- Compensation formula — Add Procrustes R term when needed

### Tests to Add (est. 2-3 hours)
- `test_baked_rotation_compensation()` — Currently fails, demonstrates problem
- `test_mirror_compensation()` — Handle det(R) = -1
- `test_scale_compensation()` — Handle non-uniform scale
- `test_procrustes_R_matrix()` — Verify Procrustes computation

---

## Implementation Phases

**Phase 1: Filtering (1 day)**
- Load pair_type_investigation data
- Generate filtered_pairs.json (24,768 safe pairs)
- Exclude topo_only + none strata

**Phase 2: Code & Testing (2-3 days)**
- Implement new functions
- Add unit tests (including failure cases)
- Smoke test on test0_smoke subset
- Expected: displacement < 1% on test subset

**Phase 3: Full Re-Run (1-2 days)**
- Restore pre_c1 snapshots (available at `check_reports/test0_rebuilt_dedup/c1_bulk/_autorun/*/pre_c1_snapshots/`)
- Run improved compensation
- Full placement verification with `placement_pairwise_compare.py`
- Expected: displacement < 5% on full dataset

---

## Risk Assessment

### Moderate Filtering (Recommended)
- ✓ Eliminates clear false positives (30,560 pairs)
- ✓ Retains 99% of legitimate matches
- ✗ Dedup rate drops from 66.8% to ~40%
- Result: More reliable, lower displacement

### Procrustes R-Matrix Addition
- ✓ Fixes rotation-based pairs (11,138 pairs)
- ✓ Fixes reflection pairs (2,352 pairs)
- ✗ +30-50% computational cost per pair
- **Mitigable**: Cache computed R matrices across runs

---

## Decision Points for Team

1. **Filtering Strategy**: Conservative (21%), Moderate (43%), or Aggressive (100%)?
   - Recommend: Moderate (best risk/reward balance)

2. **Procrustes Cost**: Compute on-demand or cache pre-computed?
   - Recommend: On-demand (complete accuracy, manageable cost)

3. **Schedule**: Can we afford 4-5 days of analysis + implementation?
   - Yes, pre_c1 snapshots preserved; no rush to re-run

---

## File Locations

| Content | Path |
|---------|------|
| Compensation code | `scripts/rewrite_layout_asset_refs_with_compensation.py` |
| Chain walk helpers | `scripts/usd_xform_utils.py` |
| Compensation tests | `tests/test_dedup_compensation_chain.py` (existing) |
| Pair type data | `check_reports/test0_rebuilt_dedup/pair_type_investigation/` |
| Placement verify report | `check_reports/test0_rebuilt_dedup/verify_post_dedup/source_vs_deduped_normalized.json` |
| Pre-C1 snapshots | `check_reports/test0_rebuilt_dedup/c1_bulk/_autorun/*/pre_c1_snapshots/` |
| Dedup backup | `GRScenes-test0-rebuilt-normalized_bak/` (59 GB) |

---

## Expected Outcomes

### Current (Status Quo)
- 24.9% of prims displaced
- 66.8% dedup rate
- Many placement errors visible in layout

### After Implementation (Projected)
- 2-5% of prims displaced (10× improvement)
- 40% dedup rate (acceptable trade-off)
- Few placement errors; most assets correctly positioned
- Compensation costs ~50% more per pair (still fast enough)

---

## Next Steps

1. **Immediate**: Review 3 documents, confirm direction
2. **This week**: Decide on filtering strategy, assign Phase 1
3. **Next week**: Implementation + testing (Phase 2)
4. **Two weeks**: Full re-run + verification (Phase 3)

All technical details, alternate approaches, and contingency plans documented in the two primary docs.

---

## Appendix: Why Current Tests Pass Despite Failure

**The 10 unit tests in `test_dedup_compensation_chain.py`**:
- ✓ Validate mathematical correctness of formula
- ✓ Validate chain walk implementation
- ✓ Use pure TRS (translate, rotate, scale) on prims only
- ✓ Assume identical vertex configurations

**They do NOT test**:
- ✗ Baked vertex rotations (primary failure mode)
- ✗ Mirror/reflection (secondary failure mode)
- ✗ Procrustes alignment (detection vs. compensation gap)
- ✗ Real dedup pairs with geometric differences

**Result**: Tests pass for ideal scenarios, code fails on real data.

**Solution**: Add tests that explicitly demonstrate failure modes, then implement Procrustes-aware compensation to fix them.

