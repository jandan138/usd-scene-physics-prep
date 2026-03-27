---
title: "GRScenes-test0 BBox-Gated Dedup Implementation Spec"
created_at: "2026-03-27"
updated_at: "2026-03-27"
maintainer: "Codex"
status: "active"
code_reference:
  - "scripts/c1_build_bulk_mapping_from_dedup_report.py"
  - "scripts/c1_autorun_categories.py"
  - "scripts/c1_bulk_apply_layout_dedup.py"
  - "scripts/c1_bulk_step6_category_promote_scan_soft_delete.py"
  - "scripts/rewrite_layout_asset_refs_with_compensation.py"
  - "scripts/compute_vertex_transform.py"
  - "scripts/placement_pairwise_compare.py"
  - "scripts/verify_dedup_bbox.py"
  - "tests/test_compute_vertex_transform.py"
  - "tests/test_dedup_compensation_chain.py"
  - "tests/test_patch_placement.py"
---

# Summary

This spec defines the fail-closed implementation path for bbox-gated dedup on
`GRScenes-test0`.

Core contract:

> `cannot prove => do not dedup`

Operational meaning:

- a pair is dedup-eligible only if the repository can prove that replacing the
  old asset reference preserves the scene instance's world-space bbox /
  footprint within hard tolerances
- if proof is unavailable, incomplete, or violated, the pair must remain
  unchanged

Current signed-off policy direction:

- the primary product contract is **coarse scene occupancy correctness**
- therefore `bbox / footprint / centroid` are the primary acceptance gates
- `RMSE` remains a secondary geometry-quality signal unless later evidence
  justifies hardening it for specific modes or categories

This spec replaces the permissive “scan all modes, rewrite, then inspect
results” posture with a gated flow:

1. pair certification
2. filtered mapping generation
3. preflight-first rewrite
4. post-run hard audit
5. only then promotion / soft-delete

# Scope

## Dataset scope

Canonical upstream baseline:

- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup`

Baseline provenance references:

- [GRScenes-test0-rebuilt-normalize-prededup Baseline Adoption](../test0_full/grscenes_test0_rebuilt_prededup_baseline_adoption_20260327.md)
- [prededup_baseline_binding_verdict.json](../../check_reports/test0_rebuilt_full/20260315_chain_fix_v1/summary/prededup_baseline_binding_verdict.json)

## Functional scope

This spec covers:

- pair eligibility and certificate generation
- rewrite-time fail-closed behavior
- authoritative post-run validation
- phased rollout and mode re-enable policy
- test coverage required to support the new contract

This spec does not cover:

- dataset-agnostic generalization beyond test0
- non-bbox product requirements such as dedup yield optimization
- legacy permissive rollout recovery for already-mutated trees

# Canonical Upstream Baseline

The implementation defined here must treat the following root as the only
upstream input baseline:

```text
/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup
```

Reason:

- it is now formally bound to the passed normalize-only rebuilt standardized
  20260315 lineage
- it is clean of direct dedup traces
- it is the stable pre-dedup root for bbox-gated work

No implementation in this spec should assume the current live
`GRScenes-test0-rebuilt-normalized` tree is safe as baseline input, because
that lineage already contains later dedup history.

# Contract

## Hard rule

For every `ref_changed` scene instance in `layout.usd`:

- dedup is allowed only when the repository can prove that the rewritten scene
  instance preserves the original world-space occupied bounds / footprint
  within hard tolerances
- if proof cannot be produced, the pair is `ineligible_for_dedup`

## Definitions

### `ref_changed`

A scene prim whose authored asset reference would change from old asset to
canonical asset in the rewrite.

### bbox

World-space axis-aligned bounding box of the scene instance after composing the
referenced asset geometry and the scene transform.

### footprint

Projected occupied footprint of the scene instance used to detect planar
orientation / extent drift that centroid-only metrics can miss.

Current default:

- flat assets use plane-aware footprint
- non-flat assets rely on world AABB and supporting metrics until a broader
  footprint definition is standardized

### pair certificate

A pre-rewrite eligibility artifact proving whether an old->canonical pair may
enter bbox-gated dedup.

### authoritative post-run validator

The scene-level validator whose exit code determines whether a bbox-gated run
passes. In this spec, the authoritative base is
`scripts/placement_pairwise_compare.py`, extended with bbox/footprint gates.

## Hard gates

The following are hard requirements for an eligible pair:

1. world-space bbox min/max delta on each axis `<= eps_bbox`
2. world-space footprint extent delta `<= eps_bbox`
3. world-space footprint axis delta `<= eps_angle`
4. world-space centroid delta `<= eps_pos`
5. no unresolved compensation state:
   - no aspect-ratio mismatch
   - no size-diff mismatch
   - no plane mismatch without validated handling
   - no unresolved transitive dependency

Recommended current defaults:

- `eps_bbox = 0.01`
- `eps_pos = 0.01`
- `eps_geom = 0.01` for secondary RMSE evaluation and A/B reporting
- `eps_angle = 1 degree`

## Diagnostics only

The following are useful but must not override hard-gate failures:

- vertex RMSE when computable
- bbox IoU summaries
- max-per-mesh displacement summaries
- unchanged-ref aggregate diagnostics

# Immediate Operating Policy

Until the full certificate and validator chain is implemented, bbox-gated mode
must default to the narrowest safe operating surface.

## Allowed in early rollout

- direct single-reference `geom_only`
- compensation succeeds
- no payload rewrite
- no asset-valued attribute rewrite
- no multi-ref changed prim

## Disabled by default in early rollout

- `shape_invariant`
- `transitive`
- current `topo_filesize`
- payload rewrites
- asset-valued attribute rewrites
- multi-ref changed prim compensation

# Implementation Architecture

## 1. Pair certificate before mapping

Before any mapping is consumed by rewrite, the pipeline must compute an
eligibility artifact for each candidate pair.

Minimum required fields:

- `old_asset`
- `canonical_asset`
- `mode`
- `eligible`
- `reject_reason`
- `bbox_delta`
- `footprint_extent_delta`
- `footprint_axis_delta`
- `centroid_delta`
- `vertex_rmse`
- `rmse_available`
- `rmse_unavailable_reason`
- `alternate_proof_kind`
- `alternate_proof_passed`
- `proof_source`

Behavior:

- only `eligible == true` pairs may enter filtered mapping output
- all other pairs remain untouched in layouts
- in phase-0 / phase-1 rollout, RMSE is recorded as secondary evidence rather
  than a universal hard gate

## 2. Preflight-first rewrite

Rewrite must no longer mutate scene references first and diagnose later.

Required behavior:

- no prim mutation until compensability is proven, or the rewrite is revertible
  per prim
- no identity fallback for `ref_changed` prims in bbox-gated mode
- no silent promotion of unsupported modes
- any unsupported or unproven case must keep the original reference and record
  a reject reason

## 3. Authoritative post-run hard gate

Scene-level validation must fail the run if any `ref_changed` prim violates the
contract.

Authoritative validator base:

- `scripts/placement_pairwise_compare.py`

Required additions:

- bbox min/max delta breakdown for `ref_changed`
- footprint extent delta breakdown for `ref_changed`
- footprint axis mismatch / swap counters
- hard-fail exit status when any gate is violated

Supplemental diagnostic:

- `scripts/verify_dedup_bbox.py`
  - keep as helper report
  - do not treat its current output as the sole gate

## 4. Evidence chain

Each bbox-gated run should leave an auditable chain:

1. upstream baseline reference
2. pair certificates
3. filtered mapping
4. rewrite output and reject ledger
5. authoritative audit result
6. promotion / step6 decision

# Phased Rollout

## Phase 0: Policy freeze

Freeze the contract in docs and CLI behavior:

- no proof => no dedup
- bbox-gated mode becomes explicit, not implicit

## Phase 1: Conservative enablement

Enable only the narrow `geom_only` surface described above, and only after a
minimal certificate path exists for that restricted surface.

Clarification:

- until the minimal certificate path for restricted `geom_only` exists, bbox-gated
  mode does not rewrite any scene refs
- Phase 1 is therefore not “allow geom_only without proof”; it is “allow only
  the smallest proof-backed geom_only surface”

Deliverables:

- minimal pair certificate path for restricted `geom_only`
- bbox-gated mode switch
- explicit disallow rules for risky modes and rewrite shapes

## Phase 2: Pair certification

Implement pair certificates and filtered mapping generation.

Deliverables:

- certificate artifact format
- reject reasons
- filtered mapping output
- secondary RMSE / residual evidence recording

## Phase 3: Rewrite fail-closed

Convert rewrite from fail-open to fail-closed.

Deliverables:

- preflight-first rewrite
- removal of identity fallback for `ref_changed` bbox-gated path
- explicit reject ledger

## Phase 4: Validator hard-gating

Promote scene-level validation to authoritative gate.

Deliverables:

- bbox/footprint hard-gate metrics in authoritative validator
- non-zero exit on violations
- run-blocking integration into orchestration

## Phase 5: RMSE Policy A/B Evaluation

Before broad non-`geom_only` enablement, run a controlled A/B comparison of:

- Policy A:
  - bbox-primary, RMSE-observe
- Policy B:
  - bbox-primary plus stricter RMSE-based gating when computable

Required measurements:

- eligible pair count delta by mode/category
- dedup yield delta
- layout drift under bbox/footprint gates
- worst-scene and worst-category concentration
- RMSE availability / unavailability rates by mode/category

Decision goal:

- quantify how much dedup is lost by hardening RMSE
- quantify how much additional layout safety is gained

## Phase 6: Mode re-enable by evidence

Re-enable broader modes only when:

- pair certification exists for that mode
- rewrite path is fail-closed
- post-run validator passes
- dedicated tests exist
- the A/B evidence supports the chosen RMSE policy for that mode/category

Recommended order:

1. `geom_only`
2. `topo_filesize` with explicit residual gate
3. `shape_invariant` only after unsafe cases reject cleanly
4. `transitive` last, after full chain context is wired and validated

# Code Change Map

## Mapping and orchestration

### `scripts/c1_build_bulk_mapping_from_dedup_report.py`

Current issue:

- chooses canonicals by usage only
- has no eligibility or reject-reason concept

Required change:

- produce pair certificates or consume them to emit filtered mapping only

### `scripts/c1_autorun_categories.py`

Current issue:

- current flow is effectively `mapping -> bulk apply -> step6`

Required change:

- change orchestration to:
  - certificate
  - filtered mapping
  - bulk apply
  - audit
  - promotion decision
- add bbox-gated mode flag and run discipline
- add A/B evaluation support for bbox-primary vs bbox+RMSE stricter policy

### `scripts/c1_bulk_apply_layout_dedup.py`

Current issue:

- consumes raw mapping pairs directly

Required change:

- consume only eligible pairs
- pass bbox-gated controls into rewrite layer
- surface reject statistics

### `scripts/c1_bulk_step6_category_promote_scan_soft_delete.py`

Current issue:

- Step6 promotion / soft-delete historically assumes the upstream run is
  promotable once earlier stages complete

Required change:

- require authoritative bbox-gated audit success before promotion or
  soft-delete
- treat bbox-gated audit failure as a hard block on Step6
- surface the audit verdict in promotion logs / summaries

## Rewrite and compensation

### `scripts/rewrite_layout_asset_refs_with_compensation.py`

Current issues:

- fail-open control flow
- mutation before proof
- identity fallback for unsupported/error cases
- no equivalent safe path for payload or asset-valued attribute rewrites

Required change:

- preflight-first per prim
- explicit reject reasons
- no mutation until proof exists
- bbox-gated disallow rules for payload / asset-attr / multi-ref shapes

### `scripts/compute_vertex_transform.py`

Current issues:

- returns transforms, not certification results
- `shape_invariant` lacks usable residual gate
- ICP RMSE is dropped from decision logic
- `transitive` live path is incomplete

Required change:

- introduce structured certification result layer
- preserve residual metrics for decision making
- expose unresolved states as rejectable outcomes

## Validation

### `scripts/placement_pairwise_compare.py`

Required role:

- authoritative post-run validator

Required change:

- add bbox/footprint hard-gate outputs
- add hard-fail exit behavior
- ensure `ref_changed` vs `ref_same` stays explicit
- preserve RMSE outputs as secondary analysis metrics even when not used as a
  universal hard gate

### `scripts/verify_dedup_bbox.py`

Required role:

- supplemental bbox diagnostic helper

Required change:

- either remain diagnostic only, or contribute helper calculations that are
  reused by the authoritative validator

# Validation and Acceptance

## Pair-level acceptance

A pair is eligible only if:

- all hard metrics are within tolerance
- no unresolved compensation state remains

RMSE handling in the current rollout:

- when computable, record `RMSE` as secondary geometry-quality evidence
- when unavailable, record the reason explicitly
- `RMSE unavailable` does not automatically reject a pair in phase-0 / phase-1
- later mode/category hardening may still require RMSE or another stronger
  approved proof source based on A/B evidence

## Run-level acceptance

For bbox-gated mode, the run passes only if all of the following hold:

- `scenes_error == 0`
- `total_no_mesh == 0`
- compared scope is complete
- `ref_changed` hard-gate failures are zero
- no supposedly eligible pair is rewritten through a reject path

## Required tests

Add or update tests for:

- pair certificate generation
- reject reason propagation
- no mutation on failed preflight
- payload/asset-attr disallow behavior
- validator hard-fail behavior
- mode-specific re-enable tests

Existing tests requiring attention:

- `tests/test_compute_vertex_transform.py`
  - currently contains older permissive `shape_invariant` expectations
- `tests/test_patch_placement.py`
  - remains useful for composed-world invariance math
- `tests/test_dedup_compensation_chain.py`
  - remains useful for chain-walk / transform correctness

# Open Decisions

These items still require user sign-off before the spec is treated as fully
locked.

1. Threshold policy
   - keep `0.01` global absolute threshold
   - or move to absolute + relative mixed thresholds

2. Footprint definition
   - global XZ projection
   - or plane-aware footprint for flat assets

3. RMSE hardening policy after A/B evaluation
   - remain secondary signal only
   - become mode-specific hard gate
   - become category-specific hard gate

4. Phase-0 operating mode
   - keep restricted `geom_only`
   - or pause all dedup until pair certification exists

Current recommended defaults:

- `eps_bbox = 0.01`
- `eps_pos = 0.01`
- `eps_geom = 0.01`
- `eps_angle = 1 degree`
- flat assets use plane-aware footprint
- RMSE is secondary evidence in phase-0 / phase-1
- run A/B evaluation before hardening RMSE into a broader gate
- early rollout keeps only restricted `geom_only`

# References

- [GRScenes-test0-rebuilt-normalize-prededup Baseline Adoption](../test0_full/grscenes_test0_rebuilt_prededup_baseline_adoption_20260327.md)
- [GRScenes-test0 Full Dedup Handoff Runbook](grscenes_test0_full_dedup_handoff_runbook.md)
- [资产去重落地方案 C1 执行手册（Scene 侧引用归一 + Instancing）](asset_dedup_c1_scene_instancing_runbook.md)
- [C1 规模化执行与审核流程（可控批处理版）](asset_dedup_c1_scaling_workflow.md)

Policy and synthesis sources:

- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.codex/worklogs/main/2026-03-25/bbox-gated-dedup-handoff.md`
- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.codex/worklogs/main/2026-03-27/baseline-confirmation-handoff.md`
- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/.codex/worklogs/main/2026-03-27/baseline-closure-handoff.md`
