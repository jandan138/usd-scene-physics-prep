---
title: "GRScenes-test0 Phase1 Status And Phase2 Probe Plan"
created_at: "2026-03-14"
updated_at: "2026-03-14"
maintainer: "Codex"
status: "active"
code_reference:
  - "scripts/normalize_asset_transforms.py"
  - "scripts/assemble_normalize_gate_bundle.py"
  - "scripts/placement_pairwise_compare.py"
  - "scripts/audit_normalize_phase2.py"
---

# Summary

This document records the currently accepted status of `GRScenes-test0` Phase 1
and the agreed next-step plan for the normalize-only continuation.

Execution handoff document:

- `docs/records/runs/test0-full/grscenes_test0_normalize_only_continuation_execution_plan_20260314.md`

Current operator judgment:

- the known Phase 1 bad assets are likely non-impacting empty-mesh placeholders
- do **not** rerun Phase 1 now
- treat the existing Phase 1 output as canonical input
- continue with `phase2 -> pairwise -> audit -> normalize-only hard gate`
- execution of this continuation will be handled in a separate Codex dialogue

# Current Status

## Canonical Phase 1 Input

Use the following as the authoritative Phase 1 output set:

- phase1 run root:
  `check_reports/test0_full/20260313_023200`
- asset output root:
  `GRScenes-test0-normalized/GRScenes_assets`
- centers file:
  `check_reports/test0_full/20260313_023200/normalize/phase1/centers_all.json`
- phase1 log:
  `check_reports/test0_full/20260313_023200/logs/phase1.log`

## Verified Metrics

- assets normalized:
  `85612 / 85647`
- errors:
  `35`
- elapsed:
  `56416.7s` (`~15.7h`)
- centers output:
  `centers_all.json` present with `85612` entries

## Important Interpretation

The wrapper run `20260313_023200` is marked failed only because it treated the
Phase 1 non-zero exit as a terminal failure:

- `summary/run_status.json`:
  `status = failed`
- wrapper error:
  `Step failed: phase1 (exit code 1)`

That wrapper status must **not** be treated as “Phase 1 produced no usable
output”. Phase 1 itself completed and emitted the canonical assets + centers
artifacts above.

# Bad Asset Assessment

## Working Assumption

The current working assumption is that the `35` Phase 1 failures are
non-impacting empty-mesh / placeholder assets and should not block moving into
Phase 2.

## Known Observations

- at least one classic known-bad asset is present:
  `cabinet/b98d6ccbeb75dfdeb60e27649a5b055a/...`
- the error set also includes multiple:
  `door_<UUID>/d41d8cd98f00b204e9800998ecf8427e/...`
- these failures are all of the form:
  `No meshes found under /Root/Instance`

## Caveat To Preserve

Although the operating assumption is “non-impacting empty mesh”, the execution
dialogue should still keep one explicit check:

- if normalize-only hard gate fails, first test whether the failure traces back
  to any of these `35` missing-center / missing-asset cases before concluding
  that Phase 2 logic is wrong

# Continuation Plan

## 1. Freeze Phase 1 As Input, Do Not Rerun

Do not mutate or overwrite:

- `check_reports/test0_full/20260313_023200/normalize/phase1/centers_all.json`
- `GRScenes-test0-normalized/GRScenes_assets`

Do not attempt to “repair” the wrapper failure in-place.

## 2. Start A Fresh Continuation Run Root

Create a new continuation report root, separate from the failed wrapper run,
for example:

- `check_reports/test0_full/20260314_phase2_probe_from_20260313_023200`

Reason:

- keep the old wrapper failure as historical record
- keep Phase 2 / gate outputs from being mixed with the old `run_status.json`

## 3. Run Phase 2 Against The Existing Canonical Inputs

Use:

- source root:
  `GRScenes-test0`
- normalized root:
  `GRScenes-test0-normalized`
- centers dir:
  `check_reports/test0_full/20260313_023200/normalize/phase1`

Expected output:

- `normalize/phase2/normalize_report.json`

## 4. Create Pre-C1 Scene Snapshots

Immediately after Phase 2, create `layout.pre_c1_*` snapshots for all normalized
scene layouts under:

- `GRScenes-test0-normalized/GRScenes100/...`

This is required so `audit_normalize_phase2.py` does not fail on
`missing_pre_c1`.

## 5. Run Normalize-Only Validation

Write these outputs under the continuation run root:

- pairwise:
  `normalize/test0_vs_normalized_pre_dedup.json`
- audit:
  `normalize/test0_normalize_phase2_audit.json`
- gate:
  `summary/normalize_gate_verdict.json`
- final verdict:
  `summary/final_verdict.json`
- operator summary:
  `summary/summary.md`

## 6. Apply Hard Gate Exactly

The continuation is considered a full normalize-only pass only if all are true:

- pairwise `displaced > 0.01 = 0`
- pairwise `ref_same > 0.01 = 0`
- audit `center_found == common_ref_prim_count`
- audit `matrix_mismatch = 0`
- audit `missing_pre_c1 = 0`
- audit `pre_c1_inert_descendant_xform_override_count = 0`
- audit `became_inert_descendant_xform_override_count = 0`

# Decision Rules

## Pass

If the hard gate is fully green:

- accept the current Phase 1 output as sufficient
- treat the `35` bad assets as operationally non-impacting
- close normalize-only and prepare the handoff into dedup planning/execution

## Fail

If the hard gate fails:

- first test whether the failing scenes / prims intersect the known Phase 1 bad
  assets
- if yes, reopen the assumption that the “empty mesh assets are harmless”
- if no, treat the failure as a true normalize / phase2 compensation issue

# Hand-Off Notes For The Next Codex Dialogue

- the next dialogue should **not** spend time re-planning Phase 1
- it should treat this document as the continuation brief
- it should explicitly preserve the distinction between:
  - wrapper-level failure on `20260313_023200`
  - actual Phase 1 output usability
- it should keep all new outputs in a fresh continuation report root
- it should not enter dedup in the same execution chain

# Minimal Operator Checklist

- confirm `centers_all.json` exists and is readable
- confirm `GRScenes-test0-normalized/GRScenes_assets` exists
- confirm `GRScenes-test0-normalized/GRScenes100` is either absent or not yet
  phase2-written before starting continuation
- run Phase 2
- create `pre_c1` snapshots
- run pairwise
- run audit
- build gate verdict
- stop before dedup
