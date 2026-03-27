---
title: "GRScenes-test0 BBox-Gated Dedup Task Breakdown"
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

This document turns the signed-off bbox-gated dedup spec into an implementation
task breakdown for `GRScenes-test0`.

Current execution status is tracked separately in:

- `docs/operations/grscenes_test0_bbox_gated_dedup_status_20260327.md`

It assumes the following are already settled and must not be reopened here:

- canonical upstream baseline:
  `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup`
- product contract:
  coarse scene occupancy correctness
- primary gates:
  `bbox / footprint / centroid`
- unresolved compensation state:
  hard reject
- early rollout RMSE policy:
  secondary evidence in phase-0 / phase-1
- required A/B evaluation:
  - Policy A: `bbox-primary, RMSE-observe`
  - Policy B: `bbox-primary + RMSE-harder`
- bbox-fail handling:
  revoke the attributable `old -> canonical` edge, then rebuild the certified
  graph and canonical selection; do not invalidate the whole duplicate group

The goal is to land the minimal safe execution path first:

1. certificate and filtered mapping
2. fail-closed rewrite
3. authoritative audit
4. audit-gated promotion

# Fixed Inputs

Baseline binding artifacts:

- `docs/test0_full/grscenes_test0_rebuilt_prededup_baseline_adoption_20260327.md`
- `check_reports/test0_rebuilt_full/20260315_chain_fix_v1/summary/prededup_baseline_binding_verdict.json`

Governing implementation spec:

- `docs/operations/grscenes_test0_bbox_gated_dedup_implementation.md`

Handoff notes already reflected into this breakdown:

- `.codex/worklogs/main/2026-03-27/bbox-gated-impl-spec-handoff.md`
- `.codex/worklogs/main/2026-03-27/baseline-closure-handoff.md`
- `.codex/worklogs/main/2026-03-27/bbox-gated-proposal-review-handoff.md`
- `.codex/worklogs/main/2026-03-27/bbox-vs-rmse-handoff.md`
- `.codex/worklogs/main/2026-03-25/bbox-gated-dedup-handoff.md`

# Current Code Reality

The current repo still runs a permissive C1 flow:

1. `c1_build_bulk_mapping_from_dedup_report.py`
2. `c1_bulk_apply_layout_dedup.py`
3. `c1_bulk_step6_category_promote_scan_soft_delete.py`

`c1_autorun_categories.py` currently orchestrates exactly that
`mapping -> bulk_apply -> step6` sequence.

Important gaps relative to the new contract:

- no pair certificate artifact exists yet
- mapping build has no eligibility or reject-reason concept
- rewrite still mutates refs once a raw mapping pair survives a narrow prefilter
- payload and asset-valued attribute rewrites are still possible
- multi-ref changed prims are logged as ambiguous, not fail-closed preflight rejects
- `placement_pairwise_compare.py` computes useful metrics but does not hard-fail
  the run
- Step6 still assumes earlier stages are promotable if scans are clean

# Artifact Layout

For phase-0 and phase-1, use a dedicated test0 workspace rooted at:

```text
check_reports/test0_bbox_gated/<run_id>/
```

Recommended structure:

```text
check_reports/test0_bbox_gated/<run_id>/
  run_manifest.json
  summary/
    run_summary.json
    ab_policy_comparison.json
    ab_policy_comparison.md
  policy_a/
    <category>/
      01_cert/
        pair_certificates.jsonl
        pair_certificate_summary.json
        certified_graph.json
        filtered_mapping.json
        filtered_mapping.stats.json
      02_apply/
        batch_summary.json
        spotcheck_list.md
        rewrite_reject_ledger.jsonl
        per_layout/*.json
      03_audit/
        placement_pairwise_compare.json
        verify_dedup_bbox.json
        audit_verdict.json
        revoked_edges.jsonl
      04_step6/
        step6_gate_decision.json
        promote_to_layout_usd_report.json
        post_promote_layout_scan_pxr.json
        post_promote_full_usd_scan_excluding_backups_pxr.json
        soft_delete_old_assets_report.json
        post_soft_delete_layout_scan_pxr.json
  policy_b/
    ...
```

Notes:

- `pair_certificates.jsonl` should be the pair-level source of truth because it
  scales better than one large nested JSON.
- `filtered_mapping.json` remains the rewrite input surface for backward
  compatibility with current C1 scripts.
- `audit_verdict.json` must be the single promotability contract for Step6.
- `revoked_edges.jsonl` is the repair input when an attributable bbox failure
  is found post-run.

# Phase 0

Phase 0 is the non-promoting implementation landing. The target state is:

- bbox-gated mode exists explicitly in CLI/orchestration
- a pair certificate can be produced for the restricted early-rollout surface
- filtered mapping is emitted from certified edges only
- the authoritative audit can write a verdict artifact and fail non-zero
- Step6 can consume the verdict contract, but actual promotion remains blocked
  until phase-1 acceptance

## Phase 0 First Scripts To Change

1. `scripts/compute_vertex_transform.py`
2. `scripts/c1_build_bulk_mapping_from_dedup_report.py`
3. `scripts/placement_pairwise_compare.py`
4. `scripts/c1_autorun_categories.py`
5. `scripts/verify_dedup_bbox.py`

Rationale:

- `compute_vertex_transform.py` is the lowest-level place that already knows
  mode dispatch, residuals, and unresolved states.
- `c1_build_bulk_mapping_from_dedup_report.py` already owns group-to-canonical
  selection and is therefore the correct first home for certified-edge rebuild.
- `placement_pairwise_compare.py` already computes `ref_changed`,
  displacement, and `vertex_rmse`; it is the least disruptive place to turn the
  audit into a hard gate.
- `c1_autorun_categories.py` must stop hardcoding the old three-stage pipeline
  and begin emitting run manifests for cert/apply/audit separation.
- `verify_dedup_bbox.py` stays supplemental, but phase-0 should align its helper
  outputs with the authoritative audit.

## Phase 0 Tasks

| Task | Scripts | Deliverable | New / Updated Artifacts | Depends On | Can Run In Parallel |
| --- | --- | --- | --- | --- | --- |
| P0-T1 | `scripts/compute_vertex_transform.py` | Introduce structured certification result instead of only raw `V` | certification result objects embedded in pair-cert rows; explicit `eligible`, `reject_reason`, `rmse_available`, `rmse_unavailable_reason`, `proof_source`, `alternate_proof_kind` | none | with P0-T3 once result schema is fixed |
| P0-T2 | `scripts/c1_build_bulk_mapping_from_dedup_report.py` | Build pair certificates, certified graph, and filtered mapping from certified edges only | `pair_certificates.jsonl`, `pair_certificate_summary.json`, `certified_graph.json`, `filtered_mapping.json`, `filtered_mapping.stats.json` | P0-T1 | with P0-T4 after output paths are agreed |
| P0-T3 | `scripts/placement_pairwise_compare.py` | Add bbox / footprint hard-gate reporting and non-zero exit on run failure | `placement_pairwise_compare.json`, `audit_verdict.json` | schema from P0-T1 for field names | with P0-T2 |
| P0-T4 | `scripts/c1_autorun_categories.py` | Change orchestration from `mapping -> bulk_apply -> step6` into `cert -> filtered_mapping -> apply -> audit -> step6_gate` | `run_manifest.json`, per-category stage directories, updated ledger rows for cert/apply/audit/step6_gate | output contracts from P0-T2 and P0-T3 | partially with P0-T2 and P0-T3 |
| P0-T5 | `scripts/verify_dedup_bbox.py` | Keep helper diagnostic aligned with bbox-gated audit schema | `verify_dedup_bbox.json` with fields that mirror authoritative audit summaries where possible | none | with P0-T3 |

## Phase 0 Detailed Notes

### P0-T1. Structured Certification Layer

Implementation focus:

- keep existing `compute_V_for_pair()` behavior for callers that still need a
  `Gf.Matrix4d`
- add a new structured entrypoint that returns a certification record with:
  - `mode`
  - `eligible`
  - `reject_reason`
  - `vertex_rmse`
  - `rmse_available`
  - `rmse_unavailable_reason`
  - `proof_source`
  - `alternate_proof_kind`
  - unresolved-state classification
- expose unresolved `shape_invariant` / `transitive` states as rejectable
  outcomes rather than as errors that downstream code silently normalizes away

This is where phase-0 should encode:

- early rollout only allows proof-backed restricted `geom_only`
- unresolved compensation state remains hard reject
- Policy A vs Policy B decision hooks can be expressed as a simple eligibility
  policy over the same recorded evidence

### P0-T2. Certified Graph and Filtered Mapping

Implementation focus:

- keep canonical selection in
  `c1_build_bulk_mapping_from_dedup_report.py`, because that script already owns
  usage-based canonical selection and conflict cleanup
- add a pair-candidate stage before canonical commit
- use only certified edges to build the graph components used for canonical
  selection
- emit filtered mapping from certified edges only

This is also the correct place to implement the refined bbox-fail recovery:

- accept a `revoked_edges.jsonl` input
- remove only the attributable `old -> canonical` edges
- rebuild connected components from the remaining certified edges
- rerun canonical selection using the current usage heuristic inside each
  remaining certified component
- re-emit `certified_graph.json` and `filtered_mapping.json`

That keeps pair pruning in the same code path that already manages graph-level
dedup semantics, instead of scattering canonical rebuild logic into Step6 or the
validator.

### P0-T3. Authoritative Audit Contract

Implementation focus:

- extend `placement_pairwise_compare.py` to compute per-prim:
  - bbox min delta by axis
  - bbox max delta by axis
  - footprint extent delta
  - footprint axis delta / swap classification
  - centroid delta
  - `ref_changed` hard-fail status
- add CLI thresholds for:
  - `eps_bbox`
  - `eps_pos`
  - `eps_angle`
  - optional `eps_geom`
- emit `audit_verdict.json` with a small top-level schema:
  - `passed`
  - `blocking_reason_counts`
  - `ref_changed_fail_count`
  - `scenes_error`
  - `total_no_mesh`
  - `policy`
  - `thresholds`
- return non-zero if:
  - `scenes_error > 0`
  - `total_no_mesh > 0`
  - compared scope is incomplete
  - any `ref_changed` prim violates a hard gate

### P0-T4. Autorun Wiring

Implementation focus:

- preserve the current category loop and ledger behavior
- replace the hardcoded three-stage execution with five explicit stages:
  - `01_cert`
  - `02_apply`
  - `03_audit`
  - `04_step6_gate`
  - `05_step6`
- add explicit policy switches rather than implicit mode trust

Minimum new CLI/config surface:

- `--bbox-gated`
- `--bbox-policy {bbox_primary_rmse_observe,bbox_primary_rmse_harder}`
- `--baseline-root`
- `--run-root`
- `--skip-step6` or `--step6-mode {off,dry_run,apply}`
- `--certificate-dir` and `--audit-dir` only if separate override is needed

Phase-0 default should be:

- bbox-gated enabled
- restricted `geom_only` only
- Step6 blocked or dry-run only

### P0-T5. Supplemental BBox Helper

Implementation focus:

- keep `verify_dedup_bbox.py` as a helper, not the gate
- align its summary keys with `placement_pairwise_compare.py` so the two reports
  can be compared mechanically
- do not add gating decisions here; keep those in `audit_verdict.json`

# Phase 1

Phase 1 is the first end-to-end restricted rollout. The target state is:

- only proof-backed restricted `geom_only` rewrites are allowed
- rewrite is fail-closed for unsupported shapes and preflight failures
- audit verdict is required before promotion or soft-delete
- Policy A and Policy B can be run from the same baseline as isolated A/B runs

## Phase 1 First Scripts To Change

1. `scripts/rewrite_layout_asset_refs_with_compensation.py`
2. `scripts/c1_bulk_apply_layout_dedup.py`
3. `scripts/c1_autorun_categories.py`
4. `scripts/c1_bulk_step6_category_promote_scan_soft_delete.py`

Rationale:

- `rewrite_layout_asset_refs_with_compensation.py` is where fail-open mutation
  currently happens and where payload / asset-attr / multi-ref shapes must be
  blocked.
- `c1_bulk_apply_layout_dedup.py` is the narrow integration layer that can
  preserve current per-layout reporting while switching to certified input and a
  reject ledger.
- `c1_autorun_categories.py` must consume the new apply and audit contracts.
- `c1_bulk_step6_category_promote_scan_soft_delete.py` must stop deciding
  promotability from scan cleanliness alone.

## Phase 1 Tasks

| Task | Scripts | Deliverable | New / Updated Artifacts | Depends On | Can Run In Parallel |
| --- | --- | --- | --- | --- | --- |
| P1-T1 | `scripts/rewrite_layout_asset_refs_with_compensation.py` | Preflight-first fail-closed rewrite | `rewrite_reject_ledger.jsonl`; richer per-layout reports with reject reasons and skipped shapes | P0-T1, P0-T2 | with P1-T2 after artifact names are fixed |
| P1-T2 | `scripts/c1_bulk_apply_layout_dedup.py` | Consume certified mapping only and surface rewrite reject stats | updated `batch_summary.json`, `spotcheck_list.md`, stable per-layout report naming under `02_apply/` | P1-T1, P0-T2 | with P1-T3 once apply output contract is fixed |
| P1-T3 | `scripts/c1_autorun_categories.py` | Full end-to-end category runner for restricted `geom_only` plus A/B policy selection | per-policy run directories, stage logs, per-category stage manifests, summary-level A/B comparison | P0-T4, P1-T2, P0-T3 | no; orchestration is the integration point |
| P1-T4 | `scripts/c1_bulk_step6_category_promote_scan_soft_delete.py` | Require authoritative audit pass before promote / soft-delete | `step6_gate_decision.json`, promotion reports that embed the audit verdict path and result | P0-T3, P1-T3 | no; must wait for audit output |

## Phase 1 Detailed Notes

### P1-T1. Preflight-First Rewrite

Implementation focus:

- do not mutate refs first and diagnose later
- reject unsupported rewrite shapes before mutation:
  - payload rewrites
  - asset-valued attribute rewrites
  - multi-ref changed prims
- reject unsupported compensation states before mutation
- remove any bbox-gated path that silently falls back to identity or implicit
  best effort for `ref_changed` prims

The rewrite report should add counters beyond the current:

- `rejected_ref_prims`
- `rejected_payload_prims`
- `rejected_asset_attr_prims`
- `rejected_multi_ref_prims`
- `rejected_unresolved_compensation_prims`

and a line-oriented `rewrite_reject_ledger.jsonl` with:

- `scene_id`
- `prim_path`
- `old_asset`
- `canonical_asset`
- `reject_reason`
- `stage`
- `policy`

### P1-T2. Bulk Apply Integration

Implementation focus:

- accept only `filtered_mapping.json` as rewrite input in bbox-gated mode
- thread certificate metadata through batch summaries for auditability
- keep current per-layout reports, but add:
  - certificate path
  - reject ledger path
  - policy name
  - baseline path

This script should remain the main producer of:

- `batch_summary.json`
- `spotcheck_list.md`

but those summaries should now reflect both changed and rejected workloads.

### P1-T3. Orchestration and A/B Runs

Implementation focus:

- run Policy A and Policy B from the same upstream baseline, in isolated work
  roots
- keep the run manifest stable enough that A/B summaries can be aggregated
  mechanically
- default restricted rollout surface to proof-backed `geom_only`

Target per-category stage order:

1. certificate build
2. filtered mapping emission
3. bulk apply
4. authoritative audit
5. Step6 gate decision
6. Step6 dry-run or apply, depending on run mode

### P1-T4. Audit-Gated Step6

Implementation focus:

- require `audit_verdict.json` as explicit input
- fail before any promote or soft-delete if `passed != true`
- record the verdict path in promotion and ledger artifacts

This keeps Step6 responsible for:

- promotion
- scan hygiene
- soft-delete

and moves promotability authority out of scan cleanliness alone.

# A/B Evaluation

The A/B evaluation should be the first operational use of the new policy
switches, but it should still run on the restricted early-rollout surface.

## Policy Definitions

Policy A:

- primary eligibility:
  bbox / footprint / centroid hard gates
- unresolved compensation state:
  hard reject
- RMSE:
  record only

Policy B:

- same primary eligibility as Policy A
- same unresolved compensation hard reject
- additional rule:
  if `vertex_rmse` is computable, require `vertex_rmse <= eps_geom`
- if `RMSE` is unavailable:
  do not auto-pass and do not auto-reject on that fact alone
  the pair remains governed by the primary proof path and the unavailable reason
  must still be recorded

## How To Run

Run both policies from the exact same baseline snapshot:

```text
/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup
```

Recommended execution shape after orchestration wiring lands:

1. materialize two isolated working roots or two isolated per-policy output trees
2. run Policy A over the restricted `geom_only` scope
3. run Policy B over the same category set and same input layout snapshot
4. stop both runs after audit or Step6 dry-run until promotability is accepted
5. aggregate the per-policy summaries into one comparison artifact

Target command shape:

```bash
python scripts/c1_autorun_categories.py \
  --dataset-root <policy_specific_work_root> \
  --bak-root <policy_specific_bak_root> \
  --report <dedup_report_json> \
  --c1-bulk-dir check_reports/test0_bbox_gated/<run_id>/<policy_name> \
  --input-layout-name layout.pre_c1_normalize_only.20260315_chain_fix_v1.usd \
  --bbox-gated \
  --bbox-policy <bbox_primary_rmse_observe|bbox_primary_rmse_harder> \
  --baseline-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup \
  --v-matrix-mode auto \
  --mode-reports-dir <mode_reports_dir> \
  --step6-mode dry_run
```

The exact flag names can change at implementation time, but the run discipline
should not:

- same baseline
- same category set
- same input snapshot
- separate policy outputs
- no actual promotion before audit-backed acceptance

## Required A/B Outputs

Each policy run must produce:

- eligible pair count by mode and category
- rejected pair count by reject reason, mode, and category
- mapping pair count by category
- layouts changed by category
- rewrite reject counts by shape and reject reason
- audit failures by category and scene
- worst-scene and worst-category concentration
- RMSE availability and unavailability rate by mode and category

The summary-level comparison must then compute:

- delta in eligible pairs between A and B
- delta in mapping pairs between A and B
- delta in changed layouts between A and B
- delta in audit failures between A and B
- concentration of losses or failures in specific categories

# Dependency Order

The minimal critical path is:

1. structured certification result
2. certified graph plus filtered mapping
3. fail-closed rewrite
4. authoritative audit verdict
5. autorun wiring of cert/apply/audit stages
6. Step6 gated on audit verdict
7. A/B restricted reruns

Explicit hard dependencies:

- `c1_build_bulk_mapping_from_dedup_report.py` depends on the certification
  result schema from `compute_vertex_transform.py`
- `rewrite_layout_asset_refs_with_compensation.py` depends on filtered mapping
  and reject classifications being available
- `c1_bulk_apply_layout_dedup.py` depends on the new rewrite contract
- `c1_autorun_categories.py` depends on both certificate output paths and audit
  verdict paths
- `c1_bulk_step6_category_promote_scan_soft_delete.py` depends on the audit
  verdict contract
- A/B reruns depend on all of the above plus first-wave tests passing

# What Can Run In Parallel

After the certificate row schema is fixed, the following can proceed in
parallel:

- `compute_vertex_transform.py` certification result work
- `placement_pairwise_compare.py` audit verdict work
- `verify_dedup_bbox.py` helper alignment work

After `filtered_mapping.json` and `audit_verdict.json` paths are fixed, the
following can proceed in parallel:

- `rewrite_layout_asset_refs_with_compensation.py`
- `c1_bulk_apply_layout_dedup.py`
- `c1_autorun_categories.py` stage-directory and ledger refactor

After the new CLI surface is stable, the following can proceed in parallel:

- unit tests for certificate and rewrite rejection
- unit tests for validator hard-fail behavior
- Step6 gating integration

# Initial Tests

The repo currently only has transform-math tests. That is not enough for the
new contract. The first wave should be:

1. Update `tests/test_compute_vertex_transform.py`
   - cover certification result fields for:
     - `geom_only`
     - `shape_invariant` reject
     - `transitive` unresolved reject
     - `rmse_available` and `rmse_unavailable_reason`
2. Update `tests/test_dedup_compensation_chain.py`
   - keep chain-walk math coverage
   - add at least one case proving that unresolved compensation state is exposed
     as a rejectable outcome instead of falling through to rewrite
3. Update `tests/test_patch_placement.py`
   - keep world-invariance math coverage
   - add a case showing the bbox-gated path does not mutate when preflight
     fails
4. Add `tests/test_bbox_gated_mapping.py`
   - certified-edge filtering
   - canonical rebuild after `revoked_edges.jsonl`
   - no whole-group invalidation on single-edge revoke
5. Add `tests/test_bbox_gated_rewrite.py`
   - payload disallow
   - asset-attr disallow
   - multi-ref changed prim disallow
   - reject ledger emission
6. Add `tests/test_placement_pairwise_compare_bbox_gate.py`
   - non-zero exit on `ref_changed` hard-gate violation
   - `audit_verdict.json` counters and threshold serialization

# Initial Acceptance

## Code-Level Acceptance

Before any rerun:

- unit tests for certificate, mapping, rewrite reject ledger, and audit hard-fail
  all pass
- `c1_autorun_categories.py` can produce the new stage layout in dry-run mode
- Step6 dry-run consumes `audit_verdict.json` and blocks when `passed != true`

## Phase-1 Restricted Rerun Acceptance

For the first restricted `geom_only` rerun from the canonical baseline:

- `filtered_mapping.json` contains only certified edges
- `rewrite_reject_ledger.jsonl` exists and has explicit reasons for every
  skipped would-be ref change
- `placement_pairwise_compare.py` exits `0`
- `audit_verdict.json` reports:
  - `passed = true`
  - `scenes_error = 0`
  - `total_no_mesh = 0`
  - `ref_changed_fail_count = 0`
- Step6 dry-run emits `step6_gate_decision.json` showing promotable status only
  because the authoritative audit passed

## First Actual Promotion Acceptance

Only after the restricted rerun passes:

- re-run on the same baseline-derived policy workspace with Step6 apply enabled
- confirm Step6 records the audit verdict path in its reports
- confirm post-promote and post-soft-delete scans remain clean
- confirm no rejected edge was promoted through a fallback path

# Immediate Implementation Order

If the work is split across owners, the safest initial order is:

1. `compute_vertex_transform.py`
   - add structured certification result
2. `c1_build_bulk_mapping_from_dedup_report.py`
   - emit certificates, certified graph, filtered mapping
3. `placement_pairwise_compare.py`
   - emit authoritative audit verdict and non-zero exit behavior
4. `c1_autorun_categories.py`
   - create explicit `cert -> apply -> audit -> step6_gate` orchestration
5. `rewrite_layout_asset_refs_with_compensation.py`
   - make rewrite fail-closed
6. `c1_bulk_apply_layout_dedup.py`
   - thread filtered mapping and reject ledger through batch reports
7. `c1_bulk_step6_category_promote_scan_soft_delete.py`
   - block on audit verdict
8. new mapping / rewrite / validator tests
9. restricted `geom_only` Policy A and Policy B dry-runs
10. restricted `geom_only` first accepted apply run

# Non-Goals For This Landing

Do not expand scope during phase-0 / phase-1 to:

- broad non-`geom_only` rollout
- payload rewrite support
- asset-valued attribute rewrite support
- multi-ref changed prim support
- permissive `transitive` recovery
- Step6 promotion without authoritative audit pass

# References

- `docs/operations/grscenes_test0_bbox_gated_dedup_implementation.md`
- `docs/operations/grscenes_test0_bbox_gated_dedup_status_20260327.md`
- `docs/test0_full/grscenes_test0_rebuilt_prededup_baseline_adoption_20260327.md`
- `check_reports/test0_rebuilt_full/20260315_chain_fix_v1/summary/prededup_baseline_binding_verdict.json`
- `.codex/worklogs/main/2026-03-27/bbox-gated-impl-spec-handoff.md`
- `.codex/worklogs/main/2026-03-27/baseline-closure-handoff.md`
- `.codex/worklogs/main/2026-03-27/bbox-gated-proposal-review-handoff.md`
- `.codex/worklogs/main/2026-03-27/bbox-vs-rmse-handoff.md`
- `.codex/worklogs/main/2026-03-25/bbox-gated-dedup-handoff.md`
