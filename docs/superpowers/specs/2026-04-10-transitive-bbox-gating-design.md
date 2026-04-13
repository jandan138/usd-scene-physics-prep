---
title: Transitive BBox Gating Design
code_reference:
  - scripts/compute_vertex_transform.py
  - scripts/c1_build_bulk_mapping_from_dedup_report.py
  - scripts/placement_pairwise_compare.py
  - scripts/c1_autorun_categories.py
  - tests/test_compute_vertex_transform.py
  - tests/test_bbox_gated_mapping.py
  - tests/test_placement_pairwise_compare_bbox_gate.py
created_at: 2026-04-10
updated_at: 2026-04-10
maintainer: OpenCode
status: approved
---

# Transitive BBox Gating Design

## Goal

Add support for certifying and auditing `transitive` dedup edges inside the
bbox-gated pipeline, while keeping the safety rule that final admission is based
on direct endpoint verification rather than matrix composition alone.

## Current State

The current codebase already has partial transitive support, but that support is
split across layers and stops before certificate admission:

- `scripts/compute_vertex_transform.py` can derive a transitive `V` by finding a
  BFS path through group members and composing per-edge transforms.
- `scripts/c1_build_bulk_mapping_from_dedup_report.py` treats any pair without a
  direct mode entry as `pair_mode="transitive"`.
- The same mapping builder currently rejects those pairs immediately with
  `reject_reason="transitive_not_supported"`.
- `scripts/placement_pairwise_compare.py` also treats `transitive` as a strict,
  unsupported mode during audit threshold selection.

This means the repository can discover transitive connectivity, but the bbox
gated certificate layer does not currently admit transitive edges into
`filtered_mapping`.

## Product Decision

The product rule for this feature is:

**A transitive path may generate a candidate endpoint transform, but the final
certificate verdict must still come from direct endpoint validation on the pair
being certified.**

In practical terms:

1. Use the transitive path only to produce a candidate `final_V` for
   `(canonical, old)`.
2. Re-run the existing bbox-gated geometric checks directly on the endpoint pair
   using that `final_V`.
3. Admit the pair only if the endpoint checks pass.

This preserves the existing safety model of the bbox-gated certificate pipeline:

- `geom_only`, `topo_filesize`, and `shape_invariant` use direct proof.
- `transitive` becomes an additional proof-construction route, not a bypass.

## Non-Goals

This design deliberately does not include the following:

- accepting a transitive edge purely because its witness path is individually
  certified
- requiring multiple witness paths to agree before admission
- selecting the numerically best witness from all possible paths
- changing Step 6 promote behavior or converting the current rollout from
  `dry_run` to promote mode
- changing `union_3way` merge logic or duplicate-group semantics

## Design Summary

The feature is implemented as four linked changes:

1. deterministic witness-path selection for transitive pairs
2. endpoint-based certificate verification using the composed `V`
3. certificate metadata for witness traceability and audit interpretation
4. audit-mode lookup that prefers certificate semantics over inferred fallback

## Witness Path Rules

### Path selection

Version 1 selects exactly one witness path per transitive pair.

The selected path must be deterministic. Ranking order is:

1. fewest hops
2. lowest-risk mode sequence
3. lexical tie-break on the resulting UID path

The risk ordering for direct edge modes is:

1. `geom_only`
2. `topo_filesize`
3. `shape_invariant`

This rule is not intended to prove global optimality. It exists to guarantee
that repeated runs on the same inputs produce the same witness and the same
certificate.

### Mixed-mode paths

Witness paths may contain a mix of direct modes.

The path is summarized into a single `transitive_effective_mode` for downstream
interpretation:

- all edges `geom_only` -> `geom_only`
- any `topo_filesize` and no `shape_invariant` -> `topo_filesize`
- any `shape_invariant` -> `shape_invariant`

This is intentionally conservative. The effective mode reflects the weakest edge
type present in the witness.

## Certificate Semantics

### Shared verification rule

Direct modes and transitive mode should use the same endpoint-verification code
path after a candidate `V` is available.

That shared verification computes:

- transformed canonical bbox against old bbox
- `bbox_delta`
- `centroid_delta`
- `vertex_rmse` when vertex counts match
- certificate precheck verdict using the existing bbox threshold

This shared verifier prevents transitive certificates from drifting into a
different proof standard than direct certificates.

### Transitive builder

Add a dedicated certificate builder for transitive pairs rather than overloading
the direct-mode builder with hidden branching.

Expected responsibility split:

- direct builder: handles `geom_only`, `topo_filesize`, `shape_invariant`
- transitive builder: resolves witness path, composes `final_V`, then calls the
  shared endpoint verifier

### Required certificate fields

Existing fields remain authoritative:

- `eligible`
- `reject_reason`
- `bbox_delta`
- `centroid_delta`
- `vertex_rmse`
- `alternate_proof_kind`
- `proof_source`

Transitive certificates add the following fields:

- `mode = "transitive"`
- `v_source = "transitive_composed"`
- `transitive_hops`
- `transitive_witness_uids`
- `transitive_witness_modes`
- `transitive_effective_mode`
- `transitive_path_hash`
- `transitive_endpoint_verified`

These fields are sufficient to explain why a transitive edge was admitted
without storing a large debug payload for every certificate.

### Proof naming

When a transitive pair passes certification:

- `alternate_proof_kind = "transitive_bbox_gated_proof"`
- `proof_source = "transitive_bbox_gated_proof"`

This keeps transitive proofs distinguishable from direct-mode proofs in summary
reports.

### Reject reasons

Version 1 uses two primary transitive-specific reject codes:

- `v_computation_failed_transitive`
- `bbox_precheck_failed_transitive`

Detailed low-level failure context should continue to live in lightweight debug
fields such as `rmse_unavailable_reason` rather than exploding the reject-code
surface.

## Audit Integration

The current audit logic infers mode from `mode_reports_dir` only. That is not
sufficient once a certified `transitive` edge has been admitted with its own
witness-derived effective mode.

### New audit input

`scripts/placement_pairwise_compare.py` should accept an optional
`--certificate-jsonl` input.

The audit script builds a certificate lookup keyed by endpoint UID pair. The
lookup stores:

- whether the pair was certified
- the pair mode
- the transitive effective mode when applicable

### Mode lookup precedence

Audit mode resolution must follow this order:

1. certificate lookup from `--certificate-jsonl`
2. legacy `mode_index` from `--mode-reports-dir`
3. legacy fallback behavior

This guarantees that audit uses the same semantics that the mapping stage used
to admit the edge.

### Audit thresholds for transitive pairs

For certified transitive pairs, audit should not use the current blanket
unsupported-mode behavior.

Instead, it should map to the certified `transitive_effective_mode`:

- effective mode `geom_only` -> strict `eps_bbox`
- effective mode `topo_filesize` -> tier2/observe bbox semantics
- effective mode `shape_invariant` -> tier2/observe bbox semantics

In other words, audit should continue to be strict or relaxed based on the proof
quality represented by the witness, not based on the string `"transitive"`
alone.

## File-Level Changes

### `scripts/compute_vertex_transform.py`

This file remains the geometry source of truth.

Planned responsibilities:

- add deterministic witness-path selection
- expose both composed `V` and witness metadata
- factor shared endpoint verification out of the direct builder
- add a transitive certificate builder that uses composed `V` plus endpoint
  verification

This file should continue to own:

- pair-mode geometry semantics
- per-mode `V` generation
- transitive witness construction
- certificate-side geometric metrics

### `scripts/c1_build_bulk_mapping_from_dedup_report.py`

This file remains the rollout-facing certificate orchestrator.

Planned responsibilities:

- stop rejecting `pair_mode == "transitive"` immediately
- call the transitive certificate builder when a direct mode is unavailable
- preserve existing summary accounting, now counting admitted transitive edges
- emit the transitive witness summary fields into `pair_certificates.jsonl`

### `scripts/placement_pairwise_compare.py`

This file remains the authoritative audit layer.

Planned responsibilities:

- load optional certificate jsonl
- resolve per-prim effective mode from certificate data first
- apply effective-mode bbox policy for certified transitive pairs
- preserve existing behavior when no certificate input is supplied

### `scripts/c1_autorun_categories.py`

This file wires the pipeline stages together.

Planned responsibilities:

- pass the category certificate jsonl into the audit stage
- preserve existing flow shape and ledger behavior

## Backward Compatibility

The feature should preserve current behavior when no transitive edge is being
certified or when audit runs without `--certificate-jsonl`.

Specifically:

- direct-mode certificates must keep their current verdicts
- direct-mode audit thresholding must keep its current behavior
- callers that do not pass certificate jsonl to audit must continue to work
- summary files and reports may gain new transitive counts and fields, but their
  existing direct-mode fields should remain stable

## Testing Strategy

### `tests/test_compute_vertex_transform.py`

Add focused tests for:

- deterministic witness-path choice
- mixed-mode witness summarization into `transitive_effective_mode`
- correct matrix-composition order for transitive paths
- endpoint verification on a transitive candidate `V`

### `tests/test_bbox_gated_mapping.py`

Add focused tests for:

- a pair that previously became `transitive_not_supported` now certifies when
  endpoint validation passes
- a transitive pair still rejects when endpoint validation fails
- the emitted certificate contains the required transitive witness fields
- summary stats count admitted and rejected transitive edges correctly

### `tests/test_placement_pairwise_compare_bbox_gate.py`

Add focused tests for:

- audit prefers certificate lookup over mode-index fallback
- certified transitive pairs use the effective-mode bbox policy rather than the
  old unsupported strict behavior
- running audit without `--certificate-jsonl` keeps legacy behavior

## Operational Impact

This feature should directly reduce the current rollout bottleneck bucket
`transitive_not_supported` by converting part of that population into either:

- `mapping_pairs` when endpoint verification passes, or
- more informative transitive-specific reject reasons when it does not

The expected operational outcome is not perfect recovery of all transitive
rejects. The goal is to recover the subset that is geometrically safe under the
same bbox-gated contract already used by direct modes.

## Risks and Mitigations

### Risk: unstable witness choice

Mitigation:

- deterministic path ranking
- explicit tests for repeatable witness selection

### Risk: cert and audit disagree about mode semantics

Mitigation:

- audit consumes certificate jsonl directly
- certificate lookup takes precedence over legacy mode inference

### Risk: transitive proof metadata becomes too large

Mitigation:

- store witness summary only, not full per-step debug payloads
- keep heavyweight diagnostics out of the stable certificate surface

### Risk: false confidence from matrix composition alone

Mitigation:

- do not admit any transitive pair without endpoint verification
- preserve fail-closed behavior on missing assets, mesh probe failure, or
  transform computation failure

## Documentation Updates

Implementation of this design should update the following documents:

- `docs/records/research/operations/grscenes_test0_dedup_rollout_status_20260410.md`
- `docs/records/research/operations/tier2_audit_40cat_failure_investigation_20260407.md`
- `docs/records/changes/2026-04-03_topo_tier2_gate_recovery_status.md`

The two older status docs should carry a clear note that they are historical
snapshots for rollout status and that the 2026-04-10 alignment note is the
latest repository-backed summary.

## Acceptance Criteria

The feature is complete when all of the following are true:

- transitive pairs are no longer categorically rejected at certificate build time
- admitted transitive pairs pass endpoint bbox-gated verification
- certificate jsonl records stable witness metadata for admitted and rejected
  transitive pairs
- audit applies effective-mode semantics to certified transitive pairs
- existing direct-mode behavior remains unchanged
- the targeted unit tests for compute, mapping, and audit all pass
