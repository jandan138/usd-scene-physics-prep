---
title: "Review: Transitive BBox Rewrite Fix"
created_at: "2026-04-22"
updated_at: "2026-04-22"
maintainer: "OpenCode"
status: "active"
code_reference:
  - "scripts/rewrite_layout_asset_refs_with_compensation.py"
  - "scripts/c1_bulk_apply_layout_dedup.py"
  - "scripts/placement_pairwise_compare.py"
  - "tests/test_bbox_gated_rewrite.py"
  - "docs/records/changes/2026-04-22_transitive_bbox_rewrite_fix.md"
doc_class: record
---

# Summary

Reviewed the transitive bbox-gated rewrite fix as a read-only code review.

# Findings

## Important

- `scripts/rewrite_layout_asset_refs_with_compensation.py:121-139` loads the last certificate row for a pair without checking `eligible` / `reject_reason`.
- This differs from `scripts/placement_pairwise_compare.py:79-119`, which only trusts certified rows.
- Operational risk: if `pair_certificates.jsonl` ever contains both an eligible and a rejected row for the same pair, rewrite behavior becomes order-dependent and can reuse a rejected witness.

## Medium

- `scripts/rewrite_layout_asset_refs_with_compensation.py:177-217` reconstructs intermediate witness asset paths from `<category_root>/<uid>/usd/<uid>.usd` and ignores any original witness USD paths.
- This is acceptable for the current GRScenes normalized asset layout, but it is brittle if a future cert pipeline emits non-canonical asset paths or renamed files.

# Assessment

- The transitive witness reconstruction is sound enough for the current rerun path because the local mode graph is rebuilt only from the certified witness chain, so `compute_V_for_pair(..., mode="transitive")` is forced onto that chain.
- The main remaining correctness risk is certificate-row trust, not the witness composition math itself.

# Missing Tests

- Duplicate certificate rows for one pair: eligible row plus later rejected row, asserting rewrite uses only certified metadata.
- Ineligible transitive certificate row with matching mapping pair, asserting bbox-gated rewrite fails closed.
- Mixed-mode transitive witness (`geom_only` + `topo_filesize` or `shape_invariant`) with a non-identity `V`, asserting both ref rewrite and matrix compensation.
- Witness metadata/path reconstruction failure case, asserting a stable reject reason instead of silent fallback.

# Commands Run

- No commands beyond source inspection and date lookup were needed for this review.
