---
title: "Dedup Gate and Threshold Discussion Notes"
code_reference:
  - scripts/compute_vertex_transform.py
  - scripts/placement_pairwise_compare.py
  - scripts/report_asset_mesh_dedup.py
  - scripts/rewrite_layout_asset_refs_with_compensation.py
created_at: 2026-04-24
updated_at: 2026-04-24
maintainer: Codex
status: active
doc_class: record
---

# Dedup Gate and Threshold Discussion Notes

## 中文摘要

本记录总结 2026-04-24 围绕 transitive bbox-gated 去重流程的讨论结论。核心判断是：

- 当前流程作为自动 rewrite / Step6 soft-delete 的安全策略是合理的，但召回偏保守。
- `79090...` 这类 bottle 不应被解释成确认的 `not_duplicate`，更准确是 `possible_duplicate`：视觉和 NN 证据较强，但未达到当前自动合并门槛。
- `baseline_bbox` 和 `nn_bbox` 不应简单共用一个阈值。`baseline_bbox` 是默认顶点顺序对齐结果；`nn_bbox` 是最近邻重配对后的补救对齐结果。
- `0.05 < nn_bbox <= 0.5` 这段不应直接人工审核，也不应直接放进正式 mapping；推荐进入自动 shadow apply + scene audit 的灰区通道。
- `geom_only` 不建议直接放宽到 topo/shape 的 `0.15` 审计阈值。`geom_only` 是 exact lane，scene audit 的严格阈值可以作为 rewrite / compensation bug 的报警器。
- transitive 的 scene audit 不宜永远按 strict `0.01` 处理；应根据 certificate witness path 继承 `geom_only` / `topo_filesize` / `shape_invariant` 的风险语义。

## Scope

This note records the design discussion around the current transitive bbox-gated
dedup pipeline, with emphasis on:

- what the three high-level gates mean;
- how `bbox_precheck_failed_topo_filesize` should be interpreted;
- why `baseline_bbox` and `nn_bbox` differ;
- how to treat visual duplicates that do not meet the strict automatic gate;
- what scene audit currently checks;
- whether `geom_only`, `topo_filesize`, `shape_invariant`, and `transitive` should
  share audit thresholds.

The discussion was prompted by the bottle group containing:

- `7861bdaa89323558eb8046679f567498`
- `79088d12b87f6758da805eb64c8a3582`
- `79090fe893611281c78d1c237c2f5b64`
- canonical candidate `0ef0ff541cbb58e437914cec5a27cfc6`

## Current Gate Model

The six-step transitive bbox-gated flow has three real gates:

| Gate | Purpose | Blocking decision |
| --- | --- | --- |
| Certificate gate | Decide whether an old asset may be rewritten to a canonical asset. | Produces `filtered_mapping.json`; rejected pairs are not applied. |
| Scene audit gate | Compare original and rewritten layouts in scene/world context. | Blocks only hard failures under the selected audit policy. |
| Step6 gate | Promote rewritten layouts, scan residual references, then soft-delete old duplicate assets. | Blocks deletion if old refs remain or scan fails. |

Union discovery is not a safety gate. It creates candidate components. The final
admission happens at certificate generation and then again through scene audit and
Step6 reference scanning.

## Bottle Example

The final bottle certificate result is:

| UID | Final outcome |
| --- | --- |
| `7861bdaa89323558eb8046679f567498` | mapped to `0ef0ff541cbb58e437914cec5a27cfc6` |
| `79088d12b87f6758da805eb64c8a3582` | mapped to `0ef0ff541cbb58e437914cec5a27cfc6` |
| `79090fe893611281c78d1c237c2f5b64` | not mapped; rejected by certificate gate |

Evidence:

- `filtered_mapping.json:1034` maps `7861...` to `0ef0...`.
- `filtered_mapping.json:1035` maps `79088...` to `0ef0...`.
- `pair_certificates.jsonl:1084` rejects `79090...` with
  `bbox_precheck_failed_topo_filesize`.

Therefore, under the current production mapping semantics, the three assets are not
all accepted as the same automatically replaceable asset. The first two are accepted
against the canonical. The third is a candidate / possible duplicate but not an
automatic duplicate.

## Units

For GRScenes-test0 records, the working convention is `metersPerUnit=0.01`, so
dedup and audit values are interpreted as centimeters in the scene context.

Examples:

- `1.0` unit is about `1 cm`;
- `0.5` unit is about `5 mm`;
- `0.05` unit is about `0.5 mm`;
- `0.01` unit is about `0.1 mm`.

This matters because a visually identical bottle can still fail a millimeter-scale
automation threshold. The failure does not imply a meter-scale or visually obvious
asset mismatch.

## Baseline BBox vs NN BBox

Both metrics ask the same question: after aligning the canonical asset to the old
asset, how far apart are the resulting bbox endpoints?

They differ in how the alignment is computed:

| Metric | Alignment assumption | Typical failure mode |
| --- | --- | --- |
| `baseline_bbox` | Vertex index `i` in canonical corresponds to vertex index `i` in old. | Fails when the assets look the same but vertex ordering differs. |
| `nn_bbox` | Rebuild correspondence by nearest neighbors, then align. | Can recover shuffled or differently exported but visually matching meshes. |

For `79090... -> 0ef0...`, the spike row is:

```text
baseline_bbox = 1.06853436
nn_bbox       = 0.40133281
nn_close_pct  = 42.8755
nn_unique_ratio = 0.539153
nn_mean_dist_norm = 0.01297176
```

This means the baseline indexed alignment fails the certificate precheck, but NN
alignment reduces the bbox error below the certificate gross precheck threshold of
`0.5`.

Current production logic still rejects the pair because Tier2 NN adoption uses a
stricter threshold: `_NN_TIER2_BBOX_THRESHOLD = 0.05`. The pair passes the other
Tier2 quality gates but does not pass `nn_bbox <= 0.05`.

## `bbox_precheck_failed_topo_filesize`

`bbox_precheck_failed_topo_filesize` means:

1. the pair was a `topo_filesize` candidate;
2. a transform `V` was computed from canonical to old;
3. transformed canonical bbox endpoints were compared against old bbox endpoints;
4. `bbox_delta.max_abs` exceeded the certificate precheck threshold of `0.5`;
5. the pair failed closed and did not enter `filtered_mapping.json`.

The relevant code path is `_evaluate_certificate_with_V(...)` in
`scripts/compute_vertex_transform.py`, where `CERT_BBOX_PRECHECK_THRESHOLD = 0.5`.

For `79090...`, the certificate row records:

```text
mode          = topo_filesize
v_source      = baseline
eligible      = false
reject_reason = bbox_precheck_failed_topo_filesize
bbox_delta.max_abs = 1.068534364958798
```

The key interpretation is not "this is definitely a different visual object." The
safer interpretation is:

> Baseline alignment could not prove that this candidate is safe for automatic
> replacement and deletion.

## Current Thresholds

### Certificate thresholds

`scripts/compute_vertex_transform.py` currently uses these key thresholds:

| Threshold | Value | Meaning |
| --- | ---: | --- |
| `_NN_FALLBACK_BASELINE_THRESHOLD` | `0.15` | If baseline bbox is above this, try NN fallback. |
| `_NN_FALLBACK_CLOSE_PCT_THRESHOLD` | `95.0` | Tier1 shuffle acceptance requires high close percentage. |
| `_NN_FALLBACK_ACCEPT_THRESHOLD` | `0.01` | Tier1 NN candidate must be extremely tight. |
| `_NN_TIER2_BBOX_THRESHOLD` | `0.05` | Tier2 NN candidate must still be very tight. |
| `_NN_TIER2_UNIQUE_THRESHOLD` | `0.5` | Tier2 nearest-neighbor uniqueness lower bound. |
| `_NN_TIER2_MEAN_DIST_THRESHOLD` | `0.02` | Tier2 normalized mean distance upper bound. |
| `_NN_TIER2_CLOSE_PCT_FLOOR` | `10.0` | Tier2 close percentage lower bound. |
| `CERT_BBOX_PRECHECK_THRESHOLD` | `0.5` | Gross certificate bbox failure threshold. |

The important asymmetry is:

- `0.5` is the certificate gross safety line;
- `0.05` is the current Tier2 NN adoption line;
- the interval `0.05 < nn_bbox <= 0.5` is currently rejected even when other NN
  quality indicators are acceptable.

### Scene audit thresholds

`scripts/placement_pairwise_compare.py` uses mode-aware audit semantics:

| Mode | Effective bbox threshold in current observe policy | Blocking behavior |
| --- | ---: | --- |
| `geom_only` | `eps_bbox` default `0.01` | hard failure |
| `topo_filesize` | `0.15` | soft warning under `bbox_primary_rmse_observe` |
| `shape_invariant` | `0.15` | soft warning under `bbox_primary_rmse_observe` |
| `transitive` | `eps_bbox` default `0.01` unless certificate-aware semantics resolve otherwise | strict by default |
| unknown | `eps_bbox` default `0.01` | strict |

The audit verdict passes only if:

```text
scenes_error == 0
no_mesh_ok
compared_scope_complete
ref_changed_hard_fail_count == 0
```

Soft warnings are recorded but do not block `bbox_primary_rmse_observe`.

## Bottle Audit Data Point

For the bottle run:

```text
candidate_pairs = 1864
eligible_pairs  = 1577
rejected_pairs  = 287
bbox_precheck_failed_topo_filesize = 220
```

The scene audit result was:

```text
passed = true
scenes_error = 0
compared_scope_complete = true
ref_changed_fail_count = 0
ref_changed_soft_fail_count = 423
```

Soft warning reasons were:

```text
bbox_min_delta_gt_eps         298
bbox_max_delta_gt_eps         268
footprint_extent_delta_gt_eps 224
centroid_delta_gt_eps          78
footprint_axis_delta_gt_eps    33
```

These are not 423 independent bbox errors. A single rewritten prim can contribute
multiple soft reasons.

The aggregate displacement breakdown showed:

```text
ref_changed centroid > 0.01: 78
ref_changed centroid > 0.1:  24
ref_changed centroid > 1.0:   0
category max for bottle:      0.131345
```

In GRScenes-test0 units, the category maximum is about `1.31 mm`.

## Is the Strategy Wrong?

The current strategy is not wrong if the goal is safe automatic rewriting and Step6
soft-delete. It is intentionally conservative and fail-closed.

It is incomplete if the goal is to recover all visually duplicate assets. The bottle
case shows a real recall gap: some visually identical or near-identical assets are
not promoted because strict automatic proof is missing.

Therefore the right taxonomy is:

| Label | Meaning | Action |
| --- | --- | --- |
| `auto_duplicate` | Strict certificate proof is strong enough. | Enter official mapping; eligible for apply and Step6. |
| `possible_duplicate` | NN or visual evidence is strong, but strict proof is not enough. | Enter automated shadow validation, not human review. |
| `not_duplicate` | Geometry remains unsafe after available alignment attempts. | Reject. |

`79090...` should be treated as `possible_duplicate`, not as a confirmed
`not_duplicate`.

## No-Human-Review Design

There is no capacity for manual review, so the gray zone should become an automated
shadow validation channel.

Recommended flow:

```text
candidate pair
  |
  |-- strict pass -> official mapping -> apply -> audit -> Step6
  |
  |-- gray pass   -> shadow mapping -> shadow apply -> audit-gated promotion
  |
  |-- hard fail   -> reject
```

Proposed gray-zone admission for `topo_filesize`:

```text
baseline_bbox > 0.5
nn_bbox <= 0.5
nn_unique_ratio >= 0.5
nn_mean_dist_norm <= 0.02
nn_close_pct >= 10
```

Then run shadow apply and scene audit against a temporary layout suffix. Promote only
if automated audit budgets pass. Step6 should remain delayed for promoted gray-zone
pairs:

1. rewrite references;
2. scan residual old references;
3. keep old assets in quarantine for one confirmation cycle;
4. soft-delete only after a clean residual scan.

This avoids manual review while preserving rollback room.

## Recommended Threshold Policy

### Keep strict automatic pass narrow

Keep the current strict lane:

```text
nn_bbox <= 0.05
```

This is appropriate for "safe without further debate" automatic mapping.

### Add audit-gated promotion for gray pairs

Do not directly change Tier2 from `0.05` to `0.5`. Instead, run candidate thresholds
through shadow audit:

```text
nn_bbox <= 0.05
nn_bbox <= 0.15
nn_bbox <= 0.30
nn_bbox <= 0.50
```

Choose the widest tier that produces no hard audit failures and stays within explicit
soft-warning budgets.

### Do not make `geom_only` match topo/shape by default

`geom_only` is the exact lane:

- geometry hash is intended to represent identical authored mesh geometry;
- `compute_V_for_pair(..., mode="geom_only")` returns identity;
- pair certificate metrics are zeroed by construction for `geom_only`;
- scene audit is the canary for rewrite / compensation bugs.

Relaxing `geom_only` audit from `0.01` to `0.15` would hide problems such as:

- wrong internal transform compensation;
- stale or incorrect mode lookup;
- wrong reference rewritten;
- hierarchy or mesh aggregation mismatch;
- matrix order or inverse mistakes.

The better approach is:

```text
geom_only production hard threshold: keep 0.01
geom_only diagnostic band: record 0.01 < drift <= 0.15
only consider 0.03 or 0.05 after repeated full-run evidence
do not jump directly to 0.15
```

### Make transitive audit inherit witness risk

Transitive should not always be audited like strict `geom_only`. If the transitive
witness path includes `topo_filesize` or `shape_invariant`, audit semantics should
inherit the effective risk tier from certificate metadata:

| Transitive witness path | Suggested audit behavior |
| --- | --- |
| all `geom_only` | strict `0.01` |
| includes `topo_filesize` | topo-like observe threshold |
| includes `shape_invariant` | shape-like observe threshold, possibly stricter until measured |
| missing / rejected certificate | strict, or block |

This matches the existing certificate-aware audit direction documented in
`docs/records/changes/2026-04-13_transitive_bbox_audit_certificate_semantics.md`.

## Open Follow-Ups

1. Add a `possible_duplicate` / shadow mapping output for gray-zone NN candidates.
2. Add a shadow apply and audit loop that can automatically promote safe gray-zone
   pairs without human review.
3. Add `geom_only_drift_report.json` for `0.01 < drift <= 0.15` instead of relaxing
   the hard threshold immediately.
4. Define soft-warning budgets for shadow promotion, such as:
   - no scene errors;
   - no prim lost or added;
   - no ref-same regressions;
   - no ref-changed hard failures;
   - no displacement above a large threshold;
   - bounded bbox / centroid / footprint soft-warning counts.
5. Ensure Step6 treats shadow-promoted pairs with delayed quarantine before deletion.

## Decision Summary

The current pipeline is safe but conservative. The preferred next design is not to
lower safety by broadening the production gate. Instead:

- keep strict certificate and audit lanes for automatic deletion;
- introduce an automated gray-zone shadow lane for likely visual duplicates;
- make scene audit the machine judge for gray-zone promotion;
- keep `geom_only` strict as a rewrite/compensation canary;
- make transitive audit semantics depend on certified witness modes.
