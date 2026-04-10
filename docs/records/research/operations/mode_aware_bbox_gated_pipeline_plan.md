---
title: "Mode-Aware BBox-Gated Dedup Pipeline — Implementation Plan"
code_reference:
  - "scripts/union_dedup_reports.py"
  - "scripts/c1_build_bulk_mapping_from_dedup_report.py"
  - "scripts/compute_vertex_transform.py"
  - "scripts/placement_pairwise_compare.py"
  - "scripts/c1_autorun_categories.py"
created_at: "2026-04-02"
updated_at: "2026-04-02"
maintainer: "Claude Code (reviewed)"
status: "plan-approved"
---

# Mode-Aware BBox-Gated Dedup Pipeline — Implementation Plan

## Context

v8 dry_run probe on bottle failed: cert passed all 1864 pairs but audit found 63% (1983/3116 prims) with bbox hard failures.

**Root cause**: The union report doesn't store per-group dedup mode. Cert reads the union report and labels ALL pairs as `geom_only`. In reality the distribution is:

| True mode | Pairs | % |
|-----------|-------|---|
| geom_only | 126 | 6.8% |
| topo_filesize | 1427 | 76.6% |
| shape_invariant | 55 | 2.9% |
| transitive (union-find only) | 256 | 13.7% |

Wrong mode label → cert issues `geom_only_exact_world_proof` for all → audit finds topo/shape pairs have inherently approximate V compensation → bbox mismatch at strict 0.01 threshold.

The apply phase (`rewrite_layout_asset_refs_with_compensation.py`) independently uses the correct mode via `build_mode_index()` from per-mode reports. So V matrices are computed correctly at apply time, but the cert and audit phases are not mode-aware.

## Scope

本轮仅支持 direct pairs（geom_only, topo_filesize, shape_invariant）。任何 pair 只要不能通过 mode_index 的 direct lookup 命中 direct mode，就统一标记为 transitive，并从 certificate eligibility、final mapping generation、apply eligibility 中排除。Transitive 在本轮只用于 traceability / diagnostics，不视为受支持的替换模式。

This round uses conservative star certification from initial_canonical. Some members may be dropped even if they are reachable through another direct node in the same union group. This is intentional to preserve direct-only semantics; recovering those members requires all-direct-pairs certification in a follow-up round.

---

## Fix 1: Union Report — Add `dominant_source_mode` + `source_mode_counts`

**File**: `scripts/union_dedup_reports.py` — `union_merge_n()` (lines 242-352)

### Changes

1. **Track edge→mode** (after line 248): Add `edge_mode: Dict[Tuple[str,str], str]` mapping each union edge to its source mode.

2. **Explicit priority** (consistent with `build_mode_index()` in `compute_vertex_transform.py`): `geom_only > topo_filesize > shape_invariant`. When the same edge appears in multiple mode reports, the higher-priority mode wins.

   ```python
   MODE_PRIORITY = {"geom_only": 2, "topo_filesize": 1, "shape_invariant": 0}
   edge_key = tuple(sorted([rep, m]))
   new_pri = MODE_PRIORITY.get(mode, -1)
   old_pri = MODE_PRIORITY.get(edge_mode.get(edge_key, ""), -1)
   if new_pri > old_pri:
       edge_mode[edge_key] = mode
   ```

3. **Per-group fields** (lines 309-313): For each output component, count edge modes within the group, pick the dominant mode (most edges, priority tie-break):
   ```json
   {
       "sig": "union_merge_N",
       "count": 10,
       "usd_paths": ["..."],
       "dominant_source_mode": "topo_filesize",
       "source_mode_counts": {"topo_filesize": 8, "shape_invariant": 2}
   }
   ```

   These fields are for **group-level traceability/diagnostics only**, NOT the primary mode determination for cert.

---

## Fix 2A: Cert — Per-pair mode from mode_index + bbox pre-check

### File: `scripts/c1_build_bulk_mapping_from_dedup_report.py`

**Change 1**: Add `--mode-reports-dir` CLI arg. **This is REQUIRED when `--bbox-gated` is set** — without it, per-pair mode lookup has no source and all pairs fall to `transitive_not_supported`. Enforce at argparse level: if `--bbox-gated` and no `--mode-reports-dir`, exit with error.

**Change 2**: Build `mode_index = _cvt.build_mode_index(args.mode_reports_dir)` at startup of `_run_bbox_gated()`.

**Change 3**: Per-pair mode resolution (replace line 353 `mode=report_mode`):

| Priority | Source | Action |
|----------|--------|--------|
| 1 | `mode_index.get((canon_uid, old_uid))` | Use direct mode (geom_only / topo_filesize / shape_invariant) |
| 2 | No direct hit | Mark as `transitive`, reject with `reject_reason="transitive_not_supported"` |
| 3 | `group.get("dominant_source_mode")` | Display/diagnostics only, NOT used as cert mode |

**Change 4**: Certificate summary overhaul — replace single `dedup_mode` field with:
- `pair_mode_counts: {geom_only: N, topo_filesize: N, shape_invariant: N, transitive: N}`
- `eligible_pairs_by_mode: {mode: count}`
- `reject_reason_counts_by_mode: {mode: {reason: count}}`

**Change 5**: Canonical consistency enforcement.

Cert currently evaluates star edges from `initial_canonical`, while component rebuild (line 396) may reselect a different final canonical. This can create uncertified or transitive final mapping edges, violating the direct-only scope.

Use a **directed** certified-edge adjacency derived from certificate rows (`canonical -> old`). Final canonical selection must be based on **outgoing certified direct coverage only**; do not infer coverage from the undirected connected component graph (which is what `_connected_components` at line 160 produces).

Concretely:
1. Build a directed adjacency: `cert_out[canonical] = {old1, old2, ...}` from eligible cert rows.
2. For each component, choose as final canonical the node with the **largest `cert_out` set size** within that component.
3. Only keep members that are in `cert_out[final_canonical]` — i.e., have a direct certified edge FROM the final canonical.
4. Drop any uncovered members.

If this directed selection proves too complex for this round, fall back to: **pin final canonical = initial_canonical**. This is safe because initial_canonical is the star center that cert evaluated against.

This ensures every edge in `filtered_mapping` corresponds to a certified direct pair.

### File: `scripts/compute_vertex_transform.py` — `build_pair_certificate()` (lines 802-879)

**Change**: For non-geom_only modes, replace hardcoded zeros (lines 858-864) with actual V + bbox computation. **Fail closed** on any error — mark metrics as unavailable, not as measured zeros.

```python
if mode == "geom_only":
    # V=Identity, mesh identical -> bbox delta is 0 by definition
    cert["bbox_delta"] = _zero_bbox_delta()
    cert["bbox_delta_available"] = True
    cert["centroid_delta"] = 0.0
    cert["vertex_rmse"] = 0.0
    cert["rmse_available"] = True
    _proof = "geom_only_exact_world_proof"
else:
    try:
        V = compute_V_for_pair(old_usd, canonical_usd, mode)
        V_np = gf_matrix4d_to_numpy(V)
        canon_transformed = (canonical_pts @ V_np[:3, :3]) + V_np[3, :3]

        bbox_old = _bbox_from_pts(old_pts)
        bbox_new = _bbox_from_pts(canon_transformed)
        delta_min = [abs(a-b) for a,b in zip(bbox_old['min'], bbox_new['min'])]
        delta_max = [abs(a-b) for a,b in zip(bbox_old['max'], bbox_new['max'])]
        max_abs = max(max(delta_min), max(delta_max))

        cert["bbox_delta"] = {"min": delta_min, "max": delta_max, "max_abs": max_abs}
        cert["bbox_delta_available"] = True
        centroid_old = np.mean(old_pts, axis=0)
        centroid_new = np.mean(canon_transformed, axis=0)
        cert["centroid_delta"] = float(np.linalg.norm(centroid_old - centroid_new))

        if len(old_pts) == len(canon_transformed):
            cert["vertex_rmse"] = float(np.sqrt(np.mean(
                np.sum((old_pts - canon_transformed)**2, axis=1))))
            cert["rmse_available"] = True
        else:
            cert["vertex_rmse"] = None
            cert["rmse_available"] = False
            cert["rmse_unavailable_reason"] = "vertex_count_mismatch"

        # Cert pre-check: gross failure gate (0.5 threshold)
        # This catches catastrophic V failures, not fine-grained bbox quality
        CERT_BBOX_PRECHECK_THRESHOLD = 0.5
        if max_abs > CERT_BBOX_PRECHECK_THRESHOLD:
            cert["eligible"] = False
            cert["reject_reason"] = f"bbox_precheck_failed_{mode}"
            return cert

    except Exception as exc:
        # FAIL CLOSED -- mark ALL metrics unavailable/None
        # Do not leave bbox_delta as _zero_bbox_delta() default from _base_pair_certificate
        cert["eligible"] = False
        cert["reject_reason"] = f"v_computation_failed_{mode}"
        cert["bbox_delta"] = None          # override base default {max_abs: 0.0}
        cert["bbox_delta_available"] = False
        cert["centroid_delta"] = None
        cert["vertex_rmse"] = None
        cert["rmse_available"] = False
        cert["rmse_unavailable_reason"] = str(exc)[:200]
        return cert

    _proof = f"{mode}_bbox_gated_proof"

cert["eligible"] = True
cert["alternate_proof_kind"] = _proof
cert["alternate_proof_passed"] = True
cert["proof_source"] = _proof
```

### File: `scripts/c1_autorun_categories.py`

**Change**: Pass `--mode-reports-dir` to cert step AND audit step (currently only passed to apply step at lines 807-808). Find where cert and audit subprocess commands are built and add the same arg. When `--bbox-gated` is set, `--mode-reports-dir` becomes a required argument at the autorun level too — validate at startup and fail early if missing.

---

## Fix 2B: Audit — Mode-aware thresholds

### File: `scripts/placement_pairwise_compare.py`

**Change 1**: Add `--mode-reports-dir` CLI arg. **REQUIRED when mode-aware thresholds are active** — without it, all prims fall to unknown -> strict threshold, which is the current behavior (safe degradation, but defeats the purpose). Build `mode_index` at startup using `compute_vertex_transform.build_mode_index()`.

**Change 2**: Per-prim mode lookup helper:

```python
def _lookup_prim_dedup_mode(left_refs, right_refs, mode_index):
    """Determine dedup mode for a ref-changed prim.

    Semantic: left=old/current, right=rewritten/canonical.
    Lookup: (canonical_uid, old_uid) in mode_index.
    """
    if not mode_index or left_refs == right_refs:
        return None
    old_uid = _uid_from_ref(left_refs[0]) if left_refs else None
    canon_uid = _uid_from_ref(right_refs[0]) if right_refs else None
    if old_uid and canon_uid:
        return (mode_index.get((canon_uid, old_uid))
                or mode_index.get((old_uid, canon_uid))
                or "transitive")
    return None
```

**Change 3**: Mode-aware eps in hard_fail logic (lines 536-553):

```python
prim_mode = _lookup_prim_dedup_mode(left_refs, right_refs, mode_index)

# Mode-aware thresholds
# Initial conservative values -- to be tightened after distribution analysis
# This is "lower false positives first, then tighten"; NOT a final standard
if prim_mode == "geom_only":
    effective_eps_bbox = eps_bbox          # 0.01 (strict, V=Identity exact)
elif prim_mode in ("topo_filesize", "shape_invariant"):
    effective_eps_bbox = 0.15              # observe phase -- NOT final standard
elif prim_mode == "transitive":
    # Transitive NOT supported this round; strict threshold
    # Count separately for diagnostics only
    effective_eps_bbox = eps_bbox
else:
    effective_eps_bbox = eps_bbox          # unknown -> strict

# Use effective_eps_bbox instead of eps_bbox in all bbox hard_fail checks below
```

**Change 4**: Add `dedup_mode` field to each compared prim entry for traceability.

**Change 5**: Add `blocking_reason_counts_by_mode` to aggregate stats:

```python
blocking_by_mode: Dict[str, Dict[str, int]] = defaultdict(lambda: defaultdict(int))
for c in compared:
    if not c.get("ref_changed"):
        continue
    m = c.get("dedup_mode") or "unknown"
    for reason in c.get("hard_failures", []):
        blocking_by_mode[m][reason] += 1
# Add blocking_by_mode to aggregate output dict
```

---

## Files to Modify (5 files)

| File | Changes |
|------|---------|
| `scripts/union_dedup_reports.py` | edge_mode tracking, `dominant_source_mode` + `source_mode_counts` per group |
| `scripts/c1_build_bulk_mapping_from_dedup_report.py` | `--mode-reports-dir` (required for bbox-gated), per-pair mode from mode_index, transitive rejection, summary by mode, canonical consistency enforcement via directed cert edges |
| `scripts/compute_vertex_transform.py` | Real V+bbox computation for non-geom_only in `build_pair_certificate()`, fail closed with `None`/unavailable on error |
| `scripts/placement_pairwise_compare.py` | `--mode-reports-dir`, mode-aware effective_eps_bbox, per-prim `dedup_mode`, `blocking_reason_counts_by_mode` |
| `scripts/c1_autorun_categories.py` | Pass `--mode-reports-dir` to cert AND audit steps; required when `--bbox-gated` |

## Schema Changes — Downstream Consumers

The new `bbox_delta_available`, `centroid_delta=None`, `vertex_rmse=None`, `bbox_delta=None` fields change the cert output schema. The following files assume these fields are always numeric/fixed-shape and must be updated to handle `None` / `bbox_delta_available=False` gracefully:

- `tests/test_compute_vertex_transform.py` — cert schema assertions
- `tests/test_bbox_gated_mapping.py` — mock cert expectations
- `scripts/summarize_bbox_ab_eval.py` — summary aggregation that may choke on None values

## Verification

1. **Union**: Run union on bottle -> check `dominant_source_mode` + `source_mode_counts` in output JSON
2. **Cert mode**: Run cert on bottle with `--mode-reports-dir` -> check `pair_mode_counts` approx {geom_only: 126, topo: 1427, shape: 55, transitive: 256 rejected}
3. **Pre-check**: Non-geom_only certs have real `bbox_delta.max_abs` values, not zeros; failed V computations show `bbox_delta_available=False`
4. **Canonical**: All edges in final `filtered_mapping` are certified direct edges
5. **Audit**: Run dry_run on bottle -> `blocking_reason_counts_by_mode` shows geom_only near-zero hard_fail, topo/shape with relaxed 0.15 threshold applied
6. **End-to-end**: Full Step 2 dry_run -> expect significant improvement in audit pass rate

## Related Documents

- `docs/operations/grscenes_test0_bbox_gated_multimode_verification_runbook.md` — v8 verification runbook (this plan addresses Step 2 failure)
- `docs/operations/grscenes_test0_bbox_gated_multimode_plan.md` — 6-change implementation plan (status: implemented)
- `docs/operations/v_compensation_translation_scaling_bug.md` — V compensation formula reference
