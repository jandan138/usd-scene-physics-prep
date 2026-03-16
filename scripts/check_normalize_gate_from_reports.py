#!/usr/bin/env python3
"""Compute a normalize-only gate verdict from pairwise + phase2 audit reports.

This is a read-only helper for smoke/full normalize validation. It consolidates
the existing pairwise placement report with audit_normalize_phase2.py output and
adds descendant-override safety checks so scenes like the refrigerator outlier
fail the gate explicitly.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, Optional, Sequence


def _load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _get(d: Dict[str, Any], *path: str, default: Any = None) -> Any:
    cur: Any = d
    for key in path:
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur


def compute_gate_verdict(
    pairwise: Dict[str, Any],
    audit: Dict[str, Any],
    displaced_threshold: int = 0,
    ref_same_threshold: int = 0,
    max_missing_centers: int = 0,
) -> Dict[str, Any]:
    pairwise_agg = _get(pairwise, "aggregate", default={}) or {}
    audit_agg = _get(audit, "aggregate", default={}) or {}
    audit_totals = _get(audit_agg, "totals", default={}) or {}

    displaced_gt = int(_get(pairwise_agg, "displaced_breakdown", "gt_0.01", default=0) or 0)
    ref_same_gt = int(_get(pairwise_agg, "ref_same_breakdown", "gt_0.01", default=0) or 0)
    common_ref = int(audit_totals.get("common_ref_prim_count", 0) or 0)
    center_found = int(audit_totals.get("center_found", 0) or 0)
    matrix_mismatch = int(audit_totals.get("matrix_mismatch", 0) or 0)
    missing_pre_c1 = list(audit.get("missing_pre_c1_scenes", []) or [])
    inert_descendant = int(audit_totals.get("pre_c1_inert_descendant_xform_override_count", 0) or 0)
    became_inert = int(audit_totals.get("became_inert_descendant_xform_override_count", 0) or 0)

    checks = {
        "pairwise_displaced_gt_0_01_ok": displaced_gt <= displaced_threshold,
        "pairwise_ref_same_gt_0_01_ok": ref_same_gt <= ref_same_threshold,
        "center_gap_within_allowance": (common_ref - center_found) <= max_missing_centers,
        "matrix_mismatch_zero": matrix_mismatch == 0,
        "missing_pre_c1_empty": len(missing_pre_c1) == 0,
        "pre_c1_inert_descendant_override_zero": inert_descendant == 0,
        "became_inert_descendant_override_zero": became_inert == 0,
    }
    passed = all(checks.values())

    worst_prim = None
    top_worst = _get(pairwise_agg, "global_top_50_worst", default=[]) or []
    if top_worst:
        worst_prim = top_worst[0]

    top_inert = _get(audit_agg, "top_50_inert_descendant_xform_overrides", default=[]) or []

    reason_parts = []
    if not checks["pairwise_displaced_gt_0_01_ok"]:
        reason_parts.append(f"pairwise_displaced_gt_0.01={displaced_gt}")
    if not checks["pairwise_ref_same_gt_0_01_ok"]:
        reason_parts.append(f"pairwise_ref_same_gt_0.01={ref_same_gt}")
    if not checks["center_gap_within_allowance"]:
        reason_parts.append(
            f"center_gap={common_ref - center_found} > max_missing_centers={max_missing_centers}"
        )
    if not checks["matrix_mismatch_zero"]:
        reason_parts.append(f"matrix_mismatch={matrix_mismatch}")
    if not checks["missing_pre_c1_empty"]:
        reason_parts.append(f"missing_pre_c1={len(missing_pre_c1)}")
    if not checks["pre_c1_inert_descendant_override_zero"]:
        reason_parts.append(f"pre_c1_inert_descendant={inert_descendant}")
    if not checks["became_inert_descendant_override_zero"]:
        reason_parts.append(f"became_inert_descendant={became_inert}")

    return {
        "status": "pass_normalize_gate" if passed else "fail_normalize_gate",
        "passed": passed,
        "checks": checks,
        "metrics": {
            "pairwise_displaced_gt_0.01": displaced_gt,
            "pairwise_ref_same_gt_0.01": ref_same_gt,
            "common_ref_prim_count": common_ref,
            "center_found": center_found,
            "matrix_mismatch": matrix_mismatch,
            "max_missing_centers": max_missing_centers,
            "skipped_missing_center": common_ref - center_found,
            "missing_pre_c1_count": len(missing_pre_c1),
            "pre_c1_inert_descendant_xform_override_count": inert_descendant,
            "became_inert_descendant_xform_override_count": became_inert,
        },
        "missing_pre_c1_scenes": missing_pre_c1,
        "worst_prim": worst_prim,
        "top_inert_descendant_xform_overrides": top_inert[:10],
        "reason": "ok" if passed else "; ".join(reason_parts),
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pairwise-report", required=True, help="placement_pairwise_compare JSON path")
    parser.add_argument("--audit-report", required=True, help="audit_normalize_phase2 JSON path")
    parser.add_argument("--out", default=None, help="Optional JSON output path")
    parser.add_argument("--displaced-threshold", type=int, default=0)
    parser.add_argument("--ref-same-threshold", type=int, default=0)
    parser.add_argument(
        "--max-missing-centers",
        type=int,
        default=0,
        help="Max allowed center_gap (common_ref - center_found). 0 = strict equality.",
    )
    args = parser.parse_args(argv)

    verdict = compute_gate_verdict(
        pairwise=_load_json(args.pairwise_report),
        audit=_load_json(args.audit_report),
        displaced_threshold=args.displaced_threshold,
        ref_same_threshold=args.ref_same_threshold,
        max_missing_centers=args.max_missing_centers,
    )

    payload = json.dumps(verdict, indent=2, ensure_ascii=False)
    if args.out:
        out_path = os.path.abspath(args.out)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as f:
            f.write(payload)
            f.write("\n")
        print(out_path)
    else:
        print(payload)

    return 0 if verdict["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
