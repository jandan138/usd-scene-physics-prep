#!/usr/bin/env python3
"""Build a normalize-only gate verdict from pairwise and audit reports.

This codifies the current smoke-run decision logic that had previously been
assembled manually into final_verdict.json / summary.md. It also incorporates
the descendant-xform-override safety signal exposed by audit_normalize_phase2.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List, Optional, Sequence


def _load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _pick_worst_prim(pairwise: Dict[str, Any]) -> Dict[str, Any]:
    aggregate = pairwise.get("aggregate", {})
    worst = (aggregate.get("global_top_50_worst") or [{}])[0]
    return {
        "scene_id": worst.get("scene_id"),
        "prim_path": worst.get("prim_path"),
        "displacement": worst.get("displacement"),
        "max_per_mesh_displacement": worst.get("max_per_mesh_displacement"),
        "ref_changed": worst.get("ref_changed"),
        "left_ref_sample": worst.get("left_ref_sample", []),
        "right_ref_sample": worst.get("right_ref_sample", []),
        "verts_left": worst.get("verts_left"),
        "verts_right": worst.get("verts_right"),
        "meshes_left": worst.get("meshes_left"),
        "meshes_right": worst.get("meshes_right"),
    }


def build_verdict(
    run_id: str,
    pairwise: Dict[str, Any],
    audit: Dict[str, Any],
    pairwise_path: str,
    audit_path: str,
    max_missing_centers: int = 0,
) -> Dict[str, Any]:
    pairwise_agg = pairwise.get("aggregate", {})
    audit_agg = audit.get("aggregate", {})
    audit_totals = audit_agg.get("totals", {})

    normalize_pairwise_gt = int(
        pairwise_agg.get("displaced_breakdown", {}).get("gt_0.01", 0)
    )
    normalize_ref_same_gt = int(
        pairwise_agg.get("ref_same_breakdown", {}).get("gt_0.01", 0)
    )
    common_ref_prim_count = int(audit_totals.get("common_ref_prim_count", 0))
    center_found = int(audit_totals.get("center_found", 0))
    matrix_mismatch = int(audit_totals.get("matrix_mismatch", 0))
    missing_pre_c1 = audit.get("missing_pre_c1_scenes", [])
    inert_descendant = int(
        audit_totals.get("pre_c1_inert_descendant_xform_override_count", 0)
    )
    became_inert_descendant = int(
        audit_totals.get("became_inert_descendant_xform_override_count", 0)
    )

    gate_failed = any(
        [
            normalize_pairwise_gt > 0,
            normalize_ref_same_gt > 0,
            (common_ref_prim_count - center_found) > max_missing_centers,
            matrix_mismatch != 0,
            bool(missing_pre_c1),
            inert_descendant > 0,
            became_inert_descendant > 0,
        ]
    )

    if gate_failed:
        status = "fail_normalize_s1"
        reason = "normalize-only gate failed before dedup"
    else:
        status = "pass_normalize_s1"
        reason = "normalize-only gate passed"

    verdict = {
        "run_id": run_id,
        "status": status,
        "reason": reason,
        "gates": {
            "normalize_pairwise_gt_0.01": normalize_pairwise_gt,
            "normalize_ref_same_gt_0.01": normalize_ref_same_gt,
            "audit_center_found": center_found,
            "audit_common_ref_prim_count": common_ref_prim_count,
            "max_missing_centers": max_missing_centers,
            "skipped_missing_center": common_ref_prim_count - center_found,
            "audit_matrix_mismatch": matrix_mismatch,
            "audit_missing_pre_c1_scene_count": len(missing_pre_c1),
            "audit_pre_c1_inert_descendant_xform_override_count": inert_descendant,
            "audit_became_inert_descendant_xform_override_count": became_inert_descendant,
        },
        "worst_prim": _pick_worst_prim(pairwise),
        "reports": {
            "pairwise_normalize_only": os.path.abspath(pairwise_path),
            "audit_normalize_phase2": os.path.abspath(audit_path),
        },
        "next_action": (
            "inspect and handle descendant xform overrides before any dedup-only run"
            if inert_descendant or became_inert_descendant
            else "inspect the normalize outlier before any dedup-only run"
        ),
    }

    top_inert = audit_agg.get("top_50_inert_descendant_xform_overrides") or []
    if top_inert:
        verdict["top_inert_descendant_xform_override"] = top_inert[0]
    return verdict


def build_summary_markdown(verdict: Dict[str, Any]) -> str:
    gates = verdict["gates"]
    lines: List[str] = [
        "# Normalize Gate Summary",
        "",
        f"- Run ID: `{verdict['run_id']}`",
        f"- Verdict: `{verdict['status']}`",
        f"- Reason: {verdict['reason']}",
        f"- Pairwise displaced > 0.01: `{gates['normalize_pairwise_gt_0.01']}`",
        f"- Pairwise ref_same > 0.01: `{gates['normalize_ref_same_gt_0.01']}`",
        (
            "- Audit center_found/common_ref_prim_count: "
            f"`{gates['audit_center_found']}` / `{gates['audit_common_ref_prim_count']}`"
            f" (gap: `{gates['skipped_missing_center']}`, max allowed: `{gates['max_missing_centers']}`)"
        ),
        f"- Audit matrix_mismatch: `{gates['audit_matrix_mismatch']}`",
        (
            "- Audit inert descendant overrides: "
            f"`{gates['audit_pre_c1_inert_descendant_xform_override_count']}`"
        ),
        (
            "- Audit became-inert descendant overrides: "
            f"`{gates['audit_became_inert_descendant_xform_override_count']}`"
        ),
    ]

    worst = verdict.get("worst_prim") or {}
    if worst.get("prim_path"):
        lines.extend(
            [
                "",
                "## Worst Prim",
                "",
                f"- Prim: `{worst.get('prim_path')}`",
                f"- Scene: `{worst.get('scene_id')}`",
                f"- Displacement: `{worst.get('displacement')}`",
                f"- Max per-mesh displacement: `{worst.get('max_per_mesh_displacement')}`",
            ]
        )

    top_inert = verdict.get("top_inert_descendant_xform_override") or {}
    if top_inert:
        lines.extend(
            [
                "",
                "## Descendant Override Finding",
                "",
                f"- Prim: `{top_inert.get('prim_path')}`",
                f"- Scene-layer props: `{top_inert.get('scene_layer_props')}`",
                f"- Composed xformOpOrder: `{top_inert.get('composed_xformOpOrder')}`",
                f"- Inert scene xform props: `{top_inert.get('inert_scene_xform_props')}`",
            ]
        )

    lines.extend(
        [
            "",
            "## Reports",
            "",
            f"- Pairwise: `{verdict['reports']['pairwise_normalize_only']}`",
            f"- Audit: `{verdict['reports']['audit_normalize_phase2']}`",
            "",
            "## Next Action",
            "",
            verdict["next_action"],
            "",
        ]
    )
    return "\n".join(lines)


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--pairwise-json", required=True)
    parser.add_argument("--audit-json", required=True)
    parser.add_argument("--out-json", default=None)
    parser.add_argument("--out-md", default=None)
    parser.add_argument(
        "--max-missing-centers",
        type=int,
        default=0,
        help="Max allowed center_gap (common_ref - center_found). 0 = strict equality.",
    )
    args = parser.parse_args(argv)

    pairwise = _load_json(args.pairwise_json)
    audit = _load_json(args.audit_json)
    verdict = build_verdict(
        run_id=args.run_id,
        pairwise=pairwise,
        audit=audit,
        pairwise_path=args.pairwise_json,
        audit_path=args.audit_json,
        max_missing_centers=args.max_missing_centers,
    )
    summary_md = build_summary_markdown(verdict)

    if args.out_json:
        os.makedirs(os.path.dirname(os.path.abspath(args.out_json)), exist_ok=True)
        with open(args.out_json, "w", encoding="utf-8") as f:
            json.dump(verdict, f, indent=2, ensure_ascii=False)
    else:
        json.dump(verdict, sys.stdout, indent=2, ensure_ascii=False)
        sys.stdout.write("\n")

    if args.out_md:
        os.makedirs(os.path.dirname(os.path.abspath(args.out_md)), exist_ok=True)
        with open(args.out_md, "w", encoding="utf-8") as f:
            f.write(summary_md)
    elif not args.out_json:
        sys.stdout.write("\n")
        sys.stdout.write(summary_md)
        sys.stdout.write("\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
