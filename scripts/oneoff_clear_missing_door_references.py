#!/usr/bin/env python3

"""Clear missing door_* reference arcs from layout.usd scenes (scheme A).

Reads a validate report produced by scripts/merge_asset_categories_test1.py --validate,
filters missing entries of kind=reference that point into GRScenes_assets/door_*,
and removes only those reference items from the authored reference list on the
corresponding prims.

This intentionally does NOT touch:
- payload missing
- asset-valued attribute missing (e.g. Material/mdl)
- any non-door_* references

Usage:
  ./scripts/isaac_python.sh scripts/oneoff_clear_missing_door_references.py \
    --validate-report check_reports/test1_category_merge_underscore_v3_validate.json \
    --dry-run --report check_reports/door_ref_clear_dryrun.json

  ./scripts/isaac_python.sh scripts/oneoff_clear_missing_door_references.py \
    --validate-report check_reports/test1_category_merge_underscore_v3_validate.json \
    --apply --report check_reports/door_ref_clear_apply.json
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
from collections import defaultdict
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple

from pxr import Sdf, Usd


def _load_missing(validate_report_path: str) -> List[Dict[str, Any]]:
    with open(validate_report_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    missing = (data.get("validate", {}) or {}).get("missing", [])
    if not isinstance(missing, list):
        return []
    out: List[Dict[str, Any]] = []
    for m in missing:
        if isinstance(m, dict):
            out.append(m)
    return out


def _is_door_ref_missing(m: Dict[str, Any]) -> bool:
    if m.get("kind") != "reference":
        return False
    raw = (m.get("path") or "").strip()
    resolved = (m.get("resolved") or "").strip()
    return ("GRScenes_assets/door_" in raw) or ("/GRScenes_assets/door_" in resolved)


def _make_reference_listop_like(original: Any, items: Sequence[Sdf.Reference]) -> Any:
    # Try to preserve explicitness.
    try:
        is_explicit = bool(original.IsExplicit())
    except Exception:
        is_explicit = False

    if is_explicit:
        return Sdf.ReferenceListOp.CreateExplicit(list(items))

    return Sdf.ReferenceListOp.Create(list(items), [])


def _remove_assetpaths_from_prim_references(
    prim: Usd.Prim, assetpaths_to_remove: Set[str]
) -> Tuple[int, int, Optional[str]]:
    """Returns (removed_count, remaining_count, note)."""

    if not prim.HasAuthoredReferences():
        return 0, 0, "no_authored_references"

    refs = prim.GetMetadata("references")
    if not refs:
        return 0, 0, "no_references_metadata"

    try:
        items = list(refs.GetAddedOrExplicitItems())
    except Exception as e:
        return 0, 0, f"failed_to_read_listop:{e}"

    kept: List[Sdf.Reference] = []
    removed = 0

    for r in items:
        ap = getattr(r, "assetPath", "")
        if ap in assetpaths_to_remove:
            removed += 1
            continue
        kept.append(Sdf.Reference(ap, r.primPath, r.layerOffset))

    if removed == 0:
        return 0, len(items), "no_match"

    if not kept:
        prim.ClearMetadata("references")
        return removed, 0, "cleared_references_metadata"

    prim.SetMetadata("references", _make_reference_listop_like(refs, kept))
    return removed, len(kept), "updated_references_metadata"


def process(
    validate_report: str,
    *,
    apply: bool,
    report_path: Optional[str],
    max_stages: Optional[int],
) -> Dict[str, Any]:
    missing = _load_missing(validate_report)
    door_missing = [m for m in missing if _is_door_ref_missing(m)]

    # stage -> prim -> set(assetPath)
    plan: Dict[str, Dict[str, Set[str]]] = defaultdict(lambda: defaultdict(set))
    for m in door_missing:
        stage = (m.get("stage") or "").strip()
        prim = (m.get("prim") or "").strip()
        ap = (m.get("path") or "").strip()
        if not stage or not prim or not ap:
            continue
        plan[stage][prim].add(ap)

    stages = sorted(plan.keys())
    if max_stages is not None:
        stages = stages[:max_stages]

    touched_stages = 0
    removed_total = 0
    prims_touched = 0

    per_stage: List[Dict[str, Any]] = []

    for stage_path in stages:
        stage = Usd.Stage.Open(stage_path)
        if stage is None:
            per_stage.append({"stage": stage_path, "error": "failed_to_open"})
            continue

        stage_removed = 0
        stage_prims_touched = 0
        prim_reports: List[Dict[str, Any]] = []

        for prim_path, aps in sorted(plan[stage_path].items()):
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                prim_reports.append({"prim": prim_path, "error": "prim_not_found", "remove": sorted(aps)})
                continue

            # Apply edits only if requested.
            if apply:
                removed, remaining, note = _remove_assetpaths_from_prim_references(prim, aps)
            else:
                # Dry-run: compute what would be removed.
                removed = 0
                remaining = 0
                note = "dry_run"
                if prim.HasAuthoredReferences():
                    refs = prim.GetMetadata("references")
                    if refs:
                        try:
                            items = list(refs.GetAddedOrExplicitItems())
                        except Exception:
                            items = []
                        remaining = len(items)
                        removed = sum(1 for r in items if getattr(r, "assetPath", "") in aps)

            if removed:
                stage_removed += removed
                stage_prims_touched += 1
                prims_touched += 1

            prim_reports.append(
                {
                    "prim": prim_path,
                    "type": prim.GetTypeName() or "",
                    "remove": sorted(aps),
                    "removed": removed,
                    "remaining_refs_after": remaining,
                    "note": note,
                }
            )

        if apply and stage_removed:
            stage.GetRootLayer().Save()

        if stage_removed:
            touched_stages += 1
            removed_total += stage_removed

        per_stage.append(
            {
                "stage": stage_path,
                "removed": stage_removed,
                "prims_touched": stage_prims_touched,
                "prim_reports": prim_reports,
            }
        )

    report: Dict[str, Any] = {
        "generated_at": _dt.datetime.now(tz=_dt.timezone.utc).isoformat(),
        "validate_report": os.path.abspath(validate_report),
        "apply": apply,
        "missing_total": len(missing),
        "door_reference_missing_total": len(door_missing),
        "stages_planned": len(plan),
        "stages_processed": len(stages),
        "touched_stages": touched_stages,
        "prims_touched": prims_touched,
        "references_removed_total": removed_total,
        "per_stage": per_stage,
    }

    if report_path:
        os.makedirs(os.path.dirname(report_path) or ".", exist_ok=True)
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print("Wrote report:", report_path)

    print("door_reference_missing_total:", report["door_reference_missing_total"], flush=True)
    print("references_removed_total:", report["references_removed_total"], flush=True)
    print("touched_stages:", report["touched_stages"], flush=True)

    return report


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--validate-report", required=True)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--report", default=None)
    ap.add_argument("--max-stages", type=int, default=None)
    args = ap.parse_args(argv)

    if args.dry_run == args.apply:
        raise SystemExit("Choose exactly one mode: --dry-run OR --apply")

    process(
        args.validate_report,
        apply=bool(args.apply),
        report_path=args.report,
        max_stages=args.max_stages,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
