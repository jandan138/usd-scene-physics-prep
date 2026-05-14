#!/usr/bin/env python3
"""Clear references to known invalid asset shells and optionally quarantine them.

This is intentionally narrow: it targets the four annotation-only asset records
found in the test0 parallel dataset integrity investigation on 2026-05-14.

The script removes only authored reference arcs whose assetPath points to one of
the target `GRScenes_assets/<category>/<uid>/usd/<uid>.usd` files. It can also
move the corresponding annotation-only asset directories out of the final
dataset into a quarantine/backup directory.
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import shutil
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple

from pxr import Sdf, Usd


DEFAULT_INVALID_ASSETS = [
    "cabinet/b98d6ccbeb75dfdeb60e27649a5b055a",
    "other/d41d8cd98f00b204e9800998ecf8427e",
    "person/351316cbb083f9f4df0cccd60cbfa848",
    "person/d41d8cd98f00b204e9800998ecf8427e",
]


def _normalize_invalid_asset(value: str) -> str:
    rel = value.strip().strip("/")
    parts = rel.split("/")
    if len(parts) != 2 or not all(parts):
        raise ValueError(f"invalid asset must be '<category>/<uid>': {value!r}")
    return rel


def _asset_usd_pattern(asset_rel: str) -> str:
    category, uid = _normalize_invalid_asset(asset_rel).split("/")
    return f"GRScenes_assets/{category}/{uid}/usd/{uid}.usd"


def _iter_layouts(dataset_root: Path) -> Iterable[Path]:
    yield from sorted((dataset_root / "GRScenes100").glob("*/*_usd/layout.usd"))


def _make_reference_listop_like(original: Any, items: Sequence[Sdf.Reference]) -> Any:
    try:
        is_explicit = bool(original.IsExplicit())
    except Exception:
        is_explicit = False

    if is_explicit:
        return Sdf.ReferenceListOp.CreateExplicit(list(items))

    return Sdf.ReferenceListOp.Create(list(items), [])


def _remove_target_refs_from_prim(
    prim: Usd.Prim,
    target_asset_paths: Set[str],
    *,
    apply: bool,
) -> Tuple[int, int, str]:
    if not prim.HasAuthoredReferences():
        return 0, 0, "no_authored_references"

    refs = prim.GetMetadata("references")
    if not refs:
        return 0, 0, "no_references_metadata"

    try:
        items = list(refs.GetAddedOrExplicitItems())
    except Exception as exc:
        return 0, 0, f"failed_to_read_listop:{exc}"

    kept: List[Sdf.Reference] = []
    removed = 0
    for ref in items:
        asset_path = getattr(ref, "assetPath", "") or ""
        if asset_path in target_asset_paths:
            removed += 1
            continue
        kept.append(Sdf.Reference(asset_path, ref.primPath, ref.layerOffset))

    if not apply or removed == 0:
        return removed, len(kept), "dry_run" if removed else "no_match"

    if not kept:
        prim.ClearMetadata("references")
        return removed, 0, "cleared_references_metadata"

    prim.SetMetadata("references", _make_reference_listop_like(refs, kept))
    return removed, len(kept), "updated_references_metadata"


def _copy_layout_backup(stage_path: Path, *, dataset_root: Path, backup_root: Path) -> str:
    rel = stage_path.relative_to(dataset_root)
    dst = backup_root / "layouts" / rel
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(stage_path, dst)
    return str(dst)


def _is_annotation_only_shell(asset_dir: Path) -> bool:
    if not asset_dir.is_dir():
        return False
    entries = list(asset_dir.iterdir())
    if not entries:
        return False
    return all(p.is_file() and p.suffix == ".json" for p in entries)


def _quarantine_asset_shell(
    asset_dir: Path,
    *,
    dataset_root: Path,
    quarantine_root: Path,
    apply: bool,
) -> Dict[str, Any]:
    rel = asset_dir.relative_to(dataset_root)
    dst = quarantine_root / rel
    shell_ok = _is_annotation_only_shell(asset_dir)

    result: Dict[str, Any] = {
        "asset_dir": str(asset_dir),
        "quarantine_path": str(dst),
        "exists": asset_dir.exists(),
        "annotation_only_shell": shell_ok,
        "quarantined": False,
    }

    if not asset_dir.exists():
        result["note"] = "asset_dir_absent"
        return result

    if not shell_ok:
        result["note"] = "not_annotation_only_shell"
        return result

    if not apply:
        result["note"] = "dry_run"
        return result

    if dst.exists():
        raise FileExistsError(f"quarantine destination already exists: {dst}")

    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(asset_dir), str(dst))
    result["quarantined"] = True
    result["note"] = "moved_to_quarantine"
    return result


def _collect_reference_plan(
    dataset_root: Path,
    invalid_assets: Sequence[str],
) -> Tuple[Dict[Path, Dict[str, Set[str]]], int, int]:
    patterns = {_asset_usd_pattern(asset) for asset in invalid_assets}
    plan: Dict[Path, Dict[str, Set[str]]] = defaultdict(lambda: defaultdict(set))
    layouts_scanned = 0
    references_planned = 0

    for layout in _iter_layouts(dataset_root):
        layouts_scanned += 1
        stage = Usd.Stage.Open(str(layout))
        if stage is None:
            continue

        for prim in stage.TraverseAll():
            if not prim.HasAuthoredReferences():
                continue
            refs = prim.GetMetadata("references")
            if not refs:
                continue
            try:
                items = list(refs.GetAddedOrExplicitItems())
            except Exception:
                continue
            for ref in items:
                asset_path = getattr(ref, "assetPath", "") or ""
                if any(pattern in asset_path for pattern in patterns):
                    plan[layout][str(prim.GetPath())].add(asset_path)
                    references_planned += 1

    return plan, layouts_scanned, references_planned


def process_dataset(
    *,
    dataset_root: Path | str,
    invalid_assets: Sequence[str],
    apply: bool,
    quarantine_root: Optional[Path | str],
    backup_root: Optional[Path | str],
    report_path: Optional[Path | str],
) -> Dict[str, Any]:
    dataset_root = Path(dataset_root)
    invalid_assets = [_normalize_invalid_asset(asset) for asset in invalid_assets]
    quarantine_path = Path(quarantine_root) if quarantine_root else None
    backup_path = Path(backup_root) if backup_root else None

    if apply and backup_path is None:
        raise ValueError("--backup-root is required with --apply")

    plan, layouts_scanned, references_planned = _collect_reference_plan(
        dataset_root, invalid_assets
    )

    touched_stages = 0
    references_removed_total = 0
    prims_touched = 0
    per_stage: List[Dict[str, Any]] = []

    for layout_path in sorted(plan):
        if apply and backup_path is not None:
            layout_backup = _copy_layout_backup(
                layout_path, dataset_root=dataset_root, backup_root=backup_path
            )
        else:
            layout_backup = None

        stage = Usd.Stage.Open(str(layout_path))
        if stage is None:
            per_stage.append({"stage": str(layout_path), "error": "failed_to_open"})
            continue

        stage_removed = 0
        stage_prims_touched = 0
        prim_reports: List[Dict[str, Any]] = []

        for prim_path, asset_paths in sorted(plan[layout_path].items()):
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                prim_reports.append(
                    {
                        "prim": prim_path,
                        "remove": sorted(asset_paths),
                        "error": "prim_not_found",
                    }
                )
                continue

            removed, remaining, note = _remove_target_refs_from_prim(
                prim,
                asset_paths,
                apply=apply,
            )
            if removed:
                stage_removed += removed
                stage_prims_touched += 1

            prim_reports.append(
                {
                    "prim": prim_path,
                    "type": prim.GetTypeName() or "",
                    "remove": sorted(asset_paths),
                    "removed": removed,
                    "remaining_refs_after": remaining,
                    "note": note,
                }
            )

        if apply and stage_removed:
            stage.GetRootLayer().Save()

        if stage_removed:
            touched_stages += 1
            references_removed_total += stage_removed
            prims_touched += stage_prims_touched

        per_stage.append(
            {
                "stage": str(layout_path),
                "backup": layout_backup,
                "removed": stage_removed,
                "prims_touched": stage_prims_touched,
                "prim_reports": prim_reports,
            }
        )

    per_asset: List[Dict[str, Any]] = []
    for asset in invalid_assets:
        asset_dir = dataset_root / "GRScenes_assets" / asset
        if quarantine_path is not None:
            asset_report = _quarantine_asset_shell(
                asset_dir,
                dataset_root=dataset_root,
                quarantine_root=quarantine_path,
                apply=apply,
            )
        else:
            asset_report = {
                "asset_dir": str(asset_dir),
                "exists": asset_dir.exists(),
                "quarantined": False,
                "note": "quarantine_disabled",
            }
        per_asset.append(asset_report)

    report: Dict[str, Any] = {
        "generated_at": _dt.datetime.now(tz=_dt.timezone.utc).isoformat(),
        "dataset_root": str(dataset_root),
        "apply": apply,
        "invalid_assets": invalid_assets,
        "layouts_scanned": layouts_scanned,
        "stages_planned": len(plan),
        "references_planned_total": references_planned,
        "touched_stages": touched_stages,
        "prims_touched": prims_touched,
        "references_removed_total": references_removed_total,
        "asset_shells_found_total": sum(1 for item in per_asset if item["exists"]),
        "asset_shells_quarantined_total": sum(
            1 for item in per_asset if item["quarantined"]
        ),
        "backup_root": str(backup_path) if backup_path else None,
        "quarantine_root": str(quarantine_path) if quarantine_path else None,
        "per_stage": per_stage,
        "per_asset": per_asset,
    }

    if report_path:
        report_file = Path(report_path)
        report_file.parent.mkdir(parents=True, exist_ok=True)
        report_file.write_text(json.dumps(report, indent=2, ensure_ascii=False))
        print("Wrote report:", report_file)

    print("layouts_scanned:", report["layouts_scanned"], flush=True)
    print("references_planned_total:", report["references_planned_total"], flush=True)
    print("references_removed_total:", report["references_removed_total"], flush=True)
    print("touched_stages:", report["touched_stages"], flush=True)
    print(
        "asset_shells_quarantined_total:",
        report["asset_shells_quarantined_total"],
        flush=True,
    )
    return report


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset-root", required=True)
    parser.add_argument(
        "--invalid-asset",
        action="append",
        default=[],
        help="Target invalid asset as '<category>/<uid>'. Defaults to the four known shells.",
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--backup-root", default=None)
    parser.add_argument("--quarantine-root", default=None)
    parser.add_argument("--report", default=None)
    args = parser.parse_args(argv)

    if args.dry_run == args.apply:
        raise SystemExit("Choose exactly one mode: --dry-run OR --apply")

    invalid_assets = args.invalid_asset or DEFAULT_INVALID_ASSETS
    process_dataset(
        dataset_root=args.dataset_root,
        invalid_assets=invalid_assets,
        apply=bool(args.apply),
        quarantine_root=args.quarantine_root,
        backup_root=args.backup_root,
        report_path=args.report,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
