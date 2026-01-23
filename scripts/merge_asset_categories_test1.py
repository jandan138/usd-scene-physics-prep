#!/usr/bin/env python3
"""Merge category folder aliases under GRScenes_assets and rewrite USD paths.

Scope (per user request)
- Only merge these aliases (underscore style canonical):
  - coffeemaker  -> coffee_maker
  - sofachair    -> sofa_chair
  - tvstand      -> tv_stand
- Do NOT merge door_* variants.

What this does
1) Dry-run mode: compute folder moves + UID collisions + estimate USD rewrites.
2) Apply mode:
   - Move asset UID folders from old category folder into new one.
   - Update per-asset *_annotation.json: set "category" to canonical name.
   - Update GRScenes_assets/Asset_annotation.json category entries for affected categories.
   - Rewrite USD internal paths in both GRScenes_assets/** and GRScenes100/** to use canonical categories.
3) Validate mode: check that all GRScenes100/**/layout.usd stages resolve referenced/payload USDs
   and asset-valued attributes (best-effort filesystem existence check).

Run (recommended):
  ./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py --subset-root GRScenes-test1 --dry-run --report check_reports/test1_merge_dryrun.json
  ./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py --subset-root GRScenes-test1 --apply   --report check_reports/test1_merge_apply.json
  ./scripts/isaac_python.sh scripts/merge_asset_categories_test1.py --subset-root GRScenes-test1 --validate --report check_reports/test1_merge_validate.json
"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import shutil
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

from pxr import Sdf, Usd


CATEGORY_MERGES: Dict[str, str] = {
    "coffeemaker": "coffee_maker",
    "sofachair": "sofa_chair",
    "tvstand": "tv_stand",
}

_NON_FILE_SCHEMES = (
    "http://",
    "https://",
    "omniverse://",
    "omni://",
    "omni:",
    "file://",
)


def _norm_abs(p: str) -> str:
    return os.path.abspath(os.path.normpath(p))


def _posix(path: str) -> str:
    return path.replace(os.sep, "/")


def _is_non_file_scheme(path: str) -> bool:
    p = (path or "").strip()
    return any(p.startswith(s) for s in _NON_FILE_SCHEMES)


def _iter_usd_files(root: str) -> Iterable[str]:
    for dirpath, _dirnames, filenames in os.walk(root):
        for fn in filenames:
            if fn.endswith((".usd", ".usda", ".usdc")):
                yield os.path.join(dirpath, fn)


def _list_uid_dirs(category_dir: str) -> List[str]:
    if not os.path.isdir(category_dir):
        return []
    out: List[str] = []
    for name in os.listdir(category_dir):
        p = os.path.join(category_dir, name)
        if os.path.isdir(p):
            out.append(name)
    out.sort()
    return out


@dataclass
class MovePlan:
    old_category: str
    new_category: str
    moves: List[Tuple[str, str]]  # (src_abs, dst_abs)
    uid_collisions: List[str]


def compute_move_plans(subset_root: str) -> List[MovePlan]:
    assets_root = os.path.join(subset_root, "GRScenes_assets")
    plans: List[MovePlan] = []

    for old_cat, new_cat in CATEGORY_MERGES.items():
        old_dir = os.path.join(assets_root, old_cat)
        new_dir = os.path.join(assets_root, new_cat)

        old_uids = set(_list_uid_dirs(old_dir))
        new_uids = set(_list_uid_dirs(new_dir))

        collisions = sorted(old_uids.intersection(new_uids))
        moves: List[Tuple[str, str]] = []
        for uid in sorted(old_uids - new_uids):
            src = os.path.join(old_dir, uid)
            dst = os.path.join(new_dir, uid)
            moves.append((src, dst))

        plans.append(
            MovePlan(
                old_category=old_cat,
                new_category=new_cat,
                moves=moves,
                uid_collisions=collisions,
            )
        )

    return plans


def _replace_category_in_str(s: str) -> Tuple[str, bool]:
    if not s:
        return s, False

    original = s
    # Handle both separator styles.
    for old_cat, new_cat in CATEGORY_MERGES.items():
        original = original.replace(f"GRScenes_assets/{old_cat}/", f"GRScenes_assets/{new_cat}/")
        original = original.replace(f"GRScenes_assets\\{old_cat}\\", f"GRScenes_assets\\{new_cat}\\")
    return original, original != s


def _rewrite_reference_listop(op: Any) -> Tuple[Any, int]:
    if not op:
        return op, 0

    changed = 0
    new_items = []
    for ref in op.GetAddedOrExplicitItems():
        ap = getattr(ref, "assetPath", "")
        new_ap, did = _replace_category_in_str(ap)
        if did:
            changed += 1
        new_ref = Sdf.Reference(new_ap, ref.primPath, ref.layerOffset)
        new_items.append(new_ref)

    # Preserve explicitness when possible.
    try:
        is_explicit = bool(op.IsExplicit())
    except Exception:
        is_explicit = False

    if is_explicit:
        return Sdf.ReferenceListOp.CreateExplicit(new_items), changed

    # Fallback: Create prepended list op.
    return Sdf.ReferenceListOp.Create(new_items, []), changed


def _rewrite_payload_listop(op: Any) -> Tuple[Any, int]:
    if not op:
        return op, 0

    changed = 0
    new_items = []
    for pl in op.GetAddedOrExplicitItems():
        ap = getattr(pl, "assetPath", "")
        new_ap, did = _replace_category_in_str(ap)
        if did:
            changed += 1
        new_pl = Sdf.Payload(new_ap, pl.primPath, pl.layerOffset)
        new_items.append(new_pl)

    try:
        is_explicit = bool(op.IsExplicit())
    except Exception:
        is_explicit = False

    if is_explicit:
        return Sdf.PayloadListOp.CreateExplicit(new_items), changed

    return Sdf.PayloadListOp.Create(new_items, []), changed


def rewrite_usd_paths_in_stage(usd_path: str, *, save: bool) -> Dict[str, int]:
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise RuntimeError(f"Failed to open USD: {usd_path}")

    refs_changed = 0
    payloads_changed = 0
    asset_attrs_changed = 0
    str_attrs_changed = 0

    for prim in stage.Traverse():
        # References
        if prim.HasAuthoredReferences():
            refs = prim.GetMetadata("references")
            new_refs, n = _rewrite_reference_listop(refs)
            if n:
                prim.SetMetadata("references", new_refs)
                refs_changed += n

        # Payloads
        if prim.HasAuthoredPayloads():
            pls = prim.GetMetadata("payloads")
            new_pls, n = _rewrite_payload_listop(pls)
            if n:
                prim.SetMetadata("payloads", new_pls)
                payloads_changed += n

        # Attributes
        for attr in prim.GetAttributes():
            t = attr.GetTypeName()
            if t in (Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.AssetArray):
                try:
                    v = attr.Get()
                except Exception:
                    continue

                if isinstance(v, Sdf.AssetPath):
                    new_path, did = _replace_category_in_str(v.path)
                    if did:
                        attr.Set(Sdf.AssetPath(new_path))
                        asset_attrs_changed += 1
                elif isinstance(v, (list, tuple)):
                    new_list = []
                    did_any = False
                    for av in v:
                        if isinstance(av, Sdf.AssetPath):
                            new_path, did = _replace_category_in_str(av.path)
                            if did:
                                did_any = True
                            new_list.append(Sdf.AssetPath(new_path))
                        else:
                            new_list.append(av)
                    if did_any:
                        attr.Set(new_list)
                        asset_attrs_changed += 1

            # Some inputs:file can be strings.
            try:
                v2 = attr.Get()
            except Exception:
                continue
            if isinstance(v2, str) and "GRScenes_assets" in v2:
                new_s, did = _replace_category_in_str(v2)
                if did:
                    attr.Set(new_s)
                    str_attrs_changed += 1

    if save and (refs_changed or payloads_changed or asset_attrs_changed or str_attrs_changed):
        stage.GetRootLayer().Save()

    return {
        "refs_changed": refs_changed,
        "payloads_changed": payloads_changed,
        "asset_attrs_changed": asset_attrs_changed,
        "str_attrs_changed": str_attrs_changed,
    }


def _update_asset_annotation_index(subset_root: str) -> Dict[str, Any]:
    """Update GRScenes_assets/Asset_annotation.json for affected categories only."""

    assets_root = os.path.join(subset_root, "GRScenes_assets")
    index_path = os.path.join(assets_root, "Asset_annotation.json")
    if not os.path.isfile(index_path):
        return {"updated": False, "reason": "Asset_annotation.json not found"}

    with open(index_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    cats = data.get("categories")
    if not isinstance(cats, dict):
        return {"updated": False, "reason": "categories missing or invalid"}

    touched = []
    for old_cat, new_cat in CATEGORY_MERGES.items():
        # Recompute from filesystem
        new_dir = os.path.join(assets_root, new_cat)
        if os.path.isdir(new_dir):
            uids = _list_uid_dirs(new_dir)
            cats[new_cat] = {"count": len(uids), "uids": uids}
            touched.append(new_cat)
        # Remove old category entry if present
        if old_cat in cats:
            cats.pop(old_cat, None)
            touched.append(old_cat)

    data["categories"] = cats

    with open(index_path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)

    return {"updated": True, "touched": sorted(set(touched)), "path": index_path}


def _update_per_asset_annotation_category(uid_dir: str, new_category: str) -> int:
    changed = 0
    uid = os.path.basename(uid_dir)
    ann_path = os.path.join(uid_dir, f"{uid}_annotation.json")
    if not os.path.isfile(ann_path):
        return 0

    try:
        with open(ann_path, "r", encoding="utf-8") as f:
            data = json.load(f)
    except Exception:
        return 0

    if data.get("category") != new_category:
        data["category"] = new_category
        with open(ann_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        changed += 1

    return changed


def apply_moves(plans: Sequence[MovePlan], *, dry_run: bool) -> Dict[str, Any]:
    moved = []
    per_asset_annotation_updates = 0

    for plan in plans:
        # Ensure destination category dir exists.
        if plan.moves:
            dst_cat_dir = os.path.dirname(plan.moves[0][1])
            if not dry_run:
                os.makedirs(dst_cat_dir, exist_ok=True)

        for src, dst in plan.moves:
            moved.append({"src": src, "dst": dst})
            if not dry_run:
                os.makedirs(os.path.dirname(dst), exist_ok=True)
                shutil.move(src, dst)
                per_asset_annotation_updates += _update_per_asset_annotation_category(dst, plan.new_category)

        # If old category dir becomes empty, remove it.
        old_dir = os.path.dirname(plan.moves[0][0]) if plan.moves else None
        if old_dir and not dry_run and os.path.isdir(old_dir):
            try:
                if not os.listdir(old_dir):
                    os.rmdir(old_dir)
            except Exception:
                pass

    return {
        "moved_count": len(moved),
        "moved": moved,
        "per_asset_annotation_updates": per_asset_annotation_updates,
    }


def validate_scenes(subset_root: str, *, max_scenes: Optional[int] = None) -> Dict[str, Any]:
    """Validate GRScenes100 scenes by checking referenced/payload USDs and asset-valued attrs exist."""

    scenes_root = os.path.join(subset_root, "GRScenes100")
    missing: List[Dict[str, str]] = []
    checked = 0

    layout_usds = []
    for dirpath, _dirnames, filenames in os.walk(scenes_root):
        for fn in filenames:
            if fn == "layout.usd":
                layout_usds.append(os.path.join(dirpath, fn))

    layout_usds.sort()

    for layout in layout_usds:
        if max_scenes is not None and checked >= max_scenes:
            break

        stage = Usd.Stage.Open(layout)
        if stage is None:
            missing.append({"kind": "stage_open", "stage": layout, "path": "", "note": "failed_to_open"})
            checked += 1
            continue

        root_layer = stage.GetRootLayer()
        anchor_dir = os.path.dirname(root_layer.realPath) if root_layer.realPath else os.path.dirname(layout)

        def check_path(kind: str, raw: str, prim_path: str):
            if not raw:
                return
            if _is_non_file_scheme(raw):
                return
            resolved = raw
            if not os.path.isabs(resolved):
                resolved = os.path.normpath(os.path.join(anchor_dir, resolved))
            if not os.path.exists(resolved):
                missing.append({"kind": kind, "stage": layout, "prim": prim_path, "path": raw, "resolved": resolved})

        for prim in stage.Traverse():
            ppath = str(prim.GetPath())

            if prim.HasAuthoredReferences():
                refs = prim.GetMetadata("references")
                if refs:
                    for ref in refs.GetAddedOrExplicitItems():
                        check_path("reference", getattr(ref, "assetPath", ""), ppath)

            if prim.HasAuthoredPayloads():
                pls = prim.GetMetadata("payloads")
                if pls:
                    for pl in pls.GetAddedOrExplicitItems():
                        check_path("payload", getattr(pl, "assetPath", ""), ppath)

            for attr in prim.GetAttributes():
                t = attr.GetTypeName()
                if t not in (Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.AssetArray):
                    continue
                try:
                    v = attr.Get()
                except Exception:
                    continue

                if isinstance(v, Sdf.AssetPath):
                    raw = (v.resolvedPath or v.path or "").strip()
                    check_path("attr", raw, f"{ppath}.{attr.GetName()}")
                elif isinstance(v, (list, tuple)):
                    for av in v:
                        if isinstance(av, Sdf.AssetPath):
                            raw = (av.resolvedPath or av.path or "").strip()
                            check_path("attr", raw, f"{ppath}.{attr.GetName()}")

        checked += 1

    return {
        "scenes_root": scenes_root,
        "layout_usd_count": len(layout_usds),
        "checked": checked,
        "missing_count": len(missing),
        "missing": missing,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--subset-root", required=True, help="Path to GRScenes-test1 (contains GRScenes_assets/ and GRScenes100/)")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--validate", action="store_true")
    ap.add_argument("--report", default=None)
    ap.add_argument("--max-usd-files", type=int, default=None, help="Limit USD rewrite scan for quick testing")
    ap.add_argument("--max-scenes", type=int, default=None, help="Limit scene validation for quick testing")
    args = ap.parse_args(argv)

    subset_root = _norm_abs(args.subset_root)
    assets_root = os.path.join(subset_root, "GRScenes_assets")
    scenes_root = os.path.join(subset_root, "GRScenes100")
    if not os.path.isdir(assets_root):
        raise SystemExit(f"Missing GRScenes_assets under: {subset_root}")
    if not os.path.isdir(scenes_root):
        raise SystemExit(f"Missing GRScenes100 under: {subset_root}")

    mode_count = sum(bool(x) for x in (args.dry_run, args.apply, args.validate))
    if mode_count != 1:
        raise SystemExit("Choose exactly one mode: --dry-run OR --apply OR --validate")

    plans = compute_move_plans(subset_root)

    report: Dict[str, Any] = {
        "generated_at": _dt.datetime.now(tz=_dt.timezone.utc).isoformat(),
        "subset_root": subset_root,
        "category_merges": CATEGORY_MERGES,
        "plans": [
            {
                "old": p.old_category,
                "new": p.new_category,
                "move_count": len(p.moves),
                "uid_collisions": p.uid_collisions,
            }
            for p in plans
        ],
    }

    # Abort if collisions exist (safer default).
    collisions_total = sum(len(p.uid_collisions) for p in plans)
    report["uid_collisions_total"] = collisions_total
    if collisions_total and args.apply:
        raise SystemExit(f"UID collisions detected ({collisions_total}). Resolve before apply.")

    if args.dry_run:
        report["move_preview"] = apply_moves(plans, dry_run=True)
        # USD rewrite preview (no save)
        usd_targets = list(_iter_usd_files(os.path.join(subset_root, "GRScenes_assets"))) + list(
            _iter_usd_files(os.path.join(subset_root, "GRScenes100"))
        )
        usd_targets.sort()
        if args.max_usd_files is not None:
            usd_targets = usd_targets[: args.max_usd_files]

        changes = []
        for p in usd_targets:
            stats = rewrite_usd_paths_in_stage(p, save=False)
            if any(stats.values()):
                changes.append({"usd": p, **stats})
        report["usd_rewrite_preview_count"] = len(changes)
        report["usd_rewrite_preview"] = changes

    elif args.apply:
        report["moves"] = apply_moves(plans, dry_run=False)
        report["asset_annotation_index"] = _update_asset_annotation_index(subset_root)

        usd_targets = list(_iter_usd_files(os.path.join(subset_root, "GRScenes_assets"))) + list(
            _iter_usd_files(os.path.join(subset_root, "GRScenes100"))
        )
        usd_targets.sort()
        if args.max_usd_files is not None:
            usd_targets = usd_targets[: args.max_usd_files]

        rewritten = []
        for p in usd_targets:
            stats = rewrite_usd_paths_in_stage(p, save=True)
            if any(stats.values()):
                rewritten.append({"usd": p, **stats})
        report["usd_rewritten_count"] = len(rewritten)
        report["usd_rewritten"] = rewritten

    elif args.validate:
        report["validate"] = validate_scenes(subset_root, max_scenes=args.max_scenes)

    if args.report:
        os.makedirs(os.path.dirname(args.report) or ".", exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print("Wrote report:", args.report)

    # Print a short summary.
    print("uid_collisions_total:", report.get("uid_collisions_total"))
    if args.dry_run:
        print("dry_run move_count:", report.get("move_preview", {}).get("moved_count"))
        print("usd_rewrite_preview_count:", report.get("usd_rewrite_preview_count"))
    if args.apply:
        print("apply moved_count:", report.get("moves", {}).get("moved_count"))
        print("usd_rewritten_count:", report.get("usd_rewritten_count"))
    if args.validate:
        v = report.get("validate", {})
        print("validated scenes:", v.get("checked"), "/", v.get("layout_usd_count"), "missing:", v.get("missing_count"))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
