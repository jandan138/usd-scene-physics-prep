#!/usr/bin/env python3
"""Repackage GRScenes smoke subset assets into normalize-friendly layout.

This helper only mutates an experimental subset root. It moves each main asset
USD from:

    GRScenes_assets/<category>/<uid>/<uid>.usd

to:

    GRScenes_assets/<category>/<uid>/usd/<uid>.usd

and rewrites USD references/payloads/asset-valued USD attributes inside the
subset so they point at the new asset path.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
from typing import Dict, Iterable, List, Tuple

from pxr import Sdf, Usd


def _norm_rel(path: str) -> str:
    return path.replace("\\", "/")


def _iter_asset_moves(assets_root: str) -> Iterable[Tuple[str, str]]:
    for category in sorted(os.listdir(assets_root)):
        cat_dir = os.path.join(assets_root, category)
        if not os.path.isdir(cat_dir):
            continue
        for uid in sorted(os.listdir(cat_dir)):
            uid_dir = os.path.join(cat_dir, uid)
            if not os.path.isdir(uid_dir):
                continue
            src = os.path.join(uid_dir, f"{uid}.usd")
            dst = os.path.join(uid_dir, "usd", f"{uid}.usd")
            if os.path.isfile(src):
                yield src, dst


def _rewrite_asset_path(asset_path: str, replacements: Dict[str, str]) -> str:
    path = _norm_rel(asset_path)
    for old, new in replacements.items():
        if old in path:
            return path.replace(old, new)
    return asset_path


def _rewrite_asset_value(value, replacements: Dict[str, str]):
    if isinstance(value, Sdf.AssetPath):
        new_path = _rewrite_asset_path(value.path, replacements)
        if new_path != value.path:
            return Sdf.AssetPath(new_path)
        return value
    if isinstance(value, (list, tuple)):
        changed = False
        out = []
        for item in value:
            if isinstance(item, Sdf.AssetPath):
                new_item = Sdf.AssetPath(_rewrite_asset_path(item.path, replacements))
                changed = changed or (new_item.path != item.path)
                out.append(new_item)
            else:
                out.append(item)
        if changed:
            return out
    return value


def _rewrite_stage(usd_path: str, replacements: Dict[str, str]) -> Dict[str, int]:
    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise RuntimeError(f"failed_to_open_stage: {usd_path}")

    ref_updates = 0
    payload_updates = 0
    attr_updates = 0

    for prim in stage.Traverse():
        refs = prim.GetMetadata("references")
        if refs:
            items = list(refs.GetAddedOrExplicitItems())
            new_items = []
            changed = False
            for item in items:
                new_asset_path = _rewrite_asset_path(item.assetPath, replacements)
                changed = changed or (new_asset_path != item.assetPath)
                new_items.append(
                    Sdf.Reference(
                        new_asset_path,
                        item.primPath,
                        item.layerOffset,
                        item.customData,
                    )
                )
            if changed:
                prim.SetMetadata("references", Sdf.ReferenceListOp.CreateExplicit(new_items))
                ref_updates += 1

        payloads = prim.GetMetadata("payloads")
        if payloads:
            items = list(payloads.GetAddedOrExplicitItems())
            new_items = []
            changed = False
            for item in items:
                new_asset_path = _rewrite_asset_path(item.assetPath, replacements)
                changed = changed or (new_asset_path != item.assetPath)
                new_items.append(
                    Sdf.Payload(
                        new_asset_path,
                        item.primPath,
                        item.layerOffset,
                    )
                )
            if changed:
                prim.SetMetadata("payloads", Sdf.PayloadListOp.CreateExplicit(new_items))
                payload_updates += 1

        for attr in prim.GetAttributes():
            try:
                value = attr.Get()
            except Exception:
                continue
            new_value = _rewrite_asset_value(value, replacements)
            if new_value != value:
                attr.Set(new_value)
                attr_updates += 1

    stage.GetRootLayer().Save()
    return {
        "reference_updates": ref_updates,
        "payload_updates": payload_updates,
        "asset_attr_updates": attr_updates,
    }


def _iter_usd_files(root: str) -> Iterable[str]:
    for dirpath, _, filenames in os.walk(root):
        for filename in sorted(filenames):
            if filename.lower().endswith((".usd", ".usda", ".usdc")):
                yield os.path.join(dirpath, filename)


def main() -> int:
    parser = argparse.ArgumentParser(description="Rewrite smoke subset asset layout to include /usd/ directories.")
    parser.add_argument("--subset-root", required=True)
    parser.add_argument("--report-out", required=True)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()

    subset_root = os.path.abspath(args.subset_root)
    assets_root = os.path.join(subset_root, "GRScenes_assets")
    if not os.path.isdir(assets_root):
        raise SystemExit(f"assets root not found: {assets_root}")

    replacements: Dict[str, str] = {}
    moves: List[Dict[str, str]] = []
    for src, dst in _iter_asset_moves(assets_root):
        uid = os.path.splitext(os.path.basename(src))[0]
        category = os.path.basename(os.path.dirname(src))
        old_rel = f"GRScenes_assets/{category}/{uid}/{uid}.usd"
        new_rel = f"GRScenes_assets/{category}/{uid}/usd/{uid}.usd"
        replacements[old_rel] = new_rel
        moves.append({"src": src, "dst": dst, "old_rel": old_rel, "new_rel": new_rel})

    if not args.dry_run:
        for move in moves:
            os.makedirs(os.path.dirname(move["dst"]), exist_ok=True)
            if not os.path.exists(move["dst"]):
                shutil.move(move["src"], move["dst"])

    rewrite_stats: List[Dict[str, object]] = []
    for usd_path in _iter_usd_files(subset_root):
        stats = _rewrite_stage(usd_path, replacements) if not args.dry_run else {
            "reference_updates": 0,
            "payload_updates": 0,
            "asset_attr_updates": 0,
        }
        if any(stats.values()):
            rewrite_stats.append({"usd": usd_path, **stats})

    report = {
        "subset_root": subset_root,
        "moves_total": len(moves),
        "rewritten_usd_files": len(rewrite_stats),
        "moves": moves[:50],
        "rewrite_stats": rewrite_stats[:200],
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.report_out)), exist_ok=True)
    with open(args.report_out, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print(
        json.dumps(
            {
                "moves_total": len(moves),
                "rewritten_usd_files": len(rewrite_stats),
                "report_out": os.path.abspath(args.report_out),
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
