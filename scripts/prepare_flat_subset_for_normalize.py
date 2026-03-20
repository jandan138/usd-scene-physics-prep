#!/usr/bin/env python3
"""Prepare a flat GRScenes subset for normalize_asset_transforms.py.

Some dataset snapshots store asset USDs as:
  GRScenes_assets/<category>/<uid>/<uid>.usd

But normalize_asset_transforms.py expects:
  GRScenes_assets/<category>/<uid>/usd/<uid>.usd

This script copies a subset root, reshapes asset directories into the expected
layout, then rewrites USD references in-place inside the copied subset so they
point to the new /usd/<uid>.usd paths.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import shutil
import sys
from pathlib import Path
from typing import Dict, List, Tuple


def _load_rewriter_module():
    scripts_dir = Path(__file__).resolve().parent
    mod_path = scripts_dir / "rewrite_layout_asset_refs_with_compensation.py"
    spec = importlib.util.spec_from_file_location(
        "rewrite_layout_asset_refs_with_compensation",
        mod_path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load module spec from: {mod_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _iter_usd_files(root: str) -> List[str]:
    out: List[str] = []
    for dirpath, _, filenames in os.walk(root):
        for name in filenames:
            if name.lower().endswith((".usd", ".usda", ".usdc")):
                out.append(os.path.join(dirpath, name))
    out.sort()
    return out


def _copy_root(src_root: str, dst_root: str) -> None:
    if os.path.exists(dst_root):
        raise RuntimeError(f"Destination already exists: {dst_root}")
    shutil.copytree(src_root, dst_root, symlinks=True)


def _reshape_assets(dst_root: str) -> Tuple[Dict[str, str], List[Dict[str, str]]]:
    assets_root = os.path.join(dst_root, "GRScenes_assets")
    mapping: Dict[str, str] = {}
    reshaped: List[Dict[str, str]] = []

    for category in sorted(os.listdir(assets_root)):
        cat_dir = os.path.join(assets_root, category)
        if not os.path.isdir(cat_dir):
            continue
        for uid in sorted(os.listdir(cat_dir)):
            uid_dir = os.path.join(cat_dir, uid)
            if not os.path.isdir(uid_dir):
                continue

            flat_usd = os.path.join(uid_dir, f"{uid}.usd")
            usd_dir = os.path.join(uid_dir, "usd")
            nested_usd = os.path.join(usd_dir, f"{uid}.usd")

            if os.path.isfile(nested_usd):
                continue
            if not os.path.isfile(flat_usd):
                continue

            os.makedirs(usd_dir, exist_ok=True)
            shutil.move(flat_usd, nested_usd)
            mapping[os.path.abspath(flat_usd)] = os.path.abspath(nested_usd)
            reshaped.append(
                {
                    "category": category,
                    "uid": uid,
                    "old": flat_usd,
                    "new": nested_usd,
                }
            )

    return mapping, reshaped


def _rewrite_usds(
    *,
    dst_root: str,
    mapping: Dict[str, str],
    report_dir: str,
) -> Dict[str, object]:
    rw = _load_rewriter_module()
    usd_files = _iter_usd_files(dst_root)
    mapping_pairs = [
        rw.MappingPair(old_abs=old_abs, canonical_abs=new_abs)
        for old_abs, new_abs in sorted(mapping.items())
    ]

    rewritten = []
    total_refs_changed = 0
    total_payloads_changed = 0
    total_asset_attrs_changed = 0

    for usd_path in usd_files:
        rel = os.path.relpath(usd_path, dst_root).replace(os.sep, "/")
        report_path = os.path.join(report_dir, rel.replace("/", "__") + ".json")
        summary = rw.rewrite_layout(
            layout_usd=usd_path,
            out_usd=None,
            subset_root=dst_root,
            mapping_pairs=mapping_pairs,
            apply_compensation=False,
            set_instanceable=False,
            dry_run=False,
            report_out=report_path,
            max_preview=0,
        )
        counts = summary.get("counts", {})
        changed = any(
            int(counts.get(key, 0))
            for key in ("refs_changed", "payloads_changed", "asset_attrs_changed")
        )
        if changed:
            rewritten.append(
                {
                    "usd": rel,
                    "counts": counts,
                }
            )
        total_refs_changed += int(counts.get("refs_changed", 0))
        total_payloads_changed += int(counts.get("payloads_changed", 0))
        total_asset_attrs_changed += int(counts.get("asset_attrs_changed", 0))

    return {
        "usd_files_scanned": len(usd_files),
        "usd_files_changed": len(rewritten),
        "total_refs_changed": total_refs_changed,
        "total_payloads_changed": total_payloads_changed,
        "total_asset_attrs_changed": total_asset_attrs_changed,
        "changed_usds": rewritten[:100],
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src-root", required=True, help="Flat subset root")
    ap.add_argument("--dst-root", required=True, help="Prepared subset root")
    ap.add_argument("--report-dir", required=True, help="Directory for JSON reports")
    args = ap.parse_args()

    src_root = os.path.abspath(args.src_root)
    dst_root = os.path.abspath(args.dst_root)
    report_dir = os.path.abspath(args.report_dir)

    os.makedirs(report_dir, exist_ok=True)

    print(f"Copying subset: {src_root} -> {dst_root}")
    _copy_root(src_root, dst_root)

    print("Reshaping assets into /usd/<uid>.usd layout")
    mapping, reshaped = _reshape_assets(dst_root)
    mapping_json = os.path.join(report_dir, "flat_to_nested_asset_mapping.json")
    with open(mapping_json, "w", encoding="utf-8") as f:
        json.dump(mapping, f, indent=2, ensure_ascii=False)

    print(f"Rewriting USD references using {len(mapping)} asset path mappings")
    rewrite_summary = _rewrite_usds(
        dst_root=dst_root,
        mapping=mapping,
        report_dir=os.path.join(report_dir, "rewrite_reports"),
    )

    summary = {
        "src_root": src_root,
        "dst_root": dst_root,
        "report_dir": report_dir,
        "assets_reshaped": len(reshaped),
        "reshaped_preview": reshaped[:50],
        "mapping_json": mapping_json,
        "rewrite_summary": rewrite_summary,
    }
    summary_path = os.path.join(report_dir, "prepare_flat_subset_summary.json")
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print(f"Wrote summary: {summary_path}")
    print(
        f"assets_reshaped={len(reshaped)} "
        f"usd_files_changed={rewrite_summary['usd_files_changed']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
