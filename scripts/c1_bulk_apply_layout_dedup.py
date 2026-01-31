#!/usr/bin/env python3
"""Apply a large old->canonical mapping to all layout.usd files (Step 3B at scale).

This writes per-layout reports and outputs a new layout USD next to the original.
It does NOT promote to layout.usd and does NOT delete assets.

Typical usage
  python scripts/c1_bulk_apply_layout_dedup.py \
    --dataset-root GRScenes-test1 \
    --mapping-json check_reports/c1_bulk/plant_geom_only_mapping.json \
    --group-label c1_plant_bulk \
    --out-name layout.c1_plant_bulk_dedup_v1.usd \
    --report-dir check_reports/c1_bulk/plant_batch \
    --set-instanceable

Then manually spotcheck, and finally run a Step6/bulk promote+scan+delete.
"""

from __future__ import annotations

import argparse
import json
import os
import importlib.util
import sys
from pathlib import Path
from typing import Dict, List

def _load_rewriter_module():
    scripts_dir = Path(__file__).resolve().parent
    mod_path = scripts_dir / "rewrite_layout_asset_refs_with_compensation.py"
    spec = importlib.util.spec_from_file_location("rewrite_layout_asset_refs_with_compensation", mod_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load module spec from: {mod_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset-root", default="GRScenes-test1")
    ap.add_argument("--layout-root", default=None, help="If set, scan layouts under this dir (default: <dataset>/GRScenes100)")
    ap.add_argument("--mapping-json", required=True)
    ap.add_argument("--group-label", required=True, help="Label used in report filenames")
    ap.add_argument("--out-name", required=True, help="Output filename written next to layout.usd")
    ap.add_argument(
        "--scene-files",
        default="layout.usd,start_result_interaction.usd,start_result_navigation.usd",
        help="Comma-separated scene USD filenames to rewrite in each scene folder (outputs use the out-name suffix)",
    )
    ap.add_argument("--report-dir", required=True)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--set-instanceable", action="store_true")
    ap.add_argument("--no-compensation", action="store_true")
    ap.add_argument("--max-preview", type=int, default=0)
    args = ap.parse_args()

    rw = _load_rewriter_module()

    dataset_root = Path(args.dataset_root)
    layout_root = Path(args.layout_root) if args.layout_root else (dataset_root / "GRScenes100")
    report_dir = Path(args.report_dir)
    report_dir.mkdir(parents=True, exist_ok=True)

    layouts = sorted(layout_root.glob("**/layout.usd"))
    print(f"layouts={len(layouts)} under {layout_root}")

    scene_files = [s.strip() for s in str(args.scene_files).split(",") if s.strip()]
    if "layout.usd" not in scene_files:
        scene_files = ["layout.usd"] + scene_files

    # Derive suffix from out-name, e.g. layout.c1_x.usd -> .c1_x
    out_bn = Path(args.out_name).name
    if not out_bn.endswith(".usd") or not out_bn.startswith("layout"):
        raise SystemExit(f"--out-name must look like 'layout.<suffix>.usd' (got: {args.out_name})")
    out_stem = out_bn[: -len(".usd")]
    out_suffix = out_stem[len("layout") :]
    if not out_suffix:
        raise SystemExit(f"--out-name must include a suffix after 'layout' (got: {args.out_name})")

    summaries: List[Dict] = []
    changed_layouts = 0

    mapping_pairs = rw._load_mapping(args.mapping_json, str(dataset_root))

    for i, layout in enumerate(layouts, start=1):
        scene_dir = layout.parent
        layout_changed = False

        for scene_fn in scene_files:
            src = scene_dir / scene_fn
            if not src.exists():
                continue

            if scene_fn == "layout.usd":
                out_path = scene_dir / args.out_name
            else:
                base = scene_fn[: -len(".usd")] if scene_fn.endswith(".usd") else scene_fn
                out_path = scene_dir / f"{base}{out_suffix}.usd"

            report_out = str(
                report_dir
                / f"{scene_dir.name}_{args.group_label}_{src.stem}_{'dryrun' if args.dry_run else 'apply'}.json"
            )

            summary = rw.rewrite_layout(
                layout_usd=str(src),
                out_usd=str(out_path),
                subset_root=str(dataset_root),
                mapping_pairs=mapping_pairs,
                apply_compensation=not args.no_compensation,
                set_instanceable=bool(args.set_instanceable),
                dry_run=bool(args.dry_run),
                report_out=report_out,
                max_preview=int(args.max_preview),
            )
            summary["scene_file"] = scene_fn

            did_change = any(
                (summary.get("counts") or {}).get(k, 0)
                for k in ("refs_changed", "payloads_changed", "asset_attrs_changed")
            )
            if scene_fn == "layout.usd" and did_change:
                layout_changed = True

            summaries.append(summary)

        if layout_changed:
            changed_layouts += 1

        if i % 5 == 0 or i == len(layouts):
            print(f"  processed {i}/{len(layouts)} changed_layouts={changed_layouts}")

    batch_summary = {
        "dataset_root": str(dataset_root),
        "layout_root": str(layout_root),
        "mapping_json": args.mapping_json,
        "out_name": args.out_name,
        "scene_files": scene_files,
        "dry_run": bool(args.dry_run),
        "changed_layouts": changed_layouts,
        "layouts_total": len(layouts),
        "per_layout": summaries,
    }

    (report_dir / "batch_summary.json").write_text(json.dumps(batch_summary, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"WROTE {report_dir / 'batch_summary.json'}")

    # Simple spotcheck list
    ranked = []
    for s in summaries:
        c = s.get("counts") or {}
        score = int(c.get("refs_changed", 0)) + int(c.get("payloads_changed", 0)) + int(c.get("asset_attrs_changed", 0))
        ranked.append((score, int(c.get("xform_compensated", 0)), s.get("layout_in"), s.get("layout_out")))
    ranked.sort(key=lambda x: (-x[0], -x[1], str(x[2] or "")))

    md_lines = [
        f"# Spotcheck list: {args.group_label}",
        "",
        f"- layouts_total: {len(layouts)}",
        f"- changed_layouts: {changed_layouts}",
        f"- mapping_json: {args.mapping_json}",
        f"- out_name: {args.out_name}",
        f"- dry_run: {bool(args.dry_run)}",
        "",
        "## Top changed layouts",
        "",
    ]
    top_n = 15
    for score, xfc, lin, lout in ranked[:top_n]:
        if score <= 0:
            continue
        md_lines.append(f"- score={score} xform_compensated={xfc} in={lin} out={lout}")
    (report_dir / "spotcheck_list.md").write_text("\n".join(md_lines) + "\n", encoding="utf-8")
    print(f"WROTE {report_dir / 'spotcheck_list.md'}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
