#!/usr/bin/env python3
"""Merge per-category geom_only dedup reports into a single combined report.

Usage:
    python scripts/merge_dedup_reports.py \
        --reports-dir check_reports/normalized_dedup_tolerance \
        --output check_reports/normalized_dedup_tolerance/merged_geom_only.json
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser(description="Merge per-category dedup reports")
    ap.add_argument("--reports-dir", required=True, help="Directory with per-category subdirs")
    ap.add_argument("--output", required=True, help="Output merged JSON path")
    args = ap.parse_args()

    reports_dir = Path(args.reports_dir)
    all_duplicates = []
    all_assets = []
    all_errors = []
    sources = []

    for subdir in sorted(reports_dir.iterdir()):
        if not subdir.is_dir():
            continue
        report_file = subdir / f"{subdir.name}_asset_mesh_dedup_geom_only.json"
        if not report_file.exists():
            continue

        data = json.loads(report_file.read_text(encoding="utf-8"))
        all_duplicates.extend(data.get("duplicates", []))
        all_assets.extend(data.get("assets", []))
        all_errors.extend(data.get("errors", []))
        sources.append(str(report_file))

    merged = {
        "meta": {
            "merged_from": sources,
            "categories": len(sources),
            "total_duplicate_groups": len(all_duplicates),
            "total_assets": len(all_assets),
            "total_errors": len(all_errors),
        },
        "duplicates": all_duplicates,
        "assets": all_assets,
        "errors": all_errors,
    }

    out = Path(args.output)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(merged, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Merged {len(sources)} categories -> {len(all_duplicates)} dup groups, {len(all_assets)} assets -> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
