#!/usr/bin/env python3
"""Summarize dedup reports across all categories.

Reads per-category JSON reports produced by report_asset_mesh_dedup.py,
aggregates statistics, and outputs a sorted CSV + summary JSON.

Usage:
    python scripts/summarize_dedup_reports.py \
        --reports-dir check_reports/dedup_tolerance \
        --out-csv check_reports/dedup_tolerance/summary.csv \
        --out-json check_reports/dedup_tolerance/summary.json
"""

import argparse
import csv
import json
import os
import sys
from pathlib import Path


def find_reports(reports_dir: str) -> list[tuple[str, str]]:
    """Return list of (category, report_path) sorted by category."""
    results = []
    for entry in sorted(os.scandir(reports_dir), key=lambda e: e.name):
        if not entry.is_dir():
            continue
        cat = entry.name
        # Look for *_geom_only.json
        for f in os.scandir(entry.path):
            if f.name.endswith("_geom_only.json") and f.is_file():
                results.append((cat, f.path))
                break
    return results


def parse_report(path: str) -> dict:
    """Extract key stats from a dedup report JSON."""
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    meta = data.get("meta", {})
    duplicates = data.get("duplicates", [])
    assets = data.get("assets", [])
    errors = data.get("errors", [])

    total_assets = len(assets) + len(errors)
    num_groups = len(duplicates)
    assets_in_groups = sum(g.get("count", 0) for g in duplicates)
    # Saveable = assets in groups minus one per group (the canonical)
    saveable = assets_in_groups - num_groups if num_groups > 0 else 0
    dedup_rate = assets_in_groups / total_assets * 100 if total_assets > 0 else 0.0

    # Largest group
    max_group_size = max((g.get("count", 0) for g in duplicates), default=0)

    return {
        "total_assets": total_assets,
        "num_groups": num_groups,
        "assets_in_groups": assets_in_groups,
        "saveable": saveable,
        "dedup_rate": round(dedup_rate, 1),
        "max_group_size": max_group_size,
        "num_errors": len(errors),
        "merge_tolerance": meta.get("merge_tolerance", 0),
        "float_eps": meta.get("float_quantize_eps", 0),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Summarize dedup reports")
    parser.add_argument(
        "--reports-dir", required=True,
        help="Root directory containing per-category report subdirectories",
    )
    parser.add_argument("--out-csv", default=None, help="Output CSV path")
    parser.add_argument("--out-json", default=None, help="Output JSON path")
    args = parser.parse_args()

    reports = find_reports(args.reports_dir)
    if not reports:
        print(f"No reports found in {args.reports_dir}", file=sys.stderr)
        return 1

    rows = []
    totals = {
        "total_assets": 0, "num_groups": 0, "assets_in_groups": 0,
        "saveable": 0, "num_errors": 0,
    }

    for cat, path in reports:
        stats = parse_report(path)
        row = {"category": cat, **stats}
        rows.append(row)
        for k in totals:
            totals[k] += stats[k]

    # Sort by saveable descending (highest ROI first)
    rows.sort(key=lambda r: r["saveable"], reverse=True)

    # Print table
    print(f"\n{'Category':<25} {'Total':>7} {'Groups':>7} {'InGrp':>7} {'Save':>7} {'Rate':>7} {'MaxGrp':>7}")
    print("-" * 75)
    for r in rows:
        print(
            f"{r['category']:<25} {r['total_assets']:>7} {r['num_groups']:>7} "
            f"{r['assets_in_groups']:>7} {r['saveable']:>7} {r['dedup_rate']:>6.1f}% {r['max_group_size']:>7}"
        )
    print("-" * 75)
    overall_rate = (
        totals["assets_in_groups"] / totals["total_assets"] * 100
        if totals["total_assets"] > 0 else 0
    )
    print(
        f"{'TOTAL':<25} {totals['total_assets']:>7} {totals['num_groups']:>7} "
        f"{totals['assets_in_groups']:>7} {totals['saveable']:>7} {overall_rate:>6.1f}%"
    )

    # Write CSV
    out_csv = args.out_csv or os.path.join(args.reports_dir, "summary.csv")
    os.makedirs(os.path.dirname(out_csv), exist_ok=True)
    fields = [
        "category", "total_assets", "num_groups", "assets_in_groups",
        "saveable", "dedup_rate", "max_group_size", "num_errors",
    ]
    with open(out_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    print(f"\nCSV written: {out_csv}")

    # Write JSON
    out_json = args.out_json or os.path.join(args.reports_dir, "summary.json")
    summary = {
        "total_categories": len(rows),
        "total_assets": totals["total_assets"],
        "total_groups": totals["num_groups"],
        "total_assets_in_groups": totals["assets_in_groups"],
        "total_saveable": totals["saveable"],
        "overall_dedup_rate": round(overall_rate, 1),
        "categories": rows,
    }
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    print(f"JSON written: {out_json}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
