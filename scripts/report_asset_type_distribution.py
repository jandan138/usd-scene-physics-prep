#!/usr/bin/env python3
"""Report normalized asset distributions by asset_type and category.

This script scans GRScenes normalized asset annotations and writes two JSON
reports under an output directory:
  - rigid_category_distribution.json
  - articulation_category_distribution.json

Each report contains:
  {
    "asset_type": "...",
    "total_assets": N,
    "category_counts": {
      "category_a": 12,
      "category_b": 34
    }
  }
"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, Tuple


SUPPORTED_ASSET_TYPES = ("rigid", "articulation")


def _iter_annotation_files(assets_root: Path) -> Iterable[Path]:
    for category_dir in sorted(assets_root.iterdir()):
        if not category_dir.is_dir():
            continue
        for uid_dir in sorted(category_dir.iterdir()):
            if not uid_dir.is_dir():
                continue
            annotation_path = uid_dir / f"{uid_dir.name}_annotation.json"
            if annotation_path.is_file():
                yield annotation_path


def _load_annotation(path: Path) -> Dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def _collect_distribution(
    assets_root: Path,
) -> Tuple[Dict[str, Counter], Counter, int, int]:
    per_type_category_counts = {asset_type: Counter() for asset_type in SUPPORTED_ASSET_TYPES}
    unexpected_asset_types = Counter()
    scanned_annotations = 0
    missing_annotations = 0

    for category_dir in sorted(assets_root.iterdir()):
        if not category_dir.is_dir():
            continue

        for uid_dir in sorted(category_dir.iterdir()):
            if not uid_dir.is_dir():
                continue

            annotation_path = uid_dir / f"{uid_dir.name}_annotation.json"
            if not annotation_path.is_file():
                missing_annotations += 1
                continue

            scanned_annotations += 1
            data = _load_annotation(annotation_path)

            asset_type = data.get("asset_type")
            category = data.get("category") or category_dir.name
            if asset_type not in SUPPORTED_ASSET_TYPES:
                unexpected_asset_types[str(asset_type)] += 1
                continue

            per_type_category_counts[asset_type][category] += 1

    return per_type_category_counts, unexpected_asset_types, scanned_annotations, missing_annotations


def _build_report(asset_type: str, counts: Counter) -> Dict:
    sorted_counts = dict(sorted(counts.items()))
    total_assets = sum(sorted_counts.values())
    return {
        "asset_type": asset_type,
        "total_assets": total_assets,
        "category_counts": sorted_counts,
    }


def _write_report(report: Dict, path: Path) -> None:
    with path.open("w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
        f.write("\n")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate rigid/articulation category distribution JSON reports."
    )
    parser.add_argument(
        "--assets-root",
        default="GRScenes-test1-normalized/GRScenes_assets",
        help="Path to normalized GRScenes_assets directory.",
    )
    parser.add_argument(
        "--out-dir",
        default="check_reports",
        help="Directory to write JSON reports into.",
    )
    args = parser.parse_args()

    assets_root = Path(args.assets_root).resolve()
    out_dir = Path(args.out_dir).resolve()

    if not assets_root.is_dir():
        print(f"Error: assets root does not exist: {assets_root}", file=sys.stderr)
        return 1

    per_type_counts, unexpected_types, scanned_annotations, missing_annotations = _collect_distribution(
        assets_root
    )

    out_dir.mkdir(parents=True, exist_ok=True)

    outputs = {
        "rigid": out_dir / "rigid_category_distribution.json",
        "articulation": out_dir / "articulation_category_distribution.json",
    }

    for asset_type, output_path in outputs.items():
        report = _build_report(asset_type, per_type_counts[asset_type])
        _write_report(report, output_path)
        print(
            f"[written] asset_type={asset_type} total_assets={report['total_assets']} "
            f"categories={len(report['category_counts'])} path={output_path}"
        )

    print(f"[summary] scanned_annotations={scanned_annotations} missing_annotations={missing_annotations}")

    if unexpected_types:
        print(f"[warning] unexpected asset_type values skipped: {dict(unexpected_types)}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
