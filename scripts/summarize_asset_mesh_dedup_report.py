#!/usr/bin/env python3
"""Summarize a large asset mesh dedup report JSON.

The raw reports are intentionally verbose (include per-asset + per-mesh info)
which makes them hard to inspect directly. This tool extracts a compact summary
and optionally writes out the top duplicate groups.

Usage:
  python3 scripts/summarize_asset_mesh_dedup_report.py \
    --input check_reports/test1_asset_mesh_dedup_geom_only.json \
    --out check_reports/test1_asset_mesh_dedup_geom_only.summary.json \
    --top 50
"""

from __future__ import annotations

import argparse
import json
import os
from collections import Counter
from typing import Any, Dict, List


def _safe_int(x: Any, default: int = 0) -> int:
    try:
        return int(x)
    except Exception:
        return default


def _summarize(report: Dict[str, Any], *, top: int) -> Dict[str, Any]:
    meta = report.get("meta", {}) or {}
    mode = meta.get("mode")

    dups: List[Dict[str, Any]] = report.get("duplicates", []) or []

    group_sizes = [_safe_int(g.get("count"), 0) for g in dups]
    group_sizes = [s for s in group_sizes if s > 1]

    size_hist = Counter(group_sizes)
    # How many asset USDs are involved in *any* duplicate group
    assets_in_dups = sum(group_sizes)

    max_group = max(group_sizes) if group_sizes else 0
    top_groups = dups[: max(0, int(top))]

    # Count unique categories present in duplicates (best-effort parse)
    categories = Counter()
    for g in top_groups:
        for p in g.get("usd_paths", []) or []:
            parts = str(p).replace("\\", "/").split("/")
            # .../GRScenes_assets/<category>/<uid>/usd/<uid>.usd
            if "GRScenes_assets" in parts:
                i = parts.index("GRScenes_assets")
                if i + 1 < len(parts):
                    categories[parts[i + 1]] += 1

    return {
        "meta": {
            "dataset": meta.get("dataset"),
            "mode": mode,
            "assets_root": meta.get("assets_root"),
            "asset_usd_count": meta.get("asset_usd_count"),
            "error_count": meta.get("error_count"),
            "duplicate_group_count": meta.get("duplicate_group_count"),
            "elapsed_sec": meta.get("elapsed_sec"),
            "float_quantize_eps": meta.get("float_quantize_eps"),
        },
        "duplicates_summary": {
            "groups": len(group_sizes),
            "assets_in_duplicate_groups": assets_in_dups,
            "max_group_size": max_group,
            "group_size_histogram": {
                str(k): size_hist[k] for k in sorted(size_hist.keys())
            },
            "top_groups": top_groups,
            "top_group_categories_hint": [
                {"category": c, "paths_in_top": n} for c, n in categories.most_common(20)
            ],
        },
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--out", default=None)
    ap.add_argument("--top", type=int, default=50)
    args = ap.parse_args()

    in_path = args.input
    out_path = args.out
    if not out_path:
        out_path = in_path + ".summary.json"

    with open(in_path, "r", encoding="utf-8") as f:
        report = json.load(f)

    summary = _summarize(report, top=args.top)

    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)

    print(out_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
