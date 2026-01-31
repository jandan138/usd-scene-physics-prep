#!/usr/bin/env python3
"""Build a large old->canonical mapping from the geom_only dedup report.

Goal
- Turn "duplicate groups" (sig) into a single mapping JSON that can be consumed by
  scripts/rewrite_layout_asset_refs_with_compensation.py at scale.

Canonical selection heuristic
- Prefer the asset USD that is most frequently referenced in layout.usd files.
- If none are referenced (rare), fall back to the first member.

Outputs
- mapping JSON: {old_asset_usd_rel: canonical_asset_usd_rel, ...}
- stats JSON: counts and canonical selections

Notes
- This does not rewrite any USDs; it only produces inputs for the C1 pipeline.
"""

from __future__ import annotations

import argparse
import json
import os
from collections import Counter
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import ijson  # type: ignore

try:
    from pxr import Usd  # type: ignore

    _PXR_ERR: Optional[Exception] = None
except Exception as e:  # pragma: no cover
    Usd = None  # type: ignore
    _PXR_ERR = e


def _norm_rel_asset_path(p: str) -> Optional[str]:
    p = (p or "").replace("\\", "/").strip()
    if p.startswith("@") and p.endswith("@"):  # Sdf.AssetPath string form
        p = p[1:-1]

    # Expect report-style: GRScenes-test1/GRScenes_assets/<cat>/<uid>/usd/<uid>.usd
    marker = "GRScenes-test1/GRScenes_assets/"
    if marker in p:
        return p[p.find(marker) :].lstrip("/")

    # Sometimes we see GRScenes_assets/... without dataset prefix
    marker2 = "GRScenes_assets/"
    if marker2 in p:
        return "GRScenes-test1/" + p[p.find(marker2) :].lstrip("/")

    return None


def _abs_from_layout_ref(base_dir: str, asset_path: str) -> str:
    s = (asset_path or "").replace("\\", "/").strip()
    if s.startswith("@") and s.endswith("@"):
        s = s[1:-1]
    if not s:
        return ""
    if os.path.isabs(s):
        return os.path.abspath(s)
    return os.path.abspath(os.path.join(base_dir, s))


def _abs_to_report_style(abs_path: str) -> Optional[str]:
    p = (abs_path or "").replace("\\", "/")
    marker = "/GRScenes-test1/GRScenes_assets/"
    idx = p.find(marker)
    if idx >= 0:
        return p[idx + 1 :]
    marker2 = "/GRScenes_assets/"
    idx2 = p.find(marker2)
    if idx2 >= 0:
        return "GRScenes-test1/" + p[idx2 + 1 :]
    return None


def _iter_duplicate_groups(report_path: Path) -> Iterable[Dict]:
    with open(report_path, "rb") as f:
        for item in ijson.items(f, "duplicates.item"):
            yield item


def _count_layout_asset_usage(dataset_root: Path) -> Counter:
    if _PXR_ERR is not None:
        raise RuntimeError(f"pxr.Usd not available: {_PXR_ERR}")

    layout_files = sorted((dataset_root / "GRScenes100").glob("**/layout.usd"))
    counts: Counter = Counter()

    for layout in layout_files:
        base_dir = str(layout.parent)
        stage = Usd.Stage.Open(str(layout), load=Usd.Stage.LoadNone)
        if stage is None:
            continue

        for prim in stage.Traverse():
            if not prim or not prim.IsValid():
                continue

            if prim.HasAuthoredReferences():
                refs = prim.GetMetadata("references")
                if refs:
                    try:
                        items = list(refs.GetAddedOrExplicitItems())
                    except Exception:
                        items = []
                    for r in items:
                        ap = getattr(r, "assetPath", "") or ""
                        rel = _abs_to_report_style(_abs_from_layout_ref(base_dir, ap))
                        if rel:
                            counts[rel] += 1

            if prim.HasAuthoredPayloads():
                pls = prim.GetMetadata("payloads")
                if pls:
                    try:
                        items = list(pls.GetAddedOrExplicitItems())
                    except Exception:
                        items = []
                    for r in items:
                        ap = getattr(r, "assetPath", "") or ""
                        rel = _abs_to_report_style(_abs_from_layout_ref(base_dir, ap))
                        if rel:
                            counts[rel] += 1

    return counts


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--report",
        default="check_reports/test1_asset_mesh_dedup_geom_only.json",
        help="Large geom_only dedup report JSON",
    )
    ap.add_argument(
        "--dataset-root",
        default="GRScenes-test1",
        help="Dataset root containing GRScenes_assets and GRScenes100",
    )
    ap.add_argument(
        "--category",
        default=None,
        help="If set, only build mapping for this category (e.g. plant).",
    )
    ap.add_argument(
        "--out-mapping-json",
        required=True,
        help="Output mapping JSON path (dict old->canonical, report-style rel paths)",
    )
    ap.add_argument(
        "--out-stats-json",
        required=True,
        help="Output stats JSON path",
    )
    ap.add_argument(
        "--progress-every",
        type=int,
        default=200,
        help="Print progress every N duplicate groups",
    )
    args = ap.parse_args()

    report = Path(args.report)
    dataset_root = Path(args.dataset_root)

    usage = _count_layout_asset_usage(dataset_root)

    mapping: Dict[str, str] = {}
    group_count = 0
    group_kept = 0
    assets_total = 0
    assets_kept = 0

    canonical_selections: List[Dict] = []

    for g in _iter_duplicate_groups(report):
        group_count += 1
        sig = str(g.get("sig") or "")
        paths = [p for p in (g.get("usd_paths") or []) if isinstance(p, str)]
        if not paths:
            continue

        # Filter category if requested
        if args.category:
            want = str(args.category)
            paths = [p for p in paths if f"GRScenes-test1/GRScenes_assets/{want}/" in p]
            if not paths:
                continue

        # Choose canonical by layout usage count
        scored = [(usage.get(p, 0), p) for p in paths]
        scored.sort(key=lambda x: (-x[0], x[1]))
        canonical = scored[0][1]

        group_kept += 1
        assets_total += len(paths)
        assets_kept += 1

        for p in paths:
            if p == canonical:
                continue
            mapping[p] = canonical

        canonical_selections.append(
            {
                "sig": sig,
                "member_count": len(paths),
                "canonical": canonical,
                "canonical_usage_in_layouts": int(usage.get(canonical, 0)),
            }
        )

        if args.progress_every and group_count % args.progress_every == 0:
            print(
                f"groups_seen={group_count} kept={group_kept} mapping_pairs={len(mapping)}",
                flush=True,
            )

    out_mapping = Path(args.out_mapping_json)
    out_stats = Path(args.out_stats_json)
    out_mapping.parent.mkdir(parents=True, exist_ok=True)
    out_stats.parent.mkdir(parents=True, exist_ok=True)

    out_mapping.write_text(json.dumps(mapping, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    stats = {
        "report": str(report),
        "dataset_root": str(dataset_root),
        "category": args.category,
        "groups_seen": group_count,
        "groups_included": group_kept,
        "assets_included_total": assets_total,
        "canonicals_kept": assets_kept,
        "mapping_pairs": len(mapping),
        "canonical_selections": canonical_selections,
    }
    out_stats.write_text(json.dumps(stats, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    print(
        f"DONE groups_included={group_kept} mapping_pairs={len(mapping)} out={out_mapping}",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
