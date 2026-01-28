#!/usr/bin/env python3
"""Find specific asset USDs in a large asset-mesh-dedup report.

Why this exists
- The *.summary.json only includes "top N" duplicate groups, so most assets won't
  be searchable there.
- The full report JSON can be hundreds of MB; this tool loads it once and then
  answers: "Is this asset present?", "What mesh_count/signature did it get?",
  and "Which duplicate group (if any) does it belong to?".

Usage
  python3 scripts/find_assets_in_dedup_report.py \
    --report check_reports/test1_asset_mesh_dedup_geom_only.json \
    --usd \
      /abs/path/.../GRScenes-test1/GRScenes_assets/<cat>/<uid>/usd/<uid>.usd \
      GRScenes-test1/GRScenes_assets/<cat>/<uid>/usd/<uid>.usd

Notes
- The report typically stores paths as workspace-relative paths like
  "GRScenes-test1/...". This script normalizes absolute inputs.
"""

from __future__ import annotations

import argparse
import json
import os
from typing import Any, Dict, Iterable, List, Optional, Set, Tuple


def _normalize_usd_path(p: str) -> str:
    """Normalize user input into the form used in the report.

    The report often stores paths as relative strings starting at GRScenes-test1/...
    but users frequently paste absolute /cpfs/... paths.
    """
    p = str(p).strip().replace("\\", "/")
    if not p:
        return p

    # If absolute, try to strip everything before the dataset folder.
    # This is intentionally simple and robust.
    marker = "/GRScenes-test1/"
    if marker in p:
        # Keep everything from GRScenes-test1/... onward.
        return "GRScenes-test1/" + p.split(marker, 1)[1]

    # Also handle the common case where the path is already relative.
    if p.startswith("GRScenes-test1/"):
        return p

    return p


def _pick_asset_sig_key(report_mode: Optional[str]) -> str:
    """Map report mode to the asset signature key used in assets[]."""
    mode = (report_mode or "").strip()
    if mode == "geom_only":
        return "asset_geom_sig"
    if mode == "scale_only":
        return "asset_scale_sig"
    if mode == "full_matrix":
        return "asset_full_matrix_sig"
    # Fallback; geom is the least surprising.
    return "asset_geom_sig"


def _iter_targets(argv_usds: Iterable[str]) -> List[str]:
    out: List[str] = []
    for raw in argv_usds:
        n = _normalize_usd_path(raw)
        if n and n not in out:
            out.append(n)
    return out


def _load_report(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def _find_in_assets(report: Dict[str, Any], targets: Set[str]) -> Dict[str, Dict[str, Any]]:
    out: Dict[str, Dict[str, Any]] = {}
    assets = report.get("assets", []) or []

    for a in assets:
        p = str(a.get("usd_path", ""))
        if p in targets:
            out[p] = {
                "usd_path": p,
                "category": a.get("category"),
                "uid": a.get("uid"),
                "mesh_count": a.get("mesh_count"),
                "asset_geom_sig": a.get("asset_geom_sig"),
                "asset_scale_sig": a.get("asset_scale_sig"),
                "asset_full_matrix_sig": a.get("asset_full_matrix_sig"),
            }
            if len(out) == len(targets):
                break

    return out


def _find_in_duplicates(
    report: Dict[str, Any],
    targets: Set[str],
) -> Tuple[Dict[str, Dict[str, Any]], Dict[str, Dict[str, Any]]]:
    """Return (per_target, sig_to_group_summary for groups that contain any target)."""

    per_target: Dict[str, Dict[str, Any]] = {}
    groups_by_sig: Dict[str, Dict[str, Any]] = {}

    dups = report.get("duplicates", []) or []
    remaining = set(targets)

    for g in dups:
        sig = str(g.get("sig", ""))
        paths = [str(x) for x in (g.get("usd_paths", []) or [])]

        hit = [p for p in paths if p in remaining]
        if not hit:
            continue

        groups_by_sig.setdefault(
            sig,
            {
                "sig": sig,
                "count": int(g.get("count") or len(paths)),
                "usd_paths": paths,
            },
        )

        for p in hit:
            per_target[p] = {
                "sig": sig,
                "count": int(g.get("count") or len(paths)),
            }
            remaining.discard(p)

        if not remaining:
            break

    return per_target, groups_by_sig


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", required=True, help="Full report JSON (not the .summary.json)")
    ap.add_argument("--usd", nargs="+", required=True, help="One or more asset usd paths")
    ap.add_argument("--show-members", type=int, default=10, help="Show up to N members in group")
    args = ap.parse_args()

    report_path = args.report
    targets_list = _iter_targets(args.usd)
    targets = set(targets_list)

    if not targets_list:
        print("No valid --usd provided")
        return 2

    report = _load_report(report_path)
    meta = report.get("meta", {}) or {}
    mode = meta.get("mode")
    sig_key = _pick_asset_sig_key(mode)

    assets_hit = _find_in_assets(report, targets)
    dups_hit, groups = _find_in_duplicates(report, targets)

    print(f"report={report_path}")
    print(f"mode={mode} asset_sig_key={sig_key}")
    print()

    for t in targets_list:
        print(f"TARGET: {t}")
        a = assets_hit.get(t)
        if not a:
            print("  in_assets: NO")
        else:
            print("  in_assets: YES")
            print(f"  category={a.get('category')} uid={a.get('uid')} mesh_count={a.get('mesh_count')}")
            print(f"  asset_geom_sig={a.get('asset_geom_sig')}")
            print(f"  asset_scale_sig={a.get('asset_scale_sig')}")
            print(f"  asset_full_matrix_sig={a.get('asset_full_matrix_sig')}")

        d = dups_hit.get(t)
        if not d:
            print("  in_duplicates: NO (unique or not in duplicates list)")
        else:
            sig = d["sig"]
            print(f"  in_duplicates: YES sig={sig} group_count={d['count']}")
            grp = groups.get(sig)
            if grp:
                members = grp.get("usd_paths", [])
                n = int(args.show_members)
                preview = members[: max(0, n)]
                print(f"  group_members_preview({len(preview)}/{len(members)}):")
                for m in preview:
                    print(f"    - {m}")
        print()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
