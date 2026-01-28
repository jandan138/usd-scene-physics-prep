#!/usr/bin/env python3
"""Query a large asset mesh dedup report JSON for specific assets.

Why this exists
- The full reports (e.g. check_reports/test1_asset_mesh_dedup_geom_only.json) can be ~300MB.
- VS Code extensions cannot open/sync those files reliably.
- The *.summary.json only keeps statistics + top N groups, so a random asset may not appear.

This script streams the big JSON and answers:
- Does a given asset USD appear in the report's `assets[]`?
- What is its mesh_count and asset signature(s)?
- In the report's `mode`, which duplicate group (sig) does it belong to and what is the group size?

Dependencies
- Requires `ijson` for streaming JSON parsing.
  Install: `python3 -m pip install ijson`

Examples
- Query two assets in the geom_only report:
  python3 scripts/query_asset_mesh_dedup_report.py \
    --report check_reports/test1_asset_mesh_dedup_geom_only.json \
    --usd /abs/path/.../GRScenes-test1/GRScenes_assets/.../usd/<uid>.usd \
    --usd GRScenes-test1/GRScenes_assets/.../usd/<uid>.usd

- Limit group members printed:
  python3 scripts/query_asset_mesh_dedup_report.py --report ... --usd ... --max-paths 30
"""

from __future__ import annotations

import argparse
import os
import sys
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Set, Tuple


@dataclass
class FoundAsset:
    usd_path: str
    category: Optional[str]
    uid: Optional[str]
    mesh_count: int
    asset_geom_sig: str
    asset_scale_sig: str
    asset_full_matrix_sig: str


def _normalize_query_path(p: str, *, repo_root: str) -> str:
    # Reports store `usd_path` as whatever path was yielded during scanning.
    # In our runs, that's typically a repo-relative path like:
    #   GRScenes-test1/GRScenes_assets/<category>/<uid>/usd/<uid>.usd
    # Users may paste an absolute path; normalize to match report style.
    p = str(p).replace("\\", "/").strip()

    # If it is inside this repo, make it repo-relative.
    if os.path.isabs(p):
        repo_root_abs = os.path.abspath(repo_root)
        p_abs = os.path.abspath(p)
        if p_abs.startswith(repo_root_abs + os.sep):
            rel = os.path.relpath(p_abs, repo_root_abs)
            return rel.replace("\\", "/")

    # If it contains a dataset-relative segment, cut to that.
    marker = "GRScenes-test1/GRScenes_assets/"
    idx = p.find(marker)
    if idx >= 0:
        return p[idx:].lstrip("/")

    return p.lstrip("/")


def _mode_sig_key(mode: str) -> Tuple[str, str]:
    # (asset dict key, human label)
    if mode == "geom_only":
        return "asset_geom_sig", "asset_geom_sig (geom_only)"
    if mode == "scale_only":
        return "asset_scale_sig", "asset_scale_sig (scale_only)"
    if mode == "full_matrix":
        return "asset_full_matrix_sig", "asset_full_matrix_sig (full_matrix)"
    raise ValueError(f"Unknown report mode: {mode!r}")


def _iter_assets(report_path: str) -> Iterable[Dict]:
    import ijson  # type: ignore

    with open(report_path, "rb") as f:
        for item in ijson.items(f, "assets.item"):
            yield item


def _iter_duplicates(report_path: str) -> Iterable[Dict]:
    import ijson  # type: ignore

    with open(report_path, "rb") as f:
        for item in ijson.items(f, "duplicates.item"):
            yield item


def _read_meta(report_path: str) -> Dict:
    import ijson  # type: ignore

    with open(report_path, "rb") as f:
        # meta is a small dict at the top of the file.
        meta = next(ijson.items(f, "meta"), {})
    return meta or {}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--report", required=True, help="Path to a large report JSON (not the .summary.json)")
    ap.add_argument("--usd", action="append", default=[], help="Asset usd_path to query (repeatable)")
    ap.add_argument(
        "--max-paths",
        type=int,
        default=50,
        help="Max member paths to print per duplicate group (0 prints none)",
    )
    args = ap.parse_args()

    if not args.usd:
        ap.error("At least one --usd is required")

    try:
        import ijson  # noqa: F401
    except Exception as e:
        print("ERROR: This script requires the 'ijson' package for streaming parsing.")
        print("Install it with: python3 -m pip install ijson")
        print(f"Underlying import error: {e}")
        return 2

    repo_root = os.getcwd()
    queries_raw: List[str] = list(args.usd)
    queries_norm = [_normalize_query_path(p, repo_root=repo_root) for p in queries_raw]
    query_set: Set[str] = set(queries_norm)

    meta = _read_meta(args.report)
    mode = str(meta.get("mode") or "")
    if not mode:
        print("ERROR: Could not read meta.mode from report (is this a valid report JSON?)")
        return 2

    sig_key, sig_label = _mode_sig_key(mode)

    found: Dict[str, FoundAsset] = {}

    # Pass 1: find matching assets and their signatures.
    for a in _iter_assets(args.report):
        usd_path = str(a.get("usd_path") or "")
        if usd_path in query_set:
            found[usd_path] = FoundAsset(
                usd_path=usd_path,
                category=a.get("category"),
                uid=a.get("uid"),
                mesh_count=int(a.get("mesh_count") or 0),
                asset_geom_sig=str(a.get("asset_geom_sig") or ""),
                asset_scale_sig=str(a.get("asset_scale_sig") or ""),
                asset_full_matrix_sig=str(a.get("asset_full_matrix_sig") or ""),
            )
            if len(found) == len(query_set):
                break

    print("=== Report ===")
    print(f"report={args.report}")
    print(f"mode={mode}")
    print(f"sig_key={sig_key} ({sig_label})")
    print("")

    # Print existence + basic info.
    print("=== Assets ===")
    missing = [p for p in queries_norm if p not in found]
    for raw, norm in zip(queries_raw, queries_norm):
        if norm in found:
            fa = found[norm]
            mode_sig = getattr(fa, sig_key)
            print(f"FOUND: {raw}")
            print(f"  normalized={norm}")
            print(f"  category={fa.category} uid={fa.uid}")
            print(f"  mesh_count={fa.mesh_count}")
            print(f"  {sig_label}={mode_sig}")
            if fa.mesh_count == 0:
                print("  NOTE: mesh_count==0 -> this asset's signature comes from empty mesh list")
        else:
            print(f"MISSING: {raw}")
            print(f"  normalized={norm}")
    print("")

    if missing:
        print("Some queries were not found in report assets[].")
        return 1

    # Pass 2: find the duplicates group(s) by signature.
    wanted_sigs: Set[str] = set(getattr(found[p], sig_key) for p in found)
    sig_to_group: Dict[str, Dict] = {}
    for g in _iter_duplicates(args.report):
        sig = str(g.get("sig") or "")
        if sig in wanted_sigs:
            sig_to_group[sig] = g
            if len(sig_to_group) == len(wanted_sigs):
                break

    print("=== Duplicate Groups (report mode) ===")
    for raw, norm in zip(queries_raw, queries_norm):
        fa = found[norm]
        mode_sig = getattr(fa, sig_key)
        g = sig_to_group.get(mode_sig)
        if not g:
            print(f"NO DUP GROUP: {raw} (sig={mode_sig})")
            continue
        paths = list(g.get("usd_paths") or [])
        count = int(g.get("count") or len(paths) or 0)
        print(f"GROUP: {raw}")
        print(f"  sig={mode_sig}")
        print(f"  group_count={count}")
        if args.max_paths > 0:
            show = paths[: args.max_paths]
            print(f"  members_shown={len(show)}/{len(paths)}")
            for p in show:
                print(f"    {p}")
    print("")

    # Pairwise same-group check for the common case of 2 inputs.
    if len(queries_norm) == 2:
        a1 = found[queries_norm[0]]
        a2 = found[queries_norm[1]]
        s1 = getattr(a1, sig_key)
        s2 = getattr(a2, sig_key)
        print("=== Same Asset? (by report mode signature) ===")
        print("YES" if s1 == s2 else "NO")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
