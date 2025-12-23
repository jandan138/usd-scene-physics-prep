#!/usr/bin/env python3
"""Check external file dependencies of a USD stage.

Goal
- Given a USD file (including "flattened" outputs), enumerate external asset
  paths it still depends on (textures, referenced USDs, payloads, etc.).
- Detect missing files on the local filesystem where possible.

Notes
- Asset path resolution can be resolver-specific (Ar). We use a best-effort
  approach:
  - Prefer Sdf.AssetPath.resolvedPath when available.
  - Otherwise, if the path is relative and the stage has a root layer path,
    resolve it relative to the root layer directory.
- Non-filesystem schemes (e.g. omni:, http:) are reported but not existence-
  checked.

Usage
  ./scripts/isaac_python.sh scripts/check_usd_external_assets.py \
    --input /abs/path/to/file.usd

Optional
  --report /abs/path/to/report.json
"""

from __future__ import annotations

import argparse
import json
import os
import re
import glob
from collections import defaultdict
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple

from pxr import Sdf, Usd


_NON_FILE_SCHEMES = (
    "http://",
    "https://",
    "omniverse://",
    "omni://",
    "omni:",
    "file://",
)


def _is_non_file_scheme(path: str) -> bool:
    p = path.strip()
    return any(p.startswith(s) for s in _NON_FILE_SCHEMES)


def _looks_like_udim(path: str) -> bool:
    return "<UDIM>" in path or "%(UDIM)" in path or "$UDIM" in path


def _udim_glob_candidates(path: str) -> List[str]:
    # Common UDIM tokens.
    p = path
    p = p.replace("<UDIM>", "[0-9][0-9][0-9][0-9]")
    p = p.replace("$UDIM", "[0-9][0-9][0-9][0-9]")
    # Some pipelines use printf style, e.g. %(UDIM)d
    p = re.sub(r"%\(UDIM\)[a-zA-Z]", "[0-9][0-9][0-9][0-9]", p)
    return glob.glob(p)


def _safe_str(x: Any) -> str:
    try:
        return str(x)
    except Exception:
        return repr(x)


@dataclass(frozen=True)
class AssetRef:
    prop_path: str
    asset_path: str
    resolved_path: str


def _iter_asset_values(value: Any) -> Iterable[Sdf.AssetPath]:
    if value is None:
        return
    if isinstance(value, Sdf.AssetPath):
        yield value
        return
    if isinstance(value, (list, tuple)):
        for v in value:
            if isinstance(v, Sdf.AssetPath):
                yield v


def _resolve_best_effort(asset: Sdf.AssetPath, *, anchor_dir: Optional[str]) -> str:
    # Prefer resolver-provided resolvedPath.
    if getattr(asset, "resolvedPath", ""):
        return asset.resolvedPath

    raw = (asset.path or "").strip()
    if not raw:
        return ""

    if _is_non_file_scheme(raw):
        return raw

    if os.path.isabs(raw):
        return raw

    if anchor_dir:
        return os.path.normpath(os.path.join(anchor_dir, raw))

    return raw


def analyze_usd_dependencies(input_path: str) -> Dict[str, Any]:
    stage = Usd.Stage.Open(input_path)
    if stage is None:
        raise RuntimeError(f"Failed to open USD: {input_path}")

    root_layer = stage.GetRootLayer()
    root_layer_path = root_layer.realPath or root_layer.identifier
    anchor_dir = os.path.dirname(root_layer.realPath) if root_layer.realPath else None

    # Collect used layers (best-effort; may omit unresolved payloads).
    used_layers = []
    for layer in stage.GetUsedLayers():
        used_layers.append(layer.realPath or layer.identifier)

    # Collect asset-valued attributes.
    asset_refs: List[AssetRef] = []
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            t = attr.GetTypeName()
            if t not in (Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.AssetArray):
                continue

            try:
                v = attr.Get()
            except Exception:
                v = None

            for av in _iter_asset_values(v):
                raw = (av.path or "").strip()
                if not raw:
                    continue
                resolved = _resolve_best_effort(av, anchor_dir=anchor_dir)
                asset_refs.append(
                    AssetRef(
                        prop_path=_safe_str(attr.GetPath()),
                        asset_path=raw,
                        resolved_path=resolved,
                    )
                )

    # De-dup by (prop_path, asset_path)
    uniq: Dict[Tuple[str, str], AssetRef] = {}
    for r in asset_refs:
        uniq[(r.prop_path, r.asset_path)] = r
    asset_refs = list(uniq.values())
    asset_refs.sort(key=lambda r: (r.resolved_path, r.prop_path))

    # Existence checks
    missing: List[Dict[str, str]] = []
    present: List[Dict[str, str]] = []
    scheme_only: List[Dict[str, str]] = []

    by_ext = defaultdict(int)

    for r in asset_refs:
        resolved = r.resolved_path or r.asset_path
        ext = os.path.splitext(resolved)[1].lower()
        if ext:
            by_ext[ext] += 1

        if _is_non_file_scheme(resolved):
            scheme_only.append({"prop": r.prop_path, "asset": r.asset_path, "resolved": r.resolved_path})
            continue

        # UDIM patterns: consider present if at least one tile exists.
        if _looks_like_udim(resolved):
            candidates = _udim_glob_candidates(resolved)
            if candidates:
                present.append({"prop": r.prop_path, "asset": r.asset_path, "resolved": r.resolved_path, "note": f"udim_tiles={len(candidates)}"})
            else:
                missing.append({"prop": r.prop_path, "asset": r.asset_path, "resolved": r.resolved_path, "note": "udim_no_tiles_found"})
            continue

        if resolved and os.path.exists(resolved):
            present.append({"prop": r.prop_path, "asset": r.asset_path, "resolved": r.resolved_path})
        else:
            missing.append({"prop": r.prop_path, "asset": r.asset_path, "resolved": r.resolved_path})

    # Sort extension stats
    by_ext_sorted = sorted(by_ext.items(), key=lambda kv: (-kv[1], kv[0]))

    return {
        "input": input_path,
        "root_layer": root_layer_path,
        "used_layers": used_layers,
        "asset_attributes_total": len(asset_refs),
        "present_count": len(present),
        "missing_count": len(missing),
        "scheme_only_count": len(scheme_only),
        "by_extension": [{"ext": k, "count": v} for k, v in by_ext_sorted],
        "present": present,
        "missing": missing,
        "non_file_scheme": scheme_only,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--report", default=None, help="Optional JSON report output path")
    ap.add_argument("--print-present", action="store_true", help="Also print present assets")
    args = ap.parse_args()

    report = analyze_usd_dependencies(args.input)

    print("=== USD external dependency report ===")
    print("input:", report["input"])
    print("root_layer:", report["root_layer"])
    print("used_layers:", len(report["used_layers"]))
    print("asset_attributes:", report["asset_attributes_total"])
    print("present:", report["present_count"], "missing:", report["missing_count"], "non_file_scheme:", report["scheme_only_count"])

    if report["by_extension"]:
        print("\nby_extension:")
        for item in report["by_extension"]:
            print(f"  {item['ext']}: {item['count']}")

    if report["missing"]:
        print("\nMISSING assets:")
        for m in report["missing"]:
            note = f" ({m['note']})" if m.get("note") else ""
            print(f"- {m['resolved'] or m['asset']}{note}\n  prop: {m['prop']}")

    if report["non_file_scheme"]:
        print("\nNon-filesystem assets (not existence-checked):")
        for m in report["non_file_scheme"]:
            print(f"- {m['resolved'] or m['asset']}\n  prop: {m['prop']}")

    if args.print_present and report["present"]:
        print("\nPresent assets:")
        for m in report["present"]:
            note = f" ({m['note']})" if m.get("note") else ""
            print(f"- {m['resolved'] or m['asset']}{note}\n  prop: {m['prop']}")

    if args.report:
        os.makedirs(os.path.dirname(args.report) or ".", exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print("\nWrote report:", args.report)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
