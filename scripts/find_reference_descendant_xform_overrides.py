#!/usr/bin/env python3
"""Find scene-layer xform overrides authored under referenced prim subtrees.

Normalize currently assumes the effective geometry of a reference is determined
by the referenced asset plus the top-level referencing prim transform. This
script finds cases where the layout layer also authors xformOps deeper inside
that referenced subtree, which can invalidate that assumption.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List, Optional, Sequence

try:
    from pxr import Usd
except ImportError as exc:
    print(f"Error: pxr (USD) not available: {exc}", file=sys.stderr)
    print("Run via ./scripts/isaac_python.sh or a USD-enabled Python.", file=sys.stderr)
    sys.exit(1)


def _iter_layouts(scenes_root: str):
    for sub in ("home", "commercial"):
        base = os.path.join(scenes_root, sub)
        if not os.path.isdir(base):
            continue
        for scene_id in sorted(os.listdir(base)):
            layout = os.path.join(base, scene_id, "layout.usd")
            if os.path.isfile(layout):
                yield layout, f"{sub}/{scene_id}"


def scan_layout(layout_path: str) -> Dict[str, Any]:
    stage = Usd.Stage.Open(layout_path)
    if stage is None:
        raise RuntimeError(f"failed_to_open_layout: {layout_path}")

    ref_roots = [str(prim.GetPath()) for prim in stage.Traverse() if prim.HasAuthoredReferences()]
    findings: List[Dict[str, Any]] = []

    for prim in stage.Traverse():
        path = str(prim.GetPath())
        parent_ref = None
        for ref_root in ref_roots:
            if path != ref_root and path.startswith(ref_root + "/"):
                parent_ref = ref_root
                break
        if parent_ref is None:
            continue

        local_specs = [
            spec for spec in prim.GetPrimStack()
            if os.path.basename(spec.layer.identifier) == "layout.usd"
        ]
        if not local_specs:
            continue

        props = set()
        for spec in local_specs:
            props.update(spec.properties.keys())
        xform_props = sorted(
            prop for prop in props if prop.startswith("xformOp:") or prop == "xformOpOrder"
        )
        if not xform_props:
            continue

        findings.append({
            "referenced_root": parent_ref,
            "prim_path": path,
            "xform_props": xform_props,
        })

    return {
        "layout": os.path.abspath(layout_path),
        "finding_count": len(findings),
        "findings": findings,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenes-root", required=True)
    ap.add_argument("--scene-filter", default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args(argv)

    results = []
    for layout_path, scene_id in _iter_layouts(os.path.abspath(args.scenes_root)):
        if args.scene_filter and args.scene_filter not in scene_id:
            continue
        result = scan_layout(layout_path)
        result["scene_id"] = scene_id
        results.append(result)

    summary = {
        "layouts_scanned": len(results),
        "layouts_with_findings": sum(1 for r in results if r["finding_count"] > 0),
        "total_findings": sum(r["finding_count"] for r in results),
        "results": results,
    }

    text = json.dumps(summary, indent=2, ensure_ascii=False)
    if args.out:
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"Wrote report: {args.out}")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
