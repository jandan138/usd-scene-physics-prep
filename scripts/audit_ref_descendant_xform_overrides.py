#!/usr/bin/env python3
"""Audit scene-authored xform overrides beneath referenced asset roots.

Normalize currently rewrites referenced assets in ways that can change the
composed xformOpOrder below the reference root. If a scene layer also authored
descendant xform opinions such as /Instance.xformOp:scale, those opinions may
become inert after normalization even though the attribute still exists in the
layout layer.

This script scans a composed scene and reports descendant prims that:
  - have a local scene-layer spec on top of a referenced asset prim stack
  - author xform-related properties in the scene layer
  - may no longer participate in the composed xformOpOrder
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List, Optional, Sequence

try:
    from pxr import Usd, UsdGeom
except ImportError as exc:
    print(f"Error: pxr (USD) not available: {exc}", file=sys.stderr)
    print("Run with a Python environment that has USD installed.", file=sys.stderr)
    sys.exit(1)


XFORM_PROPS = {
    "xformOpOrder",
    "xformOp:transform",
    "xformOp:translate",
    "xformOp:orient",
    "xformOp:scale",
}


def _scene_identifier(path: str) -> str:
    return os.path.abspath(path)


def audit_scene(scene_path: str) -> Dict[str, Any]:
    scene_path = os.path.abspath(scene_path)
    stage = Usd.Stage.Open(scene_path, load=Usd.Stage.LoadNone)
    if stage is None:
        raise RuntimeError(f"Failed to open scene: {scene_path}")

    findings: List[Dict[str, Any]] = []

    for prim in stage.Traverse():
        if not prim or not prim.IsValid():
            continue

        stack = prim.GetPrimStack()
        if len(stack) < 2:
            continue

        top_spec = stack[0]
        if top_spec.layer.identifier != scene_path:
            continue

        referenced_specs = [spec for spec in stack[1:] if "GRScenes_assets" in spec.layer.identifier]
        if not referenced_specs:
            continue

        # Ignore the referenced root itself. The scene root object prim is
        # expected to author its own xform opinions; we care about descendants
        # beneath that root, where normalize may silently invalidate overrides.
        if str(referenced_specs[0].path) == "/Root":
            continue

        scene_props = sorted(set(top_spec.properties.keys()) & XFORM_PROPS)
        if not scene_props:
            continue

        xform = UsdGeom.Xformable(prim)
        composed_order = []
        if xform:
            composed_order = [op.GetOpName() for op in xform.GetOrderedXformOps()]

        inert_scene_props = [
            prop for prop in scene_props
            if prop.startswith("xformOp:") and prop not in composed_order
        ]

        findings.append({
            "prim_path": str(prim.GetPath()),
            "scene_layer_props": scene_props,
            "composed_xformOpOrder": composed_order,
            "inert_scene_xform_props": inert_scene_props,
            "asset_layer_path": str(referenced_specs[0].path),
            "asset_layer_identifier": referenced_specs[0].layer.identifier,
        })

    findings.sort(
        key=lambda item: (
            len(item["inert_scene_xform_props"]),
            item["prim_path"],
        ),
        reverse=True,
    )

    return {
        "scene": scene_path,
        "finding_count": len(findings),
        "findings": findings,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", required=True, help="Scene layout USD to audit")
    parser.add_argument("--out", default=None, help="Optional JSON output path")
    args = parser.parse_args(argv)

    result = audit_scene(args.scene)

    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)
        print(args.out)
    else:
        json.dump(result, sys.stdout, indent=2)
        sys.stdout.write("\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
