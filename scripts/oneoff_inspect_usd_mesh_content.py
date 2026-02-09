#!/usr/bin/env python3
"""Inspect whether a USD file contains any usable mesh data.

This opens a stage and counts UsdGeom.Mesh prims, plus total points/faces.

Usage:
  ./scripts/isaac_python.sh scripts/oneoff_inspect_usd_mesh_content.py --usd <path>
"""

from __future__ import annotations

import argparse
import os

from pxr import Usd, UsdGeom


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", required=True, help="Path to .usd/.usda/.usdc")
    args = ap.parse_args()

    p = args.usd
    abs_p = os.path.abspath(p)
    print("path:", abs_p, flush=True)
    print("exists:", os.path.exists(p), flush=True)
    if os.path.exists(p):
        print("size_bytes:", os.path.getsize(p), flush=True)

    stage = Usd.Stage.Open(p)
    print("stage_open:", stage is not None, flush=True)
    if stage is None:
        return 2

    dp = stage.GetDefaultPrim()
    print("default_prim:", dp.GetPath() if dp else None, flush=True)

    prim_count = 0
    mesh_paths = []
    points_total = 0
    faces_total = 0
    meshes_with_points = 0

    for prim in stage.Traverse():
        prim_count += 1
        if prim.IsA(UsdGeom.Mesh):
            mesh_paths.append(str(prim.GetPath()))
            m = UsdGeom.Mesh(prim)
            pts = m.GetPointsAttr().Get() or []
            fvc = m.GetFaceVertexCountsAttr().Get() or []
            if pts:
                meshes_with_points += 1
            points_total += len(pts)
            faces_total += len(fvc)

    print("prim_count:", prim_count, flush=True)
    print("mesh_prim_count:", len(mesh_paths), flush=True)
    print("meshes_with_points:", meshes_with_points, flush=True)
    print("points_total:", points_total, flush=True)
    print("faces_total:", faces_total, flush=True)

    for mp in mesh_paths[:50]:
        print("mesh_prim:", mp, flush=True)

    # Print a few root children for context
    root_children = list(stage.GetPseudoRoot().GetChildren())
    print("root_children_count:", len(root_children), flush=True)
    for c in root_children[:20]:
        print("root_child:", c.GetPath(), "type=", c.GetTypeName(), flush=True)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
