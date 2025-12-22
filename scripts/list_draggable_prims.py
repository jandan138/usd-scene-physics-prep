#!/usr/bin/env python3
"""List prims that are likely draggable in Isaac Sim (Shift + LMB drag).

Heuristic (practical for Isaac Sim):
- Prim has UsdPhysics.RigidBodyAPI and `physics:rigidBodyEnabled == True`
- Prim's subtree contains at least one prim with `physics:collisionEnabled == True`

This matches the typical interaction where Isaac applies a continuous force to a
selected rigid body that has collision shapes.

Usage:
  ./scripts/isaac_python.sh scripts/list_draggable_prims.py \
    --input /path/to/scene.usd
"""

from __future__ import annotations

import argparse
from typing import Dict, List, Tuple

from pxr import Usd, UsdPhysics


def _get_bool_attr(prim: Usd.Prim, name: str):
    a = prim.GetAttribute(name)
    if not a:
        return None
    try:
        return a.Get()
    except Exception:
        return None


def _count_enabled_colliders_in_subtree(root: Usd.Prim) -> int:
    if not root or not root.IsValid():
        return 0

    count = 0
    stack = [root]
    while stack:
        p = stack.pop()
        enabled = _get_bool_attr(p, "physics:collisionEnabled")
        if enabled is True:
            count += 1
        stack.extend(p.GetChildren())
    return count


def _count_enabled_colliders_with_glb_probe(stage: Usd.Stage, root: Usd.Prim) -> int:
    """Count colliders under root, with a GLB-payload-friendly fallback.

    In some scenes, GLB payload prims show placeholder child prims as 'over'
    prims (IsDefined()==False). Those may not be enumerated by GetChildren().
    We therefore probe a couple of well-known geometry paths used by the
    SimBench tasks.
    """

    count = _count_enabled_colliders_in_subtree(root)
    if count > 0:
        return count

    base = str(root.GetPath())
    for suffix in ("/geometry_0/geometry_0", "/geometry_0"):
        prim = stage.GetPrimAtPath(base + suffix)
        if prim and prim.IsValid() and _get_bool_attr(prim, "physics:collisionEnabled") is True:
            count += 1

    return count


def list_draggable_prims(stage: Usd.Stage, subtree_root_path: str) -> List[Dict]:
    subtree_root = stage.GetPrimAtPath(subtree_root_path)
    if not subtree_root or not subtree_root.IsValid():
        raise RuntimeError(f"Missing subtree root prim: {subtree_root_path}")

    results: List[Dict] = []
    no_collider_rigids: List[Dict] = []
    for prim in Usd.PrimRange(subtree_root):
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            continue

        enabled = _get_bool_attr(prim, "physics:rigidBodyEnabled")
        if enabled is not True:
            continue

        collider_count = _count_enabled_colliders_with_glb_probe(stage, prim)
        item = {
            "path": str(prim.GetPath()),
            "type": prim.GetTypeName(),
            "colliders_in_subtree": collider_count,
            "mass": _get_bool_attr(prim, "physics:mass"),
        }

        if collider_count > 0:
            results.append(item)
        else:
            no_collider_rigids.append(item)

    # stable output
    results.sort(key=lambda x: x["path"])
    no_collider_rigids.sort(key=lambda x: x["path"])
    return results, no_collider_rigids


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument(
        "--input",
        required=True,
        help="USD file to inspect (ideally the processed scene).",
    )
    ap.add_argument(
        "--root",
        default="/root",
        help="Subtree to search within (default: /root)",
    )
    args = ap.parse_args()

    stage = Usd.Stage.Open(args.input)
    if stage is None:
        raise RuntimeError(f"Failed to open stage: {args.input}")

    draggable, rigid_no_collider = list_draggable_prims(stage, args.root)

    print("=== Draggable prim candidates (Shift+LMB force-drag) ===")
    print("input:", args.input)
    print("search_root:", args.root)
    print("count:", len(draggable))
    for item in draggable:
        print(f"- {item['path']}  type={item['type']}  colliders={item['colliders_in_subtree']}  mass={item['mass']}")

    if rigid_no_collider:
        print("\n=== Rigid bodies with NO colliders in subtree (usually not draggable) ===")
        print("count:", len(rigid_no_collider))
        for item in rigid_no_collider:
            print(f"- {item['path']}  type={item['type']}  colliders={item['colliders_in_subtree']}  mass={item['mass']}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
