#!/usr/bin/env python3
"""One-off utility: make a prim static collider-only in a USD.

This is intended for ad-hoc fixes (e.g. a specific imported GLB object) where we
want:
- NOT a rigid body (so it doesn't respond to gravity)
- Keep existing colliders (collisionEnabled) intact

Usage (Isaac python):
  ./scripts/isaac_python.sh scripts/oneoff_make_static_collider_only.py \
    --input  /path/to/in.usd \
    --output /path/to/out.usd \
    --prim   /root/__04

Notes:
- This script removes RigidBodyAPI and clears mass properties on the target prim.
- It also authors `physics:rigidBodyEnabled = False` on the target prim and its
  descendants to override stronger opinions from referenced layers.
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from pxr import Usd, UsdPhysics


def _force_disable_rigid_recursive(prim: Usd.Prim) -> None:
    a = prim.GetAttribute("physics:rigidBodyEnabled")
    if a:
        a.Set(False)

    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        prim.RemoveAPI(UsdPhysics.RigidBodyAPI)

    if prim.HasAPI(UsdPhysics.MassAPI):
        prim.RemoveAPI(UsdPhysics.MassAPI)

    for attr_name in (
        "physics:mass",
        "physics:density",
        "physics:centerOfMass",
        "physics:diagonalInertia",
        "physics:principalAxes",
    ):
        a2 = prim.GetAttribute(attr_name)
        if a2:
            a2.Clear()

    for ch in prim.GetChildren():
        _force_disable_rigid_recursive(ch)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--prim", required=True, help="Prim path to make static (collider-only)")
    args = ap.parse_args()

    in_path = Path(args.input)
    out_path = Path(args.output)
    if not in_path.exists():
        raise FileNotFoundError(str(in_path))
    out_path.parent.mkdir(parents=True, exist_ok=True)

    shutil.copy2(in_path, out_path)

    stage = Usd.Stage.Open(str(out_path))
    if stage is None:
        raise RuntimeError(f"Failed to open stage: {out_path}")

    prim = stage.GetPrimAtPath(args.prim)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Missing prim: {args.prim}")

    _force_disable_rigid_recursive(prim)

    stage.GetRootLayer().Save()

    print("=== oneoff_make_static_collider_only.py result ===")
    print("input :", in_path)
    print("output:", out_path)
    print("static_prim:", args.prim)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
