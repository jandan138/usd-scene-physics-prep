#!/usr/bin/env python3
"""Bind a physics material (friction/restitution) to specific prims.

This is a pragmatic fix for "too bouncy" contacts.

What it does
- Creates (or reuses) a UsdShade.Material prim (default under /_PhysicsMaterials).
- Applies UsdPhysics.MaterialAPI on that material prim.
- Authors:
  - physics:restitution (default 0.0)
  - physics:staticFriction / physics:dynamicFriction (defaults 0.8 / 0.6)
- Binds that material to target prim(s) using a *physics-purpose* material binding:
  relationship name: material:binding:physics
  (This avoids overriding render materials bound via material:binding).

Why this helps
- Restitution is the "bounciness" coefficient. Higher restitution -> more rebound.
- Many assets use engine defaults or materials with non-zero restitution.
  Setting restitution to 0 removes elastic rebound, so contacts should settle.

Usage
  ./scripts/isaac_python.sh scripts/oneoff_bind_physics_material.py \
    --input  /path/to/scene.usd \
    --output /path/to/scene_physmat.usd \
    --bind-prim /root/obj__13/geometry_0/geometry_0 \
    --bind-prim /root/obj_table/.../component3 \
    --restitution 0.0

Notes
- If you pass an Xform prim and want to bind to its meshes, use --bind-descendant-meshes.
"""

from __future__ import annotations

import argparse
import os
from typing import Iterable, List, Optional

from pxr import Sdf, Usd, UsdPhysics, UsdShade, UsdGeom


PHYSICS_BIND_REL = "material:binding:physics"


def _derive_default_output(input_path: str) -> str:
    base, ext = os.path.splitext(input_path)
    if ext.lower() not in (".usd", ".usda", ".usdc"):
        ext = ".usd"
    return base + "_physmat" + ext


def _ensure_physics_material(
    stage: Usd.Stage,
    material_path: str,
    restitution: float,
    static_friction: float,
    dynamic_friction: float,
) -> UsdShade.Material:
    mat = UsdShade.Material.Define(stage, Sdf.Path(material_path))
    mprim = mat.GetPrim()

    pmat = UsdPhysics.MaterialAPI.Apply(mprim)
    pmat.CreateRestitutionAttr(restitution)
    pmat.CreateStaticFrictionAttr(static_friction)
    pmat.CreateDynamicFrictionAttr(dynamic_friction)
    return mat


def _iter_target_prims(stage: Usd.Stage, prim_path: str, bind_descendant_meshes: bool) -> Iterable[Usd.Prim]:
    prim = stage.GetPrimAtPath(Sdf.Path(prim_path))
    if not prim or not prim.IsValid():
        return

    if not bind_descendant_meshes:
        yield prim
        return

    # If user asked for descendant meshes, bind to meshes under this prim.
    # This avoids binding materials to container Xforms.
    any_mesh = False
    for p in Usd.PrimRange(prim):
        if p.IsA(UsdGeom.Mesh):
            any_mesh = True
            yield p
    if not any_mesh:
        # fallback to the prim itself
        yield prim


def _bind_physics_material(prim: Usd.Prim, material: UsdShade.Material) -> None:
    # Write purpose-specific binding relationship directly.
    rel = prim.GetRelationship(PHYSICS_BIND_REL)
    if not rel:
        rel = prim.CreateRelationship(PHYSICS_BIND_REL, custom=False)
    rel.SetTargets([material.GetPath()])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", default=None)
    ap.add_argument(
        "--material-path",
        default="/_PhysicsMaterials/pmat_no_bounce",
        help="Where to author the UsdShade.Material prim",
    )
    ap.add_argument("--restitution", type=float, default=0.0)
    ap.add_argument("--static-friction", type=float, default=0.8)
    ap.add_argument("--dynamic-friction", type=float, default=0.6)
    ap.add_argument("--bind-prim", action="append", required=True, help="Prim path to bind (repeatable)")
    ap.add_argument(
        "--bind-descendant-meshes",
        action="store_true",
        help="If bind-prim is an Xform, bind to all descendant meshes",
    )
    args = ap.parse_args()

    out = args.output or _derive_default_output(args.input)

    stage = Usd.Stage.Open(args.input)
    if stage is None:
        raise RuntimeError(f"Failed to open USD: {args.input}")

    mat = _ensure_physics_material(
        stage,
        args.material_path,
        restitution=args.restitution,
        static_friction=args.static_friction,
        dynamic_friction=args.dynamic_friction,
    )

    bound = 0
    missing: List[str] = []
    for p in args.bind_prim:
        prim = stage.GetPrimAtPath(Sdf.Path(p))
        if not prim or not prim.IsValid():
            missing.append(p)
            continue
        for t in _iter_target_prims(stage, p, args.bind_descendant_meshes):
            _bind_physics_material(t, mat)
            bound += 1

    stage.GetRootLayer().Export(out)
    print("Wrote:", out)
    print("Physics material:", args.material_path)
    print("restitution:", args.restitution, "staticFriction:", args.static_friction, "dynamicFriction:", args.dynamic_friction)
    print("bound_targets:", bound)
    if missing:
        print("missing_prims:")
        for p in missing:
            print("-", p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
