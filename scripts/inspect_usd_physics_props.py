#!/usr/bin/env python3
"""Inspect USD physics-related properties for specific prims.

This script is intended for quick debugging of issues like:
- dynamic object penetrates static surface then pops back out
- missing collider on a mesh
- wrong approximation token (e.g., convexHull vs none)
- instance proxy vs prototype confusion

It prints:
- prim type, active, instance/proxy info
- applied schemas
- key UsdPhysics attributes (rigidBodyEnabled, collisionEnabled, approximation)
- mass properties if present
- subtree collider summary (how many meshes have collision enabled)
- world-space axis-aligned bbox for the prim (best-effort)

Usage:
  ./scripts/isaac_python.sh scripts/inspect_usd_physics_props.py \
    --input /abs/path/scene.usd \
    --prim /root/a/b \
    --prim /root/c/d

"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Tuple

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


PHYS_ATTRS = (
    "physics:rigidBodyEnabled",
    "physics:collisionEnabled",
    "physics:approximation",
    "physics:kinematicEnabled",
    "physics:linearDamping",
    "physics:angularDamping",
    "physxRigidBody:enableCCD",
    "physxRigidBody:maxDepenetrationVelocity",
    "physxCollision:contactOffset",
    "physxCollision:restOffset",
)


PHYS_MATERIAL_ATTRS = (
    "physics:staticFriction",
    "physics:dynamicFriction",
    "physics:restitution",
)


def _get_attr_str(prim: Usd.Prim, name: str) -> str:
    a = prim.GetAttribute(name)
    if not a:
        return "(none)"
    try:
        v = a.Get()
    except Exception as e:
        return f"(error: {e})"
    if v is None:
        return "(unset)"
    return str(v)


def _applied_schemas(prim: Usd.Prim) -> List[str]:
    try:
        return list(prim.GetAppliedSchemas())
    except Exception:
        return []


def _has_api(prim: Usd.Prim, api_name: str) -> bool:
    return api_name in set(_applied_schemas(prim))


def _mass_info(prim: Usd.Prim) -> Dict[str, str]:
    info: Dict[str, str] = {}

    # MassAPI is a single-apply API.
    try:
        mass_api = UsdPhysics.MassAPI(prim)
        if mass_api and mass_api.GetPrim().IsValid() and _has_api(prim, "PhysicsMassAPI"):
            for n in (
                "physics:mass",
                "physics:density",
                "physics:centerOfMass",
                "physics:diagonalInertia",
                "physics:principalAxes",
            ):
                info[n] = _get_attr_str(prim, n)
    except Exception:
        pass

    return info


def _iter_descendants_including_self(prim: Usd.Prim) -> Iterable[Usd.Prim]:
    yield prim
    for p in Usd.PrimRange(prim):
        yield p


def _subtree_collider_summary(root: Usd.Prim) -> Dict[str, int]:
    counts = {
        "mesh_total": 0,
        "mesh_collision_enabled": 0,
        "rb_enabled_prims": 0,
    }

    for p in Usd.PrimRange(root):
        if p.IsA(UsdGeom.Mesh):
            counts["mesh_total"] += 1
            ce = p.GetAttribute("physics:collisionEnabled")
            if ce and ce.Get() is True:
                counts["mesh_collision_enabled"] += 1
        rbe = p.GetAttribute("physics:rigidBodyEnabled")
        if rbe and rbe.Get() is True:
            counts["rb_enabled_prims"] += 1

    return counts


def _world_aabb(stage: Usd.Stage, prim: Usd.Prim) -> Optional[Tuple[Gf.Vec3d, Gf.Vec3d]]:
    # Best-effort: compute AABB from UsdGeom.BBoxCache.
    try:
        time = Usd.TimeCode.Default()
        purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]
        bbox_cache = UsdGeom.BBoxCache(time, purposes, useExtentsHint=True)
        bbox = bbox_cache.ComputeWorldBound(prim)
        r = bbox.ComputeAlignedRange()
        if r.IsEmpty():
            return None
        return (r.GetMin(), r.GetMax())
    except Exception:
        return None


def _compute_bound_materials(stage: Usd.Stage, prim: Usd.Prim) -> List[UsdShade.Material]:
    mats: List[UsdShade.Material] = []
    try:
        mb = UsdShade.MaterialBindingAPI(prim)
        if not mb:
            return mats
        direct = mb.GetDirectBinding()
        if direct and direct.GetMaterialPath() and direct.GetMaterialPath() != Sdf.Path.emptyPath:
            mprim = stage.GetPrimAtPath(direct.GetMaterialPath())
            if mprim and mprim.IsValid():
                mats.append(UsdShade.Material(mprim))
        # Also compute resolved bound material (walks up namespace)
        resolved = mb.ComputeBoundMaterial()
        if resolved and resolved[0]:
            mats.append(resolved[0])
    except Exception:
        return mats

    # de-dup by prim path
    uniq = {}
    for m in mats:
        try:
            uniq[m.GetPrim().GetPath().pathString] = m
        except Exception:
            pass
    return list(uniq.values())


def _print_physics_material(stage: Usd.Stage, prim: Usd.Prim) -> None:
    mats = _compute_bound_materials(stage, prim)
    if not mats:
        return
    print("\nmaterials:")
    for m in mats:
        mp = m.GetPrim()
        if not mp or not mp.IsValid():
            continue
        print("  material:", mp.GetPath())
        # If the material prim itself has UsdPhysics.MaterialAPI, show the key attrs.
        try:
            pm = UsdPhysics.MaterialAPI(mp)
            has = False
            for a in PHYS_MATERIAL_ATTRS:
                attr = mp.GetAttribute(a)
                if attr:
                    has = True
                    try:
                        v = attr.Get()
                    except Exception as e:
                        v = f"(error: {e})"
                    print(f"    {a}: {v}")
            if not has:
                # Sometimes attrs are authored but not created; still show that API exists.
                if "PhysicsMaterialAPI" in set(mp.GetAppliedSchemas()):
                    print("    PhysicsMaterialAPI applied (no friction/restitution attrs found)")
        except Exception:
            pass


def _print_prim_report(stage: Usd.Stage, prim: Usd.Prim, title: str) -> None:
    print("\n" + "=" * 80)
    print(title)
    print("path:", prim.GetPath())
    print("type:", prim.GetTypeName(), "active:", prim.IsActive())
    print("isInstance:", prim.IsInstance(), "isInstanceProxy:", prim.IsInstanceProxy())

    schemas = _applied_schemas(prim)
    print("appliedSchemas:", schemas if schemas else "(none)")

    # Show physics-purpose material binding if present.
    rel = prim.GetRelationship("material:binding:physics")
    if rel:
        try:
            targets = rel.GetTargets()
        except Exception:
            targets = []
        if targets:
            print("physicsMaterialBinding:", [t.pathString for t in targets])

    print("\nphysics attrs on prim:")
    for n in PHYS_ATTRS:
        print(f"  {n}: {_get_attr_str(prim, n)}")

    # If this prim is itself a physics material, show friction/restitution.
    if "PhysicsMaterialAPI" in set(schemas):
        print("\nphysics material attrs:")
        for n in PHYS_MATERIAL_ATTRS:
            print(f"  {n}: {_get_attr_str(prim, n)}")

    mi = _mass_info(prim)
    if mi:
        print("\nmass attrs on prim:")
        for k, v in mi.items():
            print(f"  {k}: {v}")

    # If this prim is a mesh, print mesh-specific hints.
    if prim.IsA(UsdGeom.Mesh):
        m = UsdGeom.Mesh(prim)
        try:
            pts = m.GetPointsAttr().Get()
            print("\nmesh points:", 0 if pts is None else len(pts))
        except Exception as e:
            print("\nmesh points: (error)", e)

    summary = _subtree_collider_summary(prim)
    print("\nsubtree summary:")
    for k, v in summary.items():
        print(f"  {k}: {v}")

    aabb = _world_aabb(stage, prim)
    if aabb:
        mn, mx = aabb
        print("\nworld AABB:")
        print("  min:", mn)
        print("  max:", mx)

        # Helpful derived measures
        dz = float(mx[2] - mn[2])
        print("  sizeZ:", dz)

    _print_physics_material(stage, prim)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--prim", action="append", required=True, help="Prim path to inspect (repeatable)")
    ap.add_argument("--compare-aabb", nargs=2, metavar=("A", "B"), help="Compare world AABB overlap for two prim paths")
    args = ap.parse_args()

    stage = Usd.Stage.Open(args.input)
    if stage is None:
        raise RuntimeError(f"Failed to open USD: {args.input}")

    print("stage:", args.input)
    try:
        print("metersPerUnit:", stage.GetMetadata("metersPerUnit"))
        print("upAxis:", stage.GetMetadata("upAxis"))
    except Exception:
        pass

    # Print requested prims
    for p in args.prim:
        prim = stage.GetPrimAtPath(Sdf.Path(p))
        if not prim or not prim.IsValid():
            print("\n" + "=" * 80)
            print("MISSING prim:", p)
            continue

        _print_prim_report(stage, prim, title=f"Prim report: {p}")

        # If it's an instance proxy, also inspect prototype prim.
        if prim.IsInstanceProxy():
            try:
                proto = prim.GetPrimInPrototype()
            except Exception:
                proto = None
            if proto and proto.IsValid():
                _print_prim_report(stage, proto, title=f"Prototype report for instance proxy: {p} -> {proto.GetPath()}")

    # Optional overlap check
    if args.compare_aabb:
        a_path, b_path = args.compare_aabb
        a = stage.GetPrimAtPath(Sdf.Path(a_path))
        b = stage.GetPrimAtPath(Sdf.Path(b_path))
        if a and a.IsValid() and b and b.IsValid():
            aa = _world_aabb(stage, a)
            bb = _world_aabb(stage, b)
            print("\n" + "=" * 80)
            print("AABB compare")
            print("A:", a_path)
            print("B:", b_path)
            if not aa or not bb:
                print("Cannot compute one or both AABBs")
            else:
                a_min, a_max = aa
                b_min, b_max = bb
                overlap = (
                    (a_min[0] <= b_max[0] and a_max[0] >= b_min[0])
                    and (a_min[1] <= b_max[1] and a_max[1] >= b_min[1])
                    and (a_min[2] <= b_max[2] and a_max[2] >= b_min[2])
                )
                dz_pen = (b_max[2] - a_min[2])  # if positive, A is below B top
                print("overlap:", overlap)
                print("B_topZ - A_bottomZ:", float(dz_pen))
        else:
            print("\nAABB compare: one or both prims missing")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
