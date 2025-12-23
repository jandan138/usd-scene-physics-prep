#!/usr/bin/env python3
"""Add a proxy box collider for a mesh (typically a tabletop) and optionally disable the original mesh collider.

Why this helps
- Triangle-mesh collision (approx=none) against small/complex convex shapes can
  produce jittery contacts ("bouncy" / "popping") even with restitution=0.
- A simple box collider provides a smoother contact surface and more stable
  normals, reducing solver oscillation.

What it does
- Computes the target prim's world AABB.
- Authors a child prim (Xform + Cube) that acts as a box collider:
  - Cube size=1 and a non-uniform scale is applied to match the AABB size.
  - Translation is set to the AABB center.
  - Transform is authored in the parent space (so it stays aligned).
- Applies UsdPhysics.CollisionAPI to the cube.
- Optionally sets physics:collisionEnabled=False on the original mesh (strong
  override), to avoid double-collision.

Usage
  ./scripts/isaac_python.sh scripts/oneoff_add_proxy_box_collider.py \
    --input  /path/to/scene.usd \
    --output /path/to/scene_box.usd \
    --target /root/obj_table/.../component3 \
    --name tabletop_proxy \
    --min-thickness 0.03 \
    --disable-original-collision

Notes
- This creates an axis-aligned box in world space aligned to the current mesh AABB.
  It is not an oriented box; for most tabletops this is good enough.
"""

from __future__ import annotations

import argparse
import os
from typing import Optional, Tuple

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def _derive_default_output(input_path: str) -> str:
    base, ext = os.path.splitext(input_path)
    if ext.lower() not in (".usd", ".usda", ".usdc"):
        ext = ".usd"
    return base + "_box" + ext


def _compute_world_aabb(stage: Usd.Stage, prim: Usd.Prim) -> Tuple[Gf.Vec3d, Gf.Vec3d]:
    time = Usd.TimeCode.Default()
    purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]
    bbox_cache = UsdGeom.BBoxCache(time, purposes, useExtentsHint=True)
    bbox = bbox_cache.ComputeWorldBound(prim)
    r = bbox.ComputeAlignedRange()
    if r.IsEmpty():
        raise RuntimeError(f"Empty AABB for prim: {prim.GetPath()}")
    return r.GetMin(), r.GetMax()


def _aabb_corners(mn: Gf.Vec3d, mx: Gf.Vec3d) -> Tuple[Gf.Vec3d, ...]:
    return (
        Gf.Vec3d(mn[0], mn[1], mn[2]),
        Gf.Vec3d(mn[0], mn[1], mx[2]),
        Gf.Vec3d(mn[0], mx[1], mn[2]),
        Gf.Vec3d(mn[0], mx[1], mx[2]),
        Gf.Vec3d(mx[0], mn[1], mn[2]),
        Gf.Vec3d(mx[0], mn[1], mx[2]),
        Gf.Vec3d(mx[0], mx[1], mn[2]),
        Gf.Vec3d(mx[0], mx[1], mx[2]),
    )


def _world_aabb_to_parent_local_box(
    stage: Usd.Stage,
    parent: Usd.Prim,
    world_mn: Gf.Vec3d,
    world_mx: Gf.Vec3d,
) -> Tuple[Gf.Vec3d, Gf.Vec3d]:
    """Return (center_local, size_local) such that after parent transforms, the box matches the world AABB.

    This accounts for arbitrary parent scale by transforming the world AABB corners into parent-local space
    and deriving a local AABB.
    """
    w2l = _world_to_local(stage, parent)
    corners_local = [w2l.Transform(c) for c in _aabb_corners(world_mn, world_mx)]
    xs = [c[0] for c in corners_local]
    ys = [c[1] for c in corners_local]
    zs = [c[2] for c in corners_local]
    lmn = Gf.Vec3d(min(xs), min(ys), min(zs))
    lmx = Gf.Vec3d(max(xs), max(ys), max(zs))
    center_local = (lmn + lmx) * 0.5
    size_local = (lmx - lmn)
    return center_local, size_local


def _local_to_world(stage: Usd.Stage, prim: Usd.Prim) -> Gf.Matrix4d:
    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    return xcache.GetLocalToWorldTransform(prim)


def _world_to_local(stage: Usd.Stage, prim: Usd.Prim) -> Gf.Matrix4d:
    return _local_to_world(stage, prim).GetInverse()


def _ensure_proxy_box(
    stage: Usd.Stage,
    parent: Usd.Prim,
    name: str,
    center_local: Gf.Vec3d,
    size_local: Gf.Vec3d,
) -> Usd.Prim:
    # Create a container Xform to hold the cube.
    xform_path = parent.GetPath().AppendChild(name)
    xprim = stage.DefinePrim(xform_path, "Xform")
    xf = UsdGeom.Xformable(xprim)

    # We'll scale a unit cube (size=1, extents [-0.5,0.5]) by size_local and translate to center_local.
    # Use explicit xformOps in the common order so composition is predictable:
    # localTransform = Translate * Scale
    xf.ClearXformOpOrder()
    t_op = xf.AddTranslateOp()
    s_op = xf.AddScaleOp()
    t_op.Set(Gf.Vec3d(float(center_local[0]), float(center_local[1]), float(center_local[2])))
    s_op.Set(Gf.Vec3f(float(size_local[0]), float(size_local[1]), float(size_local[2])))

    cube_path = xform_path.AppendChild("collider")
    cube_prim = stage.DefinePrim(cube_path, "Cube")
    cube = UsdGeom.Cube(cube_prim)
    cube.CreateSizeAttr(1.0)

    # Apply collision API to cube.
    UsdPhysics.CollisionAPI.Apply(cube_prim)
    cube_prim.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool).Set(True)

    return cube_prim


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", default=None)
    ap.add_argument("--target", required=True, help="Mesh prim path to approximate with a proxy box")
    ap.add_argument("--name", default="_proxyBoxCollider")
    ap.add_argument("--min-thickness", type=float, default=0.03, help="Minimum box thickness along Z (meters if metersPerUnit=1)")
    ap.add_argument("--disable-original-collision", action="store_true")
    ap.add_argument("--expand", type=float, default=0.0, help="Expand AABB by this amount on each axis")
    args = ap.parse_args()

    out = args.output or _derive_default_output(args.input)

    stage = Usd.Stage.Open(args.input)
    if stage is None:
        raise RuntimeError(f"Failed to open USD: {args.input}")

    target = stage.GetPrimAtPath(Sdf.Path(args.target))
    if not target or not target.IsValid():
        raise RuntimeError(f"Target prim not found: {args.target}")

    mn, mx = _compute_world_aabb(stage, target)
    center_world = (mn + mx) * 0.5
    size_world = (mx - mn)

    # Expand if requested
    if args.expand != 0.0:
        e = float(args.expand)
        size_world = Gf.Vec3d(
            float(size_world[0] + 2 * e),
            float(size_world[1] + 2 * e),
            float(size_world[2] + 2 * e),
        )

    # Enforce minimum thickness along Z
    if float(size_world[2]) < float(args.min_thickness):
        size_world = Gf.Vec3d(float(size_world[0]), float(size_world[1]), float(args.min_thickness))

    parent = target.GetParent()
    # Convert the desired world-space box to a parent-local transform so it remains correct
    # even when the parent has scale.
    center_local, size_local = _world_aabb_to_parent_local_box(stage, parent, mn, mx)

    # If we enforced thickness in world, re-apply thickness in local as well (best-effort).
    # This is approximate for scaled parents but improves stability.
    if float(size_local[2]) < float(args.min_thickness):
        size_local = Gf.Vec3d(float(size_local[0]), float(size_local[1]), float(args.min_thickness))

    _ensure_proxy_box(stage, parent, args.name, center_local, size_local)

    if args.disable_original_collision:
        # Strong override: disable collision on the original mesh.
        target.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool).Set(False)

    stage.GetRootLayer().Export(out)
    print("Wrote:", out)
    print("target:", args.target)
    print("proxy_parent:", parent.GetPath())
    print("proxy_name:", args.name)
    print("world_aabb_min:", mn)
    print("world_aabb_max:", mx)
    print("proxy_size_world:", size_world)
    print("proxy_size_local:", size_local)
    print("disabled_original_collision:", bool(args.disable_original_collision))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
