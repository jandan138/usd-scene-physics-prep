#!/usr/bin/env python3
"""Add multiple box proxy colliders for a single mesh (e.g., spoon handle + bowl).

Motivation
- A single mesh collider with convexDecomposition can create many small convex
  pieces and unstable contact normals, especially against thin/static surfaces.
- A small number of simple primitive colliders (Boxes/Capsules) is often more
  stable in PhysX.

What it does
- Computes the mesh's axis-aligned bound in the rigid prim's local space.
- Splits the bound along the longest axis into N segments (default 2).
- Creates one Cube collider per segment under a proxy container Xform.
- Optionally disables collision on the original mesh.
- Optionally binds a physics material via material:binding:physics.

Usage (example)
  ./scripts/isaac_python.sh scripts/oneoff_add_spoon_multi_box_proxy.py \
    --input  /path/scene.usd \
    --output /path/scene_spoonproxy.usd \
    --rigid-prim /root/obj__13 \
    --mesh-prim  /root/obj__13/geometry_0/geometry_0 \
    --boxes 2 \
    --split-ratio 0.68 \
    --disable-original-collision \
    --bind-physics-material /_PhysicsMaterials/pmat_no_bounce
"""

from __future__ import annotations

import argparse
import os
from typing import List, Tuple

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


def _derive_default_output(input_path: str) -> str:
    base, ext = os.path.splitext(input_path)
    if ext.lower() not in (".usd", ".usda", ".usdc"):
        ext = ".usd"
    return base + "_spoonproxy" + ext


def _compute_relative_aabb(stage: Usd.Stage, prim: Usd.Prim, relative_to: Usd.Prim) -> Tuple[Gf.Vec3d, Gf.Vec3d]:
    time = Usd.TimeCode.Default()
    purposes = [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy]
    bbox_cache = UsdGeom.BBoxCache(time, purposes, useExtentsHint=True)
    bbox = bbox_cache.ComputeRelativeBound(prim, relative_to)
    r = bbox.ComputeAlignedRange()
    if r.IsEmpty():
        raise RuntimeError(f"Empty relative AABB for prim: {prim.GetPath()} relative_to: {relative_to.GetPath()}")
    return r.GetMin(), r.GetMax()


def _axis_name(axis: int) -> str:
    return ("X", "Y", "Z")[axis]


def _ensure_cube_collider(
    stage: Usd.Stage,
    parent: Usd.Prim,
    name: str,
    center_local: Gf.Vec3d,
    size_local: Gf.Vec3d,
    bind_physics_material: str | None,
    contact_offset: float | None,
    rest_offset: float | None,
) -> Usd.Prim:
    xform_path = parent.GetPath().AppendChild(name)
    xprim = stage.DefinePrim(xform_path, "Xform")
    xf = UsdGeom.Xformable(xprim)
    xf.ClearXformOpOrder()
    t_op = xf.AddTranslateOp()
    s_op = xf.AddScaleOp()
    t_op.Set(Gf.Vec3d(float(center_local[0]), float(center_local[1]), float(center_local[2])))
    s_op.Set(Gf.Vec3f(float(size_local[0]), float(size_local[1]), float(size_local[2])))

    cube_path = xform_path.AppendChild("collider")
    cube_prim = stage.DefinePrim(cube_path, "Cube")
    cube = UsdGeom.Cube(cube_prim)
    cube.CreateSizeAttr(1.0)

    UsdPhysics.CollisionAPI.Apply(cube_prim)
    cube_prim.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool).Set(True)

    if contact_offset is not None:
        cube_prim.CreateAttribute("physxCollision:contactOffset", Sdf.ValueTypeNames.Float).Set(float(contact_offset))
    if rest_offset is not None:
        cube_prim.CreateAttribute("physxCollision:restOffset", Sdf.ValueTypeNames.Float).Set(float(rest_offset))

    if bind_physics_material:
        rel = cube_prim.CreateRelationship("material:binding:physics", custom=True)
        rel.SetTargets([Sdf.Path(bind_physics_material)])

    return cube_prim


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", default=None)
    ap.add_argument("--rigid-prim", default="/root/obj__13")
    ap.add_argument("--mesh-prim", default="/root/obj__13/geometry_0/geometry_0")
    ap.add_argument("--proxy-name", default="_spoon_proxy")
    ap.add_argument("--boxes", type=int, default=2, help="Number of boxes (>=2 recommended)")
    ap.add_argument(
        "--split-ratio",
        type=float,
        default=0.68,
        help="Only used when boxes=2: fraction of length for box0 (rest for box1).",
    )
    ap.add_argument(
        "--flip",
        action="store_true",
        help="Swap which end gets the longer segment when boxes=2.",
    )
    ap.add_argument(
        "--expand-perp",
        type=float,
        default=1.12,
        help="Scale factor applied to the non-length axes (makes contact more robust).",
    )
    ap.add_argument(
        "--expand-length",
        type=float,
        default=1.04,
        help="Scale factor applied to the length axis.",
    )
    ap.add_argument(
        "--min-thickness",
        type=float,
        default=0.008,
        help="Minimum box size on the smallest axis in rigid-local units.",
    )
    ap.add_argument("--disable-original-collision", action="store_true")
    ap.add_argument(
        "--bind-physics-material",
        default="/_PhysicsMaterials/pmat_no_bounce",
        help="Target physics material prim path. Use '' to skip binding.",
    )
    ap.add_argument("--contact-offset", type=float, default=0.01)
    ap.add_argument("--rest-offset", type=float, default=0.0)
    args = ap.parse_args()

    if args.boxes < 1:
        raise ValueError("--boxes must be >= 1")
    if args.boxes == 1:
        # It still works, but user asked for handle+head (multi-box).
        raise ValueError("--boxes=1 not supported here; use >=2")
    if not (0.05 <= float(args.split_ratio) <= 0.95):
        raise ValueError("--split-ratio should be in [0.05, 0.95]")

    out = args.output or _derive_default_output(args.input)
    stage = Usd.Stage.Open(args.input)
    if stage is None:
        raise RuntimeError(f"Failed to open USD: {args.input}")

    rigid = stage.GetPrimAtPath(Sdf.Path(args.rigid_prim))
    if not rigid or not rigid.IsValid():
        raise RuntimeError(f"Rigid prim not found: {args.rigid_prim}")
    mesh = stage.GetPrimAtPath(Sdf.Path(args.mesh_prim))
    if not mesh or not mesh.IsValid():
        raise RuntimeError(f"Mesh prim not found: {args.mesh_prim}")

    # Compute mesh bound in rigid-local space (so proxy stays aligned even if rigid moves/rotates).
    mn, mx = _compute_relative_aabb(stage, mesh, rigid)
    size = mx - mn
    extents = [float(size[0]), float(size[1]), float(size[2])]
    length_axis = max(range(3), key=lambda i: extents[i])
    length = extents[length_axis]
    if length <= 0:
        raise RuntimeError("Degenerate bound; cannot build proxy")

    proxy_container = stage.DefinePrim(rigid.GetPath().AppendChild(args.proxy_name), "Xform")

    bind_mat = str(args.bind_physics_material) if str(args.bind_physics_material).strip() else None

    # Build segments along the length axis.
    cuts: List[float] = []
    if args.boxes == 2:
        a = float(args.split_ratio)
        # box0 length fraction a, box1 is (1-a)
        if args.flip:
            a = 1.0 - a
        cuts = [float(mn[length_axis]), float(mn[length_axis] + a * length), float(mx[length_axis])]
    else:
        # Uniform split for N>2.
        cuts = [float(mn[length_axis] + (i * length / args.boxes)) for i in range(args.boxes + 1)]

    created = []
    for i in range(args.boxes):
        seg_mn = Gf.Vec3d(float(mn[0]), float(mn[1]), float(mn[2]))
        seg_mx = Gf.Vec3d(float(mx[0]), float(mx[1]), float(mx[2]))
        seg_mn[length_axis] = float(cuts[i])
        seg_mx[length_axis] = float(cuts[i + 1])

        center = (seg_mn + seg_mx) * 0.5
        seg_size = seg_mx - seg_mn

        # Expand slightly for robustness.
        expanded = [float(seg_size[0]), float(seg_size[1]), float(seg_size[2])]
        for ax in range(3):
            if ax == length_axis:
                expanded[ax] *= float(args.expand_length)
            else:
                expanded[ax] *= float(args.expand_perp)

        # Enforce minimum thickness to avoid thin slivers.
        for ax in range(3):
            if expanded[ax] < float(args.min_thickness):
                expanded[ax] = float(args.min_thickness)

        name = f"box_{i}"
        cube_prim = _ensure_cube_collider(
            stage,
            proxy_container,
            name,
            center_local=center,
            size_local=Gf.Vec3d(expanded[0], expanded[1], expanded[2]),
            bind_physics_material=bind_mat,
            contact_offset=float(args.contact_offset) if args.contact_offset is not None else None,
            rest_offset=float(args.rest_offset) if args.rest_offset is not None else None,
        )
        created.append(cube_prim.GetPath().pathString)

    if args.disable_original_collision:
        mesh.CreateAttribute("physics:collisionEnabled", Sdf.ValueTypeNames.Bool).Set(False)

    stage.GetRootLayer().Export(out)
    print("Wrote:", out)
    print("rigid:", rigid.GetPath())
    print("mesh:", mesh.GetPath())
    print("relative_aabb_min:", mn)
    print("relative_aabb_max:", mx)
    print("length_axis:", _axis_name(length_axis), "length:", length)
    print("boxes:", args.boxes, "created:", created)
    print("disabled_original_collision:", bool(args.disable_original_collision))
    print("bind_physics_material:", bind_mat)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
