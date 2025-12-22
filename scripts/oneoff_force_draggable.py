#!/usr/bin/env python3
"""One-off: force a prim to be draggable in Isaac Sim.

Why this exists
- Some scenes reference geometry via GLB payloads. In environments where USD
  cannot load GLB, prims appear *untyped* and meshes have no authored points.
- Our normal preprocess script intentionally skips collider binding for meshes
  without valid points to avoid PhysX cooking errors.
- However, if Isaac/Kit *can* resolve the payload at runtime, we still want to
  author the physics APIs/attrs on the target prim paths, so the object becomes
  draggable once geometry loads.

What it does
- Copies input USD to output path.
- Applies RigidBodyAPI + MassAPI to --rigid-prim and authors
  `physics:rigidBodyEnabled = True`.
- Applies CollisionAPI + MeshCollisionAPI to --collider-mesh and authors
  `physics:collisionEnabled = True` + `physics:approximation = <token>`.

Note
- If the mesh truly has no points in Isaac as well, PhysX may still warn that
  collision can't be created.
"""

from __future__ import annotations

import argparse
import shutil

from pxr import Gf, Usd, UsdPhysics


def _force_rigidbody(prim: Usd.Prim, *, mass: float) -> None:
    # Apply API even if prim is untyped; this authors the schema + attrs.
    rb = UsdPhysics.RigidBodyAPI.Apply(prim)
    rb.GetRigidBodyEnabledAttr().Set(True)

    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.CreateMassAttr().Set(float(mass))
    mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))


def _force_mesh_collider(mesh_prim: Usd.Prim, *, approx: str) -> None:
    # Apply APIs even if prim is untyped; if it resolves to a Mesh in Isaac,
    # these authored attrs will take effect.
    collision = UsdPhysics.CollisionAPI.Apply(mesh_prim)
    collision.GetCollisionEnabledAttr().Set(True)

    mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(mesh_prim)
    mesh_collision.CreateApproximationAttr(approx)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--rigid-prim", required=True, help="Prim path to become a rigid body (e.g. /root/__05)")
    parser.add_argument(
        "--collider-mesh",
        required=True,
        help="Mesh prim path to receive CollisionAPI/MeshCollisionAPI (e.g. /root/__05/geometry_0/geometry_0)",
    )
    parser.add_argument(
        "--approx",
        default="convexHull",
        choices=["sdf", "convexHull", "convexDecomposition", "meshSimplification", "none"],
        help="UsdPhysics mesh collision approximation token.",
    )
    parser.add_argument("--mass", type=float, default=1.0)
    args = parser.parse_args()

    shutil.copy2(args.input, args.output)

    stage = Usd.Stage.Open(args.output)
    if stage is None:
        raise RuntimeError(f"Failed to open output stage: {args.output}")

    rigid = stage.GetPrimAtPath(args.rigid_prim)
    if not rigid or not rigid.IsValid():
        raise RuntimeError(f"Missing rigid prim: {args.rigid_prim}")

    mesh = stage.GetPrimAtPath(args.collider_mesh)
    if not mesh or not mesh.IsValid():
        raise RuntimeError(f"Missing collider mesh prim: {args.collider_mesh}")

    _force_rigidbody(rigid, mass=args.mass)
    _force_mesh_collider(mesh, approx=args.approx)

    stage.GetRootLayer().Save()

    print("[oneoff_force_draggable] OK")
    print("  input :", args.input)
    print("  output:", args.output)
    print("  rigid :", args.rigid_prim)
    print("  mesh  :", args.collider_mesh)
    print("  approx:", args.approx)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
