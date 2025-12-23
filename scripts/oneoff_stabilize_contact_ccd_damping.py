#!/usr/bin/env python3
"""One-off: stabilize contact by authoring CCD + damping on a rigid body.

Why this exists:
- In Isaac/PhysX, large "bounce" after contact often comes from deep penetration
  + solver depenetration impulses, not just restitution.
- Enabling CCD and adding damping typically reduces tunneling + energy blow-ups.

Notes:
- Some PhysX-specific schemas (PhysxSchema) may not be importable in this
  environment. We still author the expected namespaced attributes
  (e.g., physxRigidBody:enableCCD) with correct value types.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Iterable

from pxr import Sdf, Usd


def _ensure_parent_dir(path: str) -> None:
    Path(path).expanduser().resolve().parent.mkdir(parents=True, exist_ok=True)


def _set_or_create_attr(prim: Usd.Prim, name: str, type_name: Sdf.ValueTypeName, value) -> None:
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, type_name, custom=True)
    attr.Set(value)


def _as_list(values: Iterable[str]) -> list[str]:
    return [v for v in values if v]


def main() -> int:
    parser = argparse.ArgumentParser(description="Enable CCD + damping to reduce contact bouncing.")
    parser.add_argument("--input", required=True, help="Input USD")
    parser.add_argument("--output", required=True, help="Output USD")
    parser.add_argument(
        "--rigid-prim",
        default="/root/obj__13",
        help="Rigid body prim path (default: /root/obj__13)",
    )
    parser.add_argument(
        "--collider-prim",
        action="append",
        default=[],
        help="Optional collider prim(s) to author contactOffset/restOffset on.",
    )

    parser.add_argument("--enable-ccd", type=int, default=1, choices=[0, 1], help="Enable CCD (1) or disable (0)")
    parser.add_argument("--linear-damping", type=float, default=2.0)
    parser.add_argument("--angular-damping", type=float, default=5.0)
    parser.add_argument(
        "--max-depenetration-velocity",
        type=float,
        default=5.0,
        help="Clamp depenetration speed; helps avoid "
        "'explosive' separation when objects interpenetrate.",
    )
    parser.add_argument(
        "--contact-offset",
        type=float,
        default=0.005,
        help="Optional PhysX contactOffset for collider prim(s).",
    )
    parser.add_argument(
        "--rest-offset",
        type=float,
        default=0.0,
        help="Optional PhysX restOffset for collider prim(s).",
    )
    args = parser.parse_args()

    stage = Usd.Stage.Open(args.input)
    if not stage:
        raise RuntimeError(f"Failed to open: {args.input}")

    rigid = stage.GetPrimAtPath(args.rigid_prim)
    if not rigid or not rigid.IsValid():
        raise RuntimeError(f"Rigid prim not found: {args.rigid_prim}")

    # Generic USD physics damping attrs (UsdPhysics.RigidBodyAPI will interpret these).
    _set_or_create_attr(rigid, "physics:linearDamping", Sdf.ValueTypeNames.Float, float(args.linear_damping))
    _set_or_create_attr(rigid, "physics:angularDamping", Sdf.ValueTypeNames.Float, float(args.angular_damping))

    # PhysX-specific toggles (authored as namespaced attributes).
    _set_or_create_attr(rigid, "physxRigidBody:enableCCD", Sdf.ValueTypeNames.Bool, bool(args.enable_ccd))
    _set_or_create_attr(
        rigid,
        "physxRigidBody:maxDepenetrationVelocity",
        Sdf.ValueTypeNames.Float,
        float(args.max_depenetration_velocity),
    )

    collider_prims = _as_list(args.collider_prim)
    for collider_path in collider_prims:
        collider = stage.GetPrimAtPath(collider_path)
        if not collider or not collider.IsValid():
            raise RuntimeError(f"Collider prim not found: {collider_path}")
        _set_or_create_attr(
            collider,
            "physxCollision:contactOffset",
            Sdf.ValueTypeNames.Float,
            float(args.contact_offset),
        )
        _set_or_create_attr(
            collider,
            "physxCollision:restOffset",
            Sdf.ValueTypeNames.Float,
            float(args.rest_offset),
        )

    _ensure_parent_dir(args.output)
    stage.GetRootLayer().Export(args.output)
    print(f"Wrote: {args.output}")
    print(
        "Authored on rigid:",
        {
            "physics:linearDamping": args.linear_damping,
            "physics:angularDamping": args.angular_damping,
            "physxRigidBody:enableCCD": bool(args.enable_ccd),
            "physxRigidBody:maxDepenetrationVelocity": args.max_depenetration_velocity,
        },
    )
    if collider_prims:
        print(
            "Authored on colliders:",
            {
                "physxCollision:contactOffset": args.contact_offset,
                "physxCollision:restOffset": args.rest_offset,
                "collider_prims": collider_prims,
            },
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
