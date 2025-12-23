#!/usr/bin/env python3
"""One-off fixer for invalid / unstable MassAPI values.

Why
- Some scenes contain authored invalid MassAPI values such as:
  - physics:centerOfMass = (-inf, -inf, -inf)
  - physics:principalAxes = (0,0,0,0)
These can destabilize contacts (penetrate then pop).

What this does
- For a target rigid prim (default: /root/obj__13), ensures MassAPI exists.
- Forces a sane mass value (default: 1.0).
- Clears potentially-invalid derived fields so runtime can derive/ignore them:
  - physics:centerOfMass
  - physics:principalAxes
  - physics:diagonalInertia
  - physics:density (optional; default clears)

Usage
  ./scripts/isaac_python.sh scripts/oneoff_fix_mass_invalid_values.py \
    --input  /path/to/scene.usd \
    --output /path/to/scene_fixed.usd \
    --prim   /root/obj__13 \
    --mass   1.0

If --output is omitted, writes next to input as *_massfixed.usd.
"""

from __future__ import annotations

import argparse
import os
from typing import Optional

from pxr import Gf, Sdf, Usd, UsdPhysics


def _derive_default_output(input_path: str) -> str:
    base, ext = os.path.splitext(input_path)
    if ext.lower() not in (".usd", ".usda", ".usdc", ".usdz"):
        ext = ".usd"
    return base + "_massfixed" + ext


def fix_mass_values(
    stage: Usd.Stage,
    prim_path: str,
    mass_value: float,
    keep_density: bool,
    override_derived: bool,
) -> None:
    prim = stage.GetPrimAtPath(Sdf.Path(prim_path))
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Prim not found: {prim_path}")

    # Ensure MassAPI is applied.
    mass_api = UsdPhysics.MassAPI.Apply(prim)

    # Force a reasonable mass.
    mass_api.CreateMassAttr(mass_value)

    # IMPORTANT:
    # Many bad scenes have invalid values authored in weaker layers.
    # Clearing here would just expose those weaker opinions again.
    # So we override with sane defaults.
    if override_derived:
        mass_api.CreateCenterOfMassAttr(Gf.Vec3f(0.0, 0.0, 0.0))
        # principalAxes is a quaternion (x, y, z, w). Identity = (0,0,0,1)
        mass_api.CreatePrincipalAxesAttr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        # Diagonal inertia must be positive.
        mass_api.CreateDiagonalInertiaAttr(Gf.Vec3f(1.0, 1.0, 1.0))

    if not keep_density:
        # Setting density to 0.0 is safer than clearing when a weaker invalid value exists.
        mass_api.CreateDensityAttr(0.0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True)
    ap.add_argument("--output", default=None)
    ap.add_argument("--prim", default="/root/obj__13")
    ap.add_argument("--mass", type=float, default=1.0)
    ap.add_argument("--keep-density", action="store_true", help="Do not override physics:density")
    ap.add_argument(
        "--no-override-derived",
        action="store_true",
        help="Do not override centerOfMass/principalAxes/diagonalInertia",
    )
    args = ap.parse_args()

    out = args.output or _derive_default_output(args.input)

    stage = Usd.Stage.Open(args.input)
    if stage is None:
        raise RuntimeError(f"Failed to open USD: {args.input}")

    fix_mass_values(
        stage,
        args.prim,
        args.mass,
        keep_density=args.keep_density,
        override_derived=(not args.no_override_derived),
    )

    # Save to new root layer file.
    root_layer = stage.GetRootLayer()
    root_layer.Export(out)
    print("Wrote:", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
