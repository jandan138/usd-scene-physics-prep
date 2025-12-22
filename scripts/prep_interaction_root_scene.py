#!/usr/bin/env python3
"""Prepare an arbitrary /root-structured USD for interaction physics.

This script is designed for datasets like SimBench GRSceneUSD where the stage root
is `/root` (lowercase) and does NOT follow the repository's `/Root/Meshes` layout.

Rules (current default for task10):
- `/root/room`         -> static collider only
- `/root/obj_table`    -> static collider only (table should not move)
- `/root/obj_*`        -> rigid body + collider (except obj_table)

Output is a new USD file with PhysX collider approximation + mesh-merge collision.

Run with Isaac Sim python wrapper, e.g.
  ./scripts/isaac_python.sh scripts/prep_interaction_root_scene.py \
    --input  /path/to/scene.usd \
    --output /path/to/scene_interaction_dynamic.usd
"""

from __future__ import annotations

import argparse
import os
import shutil
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Sequence, Set, Tuple

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics


SDF = "sdf"
CONVEX_HULL = "convexHull"
CONVEX_DECOMPOSITION = "convexDecomposition"
MESH_SIMPLIFICATION = "meshSimplification"
TRIANGLE_MESH = "none"


DEFAULT_EXCLUDE_PREFIXES = (
    "/root/room/Meshes/BaseAnimation/door",
)


def _prim_path_starts_with_any(prim: Usd.Prim, prefixes: Sequence[str]) -> bool:
    p = str(prim.GetPath())
    return any(p.startswith(prefix) for prefix in prefixes)


def _remove_collider_recursive(prim: Usd.Prim) -> None:
    # normal collision APIs
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        prim.RemoveAPI(UsdPhysics.CollisionAPI)
    if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)

    attr = prim.GetAttribute("physics:collisionEnabled")
    if attr:
        attr.Clear()
    attr = prim.GetAttribute("physics:approximation")
    if attr:
        attr.Clear()

    for child in prim.GetChildren():
        _remove_collider_recursive(child)


def _remove_rigidbody_recursive(prim: Usd.Prim) -> None:
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        prim.RemoveAPI(UsdPhysics.RigidBodyAPI)

    attr = prim.GetAttribute("physics:rigidBodyEnabled")
    if attr:
        attr.Clear()

    # joints: disable
    if prim.IsA(UsdPhysics.Joint):
        attr = prim.GetAttribute("physics:jointEnabled")
        if attr:
            attr.Set(False)

    for child in prim.GetChildren():
        _remove_rigidbody_recursive(child)


def _force_disable_physics_flags_recursive(prim: Usd.Prim) -> None:
    """Author explicit False on common physics enable flags.

    Important: Clear()/RemoveAPI() won't override stronger opinions coming from
    referenced layers. Setting these attrs to False in the output layer avoids
    PhysX trying to cook collisions for bad meshes.
    """

    a = prim.GetAttribute("physics:collisionEnabled")
    if a and a.Get() is True:
        a.Set(False)

    a = prim.GetAttribute("physics:rigidBodyEnabled")
    if a and a.Get() is True:
        a.Set(False)

    for child in prim.GetChildren():
        _force_disable_physics_flags_recursive(child)


def _clear_mass_properties(prim: Usd.Prim) -> None:
    """Clear authored mass properties so PhysX can auto-compute from colliders.

    Some datasets author invalid mass (e.g., negative) which triggers PhysX warnings.
    """

    if prim.HasAPI(UsdPhysics.MassAPI):
        prim.RemoveAPI(UsdPhysics.MassAPI)

    for attr_name in (
        "physics:mass",
        "physics:density",
        "physics:centerOfMass",
        "physics:diagonalInertia",
        "physics:principalAxes",
    ):
        a = prim.GetAttribute(attr_name)
        if a:
            a.Clear()


def _force_positive_mass(prim: Usd.Prim, mass_value: float = 1.0) -> None:
    """Ensure mass is positive to avoid PhysX 'negative mass' warnings."""

    api = UsdPhysics.MassAPI.Apply(prim)
    api.CreateMassAttr().Set(float(mass_value))
    # Provide a benign inertia tensor; PhysX may override at runtime.
    api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(1.0, 1.0, 1.0))


def _mesh_has_valid_geometry(mesh_prim: Usd.Prim) -> Tuple[bool, str]:
    if mesh_prim.GetTypeName() != "Mesh":
        return False, "not a Mesh"

    mesh = UsdGeom.Mesh(mesh_prim)
    points_attr = mesh.GetPointsAttr()
    if not points_attr:
        return False, "no points attr"
    try:
        points = points_attr.Get()
    except Exception:
        return False, "failed to read points"
    if not points or len(points) == 0:
        return False, "no points"

    fvc_attr = mesh.GetFaceVertexCountsAttr()
    fvi_attr = mesh.GetFaceVertexIndicesAttr()
    if not fvc_attr or not fvi_attr:
        return False, "missing topology attrs"

    try:
        fvc = fvc_attr.Get() or []
        fvi = fvi_attr.Get() or []
    except Exception:
        return False, "failed to read topology"

    try:
        count_sum = int(sum(int(x) for x in fvc))
        index_len = int(len(fvi))
    except Exception:
        return False, "invalid topology values"

    if count_sum != index_len:
        return False, f"inconsistent topology sum(counts)={count_sum} len(indices)={index_len}"
    if count_sum <= 0:
        return False, "empty topology"

    return True, "ok"


def _iter_descendant_meshes(
    root_prim: Usd.Prim,
    *,
    exclude_prefixes: Sequence[str],
    stats: dict,
    allow_untyped_payload_geometry: bool = False,
) -> List[Usd.Prim]:
    meshes: List[Usd.Prim] = []

    def recurse(p: Usd.Prim) -> None:
        if _prim_path_starts_with_any(p, exclude_prefixes):
            return
        if p.GetTypeName() == "Mesh":
            ok, reason = _mesh_has_valid_geometry(p)
            if ok:
                meshes.append(p)
            else:
                stats["skipped_meshes"].append({"path": str(p.GetPath()), "reason": reason})
        for ch in p.GetChildren():
            recurse(ch)

    recurse(root_prim)

    # Some datasets (e.g., GLB payloads) may appear as untyped prims in this
    # environment (no points / not a Mesh), but Isaac/Kit can still resolve the
    # payload at runtime. In that case, author the collision APIs/attrs on the
    # expected geometry prim paths so the object becomes draggable in Isaac.
    if allow_untyped_payload_geometry and not meshes:
        # Important: in some stages these placeholder prims exist as 'over'
        # prims (IsDefined() == False), which means GetChildren()/PrimRange()
        # won't enumerate them. We therefore probe well-known paths directly.
        if root_prim.HasPayload() and (not _prim_path_starts_with_any(root_prim, exclude_prefixes)):
            stage = root_prim.GetStage()
            base = str(root_prim.GetPath())
            # Probe well-known leaf mesh prim paths used by SimBench GLB payloads.
            # We prefer authoring colliders on leaf meshes (e.g. geometry_0,
            # geometry_01) and only fall back to the parent `/geometry_0` when
            # no leaf mesh prims are present. This avoids accidentally authoring
            # duplicate colliders on both parent and child, while still handling
            # payloads that expose only a single geometry prim.
            leaf_candidate_paths: List[str] = [
                base + "/geometry_0/geometry_0",
            ]
            # Common sibling meshes: geometry_01, geometry_02, ...
            leaf_candidate_paths.extend(
                base + f"/geometry_0/geometry_{i:02d}" for i in range(1, 10)
            )

            forced: List[Usd.Prim] = []
            for pth in leaf_candidate_paths:
                prim = stage.GetPrimAtPath(pth)
                if prim and prim.IsValid() and prim.GetTypeName() in {"", "Mesh"}:
                    forced.append(prim)

            if not forced:
                parent = stage.GetPrimAtPath(base + "/geometry_0")
                if parent and parent.IsValid() and parent.GetTypeName() in {"", "Mesh"}:
                    forced = [parent]

            if forced:
                stats.setdefault("forced_untyped_colliders", [])
                for p in forced:
                    stats["forced_untyped_colliders"].append(str(p.GetPath()))
                meshes.extend(forced)

    return meshes


def _set_collider_with_approx(prim: Usd.Prim, approx: str) -> None:
    if approx not in {SDF, CONVEX_HULL, CONVEX_DECOMPOSITION, MESH_SIMPLIFICATION, TRIANGLE_MESH}:
        raise ValueError(f"Unsupported collision approximation: {approx}")

    # In this Isaac Sim build, pxr.PhysxSchema isn't available. We still can set
    # collision via UsdPhysics and rely on the approximation token.
    # In some environments, GLB payload geometry resolves as untyped prims.
    # Authoring the applied APIs/attrs on these prims can still help Isaac once
    # the payload is resolved.
    if prim.GetTypeName() not in {"Mesh", ""}:
        return

    collider = UsdPhysics.CollisionAPI.Apply(prim)
    mesh_collider = UsdPhysics.MeshCollisionAPI.Apply(prim)
    mesh_collider.CreateApproximationAttr(approx)
    collider.GetCollisionEnabledAttr().Set(True)


def _set_rigidbody(prim: Usd.Prim, enabled: bool = True) -> None:
    # Prefer rigid bodies on Xforms; for GLB payload roots we may only see an
    # untyped prim (typeName == ''). Avoid applying on Mesh.
    if prim.GetTypeName() == "Mesh":
        return
    if prim.GetTypeName() not in {"Xform", ""}:
        return
    api = UsdPhysics.RigidBodyAPI.Apply(prim)
    api.GetRigidBodyEnabledAttr().Set(bool(enabled))


def _bind_static(
    prim: Usd.Prim,
    *,
    approx: str,
    exclude_prefixes: Sequence[str],
    stats: dict,
) -> None:
    for mesh in _iter_descendant_meshes(
        prim,
        exclude_prefixes=exclude_prefixes,
        stats=stats,
        allow_untyped_payload_geometry=False,
    ):
        _set_collider_with_approx(mesh, approx)


def _bind_rigid(
    prim: Usd.Prim,
    *,
    approx: str,
    exclude_prefixes: Sequence[str],
    stats: dict,
) -> None:
    _clear_mass_properties(prim)
    _set_rigidbody(prim, enabled=True)
    _force_positive_mass(prim, mass_value=1.0)
    for mesh in _iter_descendant_meshes(
        prim,
        exclude_prefixes=exclude_prefixes,
        stats=stats,
        allow_untyped_payload_geometry=True,
    ):
        _set_collider_with_approx(mesh, approx)


@dataclass(frozen=True)
class Rules:
    static_paths: Set[str]
    dynamic_name_prefixes: Sequence[str] = ("obj_", "__")
    dynamic_names_exact: Set[str] = frozenset({"_"})
    root_path: str = "/root"
    exclude_prefixes: Sequence[str] = DEFAULT_EXCLUDE_PREFIXES


def _iter_root_children(stage: Usd.Stage, root_path: str) -> Iterable[Usd.Prim]:
    root = stage.GetPrimAtPath(root_path)
    if not root or not root.IsValid():
        raise RuntimeError(f"Stage does not contain root prim at {root_path}")
    return root.GetChildren()


def apply_rules(stage: Usd.Stage, rules: Rules, approx_static: str, approx_dynamic: str) -> dict:
    root = stage.GetPrimAtPath(rules.root_path)
    if not root or not root.IsValid():
        raise RuntimeError(f"Stage missing root prim: {rules.root_path}")

    # Clean existing physics under /root only (avoid touching prototypes outside this subtree)
    _remove_collider_recursive(root)
    _remove_rigidbody_recursive(root)
    _force_disable_physics_flags_recursive(root)

    stats = {
        "static_bound": [],
        "dynamic_bound": [],
        "skipped": [],
        "skipped_meshes": [],
        "exclude_prefixes": list(rules.exclude_prefixes),
    }

    # Bind explicit static paths
    for p in sorted(rules.static_paths):
        prim = stage.GetPrimAtPath(p)
        if not prim or not prim.IsValid():
            stats["skipped"].append({"path": p, "reason": "missing"})
            continue
        _bind_static(prim, approx=approx_static, exclude_prefixes=rules.exclude_prefixes, stats=stats)
        stats["static_bound"].append(p)

    # Bind dynamic top-level prims as rigid (excluding explicit static)
    for ch in _iter_root_children(stage, rules.root_path):
        name = ch.GetName()
        path = str(ch.GetPath())
        if path in rules.static_paths:
            continue
        if (name in rules.dynamic_names_exact) or any(name.startswith(pfx) for pfx in rules.dynamic_name_prefixes):
            _bind_rigid(ch, approx=approx_dynamic, exclude_prefixes=rules.exclude_prefixes, stats=stats)
            stats["dynamic_bound"].append(path)

    stage.GetRootLayer().Save()
    return stats


def _default_output_path(input_path: Path) -> Path:
    return input_path.with_name(input_path.stem + "_interaction_dynamic" + input_path.suffix)


def _ensure_parent_dir(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Input USD path")
    parser.add_argument("--output", default=None, help="Output USD path")

    parser.add_argument("--root", default="/root", help="Root prim path (default: /root)")
    parser.add_argument(
        "--static",
        action="append",
        default=[],
        help="Prim paths to treat as static collider-only (repeatable)",
    )

    parser.add_argument(
        "--exclude-prefix",
        action="append",
        default=[],
        help="Exclude prim subtrees from collider binding (repeatable).",
    )

    parser.add_argument(
        "--dynamic-prefix",
        action="append",
        default=[],
        help="Top-level prim name prefix to treat as dynamic rigid body (repeatable). Default: obj_ and __",
    )

    parser.add_argument(
        "--dynamic-name",
        action="append",
        default=[],
        help="Top-level prim name (exact match) to treat as dynamic rigid body (repeatable). Default: '_'",
    )

    parser.add_argument("--approx-static", default=CONVEX_DECOMPOSITION)
    parser.add_argument("--approx-dynamic", default=CONVEX_DECOMPOSITION)

    args = parser.parse_args()

    input_path = Path(args.input)
    if not input_path.exists():
        raise FileNotFoundError(str(input_path))

    output_path = Path(args.output) if args.output else _default_output_path(input_path)

    # Work on a copy so we don't mutate the original
    _ensure_parent_dir(output_path)
    shutil.copy2(input_path, output_path)

    stage = Usd.Stage.Open(str(output_path))
    if stage is None:
        raise RuntimeError(f"Failed to open stage: {output_path}")

    exclude_prefixes = list(DEFAULT_EXCLUDE_PREFIXES)
    if args.exclude_prefix:
        exclude_prefixes.extend(args.exclude_prefix)

    dynamic_prefixes = tuple(args.dynamic_prefix) if args.dynamic_prefix else ("obj_", "__")
    dynamic_names_exact = set(args.dynamic_name) if args.dynamic_name else {"_"}

    rules = Rules(
        static_paths=set(args.static),
        root_path=args.root,
        exclude_prefixes=tuple(exclude_prefixes),
        dynamic_name_prefixes=dynamic_prefixes,
        dynamic_names_exact=frozenset(dynamic_names_exact),
    )

    stats = apply_rules(stage, rules, approx_static=args.approx_static, approx_dynamic=args.approx_dynamic)

    print("=== prep_interaction_root_scene.py result ===")
    print(f"input : {input_path}")
    print(f"output: {output_path}")
    print(f"static_bound ({len(stats['static_bound'])}):")
    for p in stats["static_bound"]:
        print("  -", p)
    print(f"dynamic_bound ({len(stats['dynamic_bound'])}):")
    for p in stats["dynamic_bound"][:50]:
        print("  -", p)
    if len(stats["dynamic_bound"]) > 50:
        print(f"  ... and {len(stats['dynamic_bound']) - 50} more")

    if stats["skipped"]:
        print("skipped:")
        for item in stats["skipped"]:
            print("  -", item)

    if stats["exclude_prefixes"]:
        print("exclude_prefixes:")
        for p in stats["exclude_prefixes"]:
            print("  -", p)

    if stats["skipped_meshes"]:
        print(f"skipped_meshes ({len(stats['skipped_meshes'])}) [showing up to 20]:")
        for item in stats["skipped_meshes"][:20]:
            print("  -", item)

    if stats.get("forced_untyped_colliders"):
        forced = stats["forced_untyped_colliders"]
        print(f"forced_untyped_colliders ({len(forced)}) [showing up to 20]:")
        for p in forced[:20]:
            print("  -", p)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
