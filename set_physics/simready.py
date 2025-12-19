"""One-shot sim-ready USD generator.

Goal
- Input: a single scene USD (expects sibling Materials/ with .mdl as in this repo's datasets)
- Output: a physics-prepped USD that is ready to load in Isaac Sim, plus its referenced assets

This module is intended to be executed with Isaac Sim's python (Linux):
  ./python.sh -m set_physics.simready --input-usd ...

Notes
- Uses existing `parse_scene` splitter to produce a normalized target structure.
- Applies PhysX schemas via pxr/PhysxSchema, and (optionally) semantic labels via omni.*.
"""

from __future__ import annotations

import argparse
import os
import shutil
from dataclasses import dataclass
from typing import List, Optional, Set


def _abspath(path: str) -> str:
    return os.path.abspath(os.path.expanduser(path))


def _ensure_dir(path: str) -> None:
    os.makedirs(path, exist_ok=True)


def _copy_file(src: str, dst: str) -> None:
    _ensure_dir(os.path.dirname(dst))
    shutil.copyfile(src, dst)


def _derive_scene_id(input_usd: str) -> str:
    parent = os.path.basename(os.path.dirname(input_usd))
    base = os.path.splitext(os.path.basename(input_usd))[0]
    # Prefer a folder-ish id when available (matches repo layout), fallback to file stem.
    return parent if parent and parent not in (".", os.sep) else base


@dataclass(frozen=True)
class SimReadyPaths:
    out_root: str
    scene_id: str
    scene_dir: str
    normalized_scene_usd: str


def normalize_scene_to_target(
    *,
    input_usd: str,
    out_root: str,
    scene_id: Optional[str] = None,
) -> SimReadyPaths:
    """Run the repo splitter to create target-like structure under out_root."""

    from set_physics.pxr_utils.data_clean import parse_scene

    input_usd = _abspath(input_usd)
    out_root = _abspath(out_root)
    if scene_id is None:
        scene_id = _derive_scene_id(input_usd)

    # parse_scene outputs to: out_root/scenes/<scene_id>/<original_filename>
    parse_scene(input_usd, out_root, scene_id)

    scene_dir = os.path.join(out_root, "scenes", scene_id)
    normalized_scene_usd = os.path.join(scene_dir, os.path.basename(input_usd))
    if not os.path.isfile(normalized_scene_usd):
        # parse_scene preserves filename; if input was start_result_fix/new, try both.
        candidates = [
            os.path.join(scene_dir, "start_result_fix.usd"),
            os.path.join(scene_dir, "start_result_new.usd"),
        ]
        for c in candidates:
            if os.path.isfile(c):
                normalized_scene_usd = c
                break

    return SimReadyPaths(
        out_root=out_root,
        scene_id=scene_id,
        scene_dir=scene_dir,
        normalized_scene_usd=normalized_scene_usd,
    )


# ----------------------- physics helpers (pxr/PhysxSchema) -----------------------

SDF = "sdf"
CONVEX_HULL = "convexHull"
CONVEX_DECOMPOSITION = "convexDecomposition"
MESH_SIMPLIFICATION = "meshSimplification"
TRIANGLE_MESH = "none"  # triangle mesh


def _get_leaf_mesh_paths(prim) -> Set[object]:
    # Use `object` to avoid hard dependency on pxr types in editor environments.
    leaf_meshes: Set[object] = set()

    def recurse(p):
        children = p.GetChildren()
        if children:
            for ch in children:
                recurse(ch)
            return
        if p.GetTypeName() == "Mesh":
            leaf_meshes.add(p.GetPath())

    recurse(prim)
    return leaf_meshes


def _transform_to_trs(prim) -> None:
    """Convert xformOp:transform to translate/orient/scale where present."""

    from pxr import Gf, Sdf

    if prim.HasAttribute("xformOpOrder"):
        order = prim.GetAttribute("xformOpOrder").Get()
        if order is None:
            return
        if "xformOp:transform" in order:
            tf_m = prim.GetAttribute("xformOp:transform").Get()
            tf = Gf.Transform()
            tf.SetMatrix(tf_m)

            translation = tf.GetTranslation()
            orientation = Gf.Quatf(tf.GetRotation().GetQuat())
            scale = tf.GetScale()

            prim.CreateAttribute("xformOp:translate", Sdf.ValueTypeNames.Double3, custom=False).Set(translation)
            prim.CreateAttribute("xformOp:orient", Sdf.ValueTypeNames.Quatf, custom=False).Set(orientation)
            prim.CreateAttribute("xformOp:scale", Sdf.ValueTypeNames.Double3, custom=False).Set(scale)
            prim.GetAttribute("xformOpOrder").Set(["xformOp:translate", "xformOp:orient", "xformOp:scale"])

    for child in prim.GetChildren():
        _transform_to_trs(child)


def _remove_collider_recursive(prim) -> None:
    from pxr import Usd, UsdPhysics, PhysxSchema

    # normal collision APIs
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        prim.RemoveAPI(UsdPhysics.CollisionAPI)
    if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
        prim.RemoveAPI(UsdPhysics.MeshCollisionAPI)
    if prim.GetAttribute("physics:collisionEnabled"):
        prim.GetAttribute("physics:collisionEnabled").Clear()
    if prim.GetAttribute("physics:approximation"):
        prim.GetAttribute("physics:approximation").Clear()

    # mesh merge collision API + collection
    if prim.HasAPI(PhysxSchema.PhysxMeshMergeCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxMeshMergeCollisionAPI)
    collection_api = Usd.CollectionAPI.GetCollection(prim, "collection:collisionmeshes")
    if collection_api:
        collection_api.ResetCollection()

    # approximation-specific APIs
    if prim.HasAPI(PhysxSchema.PhysxConvexDecompositionCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxConvexDecompositionCollisionAPI)
    if prim.GetAttribute("physxConvexDecompositionCollision:hullVertexLimit"):
        prim.GetAttribute("physxConvexDecompositionCollision:hullVertexLimit").Clear()
    if prim.GetAttribute("physxConvexDecompositionCollision:maxConvexHulls"):
        prim.GetAttribute("physxConvexDecompositionCollision:maxConvexHulls").Clear()

    if prim.HasAPI(PhysxSchema.PhysxConvexHullCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxConvexHullCollisionAPI)
    if prim.GetAttribute("physxConvexHullCollision:hullVertexLimit"):
        prim.GetAttribute("physxConvexHullCollision:hullVertexLimit").Clear()

    if prim.HasAPI(PhysxSchema.PhysxSDFMeshCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxSDFMeshCollisionAPI)
    if prim.GetAttribute("physxSDFMeshCollision:sdfResolution"):
        prim.GetAttribute("physxSDFMeshCollision:sdfResolution").Clear()

    if prim.HasAPI(PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI)
    if prim.HasAPI(PhysxSchema.PhysxTriangleMeshCollisionAPI):
        prim.RemoveAPI(PhysxSchema.PhysxTriangleMeshCollisionAPI)

    for child in prim.GetChildren():
        _remove_collider_recursive(child)


def _remove_rigidbody_recursive(prim) -> None:
    from pxr import UsdPhysics

    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
    if prim.IsA(UsdPhysics.Joint):
        attr = prim.GetAttribute("physics:jointEnabled")
        if attr:
            attr.Set(False)

    for child in prim.GetChildren():
        _remove_rigidbody_recursive(child)


def _set_mesh_merge_collision(prim, includes, excludes) -> None:
    from pxr import PhysxSchema

    if prim.GetTypeName() in ("Xform", "Mesh"):
        api = PhysxSchema.PhysxMeshMergeCollisionAPI.Apply(prim)
        coll = api.GetCollisionMeshesCollectionAPI()
        coll.GetIncludesRel().SetTargets(list(includes))
        coll.GetExcludesRel().SetTargets(list(excludes))


def _set_rigidbody(prim, enabled: bool = True) -> None:
    from pxr import Gf, Sdf, UsdPhysics

    if prim.GetTypeName() in ("Xform", "Mesh"):
        rb = UsdPhysics.RigidBodyAPI.Apply(prim)
        rb.GetRigidBodyEnabledAttr().Set(bool(enabled))

        # Ensure a xformOp:transform exists so we can consistently convert to TRS.
        if not prim.HasAttribute("xformOp:transform"):
            prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray, custom=False).Set(["xformOp:transform"])
            tf_attr = prim.CreateAttribute("xformOp:transform", Sdf.ValueTypeNames.Matrix4d, custom=False)
            tf_attr.Set(Gf.Matrix4d(1.0))

        _transform_to_trs(prim)


def _set_collider_with_approx(prim, approx: str) -> None:
    from pxr import UsdPhysics, PhysxSchema

    if approx not in {SDF, CONVEX_HULL, CONVEX_DECOMPOSITION, MESH_SIMPLIFICATION, TRIANGLE_MESH}:
        raise ValueError(f"Unsupported collision approximation: {approx}")

    if prim.GetTypeName() in ("Xform", "Mesh"):
        collider = UsdPhysics.CollisionAPI.Apply(prim)
        mesh_collider = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mesh_collider.CreateApproximationAttr(approx)
        collider.GetCollisionEnabledAttr().Set(True)

        if approx == SDF:
            physx = PhysxSchema.PhysxSDFMeshCollisionAPI.Apply(prim)
            physx.CreateSdfResolutionAttr().Set(256)
        elif approx == CONVEX_HULL:
            physx = PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim)
            physx.CreateHullVertexLimitAttr().Set(64)
        elif approx == CONVEX_DECOMPOSITION:
            physx = PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim)
            physx.CreateHullVertexLimitAttr().Set(64)
            physx.CreateMaxConvexHullsAttr().Set(max(1, len(_get_leaf_mesh_paths(prim))))
        elif approx == MESH_SIMPLIFICATION:
            PhysxSchema.PhysxTriangleMeshSimplificationCollisionAPI.Apply(prim)
        elif approx == TRIANGLE_MESH:
            PhysxSchema.PhysxTriangleMeshCollisionAPI.Apply(prim)


def _bind_merged_mesh_static(prim, approx: str) -> None:
    _set_collider_with_approx(prim, approx)
    _set_mesh_merge_collision(prim, includes=_get_leaf_mesh_paths(prim), excludes=[])


def _bind_merged_mesh_rigid(prim, approx: str) -> None:
    _set_rigidbody(prim, enabled=True)
    _set_collider_with_approx(prim, approx)
    _set_mesh_merge_collision(prim, includes=_get_leaf_mesh_paths(prim), excludes=[])


def _get_all_joints(instance_prim) -> List[object]:
    from pxr import UsdPhysics

    joints: List[object] = []

    def recurse(p):
        for ch in p.GetChildren():
            if ch.IsA(UsdPhysics.Joint):
                joints.append(UsdPhysics.Joint(ch))
            recurse(ch)

    recurse(instance_prim)
    return joints


def _get_joint_connected_body_paths(joint) -> List[object]:
    body0_paths = list(joint.GetBody0Rel().GetTargets())
    body1_paths = list(joint.GetBody1Rel().GetTargets())
    return body0_paths + body1_paths


# ----------------------- processors -----------------------

DEFAULT_PICKABLE_CATEGORIES = {
    "basket",
    "pan",
    "pot",
    "bottle",
    "laptop",
    "mouse",
    "musical_instrument",
    "toy",
    "keyboard",
    "telephone",
    "cup",
    "tray",
    "lamp",
    "box",
    "plant",
}


def process_interaction_physics(scene_usd: str, out_usd: str, *, headless: bool = True) -> None:
    """Create a dynamic sim-ready scene for interaction tasks."""

    # Isaac Sim bootstrap (kept inside function to avoid side effects when imported).
    from isaacsim import SimulationApp

    kit = SimulationApp({"sync_loads": True, "headless": bool(headless), "renderer": "RayTracedLighting"})
    try:
        from pxr import Usd

        _copy_file(scene_usd, out_usd)
        stage = Usd.Stage.Open(out_usd)
        meshes = stage.GetPrimAtPath("/Root/Meshes")
        if not meshes or not meshes.IsValid():
            raise RuntimeError("Expected /Root/Meshes in scene; did you pass a normalized target scene?")

        for scope in meshes.GetChildren():
            scope_name = str(scope.GetName())
            for cate in scope.GetChildren():
                cate_name = str(cate.GetName())
                pickable = cate_name.lower() in DEFAULT_PICKABLE_CATEGORIES
                for inst in cate.GetChildren():
                    # inst is the model prim (model_<hash>_*). The referenced content typically appears as child "Instance".
                    _remove_collider_recursive(inst)
                    _remove_rigidbody_recursive(inst)
                    instance_prim = inst.GetChild("Instance")
                    if not instance_prim or not instance_prim.IsValid():
                        # fallback: some variants may reference directly without Instance wrapper
                        instance_prim = inst

                    if scope_name in ("BaseAnimation", "Animation"):
                        # Articulated objects: bind rigid/static on groups, then enable joints.
                        joints = _get_all_joints(instance_prim)
                        for child in instance_prim.GetChildren():
                            child_name = str(child.GetName()).lower()
                            if child_name not in ("group_static", "group_00") and child.GetTypeName() in ("Xform", "Mesh"):
                                _bind_merged_mesh_rigid(child, approx=CONVEX_DECOMPOSITION)
                            if child_name == "group_00":
                                if pickable:
                                    _bind_merged_mesh_rigid(child, approx=CONVEX_DECOMPOSITION)
                                else:
                                    _bind_merged_mesh_static(child, approx=CONVEX_DECOMPOSITION)
                            if child_name == "group_static":
                                _bind_merged_mesh_static(child, approx=CONVEX_DECOMPOSITION)
                        for j in joints:
                            j.GetJointEnabledAttr().Set(True)

                    elif scope_name == "Base":
                        _bind_merged_mesh_static(instance_prim, approx=TRIANGLE_MESH)
                    else:
                        if pickable:
                            _bind_merged_mesh_rigid(instance_prim, approx=CONVEX_DECOMPOSITION)
                        else:
                            _bind_merged_mesh_static(instance_prim, approx=TRIANGLE_MESH)

        stage.GetRootLayer().Save()
    finally:
        try:
            kit.close()
        except Exception:
            pass


def process_navigation_static(scene_usd: str, out_usd: str, *, headless: bool = True, disable_doors: bool = True) -> None:
    """Create a static + semantic scene for navigation tasks."""

    from isaacsim import SimulationApp

    kit = SimulationApp({"headless": bool(headless)})
    try:
        from pxr import Usd
        from omni.isaac.core.utils.semantics import add_update_semantics

        _copy_file(scene_usd, out_usd)
        stage = Usd.Stage.Open(out_usd)
        meshes = stage.GetPrimAtPath("/Root/Meshes")
        if not meshes or not meshes.IsValid():
            raise RuntimeError("Expected /Root/Meshes in scene; did you pass a normalized target scene?")

        def set_semantics_recursive(prim, label: str) -> None:
            if prim.GetTypeName() in ("Mesh", "Xform"):
                add_update_semantics(prim, semantic_label=label, type_label="class")
            for ch in prim.GetAllChildren():
                set_semantics_recursive(ch, label)

        for scope in meshes.GetChildren():
            for cate in scope.GetChildren():
                cate_name = str(cate.GetName())
                for inst in cate.GetChildren():
                    instname = str(inst.GetName())
                    label = f"{cate_name}/{instname}"
                    set_semantics_recursive(inst, label)

                    if disable_doors and cate_name == "door":
                        inst.SetActive(False)
                        continue

                    _remove_collider_recursive(inst)
                    _remove_rigidbody_recursive(inst)
                    instance_prim = inst.GetChild("Instance")
                    if not instance_prim or not instance_prim.IsValid():
                        instance_prim = inst

                    joints = _get_all_joints(instance_prim)
                    jointed_paths: Set[object] = set()
                    for j in joints:
                        j.GetJointEnabledAttr().Set(False)
                        jointed_paths.update(_get_joint_connected_body_paths(j))

                    if jointed_paths:
                        for p in jointed_paths:
                            jp = stage.GetPrimAtPath(p)
                            if jp and jp.IsValid():
                                _bind_merged_mesh_static(jp, approx=TRIANGLE_MESH)
                    else:
                        _bind_merged_mesh_static(instance_prim, approx=TRIANGLE_MESH)

        stage.GetRootLayer().Save()
    finally:
        try:
            kit.close()
        except Exception:
            pass


# ----------------------- CLI -----------------------


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Generate a sim-ready USD (physics-prepped) from a single input USD")
    ap.add_argument("--input-usd", required=True, help="Input scene USD (expects sibling Materials/*.mdl)")
    ap.add_argument("--out-root", required=True, help="Output root directory (will create Materials/models/scenes)")
    ap.add_argument("--scene-id", default=None, help="Scene id folder name under out_root/scenes/")
    ap.add_argument(
        "--mode",
        choices=["interaction", "navigation"],
        default="interaction",
        help="Physics mode: interaction(dynamic) or navigation(static+semantics)",
    )
    ap.add_argument("--headless", action="store_true", help="Run Isaac Sim headless (recommended)")
    ap.add_argument("--disable-doors", action="store_true", help="(navigation) set door instances inactive")
    ap.add_argument(
        "--skip-clean",
        action="store_true",
        help="Skip normalization splitter; treat input-usd as already normalized and only apply physics.",
    )
    ap.add_argument(
        "--normalized-usd",
        default=None,
        help="If --skip-clean: optional explicit path to write the physics-prepped USD (default: out_root/scenes/<scene_id>/...)",
    )

    args = ap.parse_args(argv)

    input_usd = _abspath(args.input_usd)
    out_root = _abspath(args.out_root)
    scene_id = args.scene_id or _derive_scene_id(input_usd)

    if not args.skip_clean:
        paths = normalize_scene_to_target(input_usd=input_usd, out_root=out_root, scene_id=scene_id)
        in_scene = paths.normalized_scene_usd
        if not os.path.isfile(in_scene):
            raise FileNotFoundError(f"Normalized scene USD not found under: {paths.scene_dir}")

        if args.mode == "interaction":
            out_usd = os.path.join(paths.scene_dir, "start_result_dynamic.usd")
            process_interaction_physics(in_scene, out_usd, headless=args.headless)
        else:
            out_usd = os.path.join(paths.scene_dir, "start_result_navigation.usd")
            process_navigation_static(in_scene, out_usd, headless=args.headless, disable_doors=args.disable_doors)

        print(out_usd)
        return 0

    # skip-clean mode
    _ensure_dir(out_root)
    out_usd = _abspath(args.normalized_usd) if args.normalized_usd else os.path.join(out_root, f"{scene_id}_{args.mode}.usd")
    if args.mode == "interaction":
        process_interaction_physics(input_usd, out_usd, headless=args.headless)
    else:
        process_navigation_static(input_usd, out_usd, headless=args.headless, disable_doors=args.disable_doors)
    print(out_usd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
