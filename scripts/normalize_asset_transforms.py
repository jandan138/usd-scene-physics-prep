#!/usr/bin/env python3
"""Normalize asset transforms: recenter geometry to origin, convert Y-up → Z-up, compensate scenes.

This script performs two phases:
  Phase 1 — Normalize each asset USD:
    - Bake all internal transforms (Instance scale/orient + model prim translate) into mesh points
    - Rotate from Y-up to Z-up via R_y2z = Rot_X(+90°)
    - Recenter geometry so bounding-box midpoint sits at origin
    - Write back mesh points under a clean Instance with only the original scale
    - Set stage upAxis to Z

  Phase 2 — Compensate each scene layout USD:
    - For each prim that references a normalized asset, adjust its xformOp:transform
    - Remap scene-authored descendant xform overrides beneath referenced roots
    - Formula: M_scene_new = T(center) * R_y2z_inv * M_scene_old
    - This preserves the final world-space placement of every object

Math:
  R_y2z = Rot_X(+90°): (x,y,z) → (x, -z, y)   [Y-up → Z-up]
  R_y2z_inv = Rot_X(-90°)
  center = bounding-box midpoint of all mesh points after R_y2z rotation (in /Root space)

Usage:
  python3 scripts/normalize_asset_transforms.py \
      --assets-root GRScenes-test1/GRScenes_assets \
      --scenes-root GRScenes-test1/GRScenes100 \
      --output-root GRScenes-test1-normalized \
      [--category plate] [--dry-run] [--symlink-copy] \
      [--report-dir check_reports/normalize] \
      [--phase 1|2] [--centers-dir /path/to/centers]

  Use --symlink-copy to create symlinks instead of copying support files
  (PNGs, GLBs, JSON, auxiliary USDs). The primary normalized USD is always
  written as a regular file to avoid write-through into the source asset tree.
  This drastically reduces I/O for large datasets.
  Avoid /tmp as output-root for large runs (not persistent across reboots).

  Use --phase 1 to run only Phase 1 (asset normalization) and save centers JSON.
  Use --phase 2 with --centers-dir to run only Phase 2 (scene compensation),
  loading centers from JSON files produced by Phase 1.
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import shutil
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

try:
    from pxr import Gf, Sdf, Usd, UsdGeom
except ImportError as e:
    print(f"Error: pxr (USD) not available: {e}", file=sys.stderr)
    print("Run with a Python environment that has USD installed.", file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

# Y-up → Z-up: rotate +90° around X axis
_R_Y2Z = Gf.Matrix4d(1.0)
_R_Y2Z.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 90))

# Z-up → Y-up: rotate -90° around X axis (inverse)
_R_Y2Z_INV = Gf.Matrix4d(1.0)
_R_Y2Z_INV.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), -90))

# Rotation-only 4x4 for transforming normals (same rotation, no translation)
_R_Y2Z_ROT = Gf.Matrix4d(1.0)
_R_Y2Z_ROT.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 90))

_XFORM_PROPS = {
    "xformOpOrder",
    "xformOp:transform",
    "xformOp:translate",
    "xformOp:orient",
    "xformOp:scale",
}


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _get_local_matrix(prim) -> Gf.Matrix4d:
    """Return the local transform matrix of a prim."""
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return Gf.Matrix4d(1.0)
    res = xf.GetLocalTransformation(Usd.TimeCode.Default())
    if isinstance(res, tuple):
        return res[0]
    return res


def _is_flat_layout_main_usd(item_name: str, uid: str, src_item: str) -> bool:
    """Return True when a flat-layout asset entry is the main USD payload.

    Legacy GRScenes assets may store the primary USD directly under
    ``<category>/<uid>/<uid>.usd`` instead of under ``usd/<uid>.usd``.
    That file must never be symlinked into the output tree because Phase 1
    later exports the normalized asset to the destination path.
    """
    return item_name == f"{uid}.usd" and os.path.isfile(src_item)


def _guard_normalize_output_path(src_usd: str, dst_usd: str) -> None:
    """Refuse unsafe export targets that could write back into the source tree."""
    src_abs = os.path.abspath(src_usd)
    dst_abs = os.path.abspath(dst_usd)

    if os.path.islink(dst_abs):
        raise RuntimeError(
            f"Refusing to export normalized asset through symlink target: {dst_abs}"
        )

    if not os.path.lexists(dst_abs):
        return

    try:
        if os.path.samefile(src_abs, dst_abs):
            raise RuntimeError(
                f"Refusing to export normalized asset onto source file: src={src_abs} dst={dst_abs}"
            )
    except FileNotFoundError:
        return


def _get_chain_transform(ancestor, descendant) -> Gf.Matrix4d:
    """Compute accumulated local transform from ancestor's child down to descendant (inclusive).

    This gives the transform that maps points in descendant's local space
    into ancestor's local space (NOT including ancestor's own transform).

    Row-vector convention: p_ancestor = p_descendant * M_chain, where
    M_chain = M_descendant_local * M_parent_local * ... * M_ancestor_child_local.
    """
    chain = []
    cur = descendant
    while cur and cur.GetPath() != ancestor.GetPath():
        chain.append(cur)
        cur = cur.GetParent()
    chain.reverse()  # top-down order

    result = Gf.Matrix4d(1.0)
    for p in chain:
        result = _get_local_matrix(p) * result
    return result


def _find_all_meshes(root_prim) -> List:
    """Find all UsdGeom.Mesh prims under root_prim (inclusive)."""
    meshes = []
    for prim in Usd.PrimRange(root_prim):
        mesh = UsdGeom.Mesh(prim)
        if mesh:
            pts = mesh.GetPointsAttr().Get()
            if pts and len(pts) > 0:
                meshes.append(prim)
    return meshes


def _ensure_matrix_xform(prim) -> None:
    """Ensure prim has a single xformOp:transform op."""
    prim.CreateAttribute(
        "xformOpOrder", Sdf.ValueTypeNames.TokenArray, custom=False
    ).Set(["xformOp:transform"])
    if not prim.HasAttribute("xformOp:transform"):
        prim.CreateAttribute(
            "xformOp:transform", Sdf.ValueTypeNames.Matrix4d, custom=False
        )


def _set_local_matrix(prim, m: Gf.Matrix4d) -> None:
    """Write a single xformOp:transform matrix on a prim."""
    _ensure_matrix_xform(prim)
    prim.GetAttribute("xformOp:transform").Set(m)


def _clear_scene_layer_xform_props(prim, layer_identifier: str) -> None:
    """Remove scene-layer xform properties without touching referenced asset layers."""
    stack = prim.GetPrimStack()
    if not stack:
        return
    top_spec = stack[0]
    if top_spec.layer.identifier != layer_identifier:
        return
    for prop_name in list(top_spec.properties.keys()):
        if prop_name == "xformOpOrder" or prop_name.startswith("xformOp:"):
            prim.RemoveProperty(prop_name)


def _replace_scene_layer_with_matrix_xform(
    prim,
    m: Gf.Matrix4d,
    layer_identifier: str,
) -> None:
    """Replace scene-layer xform opinions with a single matrix xform."""
    _clear_scene_layer_xform_props(prim, layer_identifier)
    _set_local_matrix(prim, m)


def _set_identity_trs_with_scale(prim, scale: Gf.Vec3d) -> None:
    """Author translate/orient/scale ops with identity TR and explicit scale.

    Keeping a named xformOp:scale preserves scene-layer overrides that target
    referenced child prims such as /Instance.
    """
    _clear_xform_ops(prim)
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return

    translate_op = xf.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble)
    orient_op = xf.AddOrientOp(UsdGeom.XformOp.PrecisionDouble)
    scale_op = xf.AddScaleOp(UsdGeom.XformOp.PrecisionDouble)

    translate_op.Set(Gf.Vec3d(0.0, 0.0, 0.0))
    orient_op.Set(Gf.Quatd(1.0, 0.0, 0.0, 0.0))
    scale_op.Set(scale)


def _clear_xform_ops(prim) -> None:
    """Remove all xformOp attributes and set empty xformOpOrder."""
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return
    # Clear order first
    prim.CreateAttribute(
        "xformOpOrder", Sdf.ValueTypeNames.TokenArray, custom=False
    ).Set([])
    # Remove individual op attributes
    for op in xf.GetOrderedXformOps():
        attr = op.GetAttr()
        if attr:
            prim.RemoveProperty(attr.GetName())


def _make_scale_matrix(s: Gf.Vec3d) -> Gf.Matrix4d:
    """Create a 4x4 scale matrix from a Vec3d."""
    m = Gf.Matrix4d(1.0)
    m.SetScale(s)
    return m


def _resolve_ref_asset_path(base_dir: str, asset_path: str) -> str:
    """Resolve a possibly-relative asset path to absolute."""
    if not asset_path:
        return ""
    s = asset_path.replace("\\", "/")
    if os.path.isabs(s):
        return os.path.normpath(s)
    return os.path.normpath(os.path.join(base_dir, s))


def _is_flat_main_usd_entry(item: str, uid: str, src_item: str) -> bool:
    """Return True when an asset-dir entry is the legacy flat-layout main USD."""
    return item == f"{uid}.usd" and os.path.isfile(src_item)


def _ensure_safe_output_usd_target(src_usd: str, dst_usd: str) -> None:
    """Reject output targets that would write through to the source asset."""
    src_abs = os.path.abspath(src_usd)
    dst_abs = os.path.abspath(dst_usd)

    if os.path.lexists(dst_abs) and os.path.islink(dst_abs):
        raise RuntimeError(
            f"Refusing to export normalized asset to symlink target: {dst_abs}"
        )

    if os.path.exists(src_abs) and os.path.exists(dst_abs):
        try:
            if os.path.samefile(src_abs, dst_abs):
                raise RuntimeError(
                    f"Refusing to export normalized asset when source and output are the same file: {src_abs}"
                )
        except FileNotFoundError:
            pass


def _ref_relative_path(root_path: str, child_path: str) -> str:
    if not child_path.startswith(root_path):
        raise ValueError(f"{child_path} is not under {root_path}")
    return child_path[len(root_path):] or "/"


def _collect_scene_descendant_xform_overrides(
    scene_stage,
    scene_path: str,
    ref_root_prim,
) -> List[Dict[str, Any]]:
    """Collect scene-authored xform overrides beneath one referenced root prim."""
    findings: List[Dict[str, Any]] = []
    if ref_root_prim is None or not ref_root_prim.IsValid():
        return findings

    scene_path = os.path.abspath(scene_path)
    root_path = str(ref_root_prim.GetPath())
    for prim in Usd.PrimRange(ref_root_prim):
        if prim.GetPath() == ref_root_prim.GetPath():
            continue
        stack = prim.GetPrimStack()
        if len(stack) < 2:
            continue

        top_spec = stack[0]
        if top_spec.layer.identifier != scene_path:
            continue

        referenced_specs = [
            spec for spec in stack[1:]
            if "GRScenes_assets" in spec.layer.identifier
        ]
        if not referenced_specs:
            continue

        scene_props = sorted(set(top_spec.properties.keys()) & _XFORM_PROPS)
        if not scene_props:
            continue

        findings.append({
            "rel_path": _ref_relative_path(root_path, str(prim.GetPath())),
            "scene_prim_path": str(prim.GetPath()),
            "scene_local_matrix": _get_local_matrix(prim),
            "scene_layer_props": scene_props,
        })

    findings.sort(key=lambda item: item["rel_path"])
    return findings


def _get_cached_asset_stage_pair(
    src_asset_abs: str,
    dst_asset_abs: str,
    cache: Dict[Tuple[str, str], Dict[str, Any]],
) -> Dict[str, Any]:
    """Open and cache source/normalized asset stages plus xform caches."""
    key = (src_asset_abs, dst_asset_abs)
    if key in cache:
        return cache[key]

    src_stage = Usd.Stage.Open(src_asset_abs)
    dst_stage = Usd.Stage.Open(dst_asset_abs)
    if src_stage is None:
        raise RuntimeError(f"Failed to open source asset stage: {src_asset_abs}")
    if dst_stage is None:
        raise RuntimeError(f"Failed to open normalized asset stage: {dst_asset_abs}")

    pair = {
        "src_stage": src_stage,
        "dst_stage": dst_stage,
        "src_cache": UsdGeom.XformCache(),
        "dst_cache": UsdGeom.XformCache(),
    }
    cache[key] = pair
    return pair


def _asset_descendant_path(rel_path: str) -> Sdf.Path:
    if rel_path in ("", "/"):
        return Sdf.Path("/Root")
    return Sdf.Path("/Root" + rel_path)


def _compute_descendant_override_remap(
    asset_stage_pair: Dict[str, Any],
    rel_desc_path: str,
    source_scene_local_matrix: Gf.Matrix4d,
    center: Gf.Vec3d,
) -> Gf.Matrix4d:
    """Remap a source-scene descendant xform override into normalized local space."""
    asset_desc_path = _asset_descendant_path(rel_desc_path)
    src_stage = asset_stage_pair["src_stage"]
    dst_stage = asset_stage_pair["dst_stage"]
    src_cache = asset_stage_pair["src_cache"]
    dst_cache = asset_stage_pair["dst_cache"]

    src_asset_prim = src_stage.GetPrimAtPath(asset_desc_path)
    dst_asset_prim = dst_stage.GetPrimAtPath(asset_desc_path)
    if not src_asset_prim or not src_asset_prim.IsValid():
        raise RuntimeError(f"Missing source asset prim: {asset_desc_path}")
    if not dst_asset_prim or not dst_asset_prim.IsValid():
        raise RuntimeError(f"Missing normalized asset prim: {asset_desc_path}")

    src_asset_local = _get_local_matrix(src_asset_prim)
    dst_asset_local = _get_local_matrix(dst_asset_prim)

    parent_path = asset_desc_path.GetParentPath()
    src_parent = src_stage.GetPrimAtPath(parent_path)
    dst_parent = dst_stage.GetPrimAtPath(parent_path)
    if not src_parent or not src_parent.IsValid():
        raise RuntimeError(f"Missing source parent prim: {parent_path}")
    if not dst_parent or not dst_parent.IsValid():
        raise RuntimeError(f"Missing normalized parent prim: {parent_path}")

    t_center = Gf.Matrix4d(1.0)
    t_center.SetTranslate(center)
    f_root = t_center * _R_Y2Z_INV

    w_asset_old_parent = src_cache.GetLocalToWorldTransform(src_parent)
    w_asset_new_parent = dst_cache.GetLocalToWorldTransform(dst_parent)
    f_parent = w_asset_new_parent * f_root * w_asset_old_parent.GetInverse()

    delta_old = src_asset_local.GetInverse() * source_scene_local_matrix
    delta_new = f_parent * delta_old * f_parent.GetInverse()
    return dst_asset_local * delta_new


def _apply_descendant_override_remaps(
    dst_stage,
    ref_root_path: str,
    remaps: List[Dict[str, Any]],
    layer_identifier: str,
) -> int:
    """Write remapped descendant overrides into the destination scene layer."""
    applied = 0
    for remap in remaps:
        dst_prim_path = ref_root_path + remap["rel_path"]
        dst_prim = dst_stage.GetPrimAtPath(dst_prim_path)
        if not dst_prim or not dst_prim.IsValid():
            raise RuntimeError(f"Missing destination descendant prim: {dst_prim_path}")
        _replace_scene_layer_with_matrix_xform(
            dst_prim,
            remap["new_local_matrix"],
            layer_identifier,
        )
        applied += 1
    return applied


# ---------------------------------------------------------------------------
# Phase 1: Normalize a single asset
# ---------------------------------------------------------------------------

def compute_asset_center(src_usd: str) -> Dict[str, Any]:
    """Compute the normalization center for an asset without writing any files.

    Used by dry-run mode. Returns center, scale, and other metadata.
    """
    stage = Usd.Stage.Open(src_usd)
    if stage is None:
        raise RuntimeError(f"Failed to open: {src_usd}")

    instance = stage.GetPrimAtPath("/Root/Instance")
    if not instance or not instance.IsValid():
        raise RuntimeError(f"No /Root/Instance in: {src_usd}")

    M_instance = _get_local_matrix(instance)
    tf = Gf.Transform()
    tf.SetMatrix(M_instance)
    original_scale = tf.GetScale()

    mesh_prims = _find_all_meshes(instance)
    if not mesh_prims:
        raise RuntimeError(f"No meshes found under /Root/Instance in: {src_usd}")

    all_rotated_points: List[Gf.Vec3d] = []
    for mesh_prim in mesh_prims:
        mesh = UsdGeom.Mesh(mesh_prim)
        points = mesh.GetPointsAttr().Get()
        M_chain = _get_chain_transform(instance, mesh_prim)
        M_internal = M_chain * M_instance
        for p in points:
            p_vec = Gf.Vec3d(p[0], p[1], p[2])
            p_root = M_internal.Transform(p_vec)
            p_rotated = _R_Y2Z.Transform(p_root)
            all_rotated_points.append(p_rotated)

    if not all_rotated_points:
        center = Gf.Vec3d(0, 0, 0)
    else:
        bbox_min = Gf.Vec3d(all_rotated_points[0])
        bbox_max = Gf.Vec3d(all_rotated_points[0])
        for p in all_rotated_points[1:]:
            for i in range(3):
                bbox_min[i] = min(bbox_min[i], p[i])
                bbox_max[i] = max(bbox_max[i], p[i])
        center = (bbox_min + bbox_max) * 0.5

    return {
        "center": [float(center[0]), float(center[1]), float(center[2])],
        "center_vec": center,
        "original_scale": [float(original_scale[0]), float(original_scale[1]), float(original_scale[2])],
        "mesh_count": len(mesh_prims),
        "total_points": len(all_rotated_points),
    }


def normalize_asset(src_usd: str, dst_usd: str) -> Dict[str, Any]:
    """Normalize a single asset USD: bake transforms, rotate Y→Z, recenter.

    Returns a dict with normalization metadata (center, scale, etc.).
    """
    _ensure_safe_output_usd_target(src_usd, dst_usd)

    stage = Usd.Stage.Open(src_usd)
    if stage is None:
        raise RuntimeError(f"Failed to open: {src_usd}")

    instance = stage.GetPrimAtPath("/Root/Instance")
    if not instance or not instance.IsValid():
        raise RuntimeError(f"No /Root/Instance in: {src_usd}")

    # 1. Get Instance's full local transform (contains scale + possible orient)
    M_instance = _get_local_matrix(instance)

    # Extract the original scale for later
    tf = Gf.Transform()
    tf.SetMatrix(M_instance)
    original_scale = tf.GetScale()

    # 2. Collect all meshes and their transforms
    mesh_prims = _find_all_meshes(instance)
    if not mesh_prims:
        raise RuntimeError(f"No meshes found under /Root/Instance in: {src_usd}")

    # For each mesh, compute the full internal transform chain:
    # M_internal = M_chain * M_instance (chain = transforms between Instance and mesh)
    mesh_data: List[Tuple[Any, Gf.Matrix4d, Any, Optional[Any]]] = []
    all_rotated_points: List[Gf.Vec3d] = []

    for mesh_prim in mesh_prims:
        mesh = UsdGeom.Mesh(mesh_prim)
        points = mesh.GetPointsAttr().Get()
        normals = mesh.GetNormalsAttr().Get()

        # Chain transform from Instance's children down to this mesh
        M_chain = _get_chain_transform(instance, mesh_prim)
        M_internal = M_chain * M_instance

        # Transform each point to /Root space, then rotate Y→Z
        for p in points:
            p_vec = Gf.Vec3d(p[0], p[1], p[2])
            # point * matrix (row-vector convention in USD)
            p_root = M_internal.Transform(p_vec)
            p_rotated = _R_Y2Z.Transform(p_root)
            all_rotated_points.append(p_rotated)

        mesh_data.append((mesh_prim, M_internal, points, normals))

    # 3. Compute geometric center (bounding box midpoint)
    if not all_rotated_points:
        center = Gf.Vec3d(0, 0, 0)
    else:
        bbox_min = Gf.Vec3d(all_rotated_points[0])
        bbox_max = Gf.Vec3d(all_rotated_points[0])
        for p in all_rotated_points[1:]:
            bbox_min[0] = min(bbox_min[0], p[0])
            bbox_min[1] = min(bbox_min[1], p[1])
            bbox_min[2] = min(bbox_min[2], p[2])
            bbox_max[0] = max(bbox_max[0], p[0])
            bbox_max[1] = max(bbox_max[1], p[1])
            bbox_max[2] = max(bbox_max[2], p[2])
        center = (bbox_min + bbox_max) * 0.5

    # 4. Compute inverse scale for writing back to mesh space
    # New Instance will have only Scale(original_scale), so mesh points need to be
    # in Instance-local space = p_centered / scale
    inv_scale = Gf.Vec3d(
        1.0 / original_scale[0] if abs(original_scale[0]) > 1e-12 else 1.0,
        1.0 / original_scale[1] if abs(original_scale[1]) > 1e-12 else 1.0,
        1.0 / original_scale[2] if abs(original_scale[2]) > 1e-12 else 1.0,
    )

    # 5. Write normalized points and normals back to each mesh
    for mesh_prim, M_internal, points_old, normals_old in mesh_data:
        mesh = UsdGeom.Mesh(mesh_prim)
        new_points = []
        for p in points_old:
            p_vec = Gf.Vec3d(p[0], p[1], p[2])
            p_root = M_internal.Transform(p_vec)
            p_rotated = _R_Y2Z.Transform(p_root)
            p_centered = p_rotated - center
            # Divide by scale to get Instance-local coordinates
            p_local = Gf.Vec3f(
                float(p_centered[0] * inv_scale[0]),
                float(p_centered[1] * inv_scale[1]),
                float(p_centered[2] * inv_scale[2]),
            )
            new_points.append(p_local)

        mesh.GetPointsAttr().Set(new_points)

        # Transform normals (rotation only, no scale/translate)
        if normals_old and len(normals_old) > 0:
            new_normals = []
            # Build rotation-only matrix: R_internal_rot * R_y2z_rot
            # Uses row-vector convention consistent with Matrix4d.Transform()
            tf_int = Gf.Transform()
            tf_int.SetMatrix(M_internal)
            R_internal_4x4 = Gf.Matrix4d(1.0)
            R_internal_4x4.SetRotate(tf_int.GetRotation())
            R_normal = R_internal_4x4 * _R_Y2Z_ROT  # combined rotation

            for n in normals_old:
                n_vec = Gf.Vec3d(n[0], n[1], n[2])
                # Transform normal through combined rotation (row-vector: n * R)
                n_rotated = R_normal.Transform(n_vec)
                # Normalize
                length = n_rotated.GetLength()
                if length > 1e-12:
                    n_rotated = n_rotated / length
                new_normals.append(Gf.Vec3f(float(n_rotated[0]), float(n_rotated[1]), float(n_rotated[2])))
            mesh.GetNormalsAttr().Set(new_normals)

    # 6. Clear all transform ops on intermediate prims (model prims under Instance)
    for prim in Usd.PrimRange(instance):
        if prim.GetPath() == instance.GetPath():
            continue  # handle Instance separately below
        _clear_xform_ops(prim)

    # 7. Set Instance to clean Scale-only transform while preserving a named
    # xformOp:scale so scene-layer child overrides continue to compose.
    _set_identity_trs_with_scale(instance, original_scale)

    # 8. Set stage metadata
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

    # 9. Export
    os.makedirs(os.path.dirname(dst_usd), exist_ok=True)
    _guard_normalize_output_path(src_usd, dst_usd)
    stage.Export(dst_usd)

    return {
        "center": [float(center[0]), float(center[1]), float(center[2])],
        "original_scale": [float(original_scale[0]), float(original_scale[1]), float(original_scale[2])],
        "mesh_count": len(mesh_data),
        "total_points": len(all_rotated_points),
        "upAxis_changed": "Y->Z",
    }


# ---------------------------------------------------------------------------
# Phase 2: Compensate scene layouts
# ---------------------------------------------------------------------------

def compensate_scene(
    src_layout: str,
    dst_layout: str,
    asset_centers: Dict[str, Gf.Vec3d],
    assets_root_abs: str,
    output_assets_root_abs: str,
) -> Dict[str, Any]:
    """Compensate a scene layout USD so that normalized assets appear in their original world positions.

    asset_centers: maps asset relative path (e.g. "plate/<uid>/usd/<uid>.usd") to its center Vec3d.
    """
    # Copy the layout file first
    os.makedirs(os.path.dirname(dst_layout), exist_ok=True)
    shutil.copy2(src_layout, dst_layout)

    source_stage = Usd.Stage.Open(src_layout)
    stage = Usd.Stage.Open(dst_layout)
    if source_stage is None:
        raise RuntimeError(f"Failed to open: {src_layout}")
    if stage is None:
        raise RuntimeError(f"Failed to open: {dst_layout}")

    base_dir = os.path.dirname(os.path.abspath(dst_layout))
    source_scene_layer = os.path.abspath(src_layout)
    destination_layer = stage.GetRootLayer().identifier
    compensated = 0
    descendant_overrides_found = 0
    descendant_overrides_remapped = 0
    skipped = 0
    errors = []
    descendant_override_examples: List[Dict[str, Any]] = []
    asset_stage_cache: Dict[Tuple[str, str], Dict[str, Any]] = {}

    for prim in stage.Traverse():
        if not prim or not prim.IsValid():
            continue
        if not prim.HasAuthoredReferences():
            continue

        refs = prim.GetMetadata("references")
        if not refs:
            continue

        items = []
        try:
            items = list(refs.GetAddedOrExplicitItems())
        except Exception:
            continue

        for ref in items:
            ap = getattr(ref, "assetPath", "") or ""
            if not ap:
                continue

            # Resolve to absolute path
            resolved = _resolve_ref_asset_path(base_dir, ap)

            # Check if this references an asset in our assets root
            # We need to find the relative path within assets root
            rel_key = None
            if resolved.startswith(assets_root_abs + "/"):
                rel_key = resolved[len(assets_root_abs) + 1:]
            elif resolved.startswith(output_assets_root_abs + "/"):
                rel_key = resolved[len(output_assets_root_abs) + 1:]
            else:
                # Try to extract from the relative path pattern
                # ../../../GRScenes_assets/<cat>/<uid>/usd/<uid>.usd
                parts = ap.replace("\\", "/").split("/")
                try:
                    idx = parts.index("GRScenes_assets")
                    rel_key = "/".join(parts[idx + 1:])
                except ValueError:
                    continue

            if rel_key is None or rel_key not in asset_centers:
                skipped += 1
                continue

            center = asset_centers[rel_key]

            # Get current scene transform
            M_scene_old = _get_local_matrix(prim)
            source_ref_prim = source_stage.GetPrimAtPath(prim.GetPath())
            descendant_override_specs = _collect_scene_descendant_xform_overrides(
                source_stage,
                source_scene_layer,
                source_ref_prim,
            )
            descendant_overrides_found += len(descendant_override_specs)

            # Compensate: M_scene_new = T(center) * R_y2z_inv * M_scene_old
            # Derivation (row-vector, p * M):
            #   q = p * M_internal * R_y2z * T(-center)
            #   q * M_scene_new = p * M_internal * M_scene_old
            #   => M_scene_new = T(center) * R_y2z_inv * M_scene_old
            T_center = Gf.Matrix4d(1.0)
            T_center.SetTranslate(center)
            M_scene_new = T_center * _R_Y2Z_INV * M_scene_old

            try:
                _set_local_matrix(prim, M_scene_new)
                compensated += 1
            except Exception as e:
                errors.append({"prim": str(prim.GetPath()), "error": str(e)})
                continue

            if not descendant_override_specs:
                continue

            src_asset_abs = os.path.join(assets_root_abs, rel_key)
            dst_asset_abs = os.path.join(output_assets_root_abs, rel_key)
            try:
                asset_stage_pair = _get_cached_asset_stage_pair(
                    src_asset_abs,
                    dst_asset_abs,
                    asset_stage_cache,
                )
            except Exception as e:
                errors.append({
                    "prim": str(prim.GetPath()),
                    "error": f"descendant_override_stage_open_failed: {e}",
                })
                continue

            remaps = []
            for spec in descendant_override_specs:
                try:
                    new_local = _compute_descendant_override_remap(
                        asset_stage_pair=asset_stage_pair,
                        rel_desc_path=spec["rel_path"],
                        source_scene_local_matrix=spec["scene_local_matrix"],
                        center=center,
                    )
                    remaps.append({
                        "rel_path": spec["rel_path"],
                        "scene_prim_path": spec["scene_prim_path"],
                        "scene_layer_props": spec["scene_layer_props"],
                        "new_local_matrix": new_local,
                    })
                except Exception as e:
                    errors.append({
                        "prim": spec["scene_prim_path"],
                        "error": f"descendant_override_remap_failed: {e}",
                    })

            try:
                applied = _apply_descendant_override_remaps(
                    dst_stage=stage,
                    ref_root_path=str(prim.GetPath()),
                    remaps=remaps,
                    layer_identifier=destination_layer,
                )
                descendant_overrides_remapped += applied
                for remap in remaps[:5]:
                    if len(descendant_override_examples) >= 20:
                        break
                    descendant_override_examples.append({
                        "ref_root_path": str(prim.GetPath()),
                        "descendant_rel_path": remap["rel_path"],
                        "scene_prim_path": remap["scene_prim_path"],
                        "scene_layer_props": remap["scene_layer_props"],
                    })
            except Exception as e:
                errors.append({
                    "prim": str(prim.GetPath()),
                    "error": f"descendant_override_apply_failed: {e}",
                })

    # Update references to point to normalized asset paths
    # The layout references use relative paths like ../../../GRScenes_assets/...
    # Since we mirror the directory structure, relative paths remain valid
    # (output has same GRScenes_assets/ and GRScenes100/ under output-root)

    stage.GetRootLayer().Save()

    return {
        "compensated": compensated,
        "descendant_overrides_found": descendant_overrides_found,
        "descendant_overrides_remapped": descendant_overrides_remapped,
        "descendant_override_examples": descendant_override_examples,
        "skipped": skipped,
        "errors": errors,
    }


# ---------------------------------------------------------------------------
# Discovery helpers
# ---------------------------------------------------------------------------

def discover_assets(
    assets_root: str,
    category: Optional[str] = None,
) -> List[Tuple[str, str, str, str]]:
    """Discover all asset USD files.

    Supports both asset layouts used in this repo:
      - normalized-style: <cat>/<uid>/usd/<uid>.usd
      - flat legacy style: <cat>/<uid>/<uid>.usd

    Returns list of (category, uid, rel_path_within_assets_root, usd_path).
    """
    results = []
    assets_root = os.path.abspath(assets_root)

    if category:
        categories = [category]
    else:
        categories = sorted([
            d for d in os.listdir(assets_root)
            if os.path.isdir(os.path.join(assets_root, d))
            and not d.startswith(".")
            and not d.endswith(".json")
        ])

    for cat in categories:
        cat_dir = os.path.join(assets_root, cat)
        if not os.path.isdir(cat_dir):
            print(f"Warning: category directory not found: {cat_dir}", file=sys.stderr)
            continue

        for uid in sorted(os.listdir(cat_dir)):
            usd_dir = os.path.join(cat_dir, uid, "usd")
            usd_file = os.path.join(usd_dir, f"{uid}.usd")
            flat_usd_file = os.path.join(cat_dir, uid, f"{uid}.usd")

            # Prefer the normalized-style layout when both exist.
            if os.path.isfile(usd_file):
                rel_path = f"{cat}/{uid}/usd/{uid}.usd"
                results.append((cat, uid, rel_path, usd_file))
                continue
            if os.path.isfile(flat_usd_file):
                rel_path = f"{cat}/{uid}/{uid}.usd"
                results.append((cat, uid, rel_path, flat_usd_file))

    return results


def discover_layouts(scenes_root: str) -> List[str]:
    """Discover all layout.usd files under scenes root."""
    results = []
    scenes_root = os.path.abspath(scenes_root)

    for scene_type in sorted(os.listdir(scenes_root)):
        type_dir = os.path.join(scenes_root, scene_type)
        if not os.path.isdir(type_dir):
            continue

        for scene_id in sorted(os.listdir(type_dir)):
            layout = os.path.join(type_dir, scene_id, "layout.usd")
            if os.path.isfile(layout):
                results.append(layout)

    return results


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Normalize asset transforms (recenter + Y-up→Z-up) and compensate scene layouts.",
    )
    ap.add_argument("--assets-root", required=True,
                     help="Root directory of assets (e.g. GRScenes-test1/GRScenes_assets)")
    ap.add_argument("--scenes-root", required=True,
                     help="Root directory of scenes (e.g. GRScenes-test1/GRScenes100)")
    ap.add_argument("--output-root", required=True,
                     help="Output root directory (will mirror input structure)")
    ap.add_argument("--category", default=None,
                     help="Process only this category (e.g. 'plate')")
    ap.add_argument("--dry-run", action="store_true",
                     help="Simulate: report what would be done without writing files")
    ap.add_argument("--symlink-copy", action="store_true",
                     help="Use symlinks instead of file copies for support files; "
                          "primary normalized USDs are still written as regular files")
    ap.add_argument("--report-dir", default=None,
                     help="Directory for JSON reports")
    ap.add_argument("--phase", choices=["1", "2"], default=None,
                     help="Run only Phase 1 (normalize assets) or Phase 2 (compensate scenes). "
                          "Default: both phases.")
    ap.add_argument("--centers-dir", default=None,
                     help="Directory containing centers_*.json files (required for --phase 2)")

    args = ap.parse_args(argv)

    if args.phase == "2" and not args.centers_dir:
        ap.error("--centers-dir is required when --phase 2 is specified")

    assets_root = os.path.abspath(args.assets_root)
    scenes_root = os.path.abspath(args.scenes_root)
    output_root = os.path.abspath(args.output_root)
    output_assets = os.path.join(output_root, "GRScenes_assets")
    output_scenes = os.path.join(output_root, os.path.basename(scenes_root))

    print(f"Assets root: {assets_root}")
    print(f"Scenes root: {scenes_root}")
    print(f"Output root: {output_root}")
    if args.category:
        print(f"Category filter: {args.category}")
    if args.dry_run:
        print("DRY RUN — no files will be written")
    if args.phase:
        print(f"Phase: {args.phase} only")
    print()

    # Ensure report directory exists early
    report_dir = args.report_dir
    if report_dir:
        os.makedirs(report_dir, exist_ok=True)

    # -----------------------------------------------------------------------
    # Phase 1: Normalize assets
    # -----------------------------------------------------------------------
    run_phase1 = args.phase != "2"
    run_phase2 = args.phase != "1"

    asset_centers: Dict[str, Gf.Vec3d] = {}  # rel_path → center
    asset_report: Dict[str, Any] = {}
    errors: List[Dict[str, Any]] = []
    elapsed_p1 = 0.0
    asset_total = 0

    if run_phase1:
        print("=" * 60)
        print("Phase 1: Normalizing assets")
        print("=" * 60)

        asset_list = discover_assets(assets_root, args.category)
        asset_total = len(asset_list)
        print(f"Found {asset_total} assets to normalize")

        t0 = time.time()
        for i, (cat, uid, rel_path, src_usd) in enumerate(asset_list):
            dst_usd = os.path.join(output_assets, rel_path)

            if (i + 1) % 500 == 0 or i == 0:
                elapsed = time.time() - t0
                rate = (i + 1) / elapsed if elapsed > 0 else 0
                print(f"  [{i+1}/{asset_total}] {cat}/{uid} ({rate:.0f} assets/s)")

            if args.dry_run:
                # In dry-run, compute center without writing any files
                try:
                    info = compute_asset_center(src_usd)
                    asset_centers[rel_path] = info["center_vec"]
                    asset_report[uid] = {
                        "category": cat,
                        "center": info["center"],
                        "original_scale": info["original_scale"],
                        "upAxis_changed": "Y->Z",
                    }
                except Exception as e:
                    errors.append({"asset": rel_path, "phase": "normalize", "error": str(e)})
                continue

            try:
                # Copy (or symlink) non-USD files (textures, json, images) to output
                src_asset_dir = os.path.join(assets_root, cat, uid)
                dst_asset_dir = os.path.join(output_assets, cat, uid)
                if os.path.isdir(src_asset_dir):
                    for item in os.listdir(src_asset_dir):
                        src_item = os.path.join(src_asset_dir, item)
                        dst_item = os.path.join(dst_asset_dir, item)
                        if _is_flat_layout_main_usd(item, uid, src_item):
                            # The main legacy USD is written by normalize_asset().
                            # Never pre-create it as a symlink or copy target.
                            continue
                        if item == "usd":
                            # Handle usd/ subdir: symlink or copy non-main USD files
                            usd_dir = os.path.join(src_asset_dir, "usd")
                            dst_usd_dir = os.path.join(dst_asset_dir, "usd")
                            os.makedirs(dst_usd_dir, exist_ok=True)
                            for usd_item in os.listdir(usd_dir):
                                if usd_item == f"{uid}.usd":
                                    continue  # will be written by normalize_asset
                                src_ui = os.path.join(usd_dir, usd_item)
                                dst_ui = os.path.join(dst_usd_dir, usd_item)
                                if os.path.exists(dst_ui) or os.path.islink(dst_ui):
                                    continue  # idempotent
                                if args.symlink_copy:
                                    os.symlink(os.path.abspath(src_ui), dst_ui)
                                elif os.path.isfile(src_ui):
                                    shutil.copy2(src_ui, dst_ui)
                                elif os.path.isdir(src_ui):
                                    shutil.copytree(src_ui, dst_ui, symlinks=True)
                        else:
                            if _is_flat_main_usd_entry(item, uid, src_item):
                                continue  # will be written by normalize_asset
                            if os.path.exists(dst_item) or os.path.islink(dst_item):
                                continue  # idempotent
                            os.makedirs(dst_asset_dir, exist_ok=True)
                            if args.symlink_copy:
                                os.symlink(os.path.abspath(src_item), dst_item)
                            elif os.path.isfile(src_item):
                                shutil.copy2(src_item, dst_item)
                            elif os.path.isdir(src_item):
                                shutil.copytree(src_item, dst_item, symlinks=True)

                info = normalize_asset(src_usd, dst_usd)
                center = Gf.Vec3d(*info["center"])
                asset_centers[rel_path] = center
                asset_report[uid] = {
                    "category": cat,
                    "center": info["center"],
                    "original_scale": info["original_scale"],
                    "upAxis_changed": info["upAxis_changed"],
                }
            except Exception as e:
                errors.append({"asset": rel_path, "phase": "normalize", "error": str(e)})
                if len(errors) <= 10:
                    print(f"  ERROR: {rel_path}: {e}", file=sys.stderr)

        elapsed_p1 = time.time() - t0
        print(f"\nPhase 1 complete: {len(asset_centers)}/{asset_total} assets normalized "
              f"({len(errors)} errors) in {elapsed_p1:.1f}s")

        # Save asset centers to JSON (always after Phase 1, for Phase 2 to consume later)
        centers_out_dir = report_dir or output_root
        os.makedirs(centers_out_dir, exist_ok=True)
        centers_file = os.path.join(centers_out_dir, f"centers_{args.category or 'all'}.json")
        centers_data = {k: list(v) for k, v in asset_centers.items()}
        with open(centers_file, "w") as f:
            json.dump(centers_data, f)
        print(f"Saved {len(centers_data)} asset centers to {centers_file}")

    # If --phase 1, exit after Phase 1
    if args.phase == "1":
        print(f"\n--phase 1: Phase 1 complete, skipping Phase 2.")
        return 1 if errors else 0

    # If --phase 2, load centers from --centers-dir
    if args.phase == "2":
        centers_dir = args.centers_dir
        center_files = glob.glob(os.path.join(centers_dir, "centers_*.json"))
        if not center_files:
            print(f"ERROR: No centers_*.json files found in {centers_dir}", file=sys.stderr)
            return 1
        for cf in center_files:
            with open(cf) as f:
                data = json.load(f)
            for k, v in data.items():
                asset_centers[k] = Gf.Vec3d(*v)
        print(f"Loaded {len(asset_centers)} centers from {len(center_files)} files in {centers_dir}")

    # -----------------------------------------------------------------------
    # Phase 2: Compensate scenes
    # -----------------------------------------------------------------------
    print()
    print("=" * 60)
    print("Phase 2: Compensating scene layouts")
    print("=" * 60)

    layout_list = discover_layouts(scenes_root)
    print(f"Found {len(layout_list)} layout files to compensate")

    scene_errors: List[Dict[str, Any]] = []
    total_compensated = 0
    total_descendant_overrides_found = 0
    total_descendant_overrides_remapped = 0
    descendant_override_examples: List[Dict[str, Any]] = []

    t1 = time.time()
    for i, src_layout in enumerate(layout_list):
        # Compute relative path from scenes_root
        rel_layout = os.path.relpath(src_layout, scenes_root)
        dst_layout = os.path.join(output_scenes, rel_layout)

        if (i + 1) % 10 == 0 or i == 0:
            print(f"  [{i+1}/{len(layout_list)}] {rel_layout}")

        if args.dry_run:
            # Dry-run: just count how many prims would be compensated
            try:
                stage = Usd.Stage.Open(src_layout)
                base_dir = os.path.dirname(os.path.abspath(src_layout))
                count = 0
                for prim in stage.Traverse():
                    if not prim.HasAuthoredReferences():
                        continue
                    refs = prim.GetMetadata("references")
                    if not refs:
                        continue
                    items = []
                    try:
                        items = list(refs.GetAddedOrExplicitItems())
                    except Exception:
                        continue
                    for ref in items:
                        ap = getattr(ref, "assetPath", "") or ""
                        parts = ap.replace("\\", "/").split("/")
                        try:
                            idx = parts.index("GRScenes_assets")
                            rel_key = "/".join(parts[idx + 1:])
                            if rel_key in asset_centers:
                                count += 1
                        except ValueError:
                            pass
                total_compensated += count
            except Exception as e:
                scene_errors.append({"layout": rel_layout, "phase": "compensate", "error": str(e)})
            continue

        try:
            result = compensate_scene(
                src_layout=src_layout,
                dst_layout=dst_layout,
                asset_centers=asset_centers,
                assets_root_abs=assets_root,
                output_assets_root_abs=output_assets,
            )
            total_compensated += result["compensated"]
            total_descendant_overrides_found += int(
                result.get("descendant_overrides_found", 0)
            )
            total_descendant_overrides_remapped += int(
                result.get("descendant_overrides_remapped", 0)
            )
            for item in result.get("descendant_override_examples", []):
                if len(descendant_override_examples) >= 20:
                    break
                descendant_override_examples.append({
                    "layout": rel_layout,
                    **item,
                })
            if result["errors"]:
                scene_errors.extend([
                    {"layout": rel_layout, "phase": "compensate", **e}
                    for e in result["errors"]
                ])
        except Exception as e:
            scene_errors.append({"layout": rel_layout, "phase": "compensate", "error": str(e)})
            print(f"  ERROR: {rel_layout}: {e}", file=sys.stderr)

    elapsed_p2 = time.time() - t1
    print(f"\nPhase 2 complete: {total_compensated} prim transforms compensated "
          f"across {len(layout_list)} layouts ({len(scene_errors)} errors) in {elapsed_p2:.1f}s")

    # -----------------------------------------------------------------------
    # Report
    # -----------------------------------------------------------------------
    all_errors = errors + scene_errors
    report = {
        "meta": {
            "assets_root": assets_root,
            "scenes_root": scenes_root,
            "output_root": output_root,
            "category_filter": args.category,
            "dry_run": args.dry_run,
            "phase": args.phase,
            "assets_processed": len(asset_centers),
            "assets_total": asset_total,
            "scenes_processed": len(layout_list),
            "prims_compensated": total_compensated,
            "descendant_overrides_found": total_descendant_overrides_found,
            "descendant_overrides_remapped": total_descendant_overrides_remapped,
            "errors_count": len(all_errors),
            "elapsed_phase1_sec": round(elapsed_p1, 2),
            "elapsed_phase2_sec": round(elapsed_p2, 2),
        },
        "assets": asset_report,
        "descendant_override_examples": descendant_override_examples,
        "errors": all_errors,
    }

    # Write report
    if report_dir:
        report_path = os.path.join(report_dir, "normalize_report.json")
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        print(f"\nReport written to: {report_path}")
    else:
        report_path = os.path.join(output_root, "normalize_report.json")
        if not args.dry_run:
            os.makedirs(output_root, exist_ok=True)
            with open(report_path, "w", encoding="utf-8") as f:
                json.dump(report, f, indent=2, ensure_ascii=False)
            print(f"\nReport written to: {report_path}")

    print(f"\nSummary: {len(asset_centers)} assets normalized, "
          f"{total_compensated} scene prims compensated, "
          f"{total_descendant_overrides_remapped} descendant overrides remapped, "
          f"{len(all_errors)} errors")

    return 1 if all_errors else 0


if __name__ == "__main__":
    sys.exit(main())
