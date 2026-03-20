#!/usr/bin/env python3
"""Compare candidate phase2 compensation formulas for one normalized scene prim.

This is a read-only diagnostic. It uses the normalized asset geometry already
written to disk, applies several candidate scene-local matrices, and measures
how closely each candidate reconstructs the original scene's world-space mesh
positions.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    from pxr import Gf, Usd, UsdGeom
except ImportError as exc:
    print(f"Error: pxr (USD) not available: {exc}", file=sys.stderr)
    print("Run with a Python environment that has USD installed.", file=sys.stderr)
    sys.exit(1)


def _norm_abs(path: str) -> str:
    return os.path.normpath(os.path.abspath(path))


def _round_vec3(vec: Sequence[float]) -> List[float]:
    return [round(float(v), 6) for v in vec]


def _get_local_matrix(prim) -> Gf.Matrix4d:
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return Gf.Matrix4d(1.0)
    result = xf.GetLocalTransformation(Usd.TimeCode.Default())
    return result[0] if isinstance(result, tuple) else result


def _collect_world_mesh_points(prim) -> Dict[str, np.ndarray]:
    root_path = str(prim.GetPath())
    xcache = UsdGeom.XformCache()
    meshes: Dict[str, np.ndarray] = {}
    for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if not child.IsA(UsdGeom.Mesh):
            continue
        points = UsdGeom.Mesh(child).GetPointsAttr().Get()
        if not points:
            continue
        world_xform = xcache.GetLocalToWorldTransform(child)
        world_points = np.empty((len(points), 3), dtype=np.float64)
        for idx, point in enumerate(points):
            wp = world_xform.Transform(Gf.Vec3d(float(point[0]), float(point[1]), float(point[2])))
            world_points[idx, 0] = wp[0]
            world_points[idx, 1] = wp[1]
            world_points[idx, 2] = wp[2]
        rel_path = str(child.GetPath())[len(root_path):]
        meshes[rel_path] = world_points
    return meshes


def _collect_asset_root_points(asset_stage_path: str) -> Dict[str, np.ndarray]:
    stage = Usd.Stage.Open(asset_stage_path)
    if stage is None:
        raise RuntimeError(f"failed_to_open_asset_stage: {asset_stage_path}")
    root = stage.GetPrimAtPath("/Root")
    if root is None or not root.IsValid():
        raise RuntimeError(f"missing_root_prim: {asset_stage_path}")
    return _collect_world_mesh_points(root)


def _collect_scene_prim_points(layout_path: str, prim_path: str) -> Tuple[Dict[str, np.ndarray], Gf.Matrix4d]:
    stage = Usd.Stage.Open(layout_path)
    if stage is None:
        raise RuntimeError(f"failed_to_open_scene: {layout_path}")
    prim = stage.GetPrimAtPath(prim_path)
    if prim is None or not prim.IsValid():
        raise RuntimeError(f"missing_prim: {prim_path}")
    return _collect_world_mesh_points(prim), _get_local_matrix(prim)


def _mesh_stats(left: np.ndarray, right: np.ndarray) -> Dict[str, object]:
    delta_vec = right - left
    delta_norm = np.linalg.norm(delta_vec, axis=1)
    left_centroid = left.mean(axis=0)
    right_centroid = right.mean(axis=0)
    return {
        "point_count": int(len(left)),
        "mean_vertex_delta": round(float(delta_norm.mean()), 6),
        "max_vertex_delta": round(float(delta_norm.max()), 6),
        "centroid_delta": round(float(np.linalg.norm(right_centroid - left_centroid)), 6),
        "centroid_delta_vec": _round_vec3(right_centroid - left_centroid),
        "delta_mean_vec": _round_vec3(delta_vec.mean(axis=0)),
        "delta_min_vec": _round_vec3(delta_vec.min(axis=0)),
        "delta_max_vec": _round_vec3(delta_vec.max(axis=0)),
    }


def _apply_matrix(points: np.ndarray, matrix: Gf.Matrix4d) -> np.ndarray:
    out = np.empty_like(points)
    for idx, point in enumerate(points):
        wp = matrix.Transform(Gf.Vec3d(float(point[0]), float(point[1]), float(point[2])))
        out[idx, 0] = wp[0]
        out[idx, 1] = wp[1]
        out[idx, 2] = wp[2]
    return out


def _pre_rotation_center(source_asset_path: str) -> Gf.Vec3d:
    stage = Usd.Stage.Open(source_asset_path)
    if stage is None:
        raise RuntimeError(f"failed_to_open_source_asset: {source_asset_path}")
    instance = stage.GetPrimAtPath("/Root/Instance")
    if instance is None or not instance.IsValid():
        raise RuntimeError(f"missing_source_instance: {source_asset_path}")

    xcache = UsdGeom.XformCache()
    points: List[Tuple[float, float, float]] = []
    for child in Usd.PrimRange(instance):
        if not child.IsA(UsdGeom.Mesh):
            continue
        mesh_points = UsdGeom.Mesh(child).GetPointsAttr().Get()
        if not mesh_points:
            continue
        world_xform = xcache.GetLocalToWorldTransform(child)
        for point in mesh_points:
            wp = world_xform.Transform(Gf.Vec3d(float(point[0]), float(point[1]), float(point[2])))
            points.append((wp[0], wp[1], wp[2]))
    cloud = np.array(points, dtype=np.float64)
    bbox_mid = (cloud.min(axis=0) + cloud.max(axis=0)) * 0.5
    return Gf.Vec3d(float(bbox_mid[0]), float(bbox_mid[1]), float(bbox_mid[2]))


def compare_variants(
    source_asset_path: str,
    normalized_asset_path: str,
    original_layout_path: str,
    prim_path: str,
    rotated_center: Sequence[float],
    top_k: int,
) -> Dict[str, object]:
    normalized_meshes = _collect_asset_root_points(normalized_asset_path)
    original_meshes, original_prim_local = _collect_scene_prim_points(original_layout_path, prim_path)

    rotated_center_vec = Gf.Vec3d(*[float(v) for v in rotated_center])
    pre_rotation_center_vec = _pre_rotation_center(source_asset_path)

    r_y2z_inv = Gf.Matrix4d(1.0)
    r_y2z_inv.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), -90))

    t_rot = Gf.Matrix4d(1.0)
    t_rot.SetTranslate(rotated_center_vec)
    t_pre = Gf.Matrix4d(1.0)
    t_pre.SetTranslate(pre_rotation_center_vec)
    t_rotated_back = Gf.Matrix4d(1.0)
    t_rotated_back.SetTranslate(r_y2z_inv.Transform(rotated_center_vec))

    variants = {
        "impl_t_rot_rinv_mold": t_rot * r_y2z_inv * original_prim_local,
        "rinv_t_rot_mold": r_y2z_inv * t_rot * original_prim_local,
        "t_pre_rinv_mold": t_pre * r_y2z_inv * original_prim_local,
        "t_rinv_center_rinv_mold": t_rotated_back * r_y2z_inv * original_prim_local,
    }

    results = []
    common_meshes = sorted(set(original_meshes.keys()) & set(normalized_meshes.keys()))
    for name, matrix in variants.items():
        per_mesh = []
        aggregate_left = []
        aggregate_right = []
        for mesh_path in common_meshes:
            left = original_meshes[mesh_path]
            right = _apply_matrix(normalized_meshes[mesh_path], matrix)
            aggregate_left.append(left)
            aggregate_right.append(right)
            stats = _mesh_stats(left, right)
            stats["mesh_path"] = mesh_path
            per_mesh.append(stats)

        left_all = np.vstack(aggregate_left)
        right_all = np.vstack(aggregate_right)
        aggregate = _mesh_stats(left_all, right_all)
        per_mesh.sort(
            key=lambda item: (item["centroid_delta"], item["max_vertex_delta"]),
            reverse=True,
        )
        results.append(
            {
                "variant": name,
                "matrix_rows": [[round(float(matrix[r][c]), 6) for c in range(4)] for r in range(4)],
                "aggregate": aggregate,
                "top_meshes_by_centroid_delta": per_mesh[:top_k],
            }
        )

    results.sort(
        key=lambda item: (
            item["aggregate"]["centroid_delta"],
            item["aggregate"]["mean_vertex_delta"],
            item["aggregate"]["max_vertex_delta"],
        )
    )

    return {
        "source_asset_path": _norm_abs(source_asset_path),
        "normalized_asset_path": _norm_abs(normalized_asset_path),
        "original_layout_path": _norm_abs(original_layout_path),
        "prim_path": prim_path,
        "rotated_center": _round_vec3(rotated_center_vec),
        "pre_rotation_center": _round_vec3(pre_rotation_center_vec),
        "variant_results": results,
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-asset", required=True)
    parser.add_argument("--normalized-asset", required=True)
    parser.add_argument("--original-layout", required=True)
    parser.add_argument("--prim-path", required=True)
    parser.add_argument("--center", nargs=3, required=True, metavar=("X", "Y", "Z"), type=float)
    parser.add_argument("--top-k", type=int, default=10)
    parser.add_argument("--out", default=None)
    args = parser.parse_args(argv)

    report = compare_variants(
        source_asset_path=args.source_asset,
        normalized_asset_path=args.normalized_asset,
        original_layout_path=args.original_layout,
        prim_path=args.prim_path,
        rotated_center=args.center,
        top_k=args.top_k,
    )

    payload = json.dumps(report, indent=2)
    if args.out:
        os.makedirs(os.path.dirname(_norm_abs(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(payload)
        print(_norm_abs(args.out))
    else:
        print(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
