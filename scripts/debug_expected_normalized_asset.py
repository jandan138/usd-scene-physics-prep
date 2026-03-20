#!/usr/bin/env python3
"""Compare expected normalized asset geometry against the written normalized USD.

This reconstructs the normalized mesh points from an original asset using the
same math as scripts/normalize_asset_transforms.py, then compares them against
the actual normalized asset that was written to disk. The goal is to isolate
whether the refrigerator outlier already exists inside phase1.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Any, Dict, List, Optional, Sequence, Tuple

import numpy as np

try:
    from pxr import Gf, Usd, UsdGeom
except ImportError as exc:
    print(f"Error: pxr (USD) not available: {exc}", file=sys.stderr)
    print("Run with a Python environment that has USD installed.", file=sys.stderr)
    sys.exit(1)


R_Y2Z = Gf.Matrix4d(1.0)
R_Y2Z.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), 90))


def _get_local_matrix(prim) -> Gf.Matrix4d:
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return Gf.Matrix4d(1.0)
    result = xf.GetLocalTransformation(Usd.TimeCode.Default())
    if isinstance(result, tuple):
        return result[0]
    return result


def _get_chain_transform(ancestor, descendant) -> Gf.Matrix4d:
    chain = []
    cur = descendant
    while cur and cur.GetPath() != ancestor.GetPath():
        chain.append(cur)
        cur = cur.GetParent()
    chain.reverse()

    result = Gf.Matrix4d(1.0)
    for prim in chain:
        result = result * _get_local_matrix(prim)
    return result


def _find_meshes(root_prim) -> List:
    meshes = []
    for prim in Usd.PrimRange(root_prim):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        points = mesh.GetPointsAttr().Get()
        if points and len(points) > 0:
            meshes.append(prim)
    return meshes


def _compute_center(src_stage) -> Tuple[Gf.Vec3d, Gf.Vec3d]:
    instance = src_stage.GetPrimAtPath("/Root/Instance")
    if not instance or not instance.IsValid():
        raise RuntimeError("No /Root/Instance in source asset")

    m_instance = _get_local_matrix(instance)
    all_rotated_points: List[Gf.Vec3d] = []

    for mesh_prim in _find_meshes(instance):
        points = UsdGeom.Mesh(mesh_prim).GetPointsAttr().Get()
        m_chain = _get_chain_transform(instance, mesh_prim)
        m_internal = m_chain * m_instance
        for point in points:
            p_root = m_internal.Transform(Gf.Vec3d(float(point[0]), float(point[1]), float(point[2])))
            all_rotated_points.append(R_Y2Z.Transform(p_root))

    if not all_rotated_points:
        return Gf.Vec3d(0, 0, 0), Gf.Vec3d(1, 1, 1)

    bbox_min = Gf.Vec3d(all_rotated_points[0])
    bbox_max = Gf.Vec3d(all_rotated_points[0])
    for point in all_rotated_points[1:]:
        for idx in range(3):
            bbox_min[idx] = min(bbox_min[idx], point[idx])
            bbox_max[idx] = max(bbox_max[idx], point[idx])

    instance_tf = Gf.Transform()
    instance_tf.SetMatrix(m_instance)
    scale = instance_tf.GetScale()
    center = (bbox_min + bbox_max) * 0.5
    return center, scale


def _expected_normalized_points(src_stage) -> Tuple[Dict[str, np.ndarray], List[float], List[float]]:
    instance = src_stage.GetPrimAtPath("/Root/Instance")
    if not instance or not instance.IsValid():
        raise RuntimeError("No /Root/Instance in source asset")

    center, scale = _compute_center(src_stage)
    inv_scale = Gf.Vec3d(
        1.0 / scale[0] if abs(scale[0]) > 1e-12 else 1.0,
        1.0 / scale[1] if abs(scale[1]) > 1e-12 else 1.0,
        1.0 / scale[2] if abs(scale[2]) > 1e-12 else 1.0,
    )

    expected: Dict[str, np.ndarray] = {}
    root = str(instance.GetPath())
    for mesh_prim in _find_meshes(instance):
        mesh = UsdGeom.Mesh(mesh_prim)
        points = mesh.GetPointsAttr().Get()
        m_chain = _get_chain_transform(instance, mesh_prim)
        m_internal = m_chain * _get_local_matrix(instance)

        new_points = []
        for point in points:
            p_root = m_internal.Transform(Gf.Vec3d(float(point[0]), float(point[1]), float(point[2])))
            p_rotated = R_Y2Z.Transform(p_root)
            p_centered = p_rotated - center
            new_points.append((
                float(p_centered[0] * inv_scale[0]),
                float(p_centered[1] * inv_scale[1]),
                float(p_centered[2] * inv_scale[2]),
            ))

        rel_path = str(mesh_prim.GetPath())[len(root):]
        expected[rel_path] = np.array(new_points, dtype=np.float64)

    return expected, [float(center[0]), float(center[1]), float(center[2])], [float(scale[0]), float(scale[1]), float(scale[2])]


def _actual_normalized_points(norm_stage) -> Dict[str, np.ndarray]:
    instance = norm_stage.GetPrimAtPath("/Root/Instance")
    if not instance or not instance.IsValid():
        raise RuntimeError("No /Root/Instance in normalized asset")

    result: Dict[str, np.ndarray] = {}
    root = str(instance.GetPath())
    for mesh_prim in _find_meshes(instance):
        mesh = UsdGeom.Mesh(mesh_prim)
        points = mesh.GetPointsAttr().Get()
        rel_path = str(mesh_prim.GetPath())[len(root):]
        result[rel_path] = np.array(
            [[float(p[0]), float(p[1]), float(p[2])] for p in points],
            dtype=np.float64,
        )
    return result


def compare_expected_to_actual(src_asset: str, normalized_asset: str, top_k: int) -> Dict[str, Any]:
    src_stage = Usd.Stage.Open(src_asset)
    norm_stage = Usd.Stage.Open(normalized_asset)
    if src_stage is None:
        raise RuntimeError(f"Failed to open source asset: {src_asset}")
    if norm_stage is None:
        raise RuntimeError(f"Failed to open normalized asset: {normalized_asset}")

    expected, center, scale = _expected_normalized_points(src_stage)
    actual = _actual_normalized_points(norm_stage)

    left_only = sorted(set(expected.keys()) - set(actual.keys()))
    right_only = sorted(set(actual.keys()) - set(expected.keys()))
    common = sorted(set(expected.keys()) & set(actual.keys()))

    per_mesh = []
    point_deltas_all = []
    for mesh_path in common:
        left_points = expected[mesh_path]
        right_points = actual[mesh_path]
        if len(left_points) != len(right_points):
            per_mesh.append({
                "mesh_path": mesh_path,
                "status": "vertex_count_mismatch",
                "expected_vertices": int(len(left_points)),
                "actual_vertices": int(len(right_points)),
            })
            continue

        point_deltas = np.linalg.norm(left_points - right_points, axis=1)
        point_deltas_all.append(point_deltas)
        left_centroid = left_points.mean(axis=0)
        right_centroid = right_points.mean(axis=0)

        per_mesh.append({
            "mesh_path": mesh_path,
            "status": "ok",
            "expected_vertices": int(len(left_points)),
            "actual_vertices": int(len(right_points)),
            "centroid_delta": round(float(np.linalg.norm(left_centroid - right_centroid)), 6),
            "point_delta_mean": round(float(point_deltas.mean()), 6),
            "point_delta_max": round(float(point_deltas.max()), 6),
        })

    ok_meshes = [item for item in per_mesh if item["status"] == "ok"]
    point_delta_concat = np.concatenate(point_deltas_all, axis=0) if point_deltas_all else np.zeros((0,), dtype=np.float64)

    return {
        "source_asset": os.path.abspath(src_asset),
        "normalized_asset": os.path.abspath(normalized_asset),
        "computed_center": [round(v, 6) for v in center],
        "computed_original_scale": [round(v, 6) for v in scale],
        "mesh_count_expected": len(expected),
        "mesh_count_actual": len(actual),
        "mesh_count_common": len(common),
        "mesh_paths_expected_only": left_only,
        "mesh_paths_actual_only": right_only,
        "aggregate_point_delta_mean": round(float(point_delta_concat.mean()), 6) if len(point_delta_concat) else 0.0,
        "aggregate_point_delta_max": round(float(point_delta_concat.max()), 6) if len(point_delta_concat) else 0.0,
        "top_meshes_by_point_delta_max": sorted(
            ok_meshes,
            key=lambda item: (item["point_delta_max"], item["centroid_delta"]),
            reverse=True,
        )[:top_k],
        "top_meshes_by_centroid_delta": sorted(
            ok_meshes,
            key=lambda item: (item["centroid_delta"], item["point_delta_max"]),
            reverse=True,
        )[:top_k],
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-asset", required=True, help="Original asset USD path")
    parser.add_argument("--normalized-asset", required=True, help="Normalized asset USD path")
    parser.add_argument("--top-k", type=int, default=20, help="Number of worst meshes to include")
    parser.add_argument("--out", default=None, help="Optional JSON output path")
    args = parser.parse_args(argv)

    result = compare_expected_to_actual(
        src_asset=args.source_asset,
        normalized_asset=args.normalized_asset,
        top_k=args.top_k,
    )

    if args.out:
        os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            json.dump(result, f, indent=2)
        print(args.out)
    else:
        json.dump(result, sys.stdout, indent=2)
        sys.stdout.write("\n")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
