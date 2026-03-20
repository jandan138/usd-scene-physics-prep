#!/usr/bin/env python3
"""Compare one scene prim's world-space geometry across two layout snapshots.

This is a targeted, read-only diagnostic for normalize investigations. It
compares mesh geometry under the same prim path in two composed scene layouts
and reports:
  - aggregate asset centroid delta
  - aggregate bbox-mid delta
  - pointwise mean / max delta by mesh path and point index
  - per-mesh centroid delta

It is intentionally stricter than placement_pairwise_compare.py, which only
tracks aggregate centroid displacement and max per-mesh centroid displacement.
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


def _collect_mesh_world_points(stage_path: str, prim_path: str) -> Dict[str, np.ndarray]:
    stage = Usd.Stage.Open(stage_path)
    if stage is None:
        raise RuntimeError(f"Failed to open stage: {stage_path}")

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"Prim not found: {prim_path}")

    xcache = UsdGeom.XformCache()
    root_path = str(prim.GetPath())
    mesh_map: Dict[str, np.ndarray] = {}

    for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if not child.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(child)
        points = mesh.GetPointsAttr().Get()
        if not points:
            continue

        world_xform = xcache.GetLocalToWorldTransform(child)
        world_points = []
        for point in points:
            wp = world_xform.Transform(Gf.Vec3d(float(point[0]), float(point[1]), float(point[2])))
            world_points.append((wp[0], wp[1], wp[2]))

        rel_path = str(child.GetPath())[len(root_path):]
        mesh_map[rel_path] = np.array(world_points, dtype=np.float64)

    return mesh_map


def _aggregate_point_cloud(mesh_map: Dict[str, np.ndarray]) -> np.ndarray:
    if not mesh_map:
        return np.zeros((0, 3), dtype=np.float64)
    return np.concatenate([mesh_map[k] for k in sorted(mesh_map.keys())], axis=0)


def _bbox_mid(points: np.ndarray) -> Optional[np.ndarray]:
    if len(points) == 0:
        return None
    return (points.min(axis=0) + points.max(axis=0)) * 0.5


def _round_vec(values: np.ndarray) -> List[float]:
    return [round(float(v), 6) for v in values]


def _delta_summary(left_points: np.ndarray, right_points: np.ndarray) -> Dict[str, List[float]]:
    deltas = right_points - left_points
    return {
        "delta_mean_vec": _round_vec(deltas.mean(axis=0)),
        "delta_min_vec": _round_vec(deltas.min(axis=0)),
        "delta_max_vec": _round_vec(deltas.max(axis=0)),
    }


def compare_scene_prim(
    left_stage: str,
    right_stage: str,
    prim_path: str,
    top_k: int,
) -> Dict[str, Any]:
    left_meshes = _collect_mesh_world_points(left_stage, prim_path)
    right_meshes = _collect_mesh_world_points(right_stage, prim_path)

    left_only = sorted(set(left_meshes.keys()) - set(right_meshes.keys()))
    right_only = sorted(set(right_meshes.keys()) - set(left_meshes.keys()))
    common = sorted(set(left_meshes.keys()) & set(right_meshes.keys()))

    per_mesh = []
    all_point_deltas = []

    for mesh_path in common:
        left_points = left_meshes[mesh_path]
        right_points = right_meshes[mesh_path]
        if len(left_points) != len(right_points):
            per_mesh.append({
                "mesh_path": mesh_path,
                "status": "vertex_count_mismatch",
                "left_vertices": int(len(left_points)),
                "right_vertices": int(len(right_points)),
            })
            continue

        point_deltas = np.linalg.norm(left_points - right_points, axis=1)
        all_point_deltas.append(point_deltas)
        left_centroid = left_points.mean(axis=0)
        right_centroid = right_points.mean(axis=0)
        left_bbox_mid = _bbox_mid(left_points)
        right_bbox_mid = _bbox_mid(right_points)

        per_mesh.append({
            "mesh_path": mesh_path,
            "status": "ok",
            "left_vertices": int(len(left_points)),
            "right_vertices": int(len(right_points)),
            "centroid_delta": round(float(np.linalg.norm(left_centroid - right_centroid)), 6),
            "centroid_delta_vec": _round_vec(right_centroid - left_centroid),
            "point_delta_mean": round(float(point_deltas.mean()), 6),
            "point_delta_max": round(float(point_deltas.max()), 6),
            "left_centroid": _round_vec(left_centroid),
            "right_centroid": _round_vec(right_centroid),
            "bbox_mid_left": _round_vec(left_bbox_mid),
            "bbox_mid_right": _round_vec(right_bbox_mid),
            "bbox_mid_delta": round(float(np.linalg.norm(left_bbox_mid - right_bbox_mid)), 6),
            "bbox_mid_delta_vec": _round_vec(right_bbox_mid - left_bbox_mid),
            **_delta_summary(left_points, right_points),
        })

    per_mesh_ok = [item for item in per_mesh if item["status"] == "ok"]
    per_mesh_sorted = sorted(
        per_mesh_ok,
        key=lambda item: (item["point_delta_max"], item["centroid_delta"]),
        reverse=True,
    )

    left_cloud = _aggregate_point_cloud({k: left_meshes[k] for k in common})
    right_cloud = _aggregate_point_cloud({k: right_meshes[k] for k in common})

    result: Dict[str, Any] = {
        "left_stage": os.path.abspath(left_stage),
        "right_stage": os.path.abspath(right_stage),
        "prim_path": prim_path,
        "mesh_count_left": len(left_meshes),
        "mesh_count_right": len(right_meshes),
        "mesh_count_common": len(common),
        "mesh_paths_left_only": left_only,
        "mesh_paths_right_only": right_only,
        "top_meshes_by_point_delta_max": per_mesh_sorted[:top_k],
        "top_meshes_by_centroid_delta": sorted(
            per_mesh_ok,
            key=lambda item: (item["centroid_delta"], item["point_delta_max"]),
            reverse=True,
        )[:top_k],
    }

    if len(left_cloud) == 0 or len(right_cloud) == 0:
        result["status"] = "no_common_geometry"
        result["per_mesh"] = per_mesh[:top_k]
        return result

    aggregate_centroid_left = left_cloud.mean(axis=0)
    aggregate_centroid_right = right_cloud.mean(axis=0)
    aggregate_bbox_mid_left = _bbox_mid(left_cloud)
    aggregate_bbox_mid_right = _bbox_mid(right_cloud)
    aggregate_point_deltas = np.concatenate(all_point_deltas, axis=0) if all_point_deltas else np.zeros((0,), dtype=np.float64)

    result.update({
        "status": "ok",
        "aggregate_vertices_left": int(len(left_cloud)),
        "aggregate_vertices_right": int(len(right_cloud)),
        "aggregate_centroid_left": _round_vec(aggregate_centroid_left),
        "aggregate_centroid_right": _round_vec(aggregate_centroid_right),
        "aggregate_centroid_delta": round(float(np.linalg.norm(aggregate_centroid_left - aggregate_centroid_right)), 6),
        "aggregate_centroid_delta_vec": _round_vec(aggregate_centroid_right - aggregate_centroid_left),
        "aggregate_bbox_mid_left": _round_vec(aggregate_bbox_mid_left),
        "aggregate_bbox_mid_right": _round_vec(aggregate_bbox_mid_right),
        "aggregate_bbox_mid_delta": round(float(np.linalg.norm(aggregate_bbox_mid_left - aggregate_bbox_mid_right)), 6),
        "aggregate_bbox_mid_delta_vec": _round_vec(aggregate_bbox_mid_right - aggregate_bbox_mid_left),
        "aggregate_point_delta_mean": round(float(aggregate_point_deltas.mean()), 6) if len(aggregate_point_deltas) else 0.0,
        "aggregate_point_delta_max": round(float(aggregate_point_deltas.max()), 6) if len(aggregate_point_deltas) else 0.0,
        "per_mesh_count_vertex_mismatch": sum(1 for item in per_mesh if item["status"] != "ok"),
        **_delta_summary(left_cloud, right_cloud),
    })

    return result


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left-stage", required=True, help="Path to the original scene layout USD")
    parser.add_argument("--right-stage", required=True, help="Path to the normalized scene layout USD")
    parser.add_argument("--prim-path", required=True, help="Prim path to compare")
    parser.add_argument("--top-k", type=int, default=20, help="Number of worst meshes to include")
    parser.add_argument("--out", default=None, help="Optional JSON output path")
    args = parser.parse_args(argv)

    result = compare_scene_prim(
        left_stage=args.left_stage,
        right_stage=args.right_stage,
        prim_path=args.prim_path,
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
