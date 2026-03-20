#!/usr/bin/env python3
"""Targeted world-space geometry diff for one scene prim across two layouts.

This is a focused read-only diagnostic for normalize investigations. It compares
the same referenced object prim across two scene layouts, then reports:
  - aggregate centroid delta across all vertices
  - aggregate bbox-mid delta
  - per-mesh centroid delta
  - per-mesh max vertex delta (same mesh path, same point index)
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
    print("Run via ./scripts/isaac_python.sh or a USD-enabled Python.", file=sys.stderr)
    sys.exit(1)


def _collect_meshes(stage_path: str, prim_path: str) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
    stage = Usd.Stage.Open(stage_path)
    if stage is None:
        raise RuntimeError(f"failed_to_open_stage: {stage_path}")

    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        raise RuntimeError(f"failed_to_open_prim: {prim_path}")

    root_len = len(str(prim.GetPath()))
    cache = UsdGeom.XformCache()
    mesh_map: Dict[str, np.ndarray] = {}
    all_pts: List[np.ndarray] = []

    for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if not child.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(child)
        points = mesh.GetPointsAttr().Get()
        if not points:
            continue
        world = cache.GetLocalToWorldTransform(child)
        arr = []
        for p in points:
            wp = world.Transform(Gf.Vec3d(float(p[0]), float(p[1]), float(p[2])))
            arr.append((wp[0], wp[1], wp[2]))
        pts = np.array(arr, dtype=np.float64)
        rel_path = str(child.GetPath())[root_len:]
        mesh_map[rel_path] = pts
        all_pts.append(pts)

    if not all_pts:
        raise RuntimeError(f"no_mesh_points_found: {prim_path}")

    return np.concatenate(all_pts, axis=0), mesh_map


def _bbox_mid(points: np.ndarray) -> np.ndarray:
    return (points.min(axis=0) + points.max(axis=0)) * 0.5


def _round_vec(v: np.ndarray) -> List[float]:
    return [round(float(x), 6) for x in v]


def compare_prim(
    left_layout: str,
    right_layout: str,
    prim_path: str,
) -> Dict[str, Any]:
    left_all, left_meshes = _collect_meshes(left_layout, prim_path)
    right_all, right_meshes = _collect_meshes(right_layout, prim_path)

    common_meshes = sorted(set(left_meshes.keys()) & set(right_meshes.keys()))
    left_only_meshes = sorted(set(left_meshes.keys()) - set(right_meshes.keys()))
    right_only_meshes = sorted(set(right_meshes.keys()) - set(left_meshes.keys()))

    per_mesh = []
    for mesh_path in common_meshes:
        left_pts = left_meshes[mesh_path]
        right_pts = right_meshes[mesh_path]
        if left_pts.shape != right_pts.shape:
            per_mesh.append({
                "mesh_path": mesh_path,
                "status": "shape_mismatch",
                "left_shape": list(left_pts.shape),
                "right_shape": list(right_pts.shape),
            })
            continue

        centroid_delta = float(np.linalg.norm(left_pts.mean(axis=0) - right_pts.mean(axis=0)))
        vertex_deltas = np.linalg.norm(left_pts - right_pts, axis=1)
        per_mesh.append({
            "mesh_path": mesh_path,
            "status": "ok",
            "vertex_count": int(left_pts.shape[0]),
            "centroid_delta": round(centroid_delta, 6),
            "max_vertex_delta": round(float(vertex_deltas.max()), 6),
            "mean_vertex_delta": round(float(vertex_deltas.mean()), 6),
        })

    per_mesh.sort(
        key=lambda item: (
            item.get("max_vertex_delta", -1.0),
            item.get("centroid_delta", -1.0),
        ),
        reverse=True,
    )

    centroid_delta = float(np.linalg.norm(left_all.mean(axis=0) - right_all.mean(axis=0)))
    bbox_mid_delta = float(np.linalg.norm(_bbox_mid(left_all) - _bbox_mid(right_all)))
    all_vertex_deltas: Optional[np.ndarray] = None
    if left_all.shape == right_all.shape:
        all_vertex_deltas = np.linalg.norm(left_all - right_all, axis=1)

    result: Dict[str, Any] = {
        "left_layout": os.path.abspath(left_layout),
        "right_layout": os.path.abspath(right_layout),
        "prim_path": prim_path,
        "aggregate": {
            "left_vertex_count": int(left_all.shape[0]),
            "right_vertex_count": int(right_all.shape[0]),
            "aggregate_centroid_delta": round(centroid_delta, 6),
            "aggregate_bbox_mid_delta": round(bbox_mid_delta, 6),
            "all_vertices_mean_delta": round(float(all_vertex_deltas.mean()), 6) if all_vertex_deltas is not None else None,
            "all_vertices_max_delta": round(float(all_vertex_deltas.max()), 6) if all_vertex_deltas is not None else None,
            "left_bbox_mid": _round_vec(_bbox_mid(left_all)),
            "right_bbox_mid": _round_vec(_bbox_mid(right_all)),
            "common_mesh_count": len(common_meshes),
            "left_only_meshes": left_only_meshes,
            "right_only_meshes": right_only_meshes,
        },
        "top_20_worst_meshes": per_mesh[:20],
    }
    return result


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--left-layout", required=True)
    ap.add_argument("--right-layout", required=True)
    ap.add_argument("--prim-path", required=True)
    ap.add_argument("--out", default=None, help="Optional JSON output path")
    args = ap.parse_args(argv)

    result = compare_prim(args.left_layout, args.right_layout, args.prim_path)
    text = json.dumps(result, indent=2, ensure_ascii=False)
    if args.out:
        os.makedirs(os.path.dirname(args.out) or ".", exist_ok=True)
        with open(args.out, "w", encoding="utf-8") as f:
            f.write(text)
        print(f"Wrote report: {args.out}")
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
