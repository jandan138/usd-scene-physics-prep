#!/usr/bin/env python3
"""Diagnose geometry deltas for one original-vs-normalized scene prim pair.

This is a read-only helper for targeted normalize investigations. It compares
the composed world-space mesh geometry under one prim in two scene layouts and
reports:
  - per-mesh centroid deltas
  - per-mesh max vertex deltas by mesh path and point index
  - aggregate asset centroid delta
  - aggregate asset bbox-mid delta

Default arguments target the known refrigerator outlier from the S1 smoke run,
but all inputs are parameterized via CLI.
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
    print("Run via ./scripts/isaac_python.sh", file=sys.stderr)
    sys.exit(1)


_DEFAULT_LEFT_LAYOUT = (
    "GRScenes-test0-smoke-S1/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd"
)
_DEFAULT_RIGHT_LAYOUT = (
    "GRScenes-test0-smoke-S1-normalized/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd"
)
_DEFAULT_PRIM_PATH = (
    "/Root/Meshes/Animation/refrigerator/"
    "model_8958330cdaa9b7dcda067c99f5916ca3_0"
)


def _norm_abs(path: str) -> str:
    return os.path.normpath(os.path.abspath(path))


def _get_reference_asset_paths(prim) -> List[str]:
    refs: List[str] = []
    for spec in prim.GetPrimStack():
        ref_items = spec.referenceList
        for ref in ref_items.GetAddedOrExplicitItems():
            asset_path = getattr(ref, "assetPath", "") or ""
            if not asset_path:
                continue
            resolved = spec.layer.ComputeAbsolutePath(asset_path)
            refs.append(_norm_abs(resolved if resolved else asset_path))
    return refs


def _load_stage_and_prim(layout_path: str, prim_path: str):
    stage = Usd.Stage.Open(layout_path)
    if stage is None:
        raise RuntimeError(f"failed_to_open_stage: {layout_path}")
    prim = stage.GetPrimAtPath(prim_path)
    if prim is None or not prim.IsValid():
        raise RuntimeError(f"failed_to_find_prim: {prim_path} in {layout_path}")
    return stage, prim


def _collect_world_mesh_points(prim) -> List[Tuple[str, np.ndarray]]:
    root_path = str(prim.GetPath())
    xcache = UsdGeom.XformCache()
    meshes: List[Tuple[str, np.ndarray]] = []
    for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if not child.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(child)
        points = mesh.GetPointsAttr().Get()
        if not points or len(points) == 0:
            continue
        world_xform = xcache.GetLocalToWorldTransform(child)
        world_pts = np.empty((len(points), 3), dtype=np.float64)
        for idx, point in enumerate(points):
            wp = world_xform.Transform(
                Gf.Vec3d(float(point[0]), float(point[1]), float(point[2]))
            )
            world_pts[idx, 0] = wp[0]
            world_pts[idx, 1] = wp[1]
            world_pts[idx, 2] = wp[2]
        rel_path = str(child.GetPath())[len(root_path) :]
        meshes.append((rel_path, world_pts))
    meshes.sort(key=lambda item: item[0])
    return meshes


def _centroid(points: np.ndarray) -> np.ndarray:
    return points.mean(axis=0)


def _bbox_mid(points: np.ndarray) -> np.ndarray:
    return (points.min(axis=0) + points.max(axis=0)) * 0.5


def _round_vec3(vec: Sequence[float]) -> List[float]:
    return [round(float(v), 6) for v in vec]


def _aggregate_points(meshes: Sequence[Tuple[str, np.ndarray]]) -> np.ndarray:
    if not meshes:
        return np.empty((0, 3), dtype=np.float64)
    return np.vstack([pts for _, pts in meshes])


def _compare_meshes(
    left_meshes: Sequence[Tuple[str, np.ndarray]],
    right_meshes: Sequence[Tuple[str, np.ndarray]],
) -> Dict[str, object]:
    left_map = {path: pts for path, pts in left_meshes}
    right_map = {path: pts for path, pts in right_meshes}
    common_paths = sorted(set(left_map.keys()) & set(right_map.keys()))
    left_only = sorted(set(left_map.keys()) - set(right_map.keys()))
    right_only = sorted(set(right_map.keys()) - set(left_map.keys()))

    per_mesh = []
    mismatched_counts = []
    for mesh_path in common_paths:
        left_pts = left_map[mesh_path]
        right_pts = right_map[mesh_path]
        if len(left_pts) != len(right_pts):
            mismatched_counts.append(
                {
                    "mesh_path": mesh_path,
                    "left_points": int(len(left_pts)),
                    "right_points": int(len(right_pts)),
                }
            )
            continue

        deltas = np.linalg.norm(left_pts - right_pts, axis=1)
        left_centroid = _centroid(left_pts)
        right_centroid = _centroid(right_pts)
        left_bbox_mid = _bbox_mid(left_pts)
        right_bbox_mid = _bbox_mid(right_pts)

        max_vertex_index = int(np.argmax(deltas)) if len(deltas) else -1
        per_mesh.append(
            {
                "mesh_path": mesh_path,
                "point_count": int(len(left_pts)),
                "centroid_left": _round_vec3(left_centroid),
                "centroid_right": _round_vec3(right_centroid),
                "centroid_delta": round(
                    float(np.linalg.norm(left_centroid - right_centroid)), 6
                ),
                "bbox_mid_left": _round_vec3(left_bbox_mid),
                "bbox_mid_right": _round_vec3(right_bbox_mid),
                "bbox_mid_delta": round(
                    float(np.linalg.norm(left_bbox_mid - right_bbox_mid)), 6
                ),
                "mean_vertex_delta": round(float(deltas.mean()), 6),
                "max_vertex_delta": round(float(deltas.max()), 6),
                "max_vertex_index": max_vertex_index,
                "max_vertex_left": (
                    _round_vec3(left_pts[max_vertex_index])
                    if max_vertex_index >= 0
                    else None
                ),
                "max_vertex_right": (
                    _round_vec3(right_pts[max_vertex_index])
                    if max_vertex_index >= 0
                    else None
                ),
            }
        )

    per_mesh.sort(
        key=lambda item: (item["max_vertex_delta"], item["centroid_delta"]),
        reverse=True,
    )
    return {
        "common_mesh_count": len(common_paths),
        "left_only_mesh_count": len(left_only),
        "right_only_mesh_count": len(right_only),
        "left_only_meshes": left_only[:20],
        "right_only_meshes": right_only[:20],
        "mismatched_point_counts": mismatched_counts,
        "per_mesh": per_mesh,
    }


def diagnose(
    left_layout: str,
    right_layout: str,
    left_prim_path: str,
    right_prim_path: str,
) -> Dict[str, object]:
    left_stage, left_prim = _load_stage_and_prim(left_layout, left_prim_path)
    right_stage, right_prim = _load_stage_and_prim(right_layout, right_prim_path)

    # Keep stage handles alive while reading composed geometry below.
    _ = (left_stage, right_stage)

    left_refs = _get_reference_asset_paths(left_prim)
    right_refs = _get_reference_asset_paths(right_prim)
    left_meshes = _collect_world_mesh_points(left_prim)
    right_meshes = _collect_world_mesh_points(right_prim)

    if not left_meshes:
        raise RuntimeError(f"no_meshes_under_left_prim: {left_prim_path}")
    if not right_meshes:
        raise RuntimeError(f"no_meshes_under_right_prim: {right_prim_path}")

    mesh_report = _compare_meshes(left_meshes, right_meshes)
    left_all = _aggregate_points(left_meshes)
    right_all = _aggregate_points(right_meshes)

    if len(left_all) != len(right_all):
        aggregate_vertex = {
            "point_count_mismatch": {
                "left_points": int(len(left_all)),
                "right_points": int(len(right_all)),
            }
        }
    else:
        all_deltas = np.linalg.norm(left_all - right_all, axis=1)
        max_idx = int(np.argmax(all_deltas)) if len(all_deltas) else -1
        aggregate_vertex = {
            "mean_vertex_delta": round(float(all_deltas.mean()), 6),
            "max_vertex_delta": round(float(all_deltas.max()), 6),
            "max_vertex_index": max_idx,
            "max_vertex_left": (
                _round_vec3(left_all[max_idx]) if max_idx >= 0 else None
            ),
            "max_vertex_right": (
                _round_vec3(right_all[max_idx]) if max_idx >= 0 else None
            ),
        }

    aggregate = {
        "left_point_count": int(len(left_all)),
        "right_point_count": int(len(right_all)),
        "left_mesh_count": int(len(left_meshes)),
        "right_mesh_count": int(len(right_meshes)),
        "asset_centroid_left": _round_vec3(_centroid(left_all)),
        "asset_centroid_right": _round_vec3(_centroid(right_all)),
        "asset_centroid_delta": round(
            float(np.linalg.norm(_centroid(left_all) - _centroid(right_all))), 6
        ),
        "asset_bbox_mid_left": _round_vec3(_bbox_mid(left_all)),
        "asset_bbox_mid_right": _round_vec3(_bbox_mid(right_all)),
        "asset_bbox_mid_delta": round(
            float(np.linalg.norm(_bbox_mid(left_all) - _bbox_mid(right_all))), 6
        ),
        **aggregate_vertex,
    }

    return {
        "left_layout": _norm_abs(left_layout),
        "right_layout": _norm_abs(right_layout),
        "left_prim_path": left_prim_path,
        "right_prim_path": right_prim_path,
        "left_refs": left_refs[:5],
        "right_refs": right_refs[:5],
        "aggregate": aggregate,
        "mesh_report": {
            "common_mesh_count": mesh_report["common_mesh_count"],
            "left_only_mesh_count": mesh_report["left_only_mesh_count"],
            "right_only_mesh_count": mesh_report["right_only_mesh_count"],
            "left_only_meshes": mesh_report["left_only_meshes"],
            "right_only_meshes": mesh_report["right_only_meshes"],
            "mismatched_point_counts": mesh_report["mismatched_point_counts"],
            "top_20_worst_by_max_vertex": mesh_report["per_mesh"][:20],
        },
    }


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Compare world-space mesh geometry for one scene prim pair. "
            "Defaults target the known refrigerator normalize outlier."
        )
    )
    parser.add_argument("--left-layout", default=_DEFAULT_LEFT_LAYOUT)
    parser.add_argument("--right-layout", default=_DEFAULT_RIGHT_LAYOUT)
    parser.add_argument("--left-prim-path", default=_DEFAULT_PRIM_PATH)
    parser.add_argument("--right-prim-path", default=_DEFAULT_PRIM_PATH)
    parser.add_argument(
        "--out",
        default=None,
        help="Optional JSON output path. Prints to stdout if omitted.",
    )
    parser.add_argument(
        "--indent",
        type=int,
        default=2,
        help="JSON indentation level for stdout/file output.",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_arg_parser()
    args = parser.parse_args(argv)

    try:
        report = diagnose(
            left_layout=args.left_layout,
            right_layout=args.right_layout,
            left_prim_path=args.left_prim_path,
            right_prim_path=args.right_prim_path,
        )
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 1

    payload = json.dumps(report, indent=args.indent, sort_keys=False)
    if args.out:
        out_path = _norm_abs(args.out)
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, "w", encoding="utf-8") as fh:
            fh.write(payload)
            fh.write("\n")
        print(out_path)
    else:
        print(payload)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
