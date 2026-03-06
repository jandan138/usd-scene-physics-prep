#!/usr/bin/env python3
"""Verify that asset normalization was performed correctly.

Checks the invariants that should hold after running normalize_asset_transforms.py:

Asset checks:
  - Stage upAxis == Z
  - /Root/Instance exists
  - /Root/Instance has only a scale transform (no rotation/translation)
  - Bounding-box center of all meshes (in /Root space) is approximately (0,0,0)

Scene layout checks:
  - Each prim with authored references has a valid xformOp:transform (Matrix4d)

Usage:
  python3 scripts/verify_normalized_assets.py \
      --assets-root GRScenes-test1-normalized/GRScenes_assets \
      [--scenes-root GRScenes-test1-normalized/GRScenes100] \
      [--category plate] \
      [--tolerance 0.01] \
      [--report-dir check_reports/normalize]
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

try:
    from pxr import Gf, Sdf, Usd, UsdGeom
except ImportError as e:
    print(f"Error: pxr (USD) not available: {e}", file=sys.stderr)
    print("Run with a Python environment that has USD installed.", file=sys.stderr)
    sys.exit(1)


# ---------------------------------------------------------------------------
# Helpers (mirrored from normalize_asset_transforms.py)
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


def _find_all_meshes(root_prim) -> list:
    """Find all UsdGeom.Mesh prims under root_prim with non-empty points."""
    meshes = []
    for prim in Usd.PrimRange(root_prim):
        mesh = UsdGeom.Mesh(prim)
        if mesh:
            pts = mesh.GetPointsAttr().Get()
            if pts and len(pts) > 0:
                meshes.append(prim)
    return meshes


def _get_chain_transform(ancestor, descendant) -> Gf.Matrix4d:
    """Compute accumulated local transform from ancestor's child down to descendant."""
    chain = []
    cur = descendant
    while cur and cur.GetPath() != ancestor.GetPath():
        chain.append(cur)
        cur = cur.GetParent()
    chain.reverse()
    result = Gf.Matrix4d(1.0)
    for p in chain:
        result = result * _get_local_matrix(p)
    return result


def _is_scale_only_matrix(m: Gf.Matrix4d, eps: float = 1e-9) -> Tuple[bool, str]:
    """Check if a 4x4 matrix is a pure scale (diagonal + no translation).

    A pure scale matrix looks like:
        [sx  0  0  0]
        [ 0 sy  0  0]
        [ 0  0 sz  0]
        [ 0  0  0  1]
    """
    # Check off-diagonal elements in the 3x3 rotation block
    for r in range(3):
        for c in range(3):
            if r != c:
                val = m[r][c]
                if abs(val) > eps:
                    return False, f"non-zero off-diagonal m[{r}][{c}]={val:.6e}"

    # Check translation row (row 3, cols 0-2 in USD row-major)
    tx, ty, tz = m[3][0], m[3][1], m[3][2]
    if abs(tx) > eps or abs(ty) > eps or abs(tz) > eps:
        return False, f"non-zero translation ({tx:.6e}, {ty:.6e}, {tz:.6e})"

    # Check m[3][3] == 1
    if abs(m[3][3] - 1.0) > eps:
        return False, f"m[3][3]={m[3][3]:.6e} (expected 1.0)"

    return True, ""


# ---------------------------------------------------------------------------
# Discovery (same logic as normalize_asset_transforms.py)
# ---------------------------------------------------------------------------

def discover_assets(assets_root: str, category: Optional[str] = None) -> List[Tuple[str, str, str]]:
    """Discover asset USD files. Returns list of (category, uid, usd_path)."""
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
            continue
        for uid in sorted(os.listdir(cat_dir)):
            usd_dir = os.path.join(cat_dir, uid, "usd")
            if not os.path.isdir(usd_dir):
                continue
            usd_file = os.path.join(usd_dir, f"{uid}.usd")
            if os.path.isfile(usd_file):
                results.append((cat, uid, usd_file))

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
# Asset verification
# ---------------------------------------------------------------------------

def verify_asset(usd_path: str, tolerance: float) -> Dict[str, Any]:
    """Verify a single normalized asset USD.

    Returns a dict with 'passed' (bool), 'checks' (dict of check results),
    and 'errors' (list of failure messages).
    """
    result: Dict[str, Any] = {
        "file": usd_path,
        "passed": True,
        "checks": {},
        "errors": [],
    }

    # Open stage
    try:
        stage = Usd.Stage.Open(usd_path)
    except Exception as e:
        result["passed"] = False
        result["errors"].append(f"Failed to open USD: {e}")
        return result

    if stage is None:
        result["passed"] = False
        result["errors"].append("Failed to open USD (returned None)")
        return result

    # Check 1: upAxis == Z
    up_axis = UsdGeom.GetStageUpAxis(stage)
    up_ok = up_axis == UsdGeom.Tokens.z
    result["checks"]["upAxis"] = {
        "expected": "Z",
        "actual": str(up_axis),
        "passed": up_ok,
    }
    if not up_ok:
        result["passed"] = False
        result["errors"].append(f"upAxis is '{up_axis}', expected 'Z'")

    # Check 2: /Root/Instance exists
    instance = stage.GetPrimAtPath("/Root/Instance")
    instance_exists = instance and instance.IsValid()
    result["checks"]["instance_exists"] = {"passed": instance_exists}
    if not instance_exists:
        result["passed"] = False
        result["errors"].append("/Root/Instance prim not found or invalid")
        return result  # Cannot continue without Instance

    # Check 3: /Root/Instance has a scale-only transform
    M_instance = _get_local_matrix(instance)
    scale_ok, scale_detail = _is_scale_only_matrix(M_instance)
    result["checks"]["instance_scale_only"] = {
        "passed": scale_ok,
        "detail": scale_detail if not scale_ok else "pure scale matrix",
    }
    if not scale_ok:
        result["passed"] = False
        result["errors"].append(f"/Root/Instance transform is not scale-only: {scale_detail}")

    # Check 4: Bounding-box center approximately (0,0,0)
    mesh_prims = _find_all_meshes(instance)
    if not mesh_prims:
        result["checks"]["bbox_center"] = {
            "passed": False,
            "detail": "no meshes found under /Root/Instance",
        }
        result["passed"] = False
        result["errors"].append("No meshes found under /Root/Instance")
        return result

    # Compute bbox in /Root space (Instance transform applied to mesh points)
    all_points: List[Gf.Vec3d] = []
    for mesh_prim in mesh_prims:
        mesh = UsdGeom.Mesh(mesh_prim)
        points = mesh.GetPointsAttr().Get()
        if not points:
            continue
        # Chain transform from Instance to mesh (including Instance itself)
        M_chain = _get_chain_transform(instance, mesh_prim)
        M_full = M_instance * M_chain
        for p in points:
            p_root = M_full.Transform(Gf.Vec3d(p[0], p[1], p[2]))
            all_points.append(p_root)

    if not all_points:
        result["checks"]["bbox_center"] = {
            "passed": False,
            "detail": "no points found in any mesh",
        }
        result["passed"] = False
        result["errors"].append("No points found in any mesh under /Root/Instance")
        return result

    bbox_min = Gf.Vec3d(all_points[0])
    bbox_max = Gf.Vec3d(all_points[0])
    for p in all_points[1:]:
        for i in range(3):
            bbox_min[i] = min(bbox_min[i], p[i])
            bbox_max[i] = max(bbox_max[i], p[i])
    center = (bbox_min + bbox_max) * 0.5

    center_dist = center.GetLength()
    center_ok = center_dist <= tolerance
    result["checks"]["bbox_center"] = {
        "passed": center_ok,
        "center": [round(center[0], 6), round(center[1], 6), round(center[2], 6)],
        "distance_from_origin": round(center_dist, 6),
        "tolerance": tolerance,
        "mesh_count": len(mesh_prims),
        "point_count": len(all_points),
    }
    if not center_ok:
        result["passed"] = False
        result["errors"].append(
            f"Bounding-box center ({center[0]:.4f}, {center[1]:.4f}, {center[2]:.4f}) "
            f"is {center_dist:.4f} from origin (tolerance={tolerance})"
        )

    return result


# ---------------------------------------------------------------------------
# Scene layout verification
# ---------------------------------------------------------------------------

def verify_layout(layout_path: str) -> Dict[str, Any]:
    """Verify a scene layout USD.

    Checks that every prim with authored references has a valid xformOp:transform.
    """
    result: Dict[str, Any] = {
        "file": layout_path,
        "passed": True,
        "ref_prims_checked": 0,
        "errors": [],
    }

    try:
        stage = Usd.Stage.Open(layout_path)
    except Exception as e:
        result["passed"] = False
        result["errors"].append(f"Failed to open USD: {e}")
        return result

    if stage is None:
        result["passed"] = False
        result["errors"].append("Failed to open USD (returned None)")
        return result

    for prim in stage.Traverse():
        if not prim or not prim.IsValid():
            continue
        if not prim.HasAuthoredReferences():
            continue

        result["ref_prims_checked"] += 1
        prim_path = str(prim.GetPath())

        # Check that xformOp:transform exists
        attr = prim.GetAttribute("xformOp:transform")
        if not attr or not attr.IsValid():
            result["passed"] = False
            result["errors"].append(f"{prim_path}: missing xformOp:transform attribute")
            continue

        val = attr.Get()
        if val is None:
            result["passed"] = False
            result["errors"].append(f"{prim_path}: xformOp:transform has no value")
            continue

        # Verify it is a valid Matrix4d
        if not isinstance(val, Gf.Matrix4d):
            result["passed"] = False
            result["errors"].append(
                f"{prim_path}: xformOp:transform is {type(val).__name__}, expected Matrix4d"
            )
            continue

        # Check that xformOpOrder includes xformOp:transform
        order_attr = prim.GetAttribute("xformOpOrder")
        if order_attr and order_attr.IsValid():
            order = order_attr.Get()
            if order is not None and "xformOp:transform" not in list(order):
                result["passed"] = False
                result["errors"].append(
                    f"{prim_path}: xformOpOrder={list(order)} does not include 'xformOp:transform'"
                )

    return result


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Verify normalization invariants on asset and scene USD files.",
    )
    ap.add_argument("--assets-root", required=True,
                     help="Root directory of normalized assets")
    ap.add_argument("--scenes-root", default=None,
                     help="Root directory of compensated scenes (optional)")
    ap.add_argument("--category", default=None,
                     help="Only verify this category (e.g. 'plate')")
    ap.add_argument("--tolerance", type=float, default=0.01,
                     help="Max distance of bbox center from origin (default: 0.01)")
    ap.add_argument("--report-dir", default=None,
                     help="Directory for JSON report output")

    args = ap.parse_args(argv)

    assets_root = os.path.abspath(args.assets_root)

    # -----------------------------------------------------------------------
    # Verify assets
    # -----------------------------------------------------------------------
    print("=" * 60)
    print("Verifying normalized assets")
    print("=" * 60)
    print(f"Assets root: {assets_root}")
    if args.category:
        print(f"Category filter: {args.category}")
    print(f"Tolerance: {args.tolerance}")
    print()

    asset_list = discover_assets(assets_root, args.category)
    print(f"Found {len(asset_list)} assets to verify")

    asset_results: Dict[str, Any] = {}
    assets_passed = 0
    assets_failed = 0

    for i, (cat, uid, usd_path) in enumerate(asset_list):
        if (i + 1) % 500 == 0 or i == 0:
            print(f"  [{i+1}/{len(asset_list)}] {cat}/{uid}")

        res = verify_asset(usd_path, args.tolerance)
        key = f"{cat}/{uid}"
        asset_results[key] = res
        if res["passed"]:
            assets_passed += 1
        else:
            assets_failed += 1
            if assets_failed <= 20:
                for err in res["errors"]:
                    print(f"  FAIL {key}: {err}")

    print(f"\nAssets: {assets_passed} passed, {assets_failed} failed, "
          f"{len(asset_list)} total")

    # -----------------------------------------------------------------------
    # Verify scene layouts (optional)
    # -----------------------------------------------------------------------
    layout_results: Dict[str, Any] = {}
    layouts_passed = 0
    layouts_failed = 0

    if args.scenes_root:
        scenes_root = os.path.abspath(args.scenes_root)
        print()
        print("=" * 60)
        print("Verifying scene layouts")
        print("=" * 60)
        print(f"Scenes root: {scenes_root}")
        print()

        layout_list = discover_layouts(scenes_root)
        print(f"Found {len(layout_list)} layouts to verify")

        for i, layout_path in enumerate(layout_list):
            rel = os.path.relpath(layout_path, scenes_root)
            if (i + 1) % 10 == 0 or i == 0:
                print(f"  [{i+1}/{len(layout_list)}] {rel}")

            res = verify_layout(layout_path)
            layout_results[rel] = res
            if res["passed"]:
                layouts_passed += 1
            else:
                layouts_failed += 1
                if layouts_failed <= 20:
                    for err in res["errors"]:
                        print(f"  FAIL {rel}: {err}")

        print(f"\nLayouts: {layouts_passed} passed, {layouts_failed} failed, "
              f"{len(layout_list)} total")

    # -----------------------------------------------------------------------
    # Summary and report
    # -----------------------------------------------------------------------
    total_passed = assets_passed + layouts_passed
    total_failed = assets_failed + layouts_failed
    total = len(asset_list) + len(layout_results)

    report = {
        "summary": {
            "assets_passed": assets_passed,
            "assets_failed": assets_failed,
            "assets_total": len(asset_list),
            "layouts_passed": layouts_passed,
            "layouts_failed": layouts_failed,
            "layouts_total": len(layout_results),
            "total_passed": total_passed,
            "total_failed": total_failed,
            "total": total,
            "all_passed": total_failed == 0,
            "tolerance": args.tolerance,
        },
        "assets": asset_results,
        "layouts": layout_results,
    }

    print()
    overall = "PASS" if total_failed == 0 else "FAIL"
    print(f"Overall: {overall} ({total_passed}/{total} passed)")

    # Write report
    if args.report_dir:
        os.makedirs(args.report_dir, exist_ok=True)
        report_path = os.path.join(args.report_dir, "verification_report.json")
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False, default=str)
        print(f"Report written to: {report_path}")

    return 0 if total_failed == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
