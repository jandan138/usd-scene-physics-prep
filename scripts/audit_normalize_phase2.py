#!/usr/bin/env python3
"""Audit normalize_asset_transforms.py Phase 2 scene compensation.

This script does not modify data. It reconstructs how Phase 2 should have
matched assets / centers, then compares the expected scene-local transform
against the actual transform authored in a pre-dedup normalized layout.

The goal is to separate:
  - path extraction / asset-center lookup coverage issues
  - skipped prims
  - whether the actual authored matrix matches the implemented formula
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import sys
import time
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    from pxr import Gf, Usd, UsdGeom
except ImportError as exc:
    print(f"Error: pxr (USD) not available: {exc}", file=sys.stderr)
    print("Run via ./scripts/isaac_python.sh", file=sys.stderr)
    sys.exit(1)


R_Y2Z_INV = Gf.Matrix4d(1.0)
R_Y2Z_INV.SetRotate(Gf.Rotation(Gf.Vec3d(1, 0, 0), -90))

MATRIX_TOL = 1e-6
XFORM_PROPS = {
    "xformOpOrder",
    "xformOp:transform",
    "xformOp:translate",
    "xformOp:orient",
    "xformOp:scale",
}


def _resolve_ref_asset_path(base_dir: str, asset_path: str) -> str:
    if not asset_path:
        return ""
    s = asset_path.replace("\\", "/")
    if os.path.isabs(s):
        return os.path.normpath(s)
    return os.path.normpath(os.path.join(base_dir, s))


def _get_local_matrix(prim) -> Gf.Matrix4d:
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return Gf.Matrix4d(1.0)
    res = xf.GetLocalTransformation(Usd.TimeCode.Default())
    if isinstance(res, tuple):
        return res[0]
    return res


def _matrix_max_abs_diff(a: Gf.Matrix4d, b: Gf.Matrix4d) -> float:
    max_diff = 0.0
    for r in range(4):
        for c in range(4):
            max_diff = max(max_diff, abs(a[r][c] - b[r][c]))
    return max_diff


def _extract_rel_key(
    asset_path: str,
    resolved_abs: str,
    assets_root_abs: str,
    output_assets_root_abs: str,
) -> Tuple[Optional[str], str]:
    if resolved_abs.startswith(assets_root_abs + "/"):
        return resolved_abs[len(assets_root_abs) + 1:], "resolved_under_assets_root"
    if resolved_abs.startswith(output_assets_root_abs + "/"):
        return resolved_abs[len(output_assets_root_abs) + 1:], "resolved_under_output_assets_root"

    parts = asset_path.replace("\\", "/").split("/")
    try:
        idx = parts.index("GRScenes_assets")
    except ValueError:
        return None, "path_extract_failed"
    return "/".join(parts[idx + 1:]), "extracted_from_asset_path"


def _select_earliest_pre_c1(scene_dir: str) -> Optional[str]:
    candidates = [
        f for f in os.listdir(scene_dir)
        if f.startswith("layout.pre_c1_") and f.endswith(".usd")
    ]
    if not candidates:
        return None

    def extract_timestamp(name: str) -> str:
        m = re.search(r"\.(\d{8}_\d{6})\.usd$", name)
        return m.group(1) if m else ""

    candidates.sort(key=extract_timestamp)
    return os.path.join(scene_dir, candidates[0])


def _read_first_reference_for_prims(layout_path: str) -> Dict[str, str]:
    refs: Dict[str, str] = {}
    stage = Usd.Stage.Open(layout_path, load=Usd.Stage.LoadNone)
    if stage is None:
        return refs
    base_dir = os.path.dirname(os.path.abspath(layout_path))
    for prim in stage.Traverse():
        if not prim or not prim.IsValid() or not prim.HasAuthoredReferences():
            continue
        ref_meta = prim.GetMetadata("references")
        if not ref_meta:
            continue
        try:
            items = list(ref_meta.GetAddedOrExplicitItems())
        except Exception:
            continue
        for item in items:
            asset_path = getattr(item, "assetPath", "") or ""
            if not asset_path:
                continue
            refs[str(prim.GetPath())] = _resolve_ref_asset_path(base_dir, asset_path)
            break
    return refs


def _audit_descendant_xform_overrides(stage, scene_path: str) -> List[Dict[str, object]]:
    findings: List[Dict[str, object]] = []
    scene_path = os.path.abspath(scene_path)

    for prim in stage.Traverse():
        if not prim or not prim.IsValid():
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

        if str(referenced_specs[0].path) == "/Root":
            continue

        scene_props = sorted(set(top_spec.properties.keys()) & XFORM_PROPS)
        if not scene_props:
            continue

        xform = UsdGeom.Xformable(prim)
        composed_order = []
        if xform:
            composed_order = [op.GetOpName() for op in xform.GetOrderedXformOps()]

        inert_scene_props = [
            prop for prop in scene_props
            if prop.startswith("xformOp:") and prop not in composed_order
        ]

        findings.append({
            "prim_path": str(prim.GetPath()),
            "scene_layer_props": scene_props,
            "composed_xformOpOrder": composed_order,
            "inert_scene_xform_props": inert_scene_props,
            "asset_layer_path": str(referenced_specs[0].path),
            "asset_layer_identifier": referenced_specs[0].layer.identifier,
        })

    findings.sort(
        key=lambda item: (
            len(item["inert_scene_xform_props"]),
            item["prim_path"],
        ),
        reverse=True,
    )
    return findings


def _load_centers(centers_dir: str) -> Dict[str, Gf.Vec3d]:
    asset_centers: Dict[str, Gf.Vec3d] = {}
    center_files = sorted(glob.glob(os.path.join(centers_dir, "centers_*.json")))
    for path in center_files:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        for rel_key, center in data.items():
            asset_centers[rel_key] = Gf.Vec3d(*center)
    return asset_centers


def _scene_dirs(dataset_root: str) -> Dict[str, str]:
    mapping = {}
    for subcategory in ("home", "commercial"):
        base = os.path.join(dataset_root, "GRScenes100", subcategory)
        if not os.path.isdir(base):
            continue
        for scene_id in sorted(os.listdir(base)):
            scene_dir = os.path.join(base, scene_id)
            if os.path.isdir(scene_dir):
                mapping[f"{subcategory}/{scene_id}"] = scene_dir
    return mapping


def audit_scene(
    scene_id: str,
    src_layout: str,
    pre_c1_layout: str,
    assets_root_abs: str,
    output_assets_root_abs: str,
    asset_centers_json: Dict[str, Tuple[float, float, float]],
) -> Dict[str, object]:
    stage_src = Usd.Stage.Open(src_layout, load=Usd.Stage.LoadNone)
    stage_pre = Usd.Stage.Open(pre_c1_layout, load=Usd.Stage.LoadNone)
    if stage_src is None or stage_pre is None:
        return {
            "scene_id": scene_id,
            "status": "error",
            "error": "failed_to_open_scene",
        }

    src_base_dir = os.path.dirname(os.path.abspath(src_layout))
    src_refs = _read_first_reference_for_prims(src_layout)
    pre_refs = _read_first_reference_for_prims(pre_c1_layout)
    src_descendant_findings = _audit_descendant_xform_overrides(stage_src, src_layout)
    pre_descendant_findings = _audit_descendant_xform_overrides(stage_pre, pre_c1_layout)

    src_prims = {str(prim.GetPath()): prim for prim in stage_src.Traverse() if prim and prim.IsValid()}
    pre_prims = {str(prim.GetPath()): prim for prim in stage_pre.Traverse() if prim and prim.IsValid()}

    common_paths = sorted(set(src_refs.keys()) & set(pre_refs.keys()))
    only_in_src = sorted(set(src_refs.keys()) - set(pre_refs.keys()))
    only_in_pre = sorted(set(pre_refs.keys()) - set(src_refs.keys()))

    counters = defaultdict(int)
    matrix_diffs: List[float] = []
    top_mismatches = []
    counters["source_descendant_xform_override_count"] = len(src_descendant_findings)
    counters["pre_c1_descendant_xform_override_count"] = len(pre_descendant_findings)
    counters["pre_c1_inert_descendant_xform_override_count"] = sum(
        1 for item in pre_descendant_findings if item.get("inert_scene_xform_props")
    )

    src_descendant_by_path = {
        item["prim_path"]: item for item in src_descendant_findings
    }
    pre_descendant_by_path = {
        item["prim_path"]: item for item in pre_descendant_findings
    }
    for prim_path, pre_item in pre_descendant_by_path.items():
        if prim_path in src_descendant_by_path and pre_item.get("inert_scene_xform_props"):
            counters["became_inert_descendant_xform_override_count"] += 1

    for prim_path in common_paths:
        src_prim = src_prims.get(prim_path)
        pre_prim = pre_prims.get(prim_path)
        if src_prim is None or pre_prim is None:
            counters["missing_prim_handle"] += 1
            continue

        src_ref_meta = src_prim.GetMetadata("references")
        if not src_ref_meta:
            counters["missing_src_ref_meta"] += 1
            continue

        try:
            items = list(src_ref_meta.GetAddedOrExplicitItems())
        except Exception:
            counters["src_ref_iter_error"] += 1
            continue

        asset_path = ""
        for item in items:
            asset_path = getattr(item, "assetPath", "") or ""
            if asset_path:
                break
        if not asset_path:
            counters["empty_src_ref"] += 1
            continue

        resolved = _resolve_ref_asset_path(src_base_dir, asset_path)
        rel_key, resolution_kind = _extract_rel_key(
            asset_path=asset_path,
            resolved_abs=resolved,
            assets_root_abs=assets_root_abs,
            output_assets_root_abs=output_assets_root_abs,
        )
        counters[resolution_kind] += 1

        if rel_key is None:
            counters["skipped_no_rel_key"] += 1
            continue
        if rel_key not in asset_centers_json:
            counters["skipped_missing_center"] += 1
            continue

        counters["center_found"] += 1
        center = Gf.Vec3d(*asset_centers_json[rel_key])

        src_local = _get_local_matrix(src_prim)
        pre_local = _get_local_matrix(pre_prim)

        t_center = Gf.Matrix4d(1.0)
        t_center.SetTranslate(center)
        expected = t_center * R_Y2Z_INV * src_local

        max_diff = _matrix_max_abs_diff(expected, pre_local)
        matrix_diffs.append(max_diff)
        if max_diff <= MATRIX_TOL:
            counters["matrix_match"] += 1
        else:
            counters["matrix_mismatch"] += 1
            if len(top_mismatches) < 30:
                top_mismatches.append({
                    "prim_path": prim_path,
                    "rel_key": rel_key,
                    "max_abs_diff": max_diff,
                    "src_ref": src_refs.get(prim_path),
                    "pre_ref": pre_refs.get(prim_path),
                })

    matrix_stats = {}
    if matrix_diffs:
        matrix_stats = {
            "mean_max_abs_diff": round(float(sum(matrix_diffs) / len(matrix_diffs)), 9),
            "max_max_abs_diff": round(float(max(matrix_diffs)), 9),
            "mismatch_gt_tol": counters["matrix_mismatch"],
        }

    return {
        "scene_id": scene_id,
        "status": "ok",
        "src_layout": src_layout,
        "pre_c1_layout": pre_c1_layout,
        "source_ref_prim_count": len(src_refs),
        "pre_c1_ref_prim_count": len(pre_refs),
        "common_ref_prim_count": len(common_paths),
        "only_in_src_count": len(only_in_src),
        "only_in_pre_c1_count": len(only_in_pre),
        "only_in_src_paths": only_in_src[:20],
        "only_in_pre_c1_paths": only_in_pre[:20],
        "counters": dict(sorted(counters.items())),
        "matrix_stats": matrix_stats,
        "top_mismatches": sorted(top_mismatches, key=lambda x: x["max_abs_diff"], reverse=True),
        "source_descendant_xform_overrides": src_descendant_findings[:20],
        "pre_c1_descendant_xform_overrides": pre_descendant_findings[:20],
    }


def _aggregate(scene_results: Iterable[Dict[str, object]]) -> Dict[str, object]:
    ok_results = [r for r in scene_results if r.get("status") == "ok"]
    error_results = [r for r in scene_results if r.get("status") != "ok"]
    totals = defaultdict(int)
    max_diff = 0.0
    mismatch_examples = []
    descendant_override_examples = []

    for result in ok_results:
        totals["source_ref_prim_count"] += int(result.get("source_ref_prim_count", 0))
        totals["pre_c1_ref_prim_count"] += int(result.get("pre_c1_ref_prim_count", 0))
        totals["common_ref_prim_count"] += int(result.get("common_ref_prim_count", 0))
        totals["only_in_src_count"] += int(result.get("only_in_src_count", 0))
        totals["only_in_pre_c1_count"] += int(result.get("only_in_pre_c1_count", 0))
        for key, value in result.get("counters", {}).items():
            totals[key] += int(value)
        max_diff = max(max_diff, float(result.get("matrix_stats", {}).get("max_max_abs_diff", 0.0)))
        for item in result.get("top_mismatches", []):
            mismatch_examples.append({
                "scene_id": result["scene_id"],
                **item,
            })
        for item in result.get("pre_c1_descendant_xform_overrides", []):
            if item.get("inert_scene_xform_props"):
                descendant_override_examples.append({
                    "scene_id": result["scene_id"],
                    **item,
                })

    mismatch_examples.sort(key=lambda x: x["max_abs_diff"], reverse=True)
    descendant_override_examples.sort(
        key=lambda x: (len(x.get("inert_scene_xform_props", [])), x["prim_path"]),
        reverse=True,
    )

    return {
        "scenes_ok": len(ok_results),
        "scenes_error": len(error_results),
        "totals": dict(sorted(totals.items())),
        "max_matrix_abs_diff": round(max_diff, 9),
        "top_50_mismatches": mismatch_examples[:50],
        "top_50_inert_descendant_xform_overrides": descendant_override_examples[:50],
        "scene_errors": [
            {"scene_id": r.get("scene_id"), "error": r.get("error")}
            for r in error_results
        ],
    }


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Audit normalize_asset_transforms.py Phase 2 on pre-dedup scenes.",
    )
    parser.add_argument("--source-root", required=True, help="Original test1 dataset root")
    parser.add_argument("--normalized-root", required=True, help="Normalized dataset root")
    parser.add_argument("--centers-dir", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--scene-filter", default=None)
    parser.add_argument("--workers", type=int, default=8)
    args = parser.parse_args(argv)

    source_root = os.path.abspath(args.source_root)
    normalized_root = os.path.abspath(args.normalized_root)
    centers_dir = os.path.abspath(args.centers_dir)
    assets_root_abs = os.path.join(source_root, "GRScenes_assets")
    output_assets_root_abs = os.path.join(normalized_root, "GRScenes_assets")

    asset_centers = _load_centers(centers_dir)
    asset_centers_json = {k: (float(v[0]), float(v[1]), float(v[2])) for k, v in asset_centers.items()}
    print(f"Loaded {len(asset_centers_json)} asset centers from {centers_dir}")

    src_scene_dirs = _scene_dirs(source_root)
    norm_scene_dirs = _scene_dirs(normalized_root)

    tasks = []
    missing_pre_c1 = []
    for scene_id, src_scene_dir in sorted(src_scene_dirs.items()):
        if args.scene_filter and args.scene_filter not in scene_id:
            continue
        norm_scene_dir = norm_scene_dirs.get(scene_id)
        if not norm_scene_dir:
            continue
        src_layout = os.path.join(src_scene_dir, "layout.usd")
        pre_c1_layout = _select_earliest_pre_c1(norm_scene_dir)
        if not pre_c1_layout:
            missing_pre_c1.append(scene_id)
            continue
        tasks.append((scene_id, src_layout, pre_c1_layout))

    print(f"Scenes with source+pre_c1 pair: {len(tasks)}")
    print(f"Scenes missing pre_c1 backup:   {len(missing_pre_c1)}")

    scene_results = []
    t0 = time.time()

    if args.workers <= 1:
        for idx, (scene_id, src_layout, pre_c1_layout) in enumerate(tasks, start=1):
            print(f"[{idx}/{len(tasks)}] {scene_id}")
            scene_results.append(
                audit_scene(
                    scene_id=scene_id,
                    src_layout=src_layout,
                    pre_c1_layout=pre_c1_layout,
                    assets_root_abs=assets_root_abs,
                    output_assets_root_abs=output_assets_root_abs,
                    asset_centers_json=asset_centers_json,
                )
            )
    else:
        with ProcessPoolExecutor(max_workers=args.workers) as executor:
            futures = {
                executor.submit(
                    audit_scene,
                    scene_id,
                    src_layout,
                    pre_c1_layout,
                    assets_root_abs,
                    output_assets_root_abs,
                    asset_centers_json,
                ): scene_id
                for scene_id, src_layout, pre_c1_layout in tasks
            }
            done = 0
            for future in as_completed(futures):
                done += 1
                scene_id = futures[future]
                try:
                    result = future.result()
                except Exception as exc:
                    result = {
                        "scene_id": scene_id,
                        "status": "error",
                        "error": f"worker_exception: {exc}",
                    }
                scene_results.append(result)
                if result.get("status") == "ok":
                    found = result.get("counters", {}).get("center_found", 0)
                    mismatch = result.get("counters", {}).get("matrix_mismatch", 0)
                    print(
                        f"  [{done}/{len(tasks)}] {scene_id}: ok "
                        f"center_found={found} matrix_mismatch={mismatch}"
                    )
                else:
                    print(f"  [{done}/{len(tasks)}] {scene_id}: ERROR {result.get('error')}")

    scene_results.sort(key=lambda r: r["scene_id"])
    aggregate = _aggregate(scene_results)
    output = {
        "source_root": source_root,
        "normalized_root": normalized_root,
        "centers_dir": centers_dir,
        "loaded_asset_centers": len(asset_centers_json),
        "matched_scene_count": len(tasks),
        "missing_pre_c1_scenes": missing_pre_c1,
        "elapsed_seconds": round(time.time() - t0, 1),
        "aggregate": aggregate,
        "per_scene": scene_results,
    }

    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with open(args.out, "w", encoding="utf-8") as f:
        json.dump(output, f, indent=2, ensure_ascii=False)

    print()
    print("=" * 72)
    print("NORMALIZE PHASE-2 AUDIT SUMMARY")
    print("=" * 72)
    print(f"Scenes ok/error:          {aggregate['scenes_ok']}/{aggregate['scenes_error']}")
    print(f"Common ref prims:         {aggregate['totals'].get('common_ref_prim_count', 0)}")
    print(f"Center found:             {aggregate['totals'].get('center_found', 0)}")
    print(f"Skipped no rel key:       {aggregate['totals'].get('skipped_no_rel_key', 0)}")
    print(f"Skipped missing center:   {aggregate['totals'].get('skipped_missing_center', 0)}")
    print(f"Matrix match/mismatch:    {aggregate['totals'].get('matrix_match', 0)}/{aggregate['totals'].get('matrix_mismatch', 0)}")
    print(f"Source descendant xform:  {aggregate['totals'].get('source_descendant_xform_override_count', 0)}")
    print(f"Pre_c1 inert descendant:  {aggregate['totals'].get('pre_c1_inert_descendant_xform_override_count', 0)}")
    print(f"Became inert descendant:  {aggregate['totals'].get('became_inert_descendant_xform_override_count', 0)}")
    print(f"Max matrix abs diff:      {aggregate['max_matrix_abs_diff']}")
    print(f"Report saved to:          {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
