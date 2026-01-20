#!/usr/bin/env python3

"""Generate a Blender-friendly layout.json from a GRScenes scene layout USD.

This script reads a scene layout USD (typically: GRScenes100/<cat>/<scene_uid>_usd/layout.usd),
finds placed object assets under GRScenes_assets/<category>/<uid>/, and writes a JSON file
that indexes the corresponding per-asset GLBs:

  GRScenes_assets/<category>/<uid>/glb/<uid>.glb

Design goals:
- Only targets *object assets* (GRScenes_assets). Scene-only geometry is intentionally ignored.
- No coordinate axis conversion (can be added later in the Blender importer if needed).
- No PointInstancer expansion (per user requirement).

Run in an environment with pxr (Isaac Sim python recommended):

  ./scripts/isaac_python.sh scripts/generate_layout_json_from_usd.py \
    --layout-usd sandbox/scene_subset_<SID>/GRScenes100/home/<SID>_usd/layout.usd \
    --subset-root sandbox/scene_subset_<SID>

"""

from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import re
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

try:
    from pxr import Gf, Sdf, Usd, UsdGeom

    _PXR_IMPORT_ERROR: Optional[Exception] = None
except Exception as e:  # pragma: no cover
    Gf = None  # type: ignore[assignment]
    Sdf = None  # type: ignore[assignment]
    Usd = None  # type: ignore[assignment]
    UsdGeom = None  # type: ignore[assignment]
    _PXR_IMPORT_ERROR = e


@dataclass(frozen=True)
class AssetRef:
    category: str
    uid: str


_GRSCENES_ASSETS_RE = re.compile(r"(?:^|/)GRScenes_assets/([^/]+)/([^/]+)/")


def _norm_abs(p: str) -> str:
    return os.path.abspath(os.path.normpath(p))


def _posix_relpath(path: str) -> str:
    return path.replace(os.sep, "/")


def _parse_asset_ref_from_path(path_str: str) -> Optional[AssetRef]:
    s = (path_str or "").replace("\\", "/")
    m = _GRSCENES_ASSETS_RE.search(s)
    if not m:
        return None
    return AssetRef(category=m.group(1), uid=m.group(2))


def _resolve_maybe_relative(base_dir: str, path_str: str) -> str:
    if not path_str:
        return ""
    if os.path.isabs(path_str):
        return os.path.normpath(path_str)
    return os.path.normpath(os.path.join(base_dir, path_str))


def _find_subset_root_from_layout(layout_usd: str) -> Optional[str]:
    """Try to infer subset root by walking parents until we see GRScenes_assets."""

    p = _norm_abs(os.path.dirname(layout_usd))
    for _ in range(12):
        if os.path.isdir(os.path.join(p, "GRScenes_assets")):
            return p
        parent = os.path.dirname(p)
        if parent == p:
            break
        p = parent
    return None


def _infer_scene_uid_and_category(layout_usd: str) -> Tuple[Optional[str], Optional[str]]:
    """Infer (scene_uid, scene_category) from a path containing GRScenes100/<cat>/<sid>_usd/."""

    parts = _posix_relpath(_norm_abs(layout_usd)).split("/")
    try:
        i = parts.index("GRScenes100")
    except ValueError:
        return None, None
    if i + 2 >= len(parts):
        return None, None
    cat = parts[i + 1]
    scene_dir = parts[i + 2]
    scene_uid = scene_dir
    if scene_uid.endswith("_usd"):
        scene_uid = scene_uid[: -len("_usd")]
    return scene_uid, cat


def _iter_asset_paths_from_prim(prim) -> List[str]:
    """Collect asset paths (strings) that may reference a GRScenes asset USD for this prim."""

    paths: List[str] = []

    # 1) references
    if prim.HasAuthoredReferences():
        refs = prim.GetMetadata("references")
        if refs:
            for ref in refs.GetAddedOrExplicitItems():
                ap = getattr(ref, "assetPath", "")
                if ap:
                    paths.append(ap)

    # 2) payloads
    if prim.HasAuthoredPayloads():
        payloads = prim.GetMetadata("payloads")
        if payloads:
            for pl in payloads.GetAddedOrExplicitItems():
                ap = getattr(pl, "assetPath", "")
                if ap:
                    paths.append(ap)

    # 3) asset-valued attributes (fallback)
    for attr in prim.GetAttributes():
        try:
            val = attr.Get()
        except Exception:
            continue
        if isinstance(val, Sdf.AssetPath):
            ap = val.path
            if ap and ap.lower().endswith((".usd", ".usda", ".usdc")):
                paths.append(ap)

    return paths


def _score_candidate_path(path_str: str, ref: AssetRef) -> int:
    s = (path_str or "").replace("\\", "/")
    score = 0
    if "/usd/" in s:
        score += 10
    if s.lower().endswith(f"/usd/{ref.uid.lower()}.usd"):
        score += 10
    if os.path.basename(s).lower() == f"{ref.uid.lower()}.usd":
        score += 5
    return score


def _choose_best_asset_ref(base_dir: str, raw_paths: Sequence[str]) -> Optional[Tuple[AssetRef, str]]:
    """Choose best (AssetRef, resolved_usd_path) from a list of raw asset paths."""

    best: Optional[Tuple[int, AssetRef, str]] = None

    for p in raw_paths:
        if not p:
            continue
        resolved = _resolve_maybe_relative(base_dir, p)
        for candidate in (p, resolved):
            ref = _parse_asset_ref_from_path(candidate)
            if ref is None:
                continue
            sc = _score_candidate_path(candidate, ref)
            if best is None or sc > best[0]:
                best = (sc, ref, resolved)

    if best is None:
        return None
    _, ref, resolved = best
    return ref, resolved


def _matrix4d_to_row_major_list(m) -> List[float]:
    # Gf.Matrix4d supports m[row][col]
    out: List[float] = []
    for r in range(4):
        for c in range(4):
            out.append(float(m[r][c]))
    return out


def generate_layout_json(layout_usd: str, subset_root: str, skip_missing_glb: bool, strict: bool) -> Dict[str, object]:
    if _PXR_IMPORT_ERROR is not None:
        raise RuntimeError(f"pxr.Usd not available: {_PXR_IMPORT_ERROR}")

    layout_usd = _norm_abs(layout_usd)
    subset_root = _norm_abs(subset_root)

    stage = Usd.Stage.Open(layout_usd)
    if not stage:
        raise RuntimeError(f"Failed to open stage: {layout_usd}")

    scene_uid, scene_category = _infer_scene_uid_and_category(layout_usd)

    meters_per_unit = None
    up_axis = None
    try:
        meters_per_unit = float(UsdGeom.GetStageMetersPerUnit(stage))
    except Exception:
        pass
    try:
        up_axis = str(UsdGeom.GetStageUpAxis(stage))
    except Exception:
        pass

    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    instances: List[Dict[str, object]] = []

    base_dir = os.path.dirname(layout_usd)

    for prim in stage.Traverse():
        if not prim.IsValid():
            continue

        raw_paths = _iter_asset_paths_from_prim(prim)
        if not raw_paths:
            continue

        chosen = _choose_best_asset_ref(base_dir, raw_paths)
        if chosen is None:
            continue
        asset_ref, resolved_asset_usd = chosen

        # Only keep meaningful placements (Xformable prims).
        try:
            xformable = UsdGeom.Xformable(prim)
            if not xformable:
                continue
        except Exception:
            continue

        try:
            world: Gf.Matrix4d = xform_cache.GetLocalToWorldTransform(prim)
        except Exception:
            continue

        glb_abs = os.path.join(
            subset_root, "GRScenes_assets", asset_ref.category, asset_ref.uid, "glb", f"{asset_ref.uid}.glb"
        )
        glb_rel = _posix_relpath(os.path.relpath(glb_abs, subset_root))
        glb_exists = os.path.isfile(glb_abs)

        if strict and not glb_exists:
            raise FileNotFoundError(f"Missing GLB for {asset_ref.category}/{asset_ref.uid}: {glb_abs}")
        if skip_missing_glb and not glb_exists:
            continue

        inst: Dict[str, object] = {
            "prim_path": str(prim.GetPath()),
            "name": prim.GetName(),
            "category": asset_ref.category,
            "uid": asset_ref.uid,
            "glb": glb_rel,
            "source_asset_usd": _posix_relpath(os.path.relpath(resolved_asset_usd, subset_root))
            if resolved_asset_usd.startswith(subset_root)
            else resolved_asset_usd,
            "transform": {"matrix_row_major": _matrix4d_to_row_major_list(world)},
            "availability": {"glb": glb_exists},
        }
        instances.append(inst)

    instances.sort(key=lambda x: (x.get("prim_path") or ""))

    counts = {
        "instances_total": len(instances),
        "instances_with_glb": sum(1 for i in instances if (i.get("availability") or {}).get("glb")),
        "instances_missing_glb": sum(1 for i in instances if not (i.get("availability") or {}).get("glb")),
    }

    payload: Dict[str, object] = {
        "schema_version": 1,
        "generated_at": _dt.datetime.now(tz=_dt.timezone.utc).isoformat(),
        "subset_root": subset_root,
        "scene_uid": scene_uid,
        "scene_category": scene_category,
        "source_layout_usd": _posix_relpath(os.path.relpath(layout_usd, subset_root)) if layout_usd.startswith(subset_root) else layout_usd,
        "asset_root": "GRScenes_assets",
        "units": {"meters_per_unit": meters_per_unit},
        "up_axis": up_axis,
        "counts": counts,
        "instances": instances,
    }

    return payload


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Generate layout.json from a scene layout.usd (GRScenes_assets instances only).")
    ap.add_argument("--layout-usd", required=True, help="Path to layout.usd")
    ap.add_argument(
        "--subset-root",
        default=None,
        help="Subset/package root that contains GRScenes_assets and GRScenes100. If omitted, inferred from --layout-usd.",
    )
    ap.add_argument(
        "--out",
        default=None,
        help="Output JSON path. Default: alongside layout.usd as layout.json",
    )
    ap.add_argument(
        "--skip-missing-glb",
        action="store_true",
        help="Skip instances whose GLB does not exist (instead of emitting them with availability.glb=false).",
    )
    ap.add_argument(
        "--strict",
        action="store_true",
        help="Fail if any instance's GLB does not exist.",
    )

    args = ap.parse_args(argv)

    layout_usd = args.layout_usd
    subset_root = args.subset_root
    if subset_root is None:
        inferred = _find_subset_root_from_layout(layout_usd)
        if inferred is None:
            raise SystemExit("Failed to infer --subset-root; please provide it explicitly")
        subset_root = inferred

    out_path = args.out
    if out_path is None:
        out_path = os.path.join(os.path.dirname(_norm_abs(layout_usd)), "layout.json")

    payload = generate_layout_json(
        layout_usd=layout_usd,
        subset_root=subset_root,
        skip_missing_glb=bool(args.skip_missing_glb),
        strict=bool(args.strict),
    )

    os.makedirs(os.path.dirname(_norm_abs(out_path)), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    print(f"Wrote: {out_path}")
    print(json.dumps(payload.get("counts", {}), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
