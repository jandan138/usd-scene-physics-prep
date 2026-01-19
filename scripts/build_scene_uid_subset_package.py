#!/usr/bin/env python3

"""Build a downloadable subset package for a *scene UID*.

Given a GRScenes package root (e.g. GRScenes-test1) and a scene UID, this script:

- Copies the full scene folder: GRScenes100/<category>/<scene_uid>_usd/**
- Scans the scene USD(s) to find referenced asset UIDs under GRScenes_assets/**
- Recursively scans asset USDs to find transitive asset dependencies
- Copies each asset UID folder: GRScenes_assets/<asset_category>/<uid>/**
- Copies the minimal required Material/mdl + Material/mdl/textures closure

This is similar to scripts/build_uid_subset_package.py, but with scene→assets expansion.

Typical usage (Isaac Python environment recommended):

  ./scripts/isaac_python.sh scripts/build_scene_uid_subset_package.py \
    --src /abs/path/to/GRScenes-test1 \
    --dst /abs/path/to/subset_scene \
    --scene-uid MV7J6NIKTKJZ2AABAAAAADA8 \
    --write-manifest --verify

"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
import re
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Set, Tuple

try:
    from pxr import Sdf, Usd

    _PXR_IMPORT_ERROR: Optional[Exception] = None
except Exception as e:  # pragma: no cover
    Sdf = None  # type: ignore[assignment]
    Usd = None  # type: ignore[assignment]
    _PXR_IMPORT_ERROR = e


def _load_uid_subset_module():
    """Load scripts/build_uid_subset_package.py as a module.

    Note: `scripts/` is not a Python package in this repo (no __init__.py), so
    we cannot import it via `import scripts...` reliably.
    """

    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(scripts_dir, "build_uid_subset_package.py")
    spec = importlib.util.spec_from_file_location("uid_subset_helpers", path)
    if spec is None or spec.loader is None:
        raise ImportError(f"Failed to load module spec: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)  # type: ignore[union-attr]
    return module


# Reuse the proven copy + material dependency closure helpers.
uid_subset = _load_uid_subset_module()


_SCENE_DIR_SUFFIX = "_usd"


@dataclass(frozen=True)
class AssetRef:
    category: str
    uid: str


def _norm_abs(p: str) -> str:
    return os.path.abspath(os.path.normpath(p))


def _strip_scene_uid(scene_uid: str) -> str:
    s = scene_uid.strip().rstrip("/")
    if s.endswith(_SCENE_DIR_SUFFIX):
        s = s[: -len(_SCENE_DIR_SUFFIX)]
    return s


def _find_scene_dir(src_root: str, scene_uid: str, scene_category: Optional[str]) -> Tuple[str, str]:
    """Return (category, scene_dir) for the given scene uid."""

    src_root = _norm_abs(src_root)
    sid = _strip_scene_uid(scene_uid)

    candidates: List[Tuple[str, str]] = []

    grscenes100 = os.path.join(src_root, "GRScenes100")
    if scene_category:
        cats = [scene_category]
    else:
        if os.path.isdir(grscenes100):
            cats = sorted([d for d in os.listdir(grscenes100) if os.path.isdir(os.path.join(grscenes100, d))])
        else:
            cats = []

    for cat in cats:
        p = os.path.join(grscenes100, cat, f"{sid}{_SCENE_DIR_SUFFIX}")
        if os.path.isdir(p):
            candidates.append((cat, p))

    if not candidates:
        raise FileNotFoundError(f"scene uid not found: {sid}")
    if len(candidates) > 1:
        raise RuntimeError(f"scene uid matched multiple categories: {candidates}")

    return candidates[0]


_GRSCENES_ASSETS_RE = re.compile(r"(?:^|/)GRScenes_assets/([^/]+)/([^/]+)/")


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


def _collect_asset_refs_from_usd(usd_path: str) -> Set[AssetRef]:
    """Collect AssetRef(category, uid) referenced by a USD file.

    Sources scanned:
    - prim metadata: references
    - prim metadata: payloads
    - asset-valued attributes that point to .usd/.usda/.usdc
    - used layers (safety net)

    We only return refs that contain GRScenes_assets/<category>/<uid>/.
    """

    if _PXR_IMPORT_ERROR is not None:
        raise RuntimeError(f"pxr.Usd not available: {_PXR_IMPORT_ERROR}")

    stage = Usd.Stage.Open(usd_path)
    if not stage:
        raise RuntimeError(f"failed to open USD: {usd_path}")

    base_dir = os.path.dirname(os.path.abspath(usd_path))

    raw_paths: Set[str] = set()

    # 1) references + payloads
    for prim in stage.Traverse():
        if prim.HasAuthoredReferences():
            refs = prim.GetMetadata("references")
            if refs:
                for ref in refs.GetAddedOrExplicitItems():
                    ap = getattr(ref, "assetPath", "")
                    if ap:
                        raw_paths.add(ap)
        if prim.HasAuthoredPayloads():
            payloads = prim.GetMetadata("payloads")
            if payloads:
                for pl in payloads.GetAddedOrExplicitItems():
                    ap = getattr(pl, "assetPath", "")
                    if ap:
                        raw_paths.add(ap)

        # 2) asset-valued attributes pointing to usd
        for attr in prim.GetAttributes():
            try:
                val = attr.Get()
            except Exception:
                continue
            if isinstance(val, Sdf.AssetPath):
                ap = val.path
                if ap and ap.lower().endswith((".usd", ".usda", ".usdc")):
                    raw_paths.add(ap)

    # 3) used layers
    try:
        for layer in stage.GetUsedLayers():
            p = layer.realPath or layer.identifier
            if p:
                raw_paths.add(p)
    except Exception:
        pass

    out: Set[AssetRef] = set()
    for p in raw_paths:
        # Many paths are relative to the usd file directory in our normalized packs.
        resolved = _resolve_maybe_relative(base_dir, p)
        for candidate in (p, resolved):
            ar = _parse_asset_ref_from_path(candidate)
            if ar is not None:
                out.add(ar)

    return out


def _resolve_asset_dir(src_root: str, asset: AssetRef) -> Tuple[str, str]:
    """Return (category, absolute_dir) for an asset.

    Prefer the category inferred from the path; fall back to a UID search if needed.
    """

    src_root = _norm_abs(src_root)
    preferred = os.path.join(src_root, "GRScenes_assets", asset.category, asset.uid)
    if os.path.isdir(preferred):
        return asset.category, preferred

    matches = uid_subset._find_uid_asset_dirs(src_root, asset.uid)
    if not matches:
        raise FileNotFoundError(f"asset uid not found under GRScenes_assets: {asset.uid} (from category {asset.category})")
    if len(matches) > 1:
        raise RuntimeError(f"asset uid has multiple matches under GRScenes_assets: {asset.uid} -> {matches}")

    cat, p = matches[0]
    return cat, p


def _iter_scene_usd_files(scene_dir: str, include_scene_usds: str) -> List[str]:
    if include_scene_usds == "layout-only":
        p = os.path.join(scene_dir, "layout.usd")
        if os.path.isfile(p):
            return [p]
        # Fallback: scan everything.
    return uid_subset._iter_usd_files_under(scene_dir)


def _verify_subset_mdl_and_textures(dst_root: str) -> Dict[str, object]:
    """Verify that referenced MDL / texture files exist (assets + scenes)."""

    usd_files: List[str] = []

    assets_root = os.path.join(dst_root, "GRScenes_assets")
    scenes_root = os.path.join(dst_root, "GRScenes100")

    if os.path.isdir(assets_root):
        usd_files.extend(uid_subset._iter_usd_files_under(assets_root))
    if os.path.isdir(scenes_root):
        usd_files.extend(uid_subset._iter_usd_files_under(scenes_root))

    missing: List[Dict[str, str]] = []
    for usd in sorted(set(usd_files)):
        for ref in uid_subset._analyze_usd_asset_attributes(usd):
            resolved = ref["resolved"] or ref["asset"]
            if not resolved:
                continue
            low = resolved.lower()
            if not (low.endswith(".mdl") or os.path.splitext(low)[1] in uid_subset._IMAGE_EXTS):
                continue
            if os.path.isabs(resolved) and os.path.exists(resolved):
                continue
            missing.append({"usd": usd, **ref})

    return {"usd_count": len(set(usd_files)), "missing_asset_refs": missing, "missing_count": len(missing)}


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="Source GRScenes package root (e.g. GRScenes-test1)")
    ap.add_argument("--dst", required=True, help="Destination subset package root")
    ap.add_argument("--scene-uid", required=True, help="Scene UID (e.g. MV7J6NIKTKJZ2AABAAAAADA8)")
    ap.add_argument("--scene-category", default=None, help="Scene category under GRScenes100 (e.g. home, commercial)")
    ap.add_argument(
        "--include-scene-usds",
        default="all",
        choices=["all", "layout-only"],
        help="Which USD files under the scene folder to scan for referenced assets",
    )
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--no-incremental", action="store_true", help="Disable incremental copy (always overwrite)")
    ap.add_argument("--write-manifest", action="store_true", help="Write <dst>/subset_manifest.json")
    ap.add_argument("--verify", action="store_true", help="Re-scan subset USDs and report missing MDL/texture files")
    ap.add_argument(
        "--create-usd-textures-symlink",
        action="store_true",
        help="Create per-asset usd/textures symlink -> <dst>/Material/mdl/textures (optional)",
    )
    ap.add_argument(
        "--force-usd-textures-symlink",
        action="store_true",
        help="If usd/textures symlink exists but points elsewhere, replace it",
    )

    args = ap.parse_args(list(argv) if argv is not None else None)

    if _PXR_IMPORT_ERROR is not None:
        print("ERROR: pxr (Usd/Sdf) is not available in this python environment.")
        print("Run with ./scripts/isaac_python.sh ...")
        print("Import error:", _PXR_IMPORT_ERROR)
        return 2

    src_root = _norm_abs(args.src)
    dst_root = _norm_abs(args.dst)
    dry_run = bool(args.dry_run)
    incremental = not bool(args.no_incremental)

    if not os.path.isdir(src_root):
        print(f"ERROR: src root not found: {src_root}")
        return 2

    try:
        scene_cat, scene_src_dir = _find_scene_dir(src_root, args.scene_uid, args.scene_category)
    except Exception as e:
        print("ERROR:", e)
        return 3

    scene_uid = _strip_scene_uid(args.scene_uid)
    scene_dst_dir = os.path.join(dst_root, "GRScenes100", scene_cat, f"{scene_uid}{_SCENE_DIR_SUFFIX}")

    if not dry_run:
        os.makedirs(dst_root, exist_ok=True)

    # 1) Copy scene folder.
    copied_scene_files, skipped_scene_files = uid_subset._copy_tree(
        scene_src_dir, scene_dst_dir, dry_run=dry_run, incremental=incremental
    )

    # 2) Compute asset UID closure via BFS over USD references.
    seed_scene_usds = _iter_scene_usd_files(scene_src_dir, args.include_scene_usds)
    queue: List[str] = list(seed_scene_usds)
    scanned_usds: Set[str] = set()

    assets: Dict[Tuple[str, str], str] = {}  # (category, uid) -> src_dir
    missing_assets: List[str] = []

    while queue:
        usd_path = queue.pop()
        usd_path = os.path.abspath(usd_path)
        if usd_path in scanned_usds:
            continue
        scanned_usds.add(usd_path)

        try:
            refs = _collect_asset_refs_from_usd(usd_path)
        except Exception as e:
            print(f"ERROR: failed scanning USD: {usd_path}: {e}")
            return 5

        for ref in sorted(refs, key=lambda r: (r.category, r.uid)):
            try:
                cat, asset_dir = _resolve_asset_dir(src_root, ref)
            except Exception as e:
                missing_assets.append(f"{ref.uid} ({ref.category}): {e}")
                continue

            key = (cat, ref.uid)
            if key in assets:
                continue

            assets[key] = asset_dir
            # Scan this asset's USDs too for transitive references.
            queue.extend(uid_subset._iter_usd_files_under(asset_dir))

    if missing_assets:
        print("ERROR: missing referenced assets:")
        for m in missing_assets:
            print("-", m)
        return 5

    # 3) Copy assets.
    copied_asset_files = 0
    skipped_asset_files = 0
    for (cat, uid), asset_src_dir in sorted(assets.items()):
        asset_dst_dir = os.path.join(dst_root, "GRScenes_assets", cat, uid)
        c, s = uid_subset._copy_tree(asset_src_dir, asset_dst_dir, dry_run=dry_run, incremental=incremental)
        copied_asset_files += c
        skipped_asset_files += s

    # 4) Copy minimal material closure.
    usd_files_scanned: List[str] = []
    usd_files_scanned.extend(seed_scene_usds)
    for asset_src_dir in assets.values():
        usd_files_scanned.extend(uid_subset._iter_usd_files_under(asset_src_dir))
    usd_files_scanned = sorted(set(usd_files_scanned))

    dep_report = uid_subset._collect_mdl_and_textures(
        usd_files=usd_files_scanned,
        src_root=src_root,
        dst_root=dst_root,
        dry_run=dry_run,
        incremental=incremental,
    )

    # 5) Optional symlink creation (assets only).
    symlink_report: Optional[Dict[str, object]] = None
    if args.create_usd_textures_symlink:
        symlink_report = uid_subset._create_usd_textures_symlinks(
            dst_root=dst_root,
            dry_run=dry_run,
            force=bool(args.force_usd_textures_symlink),
        )

    # 6) Optional verify.
    verify_report: Optional[Dict[str, object]] = None
    if args.verify:
        if dry_run:
            print("DRY-RUN: would verify subset")
        else:
            verify_report = _verify_subset_mdl_and_textures(dst_root)

    manifest: Dict[str, object] = {
        "src_root": src_root,
        "dst_root": dst_root,
        "scene_uid": scene_uid,
        "scene_category": scene_cat,
        "scene_src_dir": scene_src_dir,
        "scene_dst_dir": scene_dst_dir,
        "include_scene_usds": args.include_scene_usds,
        "scene_usd_files_scanned": seed_scene_usds,
        "assets": [
            {"category": cat, "uid": uid, "src_dir": src_dir, "dst_dir": os.path.join(dst_root, "GRScenes_assets", cat, uid)}
            for (cat, uid), src_dir in sorted(assets.items())
        ],
        "counts": {
            "assets": len(assets),
            "scene_files_copied": copied_scene_files,
            "scene_files_skipped": skipped_scene_files,
            "asset_files_copied": copied_asset_files,
            "asset_files_skipped": skipped_asset_files,
            "usd_files_scanned": len(usd_files_scanned),
        },
        "material_dependency": dep_report,
        "verify": verify_report,
        "usd_textures_symlink": symlink_report,
    }

    print("=== Scene subset package summary ===")
    print("scene:", scene_uid, "category:", scene_cat)
    print("assets:", len(assets))
    print("scene_files_copied:", copied_scene_files, "skipped:", skipped_scene_files)
    print("asset_files_copied:", copied_asset_files, "skipped:", skipped_asset_files)
    print("usd_files_scanned:", len(usd_files_scanned))
    print(
        "mdls:",
        len(dep_report.get("all_mdl_rel", [])),
        "copied:",
        dep_report.get("copied_mdls"),
        "missing:",
        len(dep_report.get("missing_mdls", [])),
    )
    print(
        "textures:",
        len(dep_report.get("textures_suffixes", [])),
        "copied_files:",
        dep_report.get("copied_textures"),
        "missing:",
        len(dep_report.get("missing_textures", [])),
    )

    if args.write_manifest:
        out_path = os.path.join(dst_root, "subset_manifest.json")
        if dry_run:
            print("DRY-RUN: would write manifest:", out_path)
        else:
            with open(out_path, "w", encoding="utf-8") as f:
                json.dump(manifest, f, ensure_ascii=False, indent=2)
            print("Wrote manifest:", out_path)

    if verify_report and verify_report.get("missing_count"):
        print("VERIFY: missing MDL/texture refs:", verify_report.get("missing_count"))
        return 4

    if dep_report.get("missing_mdls") or dep_report.get("missing_textures"):
        return 5

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
