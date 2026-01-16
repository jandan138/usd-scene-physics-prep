#!/usr/bin/env python3

import argparse
import json
import os
import shutil
import sys
from typing import Any, Dict, Iterator, Optional, Tuple

# Allow running under wrappers (e.g. isaac_python.sh) where repo root may not be on PYTHONPATH.
_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from specs_normalizer.utils.render_size import bytes_to_mb, compute_usd_render_package_size_mb


def safe_file_size_bytes(path: str) -> Optional[int]:
    if not path or not os.path.exists(path) or not os.path.isfile(path):
        return None
    try:
        return os.path.getsize(path)
    except OSError:
        return None


def safe_dir_size_bytes(path: str) -> Optional[int]:
    if not path or not os.path.exists(path) or not os.path.isdir(path):
        return None
    total = 0
    for root, _, files in os.walk(path, followlinks=False):
        for name in files:
            fp = os.path.join(root, name)
            if os.path.islink(fp):
                continue
            try:
                total += os.path.getsize(fp)
            except OSError:
                continue
    return total


def infer_usd_material_softlink(dataset_root: str, asset_dir: str) -> bool:
    usd_textures = os.path.join(asset_dir, "usd", "textures")
    if os.path.islink(usd_textures):
        return True
    if os.path.isdir(os.path.join(dataset_root, "Material", "mdl")):
        return True
    return False


def iter_annotation_paths(root: str) -> Iterator[str]:
    for dirpath, _, filenames in os.walk(root):
        for fn in filenames:
            if fn.endswith("_annotation.json"):
                yield os.path.join(dirpath, fn)


def parse_uid_from_annotation_filename(path: str) -> Optional[str]:
    base = os.path.basename(path)
    if not base.endswith("_annotation.json"):
        return None
    return base[: -len("_annotation.json")]


def looks_like_asset_annotation(asset_dir: str, uid: str) -> bool:
    # Heuristic: an asset annotation must have sibling USD at usd/<uid>.usd
    usd_path = os.path.join(asset_dir, "usd", uid + ".usd")
    return os.path.exists(usd_path)


def load_json(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Expected JSON object in {path}")
    return data


def write_json_atomic(path: str, data: Dict[str, Any]) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
        f.write("\n")
    os.replace(tmp, path)


def compute_backfill_fields(dataset_root: str, asset_dir: str, uid_for_paths: str) -> Dict[str, Any]:
    usd_path = os.path.join(asset_dir, "usd", uid_for_paths + ".usd")
    glb_path = os.path.join(asset_dir, "glb", uid_for_paths + ".glb")
    urdf_dir = os.path.join(asset_dir, "urdf")
    return {
        "glb_size": bytes_to_mb(safe_file_size_bytes(glb_path)),
        "usd_size": compute_usd_render_package_size_mb(usd_path, dataset_root=dataset_root, require_pxr=False),
        "urdf_size": bytes_to_mb(safe_dir_size_bytes(urdf_dir)),
        "orientation": 0,
        "usd_material_softlink": infer_usd_material_softlink(dataset_root=dataset_root, asset_dir=asset_dir),
    }


def apply_backfill(
    ann: Dict[str, Any],
    *,
    uid_from_name: str,
    computed: Dict[str, Any],
    overwrite: bool,
) -> bool:
    changed = False

    # Ensure stable identity fields exist (do not overwrite unless explicitly asked)
    if overwrite or ("uid" not in ann):
        ann["uid"] = str(ann.get("uid") or uid_from_name)
        changed = True

    legacy_defaults = {
        "category": ann.get("category", ""),
        "description": ann.get("description", ""),
        "material": ann.get("material", ""),
        "dimensions": ann.get("dimensions", ""),
        "mass": ann.get("mass", ""),
        "placement": ann.get("placement", []),
    }
    for k, v in legacy_defaults.items():
        if k not in ann:
            ann[k] = v
            changed = True

    for k, v in computed.items():
        if overwrite or (k not in ann):
            ann[k] = v
            changed = True

    return changed


def main() -> int:
    ap = argparse.ArgumentParser(
        description=(
            "Backfill missing fields in per-asset {uid}_annotation.json files. "
            "Only files that look like asset annotations (sibling usd/<uid>.usd exists) are updated."
        )
    )
    ap.add_argument(
        "--root",
        required=True,
        help="Root folder to scan (e.g. GRScenes-test1 or a subset root)",
    )
    ap.add_argument(
        "--dataset-root",
        default=None,
        help=(
            "Dataset root used to infer centralized materials (checks <dataset-root>/Material/mdl). "
            "Defaults to --root."
        ),
    )
    ap.add_argument("--dry-run", action="store_true", help="Do not write any files")
    ap.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing values for the new fields (otherwise only fill missing keys)",
    )
    ap.add_argument(
        "--backup",
        action="store_true",
        help="Write a .bak copy before modifying a JSON file",
    )

    args = ap.parse_args()

    root = os.path.abspath(args.root)
    dataset_root = os.path.abspath(args.dataset_root or args.root)

    scanned = 0
    updated = 0
    skipped_not_asset = 0
    errors = 0

    for ann_path in iter_annotation_paths(root):
        scanned += 1
        try:
            uid_from_name = parse_uid_from_annotation_filename(ann_path)
            if not uid_from_name:
                continue

            asset_dir = os.path.dirname(ann_path)
            if not looks_like_asset_annotation(asset_dir, uid_from_name):
                skipped_not_asset += 1
                continue

            ann = load_json(ann_path)
            computed = compute_backfill_fields(dataset_root=dataset_root, asset_dir=asset_dir, uid_for_paths=uid_from_name)
            changed = apply_backfill(
                ann,
                uid_from_name=uid_from_name,
                computed=computed,
                overwrite=args.overwrite,
            )
            if not changed:
                continue

            updated += 1
            if args.dry_run:
                continue

            if args.backup:
                shutil.copy2(ann_path, ann_path + ".bak")

            write_json_atomic(ann_path, ann)

        except Exception as e:
            errors += 1
            print(f"[ERROR] {ann_path}: {e}")

    print(
        json.dumps(
            {
                "root": root,
                "dataset_root": dataset_root,
                "scanned": scanned,
                "updated": updated,
                "skipped_not_asset": skipped_not_asset,
                "errors": errors,
                "dry_run": bool(args.dry_run),
                "overwrite": bool(args.overwrite),
            },
            ensure_ascii=False,
            indent=2,
        )
    )

    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
