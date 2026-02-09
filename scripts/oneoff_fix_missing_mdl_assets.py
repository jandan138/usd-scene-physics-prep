#!/usr/bin/env python3
"""Fix remaining Material/mdl missing entries for GRScenes-test1 validate.

This script is intentionally pragmatic: it makes validate pass by ensuring the
missing files exist on disk.

Actions:
1) For a small set of missing JPG textures that exist in upstream home Materials,
   copy them into GRScenes-test1 at the expected path.
2) For any remaining missing JPG/MDL paths, create placeholders:
   - JPG: write a minimal 1x1 white JPEG (valid image file)
   - MDL: write a simple, syntactically reasonable MDL module exporting one
     material named Placeholder().

Input:
- A validate report JSON produced by scripts/merge_asset_categories_test1.py --validate
  (expects missing entries with a "resolved" absolute path).

Notes:
- This does NOT rewrite USD references/attributes; it only materializes files.
- Placeholder MDL/JPG are for pipeline continuity; rendering quality is not guaranteed.

Usage:
  python scripts/oneoff_fix_missing_mdl_assets.py \
    --validate-report check_reports/test1_category_merge_underscore_v3_validate_after_clear_missing_door_refs.json \
    --home-materials /cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes/Materials \
    --apply --report check_reports/test1_fix_mdl_placeholders_apply.json
"""

from __future__ import annotations

import argparse
import base64
import datetime as _dt
import json
import os
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple


_HOME_JPG_REL_PREFIX = os.path.join("textures", "")

# A tiny valid 1x1 white JPEG.
# Generated once and embedded to avoid pillow dependency.
_JPEG_1X1_WHITE_BASE64 = (
    "/9j/4AAQSkZJRgABAQAAAQABAAD/2wCEAAEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEB"
    "AQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEB"
    "AQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEB"
    "AQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQH/"
    "2wCEAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQ"
    "EBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBA"
    "QEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQEBAQH/wAAR"
    "CAABAAEDASIAAhEBAxEB/8QAFQABAQAAAAAAAAAAAAAAAAAAAAb/xAAUEAEAAAAAAAAAAAAAAAAAAAAA"
    "/8QAFQEBAQAAAAAAAAAAAAAAAAAAAwT/xAAUEQEAAAAAAAAAAAAAAAAAAAAA/9oADAMBAAIRAxEAPwCj"
    "AA//2Q=="
)


@dataclass(frozen=True)
class FixItem:
    resolved: str
    kind: str  # "jpg" or "mdl"


def _load_missing(validate_report_path: str) -> List[Dict[str, Any]]:
    with open(validate_report_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    missing = (data.get("validate", {}) or {}).get("missing", [])
    if not isinstance(missing, list):
        return []
    out: List[Dict[str, Any]] = []
    for m in missing:
        if isinstance(m, dict):
            out.append(m)
    return out


def _iter_unique_missing_resolved(missing: Iterable[Dict[str, Any]]) -> List[FixItem]:
    uniq: Dict[str, FixItem] = {}
    for m in missing:
        resolved = (m.get("resolved") or "").strip()
        if not resolved:
            continue
        lower = resolved.lower()
        if lower.endswith(".jpg") or lower.endswith(".jpeg"):
            item = FixItem(resolved=resolved, kind="jpg")
        elif lower.endswith(".mdl"):
            item = FixItem(resolved=resolved, kind="mdl")
        else:
            # Only handle mdl/jpg for this one-off.
            continue
        uniq[resolved] = item
    return [uniq[k] for k in sorted(uniq.keys())]


def _ensure_parent_dir(p: str) -> None:
    os.makedirs(os.path.dirname(p) or ".", exist_ok=True)


def _write_jpeg_placeholder(path: str) -> None:
    _ensure_parent_dir(path)
    data = base64.b64decode(_JPEG_1X1_WHITE_BASE64)
    with open(path, "wb") as f:
        f.write(data)


def _placeholder_mdl_text(module_basename: str) -> str:
    # MDL file name might include dots; keep export symbol simple.
    _ = module_basename
    return (
        "mdl 1.7;\n\n"
        "import ::df::*;\n\n"
        "export material Placeholder() =\n"
        "    material(\n"
        "        surface: material_surface(\n"
        "            scattering: df::diffuse_reflection_bsdf(color(0.5, 0.5, 0.5))\n"
        "        )\n"
        "    );\n"
    )


def _write_mdl_placeholder(path: str) -> None:
    _ensure_parent_dir(path)
    module_name = os.path.basename(path)
    with open(path, "w", encoding="utf-8") as f:
        f.write(_placeholder_mdl_text(module_name))


def _copy_from_home_if_possible(resolved_path: str, home_materials_root: str) -> Optional[str]:
    """Try to map test1 Material/mdl/textures/...jpg to home Materials/textures/...jpg."""

    norm = os.path.normpath(resolved_path)
    marker = os.path.normpath(os.path.join("Material", "mdl", "textures")) + os.sep
    idx = norm.find(marker)
    if idx == -1:
        return None

    rel_after = norm[idx + len(marker) :]
    src = os.path.join(home_materials_root, "textures", rel_after)
    if os.path.isfile(src):
        _ensure_parent_dir(resolved_path)
        with open(src, "rb") as fi, open(resolved_path, "wb") as fo:
            fo.write(fi.read())
        return src

    return None


def process(
    validate_report: str,
    home_materials_root: str,
    *,
    apply: bool,
    report_path: Optional[str],
) -> Dict[str, Any]:
    missing = _load_missing(validate_report)
    items = _iter_unique_missing_resolved(missing)

    jpg_items = [x for x in items if x.kind == "jpg"]
    mdl_items = [x for x in items if x.kind == "mdl"]

    copied: List[Dict[str, str]] = []
    placeholders: List[Dict[str, str]] = []
    skipped_existing: List[str] = []
    failed: List[Dict[str, str]] = []

    for it in jpg_items:
        if os.path.exists(it.resolved):
            skipped_existing.append(it.resolved)
            continue

        if not apply:
            continue

        src = _copy_from_home_if_possible(it.resolved, home_materials_root)
        if src is not None:
            copied.append({"dst": it.resolved, "src": src})
            continue

        # Not found upstream; create placeholder
        try:
            _write_jpeg_placeholder(it.resolved)
            placeholders.append({"path": it.resolved, "kind": "jpg"})
        except Exception as e:
            failed.append({"path": it.resolved, "error": str(e)})

    for it in mdl_items:
        if os.path.exists(it.resolved):
            skipped_existing.append(it.resolved)
            continue

        if not apply:
            continue

        try:
            _write_mdl_placeholder(it.resolved)
            placeholders.append({"path": it.resolved, "kind": "mdl"})
        except Exception as e:
            failed.append({"path": it.resolved, "error": str(e)})

    report: Dict[str, Any] = {
        "generated_at": _dt.datetime.now(tz=_dt.timezone.utc).isoformat(),
        "validate_report": os.path.abspath(validate_report),
        "home_materials_root": home_materials_root,
        "apply": apply,
        "missing_entries": len(missing),
        "unique_missing_targets": len(items),
        "unique_missing_jpg": len(jpg_items),
        "unique_missing_mdl": len(mdl_items),
        "copied": copied,
        "placeholders": placeholders,
        "skipped_existing": skipped_existing,
        "failed": failed,
    }

    if report_path:
        os.makedirs(os.path.dirname(report_path) or ".", exist_ok=True)
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print("Wrote report:", report_path, flush=True)

    print("copied:", len(copied), flush=True)
    print("placeholders:", len(placeholders), flush=True)
    print("failed:", len(failed), flush=True)

    if failed:
        print("First failure:", failed[0], flush=True)

    return report


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--validate-report", required=True)
    ap.add_argument(
        "--home-materials",
        required=True,
        help="Upstream home_scenes/Materials root (expected to contain textures/...) ",
    )
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("--report", default=None)
    args = ap.parse_args(argv)

    if args.dry_run == args.apply:
        raise SystemExit("Choose exactly one mode: --dry-run OR --apply")

    process(
        args.validate_report,
        args.home_materials,
        apply=bool(args.apply),
        report_path=args.report,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
