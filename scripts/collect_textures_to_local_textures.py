#!/usr/bin/env python3
"""Collect texture files into ./textures and rewrite USD texture asset paths.

Motivation
- Some USDs reference textures via machine-specific absolute paths (e.g. /tmp/...).
- This makes the USD non-portable: opening on another machine loses textures.

What this script does
- Opens a USD stage.
- Finds asset-valued attributes that look like texture image paths.
- For texture paths NOT already under "./textures/" (relative to the USD file),
  it copies the referenced files into a local textures folder next to the output USD
  and rewrites the USD asset paths to "./textures/<name>".

Notes / limitations
- Only rewrites *USD asset attributes* (SdfAssetPath / SdfAssetPathArray).
  If textures are referenced inside MDL text files, this script will not edit those MDLs.
- Asset resolution depends on the active USD resolver. We use best-effort resolution:
  - Prefer resolver-provided resolvedPath when available.
  - Otherwise, resolve relative to the input USD directory.

Usage
  ./scripts/isaac_python.sh scripts/collect_textures_to_local_textures.py \
    --input /abs/path/to/scene.usd \
    --output /abs/path/to/scene_textures_fixed.usd

Optional
  --textures-dir-name textures   # default: textures
  --dry-run                      # do not write USD or copy files
  --report /abs/path/to/report.json
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Tuple

from pxr import Sdf, Usd


_IMAGE_EXTS = {
    ".png",
    ".jpg",
    ".jpeg",
    ".exr",
    ".tif",
    ".tiff",
    ".tga",
    ".bmp",
    ".webp",
}

_NON_FILE_SCHEMES = (
    "http://",
    "https://",
    "omniverse://",
    "omni://",
    "omni:",
    "file://",
)


def _is_non_file_scheme(path: str) -> bool:
    p = (path or "").strip()
    return any(p.startswith(s) for s in _NON_FILE_SCHEMES)


def _to_posix_path(path: str) -> str:
    return (path or "").replace("\\", "/")


def _looks_like_texture(path: str) -> bool:
    p = (path or "").strip()
    if not p:
        return False
    if _is_non_file_scheme(p):
        return False
    ext = os.path.splitext(p)[1].lower()
    return ext in _IMAGE_EXTS


def _is_under_local_textures(raw_asset_path: str, textures_dir_name: str) -> bool:
    # Treat both "textures/foo.png" and "./textures/foo.png" as local.
    p = _to_posix_path((raw_asset_path or "").strip())
    if not p:
        return False
    prefix1 = f"{textures_dir_name}/"
    prefix2 = f"./{textures_dir_name}/"
    return p.startswith(prefix1) or p.startswith(prefix2)


def _iter_asset_values(value: Any) -> Iterable[Sdf.AssetPath]:
    if value is None:
        return
    if isinstance(value, Sdf.AssetPath):
        yield value
        return
    if isinstance(value, (list, tuple)):
        for v in value:
            if isinstance(v, Sdf.AssetPath):
                yield v


def _resolve_best_effort(asset: Sdf.AssetPath, *, anchor_dir: Optional[str]) -> str:
    # Prefer resolver-provided resolvedPath.
    resolved = getattr(asset, "resolvedPath", "") or ""
    if resolved:
        return resolved

    raw = (asset.path or "").strip()
    if not raw:
        return ""

    if _is_non_file_scheme(raw):
        return raw

    if os.path.isabs(raw):
        return raw

    if anchor_dir:
        return os.path.normpath(os.path.join(anchor_dir, raw))

    return raw


def _sha1_of_file(path: str, *, chunk_size: int = 1024 * 1024) -> str:
    h = hashlib.sha1()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk_size)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def _safe_basename(name: str) -> str:
    # Remove problematic characters; keep it simple/portable.
    base = os.path.basename(name)
    base = re.sub(r"[^A-Za-z0-9._-]+", "_", base)
    base = base.strip("._") or "texture"
    return base


@dataclass
class RewriteItem:
    prop_path: str
    old_asset_path: str
    old_resolved_path: str
    new_asset_path: str
    copied_to: str
    note: str


def collect_and_rewrite(
    *,
    input_usd: str,
    output_usd: str,
    textures_dir_name: str,
    dry_run: bool,
) -> Dict[str, Any]:
    stage = Usd.Stage.Open(input_usd)
    if stage is None:
        raise RuntimeError(f"Failed to open USD: {input_usd}")

    # Anchor for resolving relative paths.
    root_layer = stage.GetRootLayer()
    anchor_dir = os.path.dirname(root_layer.realPath) if root_layer.realPath else os.path.dirname(os.path.abspath(input_usd))

    out_dir = os.path.dirname(os.path.abspath(output_usd))
    textures_dir = os.path.join(out_dir, textures_dir_name)

    rewrites: List[RewriteItem] = []
    skipped: List[Dict[str, str]] = []

    # Map resolved source path -> (dest_abs, new_rel_path)
    copied_map: Dict[str, Tuple[str, str]] = {}

    def ensure_copy(src_abs: str) -> Tuple[str, str, str]:
        """Copy src_abs into textures_dir and return (dest_abs, new_rel, note)."""
        os.makedirs(textures_dir, exist_ok=True)

        safe_name = _safe_basename(src_abs)
        dest_abs = os.path.join(textures_dir, safe_name)

        if os.path.exists(dest_abs):
            try:
                # If it's actually the same file, reuse.
                if os.path.samefile(src_abs, dest_abs):
                    return dest_abs, f"./{textures_dir_name}/{safe_name}", "already_present_samefile"
            except Exception:
                pass

            # Compare content; if different, disambiguate.
            try:
                src_sig = (os.path.getsize(src_abs), _sha1_of_file(src_abs))
                dst_sig = (os.path.getsize(dest_abs), _sha1_of_file(dest_abs))
                if src_sig == dst_sig:
                    return dest_abs, f"./{textures_dir_name}/{safe_name}", "already_present_samecontent"
            except Exception:
                # Fall back to disambiguation.
                pass

            stem, ext = os.path.splitext(safe_name)
            try:
                short = _sha1_of_file(src_abs)[:10]
            except Exception:
                short = hashlib.sha1(src_abs.encode("utf-8", errors="ignore")).hexdigest()[:10]
            safe_name = f"{stem}_{short}{ext}"
            dest_abs = os.path.join(textures_dir, safe_name)

        if not dry_run:
            shutil.copy2(src_abs, dest_abs)
        return dest_abs, f"./{textures_dir_name}/{safe_name}", "copied"

    # Traverse and rewrite asset-valued attributes.
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            t = attr.GetTypeName()
            if t not in (Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.AssetArray):
                continue

            try:
                v = attr.Get()
            except Exception:
                v = None

            # Collect asset values and decide if we should rewrite.
            assets = list(_iter_asset_values(v))
            if not assets:
                continue

            is_array = isinstance(v, (list, tuple))
            new_values: List[Sdf.AssetPath] = []
            changed = False

            for av in assets:
                raw = (av.path or "").strip()
                if not _looks_like_texture(raw):
                    new_values.append(av)
                    continue

                if _is_under_local_textures(raw, textures_dir_name):
                    new_values.append(av)
                    continue

                resolved = _resolve_best_effort(av, anchor_dir=anchor_dir)
                if not resolved or _is_non_file_scheme(resolved):
                    skipped.append(
                        {
                            "prop": str(attr.GetPath()),
                            "asset": raw,
                            "resolved": resolved,
                            "reason": "non_filesystem_or_unresolved",
                        }
                    )
                    new_values.append(av)
                    continue

                if not os.path.exists(resolved):
                    skipped.append(
                        {
                            "prop": str(attr.GetPath()),
                            "asset": raw,
                            "resolved": resolved,
                            "reason": "missing_source_file",
                        }
                    )
                    new_values.append(av)
                    continue

                # Copy (dedup by resolved path).
                if resolved in copied_map:
                    dest_abs, new_rel = copied_map[resolved]
                    note = "reused_copy"
                else:
                    dest_abs, new_rel, note = ensure_copy(resolved)
                    copied_map[resolved] = (dest_abs, new_rel)

                # Rewrite.
                new_values.append(Sdf.AssetPath(_to_posix_path(new_rel)))
                rewrites.append(
                    RewriteItem(
                        prop_path=str(attr.GetPath()),
                        old_asset_path=raw,
                        old_resolved_path=resolved,
                        new_asset_path=_to_posix_path(new_rel),
                        copied_to=dest_abs,
                        note=note,
                    )
                )
                changed = True

            if changed:
                if not dry_run:
                    if is_array:
                        attr.Set(new_values)
                    else:
                        # Single asset attr
                        attr.Set(new_values[0])

    # Export stage.
    if not dry_run:
        os.makedirs(os.path.dirname(os.path.abspath(output_usd)) or ".", exist_ok=True)
        stage.GetRootLayer().Export(output_usd)

    return {
        "input": os.path.abspath(input_usd),
        "output": os.path.abspath(output_usd),
        "textures_dir": os.path.abspath(textures_dir),
        "dry_run": dry_run,
        "rewrite_count": len(rewrites),
        "copied_unique_sources": len(copied_map),
        "rewrites": [
            {
                "prop": r.prop_path,
                "old": r.old_asset_path,
                "resolved": r.old_resolved_path,
                "new": r.new_asset_path,
                "copied_to": r.copied_to,
                "note": r.note,
            }
            for r in rewrites
        ],
        "skipped": skipped,
    }


def _default_output_path(input_usd: str) -> str:
    d = os.path.dirname(os.path.abspath(input_usd))
    base = os.path.basename(input_usd)
    stem, ext = os.path.splitext(base)
    if not ext:
        ext = ".usd"
    return os.path.join(d, f"{stem}_textures_fixed{ext}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--input", required=True, help="Input USD path")
    ap.add_argument("--output", default=None, help="Output USD path (default: <input>_textures_fixed.usd)")
    ap.add_argument("--textures-dir-name", default="textures", help="Folder name next to output USD (default: textures)")
    ap.add_argument("--dry-run", action="store_true", help="Do not copy files or write USD")
    ap.add_argument("--report", default=None, help="Optional JSON report path")
    args = ap.parse_args()

    input_usd = os.path.abspath(args.input)
    output_usd = os.path.abspath(args.output) if args.output else _default_output_path(input_usd)

    report = collect_and_rewrite(
        input_usd=input_usd,
        output_usd=output_usd,
        textures_dir_name=args.textures_dir_name,
        dry_run=args.dry_run,
    )

    print("=== collect_textures_to_local_textures ===")
    print("input:", report["input"])
    print("output:", report["output"])
    print("textures_dir:", report["textures_dir"])
    print("dry_run:", report["dry_run"])
    print("rewrite_count:", report["rewrite_count"], "copied_unique_sources:", report["copied_unique_sources"], "skipped:", len(report["skipped"]))

    if report["skipped"]:
        print("\nSkipped (first 20):")
        for item in report["skipped"][:20]:
            print("-", item.get("resolved") or item.get("asset"), "|", item.get("reason"), "|", item.get("prop"))

    if args.report:
        os.makedirs(os.path.dirname(os.path.abspath(args.report)) or ".", exist_ok=True)
        with open(args.report, "w", encoding="utf-8") as f:
            json.dump(report, f, ensure_ascii=False, indent=2)
        print("\nWrote report:", os.path.abspath(args.report))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
