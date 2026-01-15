#!/usr/bin/env python3
"""Audit and (optionally) sync missing textures referenced by MDL files.

Why:
- Isaac/Kit resolves MDL texture references literally. If a referenced file is missing, you get
  rtx.mdltranslator warnings/errors. Scenes can still load with fallbacks, but materials are incomplete.

What this script does:
- Scans all .mdl under a given mdl root for texture file references.
- Resolves those references to absolute paths in the exported package.
- Reports which files are missing.
- Optionally searches one or more source roots for the same sharded relative path and copies
  found files into the exported textures dir, preserving sharding.

Design constraints:
- No symlinks created.
- Does not flatten textures; preserves the referenced subdirectory layout (e.g., cd9/9c5/<hash>.jpg).

Typical usage:
  # 1) Audit only
  python scripts/sync_missing_mdl_textures.py \
    --mdl-root GRScenes-test1/Material/mdl \
    --report logs/missing_textures_report.json

  # 2) Try to copy from known source roots (repeatable)
  python scripts/sync_missing_mdl_textures.py \
    --mdl-root GRScenes-test1/Material/mdl \
    --search-root /cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/home_scenes/Materials \
    --search-root /cpfs/shared/simulation/zzh-grscenes/scenes/GRScenes-100/commercial_scenes/Materials \
    --copy \
    --report logs/missing_textures_report.json

Notes:
- For speed, it does NOT do a global filename search (which can be extremely expensive on large trees).
  It only checks a small set of candidate locations per missing reference.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
from dataclasses import dataclass
from typing import Iterable, Optional


IMAGE_EXTS = (".png", ".jpg", ".jpeg", ".exr", ".tif", ".tiff", ".tga", ".bmp", ".webp")


@dataclass(frozen=True)
class Ref:
    mdl_file: str
    raw: str
    resolved_abs: str


_STRING_RE = re.compile(r"(?P<q>['\"])(?P<s>(?:\\.|(?!\1).)*)\1")


def _posix(p: str) -> str:
    return p.replace("\\", "/")


def _is_texture_string(s: str) -> bool:
    low = s.lower()
    if not low.endswith(IMAGE_EXTS):
        return False
    # MDL typically references textures with ./textures/... or file:/.../textures/...
    return "/textures/" in _posix(low) or low.startswith("./textures/") or low.startswith("textures/")


def _decode_string_literal(s: str) -> str:
    # Keep it simple: only unescape common sequences used in MDL.
    return (
        s.replace("\\\"", '"')
        .replace("\\'", "'")
        .replace("\\\\", "\\")
        .replace("\\n", "\n")
        .replace("\\t", "\t")
    )


def _resolve_ref(mdl_file: str, tex_ref: str) -> Optional[str]:
    tex_ref = tex_ref.strip()

    # file: URI
    if tex_ref.startswith("file:"):
        # MDL uses file:/abs/path or file:./rel
        path = tex_ref[len("file:") :]
        if path.startswith("//"):
            # file://... is uncommon here; keep minimal handling
            path = path.lstrip("/")
            path = "/" + path
        if os.path.isabs(path):
            return path
        return os.path.normpath(os.path.join(os.path.dirname(mdl_file), path))

    # Absolute path
    if os.path.isabs(tex_ref):
        return tex_ref

    # Relative path (common: ./textures/..)
    return os.path.normpath(os.path.join(os.path.dirname(mdl_file), tex_ref))


def iter_mdl_files(mdl_root: str) -> Iterable[str]:
    for root, _, files in os.walk(mdl_root):
        for f in files:
            if f.lower().endswith(".mdl"):
                yield os.path.join(root, f)


def extract_texture_refs(mdl_file: str) -> list[Ref]:
    try:
        with open(mdl_file, "r", encoding="utf-8", errors="ignore") as fh:
            txt = fh.read()
    except OSError:
        return []

    refs: list[Ref] = []
    for m in _STRING_RE.finditer(txt):
        raw = _decode_string_literal(m.group("s"))
        if not _is_texture_string(raw):
            continue
        resolved = _resolve_ref(mdl_file, raw)
        if not resolved:
            continue
        refs.append(Ref(mdl_file=mdl_file, raw=raw, resolved_abs=resolved))
    return refs


def _textures_suffix(path_str: str) -> Optional[str]:
    # Extract suffix after the last '/textures/' component.
    p = _posix(path_str)
    low = p.lower()
    idx = low.rfind("/textures/")
    if idx == -1:
        # allow leading 'textures/...'
        if low.startswith("textures/"):
            return p[len("textures/") :]
        if low.startswith("./textures/"):
            return p[len("./textures/") :]
        return None
    return p[idx + len("/textures/") :]


def candidate_sources(search_root: str, suffix: str) -> list[str]:
    # Try a handful of plausible roots.
    # search_root may be:
    # - .../Materials
    # - .../Materials/Textures
    # - .../Material/mdl
    # - .../Material/mdl/textures
    sr = search_root.rstrip("/")
    candidates: list[str] = []

    base = sr
    candidates.append(os.path.join(base, suffix))
    candidates.append(os.path.join(base, "textures", suffix))
    candidates.append(os.path.join(base, "Textures", suffix))

    # If given a Materials dir, also try its texture dirs
    candidates.append(os.path.join(base, "Materials", "textures", suffix))
    candidates.append(os.path.join(base, "Materials", "Textures", suffix))

    return candidates


def safe_copy(src: str, dst: str) -> None:
    os.makedirs(os.path.dirname(dst), exist_ok=True)
    shutil.copy2(src, dst)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--mdl-root", required=True, help="Exported MDL root, e.g. GRScenes-test1/Material/mdl")
    ap.add_argument(
        "--textures-dir",
        default=None,
        help="Exported textures directory. Defaults to <mdl-root>/textures",
    )
    ap.add_argument(
        "--search-root",
        action="append",
        default=[],
        help="A root directory to search for missing textures (repeatable)",
    )
    ap.add_argument("--copy", action="store_true", help="If set, copy found textures into exported textures dir")
    ap.add_argument(
        "--report",
        default=None,
        help="Write a JSON report with summary/missing/found info",
    )
    ap.add_argument("--max-print", type=int, default=20, help="Max missing entries to print")

    args = ap.parse_args()

    mdl_root = os.path.abspath(args.mdl_root)
    textures_dir = os.path.abspath(args.textures_dir or os.path.join(mdl_root, "textures"))
    search_roots = [os.path.abspath(p) for p in (args.search_root or [])]

    all_refs: list[Ref] = []
    for mdl in iter_mdl_files(mdl_root):
        all_refs.extend(extract_texture_refs(mdl))

    missing: list[Ref] = [r for r in all_refs if not os.path.isfile(r.resolved_abs)]

    found_map: dict[str, str] = {}
    copied: list[tuple[str, str]] = []

    if search_roots and missing:
        for r in missing:
            suffix = _textures_suffix(r.resolved_abs) or _textures_suffix(r.raw)
            if not suffix:
                continue
            # Destination path should preserve suffix under exported textures dir
            dst_path = os.path.join(textures_dir, *suffix.split("/"))
            if os.path.isfile(dst_path):
                found_map[r.resolved_abs] = dst_path
                continue

            src_found: Optional[str] = None
            for sr in search_roots:
                for cand in candidate_sources(sr, suffix):
                    if os.path.isfile(cand):
                        src_found = cand
                        break
                if src_found:
                    break

            if src_found:
                found_map[r.resolved_abs] = src_found
                if args.copy:
                    safe_copy(src_found, dst_path)
                    copied.append((src_found, dst_path))

    summary = {
        "mdl_root": mdl_root,
        "textures_dir": textures_dir,
        "total_mdl_files": sum(1 for _ in iter_mdl_files(mdl_root)),
        "total_texture_refs": len(all_refs),
        "missing_count": len(missing),
        "unique_missing_paths": len({r.resolved_abs for r in missing}),
        "search_roots": search_roots,
        "found_candidates": len(found_map),
        "copied": len(copied),
    }

    print(json.dumps(summary, indent=2, ensure_ascii=False))
    if missing:
        print("Missing examples:")
        uniq = []
        seen = set()
        for r in missing:
            if r.resolved_abs in seen:
                continue
            seen.add(r.resolved_abs)
            uniq.append(r)
            if len(uniq) >= args.max_print:
                break
        for r in uniq:
            print("-", r.resolved_abs)

    if args.report:
        os.makedirs(os.path.dirname(os.path.abspath(args.report)) or ".", exist_ok=True)
        report_obj = {
            "summary": summary,
            "missing": [r.__dict__ for r in missing],
            "found_map": found_map,
            "copied": copied,
        }
        with open(args.report, "w", encoding="utf-8") as fh:
            json.dump(report_obj, fh, ensure_ascii=False, indent=2)
        print("Wrote report:", args.report)

    # Non-zero exit if missing remain and we didn't copy them all.
    if missing and (not args.copy):
        return 2
    if args.copy:
        # Recompute missing after copy
        still_missing = [r for r in missing if not os.path.isfile(r.resolved_abs)]
        if still_missing:
            return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
