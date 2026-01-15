#!/usr/bin/env python3
"""Build a downloadable subset package by selected asset UIDs.

Requirements (project-specific)
- Copy the entire UID folder under GRScenes_assets/<category>/<uid>/ (future-proof: may include glb, etc.).
- Do NOT include Scenes (GRScenes100 content). Optionally create an empty GRScenes100 dir.
- Material should be minimal: only MDL files (and their internal MDL deps) plus referenced textures.
- Keep textures sharding layout under Material/mdl/textures/.
- No symlinks.

Typical usage
  ./scripts/isaac_python.sh scripts/build_uid_subset_package.py \
    --src /abs/path/to/GRScenes-test1 \
    --dst /abs/path/to/subsets/bed_only \
    --uid 0001a2b3c4d5e6f7 \
    --uid 0009ffee... \
    --write-manifest

  # UIDs from file (one per line)
  ./scripts/isaac_python.sh scripts/build_uid_subset_package.py \
    --src /abs/path/to/GRScenes-test1 \
    --dst /abs/path/to/subsets/my_pack \
    --uid-file uids.txt \
    --write-manifest \
    --verify
"""

from __future__ import annotations

import argparse
import json
import glob
import os
import re
import shutil
import sys
from dataclasses import dataclass
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


try:
    from pxr import Sdf, Usd  # type: ignore
except Exception as e:  # pragma: no cover
    Sdf = None  # type: ignore
    Usd = None  # type: ignore
    _PXR_IMPORT_ERROR = e
else:
    _PXR_IMPORT_ERROR = None


_IMAGE_EXTS = (".png", ".jpg", ".jpeg", ".exr", ".tif", ".tiff", ".tga", ".bmp", ".webp", ".hdr")


def _posix(p: str) -> str:
    return p.replace("\\", "/")


def _norm_abs(p: str) -> str:
    return os.path.abspath(os.path.expanduser(p))


def _is_under(path: str, root: str) -> bool:
    try:
        return os.path.commonpath([_norm_abs(path), _norm_abs(root)]) == _norm_abs(root)
    except Exception:
        return False


def _safe_relpath(path: str, root: str) -> Optional[str]:
    if not _is_under(path, root):
        return None
    return os.path.relpath(_norm_abs(path), _norm_abs(root))


def _read_uid_file(path: str) -> List[str]:
    uids: List[str] = []
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            uids.append(s)
    return uids


def _dedup_keep_order(items: Sequence[str]) -> List[str]:
    seen: Set[str] = set()
    out: List[str] = []
    for x in items:
        if x in seen:
            continue
        seen.add(x)
        out.append(x)
    return out


def _iter_asset_values(value) -> Iterable:
    if value is None:
        return
    if Sdf is not None and isinstance(value, Sdf.AssetPath):
        yield value
        return
    if isinstance(value, (list, tuple)):
        for v in value:
            if Sdf is not None and isinstance(v, Sdf.AssetPath):
                yield v


def _resolve_best_effort(asset, *, anchor_dir: Optional[str]) -> str:
    if getattr(asset, "resolvedPath", ""):
        return asset.resolvedPath

    raw = (asset.path or "").strip()
    if not raw:
        return ""

    if raw.startswith(("http://", "https://", "omniverse://", "omni://", "omni:", "file://")):
        return raw

    if os.path.isabs(raw):
        return raw

    if anchor_dir:
        return os.path.normpath(os.path.join(anchor_dir, raw))

    return raw


@dataclass(frozen=True)
class UidAsset:
    category: str
    uid: str
    src_dir: str
    dst_dir: str


def _find_uid_asset_dirs(src_root: str, uid: str) -> List[Tuple[str, str]]:
    assets_root = os.path.join(src_root, "GRScenes_assets")
    matches: List[Tuple[str, str]] = []
    if not os.path.isdir(assets_root):
        return matches

    # Expected layout: GRScenes_assets/<category>/<uid>/
    for category in os.listdir(assets_root):
        cat_dir = os.path.join(assets_root, category)
        if not os.path.isdir(cat_dir):
            continue
        cand = os.path.join(cat_dir, uid)
        if os.path.isdir(cand):
            matches.append((category, cand))
    return matches


def _iter_files(root: str) -> Iterable[Tuple[str, str]]:
    for r, dirs, files in os.walk(root):
        dirs.sort()
        files.sort()
        for fn in files:
            src = os.path.join(r, fn)
            rel = os.path.relpath(src, root)
            yield src, rel


def _copy_tree(src_dir: str, dst_dir: str, *, dry_run: bool, incremental: bool) -> Tuple[int, int]:
    copied = 0
    skipped = 0

    for src_path, rel in _iter_files(src_dir):
        dst_path = os.path.join(dst_dir, rel)
        if incremental and os.path.exists(dst_path):
            try:
                sst = os.stat(src_path)
                dstst = os.stat(dst_path)
                if sst.st_size == dstst.st_size and int(sst.st_mtime) <= int(dstst.st_mtime):
                    skipped += 1
                    continue
            except OSError:
                pass

        if not dry_run:
            os.makedirs(os.path.dirname(dst_path), exist_ok=True)
            shutil.copy2(src_path, dst_path)
        copied += 1

    return copied, skipped


def _iter_usd_files_under(root: str) -> List[str]:
    out: List[str] = []
    for r, _, files in os.walk(root):
        for fn in files:
            if fn.lower().endswith((".usd", ".usda", ".usdc")):
                out.append(os.path.join(r, fn))
    out.sort()
    return out


def _analyze_usd_asset_attributes(usd_path: str) -> List[Dict[str, str]]:
    if Usd is None:
        raise RuntimeError(f"pxr.Usd not available: {_PXR_IMPORT_ERROR}")

    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        raise RuntimeError(f"Failed to open USD: {usd_path}")

    root_layer = stage.GetRootLayer()
    anchor_dir = os.path.dirname(root_layer.realPath) if root_layer.realPath else None

    refs: List[Dict[str, str]] = []
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            t = attr.GetTypeName()
            if t not in (Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.AssetArray):
                continue

            try:
                v = attr.Get()
            except Exception:
                v = None

            for av in _iter_asset_values(v):
                raw = (av.path or "").strip()
                if not raw:
                    continue
                resolved = _resolve_best_effort(av, anchor_dir=anchor_dir)
                refs.append({"prop": str(attr.GetPath()), "asset": raw, "resolved": resolved})

    uniq: Dict[Tuple[str, str], Dict[str, str]] = {}
    for r in refs:
        uniq[(r["prop"], r["asset"])] = r

    out = list(uniq.values())
    out.sort(key=lambda r: (r["resolved"], r["prop"]))
    return out


_MODULE_RE = re.compile(r"\b(?:import|using)\s+(?P<path>(?:\.|::)[^;\s]+)")


def _extract_mdl_module_deps(mdl_abs_path: str) -> Set[str]:
    try:
        with open(mdl_abs_path, "r", encoding="utf-8", errors="ignore") as f:
            txt = f.read()
    except OSError:
        return set()

    deps: Set[str] = set()
    for m in _MODULE_RE.finditer(txt):
        p = m.group("path")
        # examples:
        # - .::OmniUe4Function
        # - ::KooPbr::*
        # - ::KooPbr::functions
        # - ::nvidia::core_definitions
        p = p.replace("*", "")
        p = p.strip(";")

        if p.startswith(".::"):
            mod = p[len(".::") :]
        elif p.startswith("::"):
            mod = p[len("::") :]
        else:
            continue

        mod = mod.split("::", 1)[0]
        mod = mod.strip()
        if not mod:
            continue
        if mod in {"math", "state", "tex", "anno", "df", "nvidia"}:
            continue
        # allow typical module names and suffixes
        if not re.match(r"^[A-Za-z0-9_]+$", mod):
            continue
        deps.add(f"{mod}.mdl")

    return deps


def _textures_suffix(path_str: str) -> Optional[str]:
    p = _posix(path_str)
    low = p.lower()
    idx = low.rfind("/textures/")
    if idx == -1:
        if low.startswith("textures/"):
            return p[len("textures/") :]
        if low.startswith("./textures/"):
            return p[len("./textures/") :]
        return None
    return p[idx + len("/textures/") :]


_STRING_RE = re.compile(r"(?P<q>['\"])(?P<s>(?:\\.|(?!\1).)*)\1")


def _decode_string_literal(s: str) -> str:
    return (
        s.replace('\\"', '"')
        .replace("\\'", "'")
        .replace("\\\\", "\\")
        .replace("\\n", "\n")
        .replace("\\t", "\t")
    )


def _is_texture_string(s: str) -> bool:
    low = s.lower()
    if not low.endswith(_IMAGE_EXTS):
        return False
    return "/textures/" in _posix(low) or low.startswith("./textures/") or low.startswith("textures/")


def _resolve_mdl_texture_ref(mdl_abs_path: str, tex_ref: str) -> Optional[str]:
    tex_ref = tex_ref.strip()

    if tex_ref.startswith("file:"):
        path = tex_ref[len("file:") :]
        if path.startswith("//"):
            path = path.lstrip("/")
            path = "/" + path
        if os.path.isabs(path):
            return path
        return os.path.normpath(os.path.join(os.path.dirname(mdl_abs_path), path))

    if os.path.isabs(tex_ref):
        return tex_ref

    return os.path.normpath(os.path.join(os.path.dirname(mdl_abs_path), tex_ref))


def _extract_mdl_texture_refs(mdl_abs_path: str) -> List[Tuple[str, str]]:
    """Return list of (raw_ref, resolved_abs)."""
    try:
        with open(mdl_abs_path, "r", encoding="utf-8", errors="ignore") as f:
            txt = f.read()
    except OSError:
        return []

    out: List[Tuple[str, str]] = []
    for m in _STRING_RE.finditer(txt):
        raw = _decode_string_literal(m.group("s"))
        if not _is_texture_string(raw):
            continue
        resolved = _resolve_mdl_texture_ref(mdl_abs_path, raw)
        if not resolved:
            continue
        out.append((raw, resolved))
    return out


def _copy_file(src: str, dst: str, *, dry_run: bool, incremental: bool) -> bool:
    if incremental and os.path.exists(dst):
        try:
            sst = os.stat(src)
            dstst = os.stat(dst)
            if sst.st_size == dstst.st_size and int(sst.st_mtime) <= int(dstst.st_mtime):
                return False
        except OSError:
            pass

    if not dry_run:
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(src, dst)
    return True


def _collect_mdl_and_textures(
    *,
    usd_files: Sequence[str],
    src_root: str,
    dst_root: str,
    dry_run: bool,
    incremental: bool,
) -> Dict[str, object]:
    src_mdl_root = os.path.join(src_root, "Material", "mdl")
    src_tex_root = os.path.join(src_mdl_root, "textures")

    dst_mdl_root = os.path.join(dst_root, "Material", "mdl")
    dst_tex_root = os.path.join(dst_mdl_root, "textures")

    if not dry_run:
        os.makedirs(dst_mdl_root, exist_ok=True)
        os.makedirs(dst_tex_root, exist_ok=True)

    initial_mdl_abs: Set[str] = set()
    initial_tex_abs: Set[str] = set()
    usd_missing: List[Dict[str, str]] = []

    for usd in usd_files:
        for ref in _analyze_usd_asset_attributes(usd):
            resolved = ref["resolved"] or ref["asset"]
            if not resolved:
                continue
            low = resolved.lower()

            if low.endswith(".mdl"):
                if os.path.isabs(resolved) and os.path.isfile(resolved):
                    initial_mdl_abs.add(resolved)
                else:
                    usd_missing.append({"usd": usd, **ref})

            ext = os.path.splitext(low)[1]
            if ext in _IMAGE_EXTS:
                if os.path.isabs(resolved) and os.path.isfile(resolved):
                    if _is_under(resolved, src_tex_root):
                        initial_tex_abs.add(resolved)
                else:
                    # keep missing in report
                    usd_missing.append({"usd": usd, **ref})

    initial_mdl_rel: Set[str] = set()
    for p in sorted(initial_mdl_abs):
        rel = _safe_relpath(p, src_mdl_root)
        if rel is not None:
            initial_mdl_rel.add(_posix(rel))

    all_mdl_rel: Set[str] = set(initial_mdl_rel)
    processed: Set[str] = set()
    queue: List[str] = sorted(initial_mdl_rel)

    while queue:
        cur_rel = queue.pop(0)
        if cur_rel in processed:
            continue
        processed.add(cur_rel)

        cur_abs = os.path.join(src_mdl_root, *cur_rel.split("/"))
        deps = _extract_mdl_module_deps(cur_abs)
        for dep_name in sorted(deps):
            dep_abs = os.path.join(src_mdl_root, dep_name)
            if not os.path.isfile(dep_abs):
                continue
            dep_rel = _posix(os.path.relpath(dep_abs, src_mdl_root))
            if dep_rel not in all_mdl_rel:
                all_mdl_rel.add(dep_rel)
                queue.append(dep_rel)

    copied_mdls = 0
    skipped_mdls = 0
    missing_mdls: List[str] = []

    for rel in sorted(all_mdl_rel):
        src = os.path.join(src_mdl_root, *rel.split("/"))
        dst = os.path.join(dst_mdl_root, *rel.split("/"))
        if not os.path.isfile(src):
            missing_mdls.append(rel)
            continue
        if _copy_file(src, dst, dry_run=dry_run, incremental=incremental):
            copied_mdls += 1
        else:
            skipped_mdls += 1

    # Textures from MDLs
    tex_suffixes: Set[str] = set()
    tex_missing: List[str] = []

    for rel in sorted(all_mdl_rel):
        mdl_abs = os.path.join(src_mdl_root, *rel.split("/"))
        if not os.path.isfile(mdl_abs):
            continue
        for raw, resolved_abs in _extract_mdl_texture_refs(mdl_abs):
            suffix = _textures_suffix(resolved_abs) or _textures_suffix(raw)
            if not suffix:
                continue
            tex_suffixes.add(_posix(suffix))

    # Include textures referenced directly in USD
    for tex_abs in initial_tex_abs:
        rel = _safe_relpath(tex_abs, src_tex_root)
        if rel is not None:
            tex_suffixes.add(_posix(rel))

    copied_tex = 0
    skipped_tex = 0

    for suffix in sorted(tex_suffixes):
        src = os.path.join(src_tex_root, *suffix.split("/"))
        dst = os.path.join(dst_tex_root, *suffix.split("/"))

        if "<UDIM>" in suffix or "%(UDIM" in suffix or "$UDIM" in suffix:
            # Best-effort UDIM support.
            glob_pat = src.replace("<UDIM>", "[0-9][0-9][0-9][0-9]").replace("$UDIM", "[0-9][0-9][0-9][0-9]")
            tiles = sorted({p for p in glob.glob(glob_pat) if os.path.isfile(p)})  # type: ignore[name-defined]
            if not tiles:
                tex_missing.append(suffix)
                continue
            for tile in tiles:
                tile_rel = _safe_relpath(tile, src_tex_root)
                if tile_rel is None:
                    continue
                tile_dst = os.path.join(dst_tex_root, *tile_rel.split(os.sep))
                if _copy_file(tile, tile_dst, dry_run=dry_run, incremental=incremental):
                    copied_tex += 1
                else:
                    skipped_tex += 1
            continue

        if not os.path.isfile(src):
            tex_missing.append(suffix)
            continue

        if _copy_file(src, dst, dry_run=dry_run, incremental=incremental):
            copied_tex += 1
        else:
            skipped_tex += 1

    return {
        "src_mdl_root": src_mdl_root,
        "dst_mdl_root": dst_mdl_root,
        "initial_mdl_rel": sorted(initial_mdl_rel),
        "all_mdl_rel": sorted(all_mdl_rel),
        "missing_mdls": sorted(set(missing_mdls)),
        "copied_mdls": copied_mdls,
        "skipped_mdls": skipped_mdls,
        "textures_suffixes": sorted(tex_suffixes),
        "missing_textures": sorted(set(tex_missing)),
        "copied_textures": copied_tex,
        "skipped_textures": skipped_tex,
        "usd_missing_asset_refs": usd_missing,
    }


def _verify_subset(dst_root: str) -> Dict[str, object]:
    assets_root = os.path.join(dst_root, "GRScenes_assets")
    usd_files = _iter_usd_files_under(assets_root)

    missing: List[Dict[str, str]] = []
    for usd in usd_files:
        for ref in _analyze_usd_asset_attributes(usd):
            resolved = ref["resolved"] or ref["asset"]
            if not resolved:
                continue
            low = resolved.lower()
            if not (low.endswith(".mdl") or os.path.splitext(low)[1] in _IMAGE_EXTS):
                continue
            if os.path.isabs(resolved) and os.path.exists(resolved):
                continue
            missing.append({"usd": usd, **ref})

    return {"usd_count": len(usd_files), "missing_asset_refs": missing, "missing_count": len(missing)}


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True, help="Source GRScenes package root (e.g. GRScenes-test1)")
    ap.add_argument("--dst", required=True, help="Destination subset package root")
    ap.add_argument("--uid", action="append", default=[], help="Asset UID (repeatable)")
    ap.add_argument("--uid-file", default=None, help="Path to a text file containing UIDs (one per line)")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--no-incremental", action="store_true", help="Disable incremental copy (always overwrite)")
    ap.add_argument("--write-manifest", action="store_true", help="Write <dst>/subset_manifest.json")
    ap.add_argument("--verify", action="store_true", help="Re-scan subset USDs and report missing MDL/texture files")
    ap.add_argument("--create-empty-grscenes100", action="store_true", help="Create empty GRScenes100 dir (no scenes copied)")

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

    uids = list(args.uid or [])
    if args.uid_file:
        uids.extend(_read_uid_file(args.uid_file))
    uids = _dedup_keep_order([u for u in uids if u])

    if not uids:
        print("ERROR: No UIDs provided. Use --uid and/or --uid-file")
        return 2

    if not os.path.isdir(src_root):
        print(f"ERROR: src root not found: {src_root}")
        return 2

    if not dry_run:
        os.makedirs(dst_root, exist_ok=True)
        if args.create_empty_grscenes100:
            os.makedirs(os.path.join(dst_root, "GRScenes100"), exist_ok=True)

    uid_assets: List[UidAsset] = []
    missing_uids: Dict[str, List[str]] = {}

    for uid in uids:
        matches = _find_uid_asset_dirs(src_root, uid)
        if not matches:
            missing_uids[uid] = []
            continue
        if len(matches) > 1:
            missing_uids[uid] = [f"{cat}:{p}" for cat, p in matches]
            continue

        category, src_dir = matches[0]
        dst_dir = os.path.join(dst_root, "GRScenes_assets", category, uid)
        uid_assets.append(UidAsset(category=category, uid=uid, src_dir=src_dir, dst_dir=dst_dir))

    if missing_uids:
        print("ERROR: Some UIDs were not found uniquely under GRScenes_assets:")
        for uid, info in missing_uids.items():
            if not info:
                print(f"- {uid}: not found")
            else:
                print(f"- {uid}: multiple matches: {info}")
        return 3

    copied_asset_files = 0
    skipped_asset_files = 0

    for ua in uid_assets:
        c, s = _copy_tree(ua.src_dir, ua.dst_dir, dry_run=dry_run, incremental=incremental)
        copied_asset_files += c
        skipped_asset_files += s

    # Dependency scanning always uses source USDs so --dry-run is useful and side-effect free.
    usd_files_scanned: List[str] = []
    for ua in uid_assets:
        usd_files_scanned.extend(_iter_usd_files_under(ua.src_dir))
    usd_files_scanned = sorted(set(usd_files_scanned))

    dep_report = _collect_mdl_and_textures(
        usd_files=usd_files_scanned,
        src_root=src_root,
        dst_root=dst_root,
        dry_run=dry_run,
        incremental=incremental,
    )

    verify_report: Optional[Dict[str, object]] = None
    if args.verify:
        if dry_run:
            print("DRY-RUN: --verify skipped (no files written).")
        else:
            verify_report = _verify_subset(dst_root)

    manifest = {
        "src_root": src_root,
        "dst_root": dst_root,
        "uids": uids,
        "dry_run": dry_run,
        "incremental": incremental,
        "assets": [ua.__dict__ for ua in uid_assets],
        "usd_files_scanned": usd_files_scanned,
        "asset_copy": {"copied_files": copied_asset_files, "skipped_files": skipped_asset_files},
        "material": dep_report,
        "verify": verify_report,
    }

    print("=== Subset package summary ===")
    print("uids:", len(uids))
    print("asset_files_copied:", copied_asset_files, "skipped:", skipped_asset_files)
    print("usd_files_scanned:", len(usd_files_scanned))
    print("mdls:", len(dep_report.get("all_mdl_rel", [])), "copied:", dep_report.get("copied_mdls"), "missing:", len(dep_report.get("missing_mdls", [])))
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
        # Non-zero so CI / automation can catch incomplete packs.
        return 5

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
