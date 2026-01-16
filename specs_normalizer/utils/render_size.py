"""Utilities for computing per-asset render package size.

`usd_size` is intended to approximate how many bytes are needed to render an
asset correctly: the USD file plus the MDL files and textures it references.

These helpers prefer `pxr` (Usd/Sdf) when available.
"""

from __future__ import annotations

import os
import re
from typing import Iterable, List, Optional, Sequence, Set, Tuple


try:
    from pxr import Sdf, Usd  # type: ignore
except Exception:
    Sdf = None  # type: ignore
    Usd = None  # type: ignore


_IMAGE_EXTS = (".png", ".jpg", ".jpeg", ".exr", ".tif", ".tiff", ".tga", ".bmp", ".webp", ".hdr")


def _safe_file_size_bytes(path: str) -> Optional[int]:
    if not path or not os.path.exists(path) or not os.path.isfile(path):
        return None
    try:
        return os.path.getsize(path)
    except OSError:
        return None


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
        p = p.replace("*", "")
        p = p.strip(";")

        if p.startswith(".::"):
            mod = p[len(".::") :]
        elif p.startswith("::"):
            mod = p[len("::") :]
        else:
            continue

        mod = mod.split("::", 1)[0].strip()
        if not mod:
            continue
        # Ignore standard libs.
        if mod in {"math", "state", "tex", "anno", "df", "nvidia"}:
            continue
        if not re.match(r"^[A-Za-z0-9_]+$", mod):
            continue
        deps.add(f"{mod}.mdl")

    return deps


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
    low = low.replace("\\", "/")
    return "/textures/" in low or low.startswith("./textures/") or low.startswith("textures/")


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


def _extract_mdl_texture_refs(mdl_abs_path: str) -> List[str]:
    try:
        with open(mdl_abs_path, "r", encoding="utf-8", errors="ignore") as f:
            txt = f.read()
    except OSError:
        return []

    out: List[str] = []
    for m in _STRING_RE.finditer(txt):
        raw = _decode_string_literal(m.group("s"))
        if not _is_texture_string(raw):
            continue
        resolved = _resolve_mdl_texture_ref(mdl_abs_path, raw)
        if resolved:
            out.append(resolved)
    return out


def _guess_material_mdl_root(dataset_root: Optional[str], mdl_abs_paths: Sequence[str]) -> Optional[str]:
    if dataset_root:
        cand = os.path.join(dataset_root, "Material", "mdl")
        if os.path.isdir(cand):
            return cand

    # Fallback: assume all MDLs live together
    if mdl_abs_paths:
        return os.path.dirname(mdl_abs_paths[0])
    return None


def compute_usd_render_package_size_bytes(
    usd_path: str,
    *,
    dataset_root: Optional[str] = None,
    require_pxr: bool = False,
) -> Optional[int]:
    """Compute size (bytes) of USD + referenced MDLs + referenced textures.

    - Returns `None` if `usd_path` doesn't exist.
    - If `pxr` is unavailable:
      - when require_pxr=True, raises RuntimeError
      - otherwise returns just the USD file size.

    Note: This is a best-effort approximation.
    """

    usd_file_size = _safe_file_size_bytes(usd_path)
    if usd_file_size is None:
        return None

    if Usd is None:
        if require_pxr:
            raise RuntimeError("pxr.Usd not available; cannot compute render package dependencies")
        return usd_file_size

    stage = Usd.Stage.Open(usd_path)
    if stage is None:
        return usd_file_size

    root_layer = stage.GetRootLayer()
    anchor_dir = os.path.dirname(root_layer.realPath) if getattr(root_layer, "realPath", "") else os.path.dirname(usd_path)

    mdl_abs: Set[str] = set()
    img_abs: Set[str] = set()

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
                resolved = _resolve_best_effort(av, anchor_dir=anchor_dir)
                if not resolved:
                    continue
                low = resolved.lower()
                if low.endswith(".mdl") and os.path.isfile(resolved):
                    mdl_abs.add(resolved)
                ext = os.path.splitext(low)[1]
                if ext in _IMAGE_EXTS and os.path.isfile(resolved):
                    img_abs.add(resolved)

    # MDL closure: imports/using
    mdl_root = _guess_material_mdl_root(dataset_root, sorted(mdl_abs))
    if mdl_root:
        processed: Set[str] = set()
        queue: List[str] = sorted(mdl_abs)
        while queue:
            cur = queue.pop(0)
            if cur in processed:
                continue
            processed.add(cur)

            for dep_name in sorted(_extract_mdl_module_deps(cur)):
                dep_abs = os.path.join(mdl_root, dep_name)
                if os.path.isfile(dep_abs) and dep_abs not in mdl_abs:
                    mdl_abs.add(dep_abs)
                    queue.append(dep_abs)

    # Textures referenced by MDL
    tex_abs: Set[str] = set()
    for m in sorted(mdl_abs):
        for tex in _extract_mdl_texture_refs(m):
            if os.path.isfile(tex):
                tex_abs.add(tex)

    total = usd_file_size
    for p in sorted(mdl_abs | tex_abs | img_abs):
        sz = _safe_file_size_bytes(p)
        if sz:
            total += sz

    return total
