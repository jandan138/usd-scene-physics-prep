#!/usr/bin/env python3
"""Rewrite asset references in a layout.usd and optionally apply transform compensation.

This script supports the workflow described in:
- docs/operations/asset_dedup_c1_scene_instancing_runbook.md (Step 3B)

Core idea:
- Replace references/payloads/asset-path attributes that point to an "old" asset USD with a "canonical" asset USD.
- If assets carry different internal alignment transforms, adjust the *layout prim* local transform so that the
  final world placement stays unchanged.

Mathematically (row-vector convention, p' = p * M):
- p_world = p_mesh * M_internal * M_layout = p_instance * M_layout
- Two compensation regimes:
  1) geom_only (V=I, mesh vertices identical): internal-transform compensation
     M_layout_new = M_canon_int^{-1} * M_old_int * M_layout_old
  2) non-geom_only (V in instance-space from extract_instance_space_vertices):
     V maps p_canon_inst -> p_old_inst, so M_layout_new = V * M_layout_old

Important limitations (intentional):
- We compute "asset internal" as the full chain transform from defaultPrim through Instance down to the
  first mesh's parent prim.  This captures intermediate prims like Group_00, Component_N etc. that carry
  positioning transforms.  Falls back to Instance-only or identity when no mesh or no Instance child exists.
- The script will author a single `xformOp:transform` on the instance prim to represent the compensated transform.
  This may replace existing TRS op stacks; it preserves the composed matrix but not the exact op decomposition.

Use dry-run first.
"""

from __future__ import annotations

import argparse
import json
import os
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

try:
    from pxr import Gf, Sdf, Usd, UsdGeom

    _PXR_IMPORT_ERROR: Optional[Exception] = None
except Exception as e:  # pragma: no cover
    Gf = None  # type: ignore
    Sdf = None  # type: ignore
    Usd = None  # type: ignore
    UsdGeom = None  # type: ignore
    _PXR_IMPORT_ERROR = e


@dataclass(frozen=True)
class MappingPair:
    old_abs: str
    canonical_abs: str


def _norm_abs(p: str) -> str:
    p = (p or "").replace("\\", "/")
    return os.path.abspath(p)


def _posix_relpath(path: str) -> str:
    return (path or "").replace("\\", "/")


def _find_subset_root_from_layout(layout_usd: str) -> Optional[str]:
    """Infer subset root by walking parents until we see GRScenes_assets."""
    cur = os.path.abspath(layout_usd)
    cur = os.path.dirname(cur)

    while True:
        if os.path.isdir(os.path.join(cur, "GRScenes_assets")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            return None
        cur = parent


def _load_mapping(mapping_path: str, subset_root: str) -> List[MappingPair]:
    with open(mapping_path, "r", encoding="utf-8") as f:
        payload = json.load(f)

    pairs: List[Tuple[str, str]] = []

    if isinstance(payload, dict):
        for k, v in payload.items():
            if not isinstance(k, str) or not isinstance(v, str):
                continue
            pairs.append((k, v))
    elif isinstance(payload, list):
        for item in payload:
            if isinstance(item, dict) and "old" in item and "canonical" in item:
                old = item.get("old")
                canonical = item.get("canonical")
                if isinstance(old, str) and isinstance(canonical, str):
                    pairs.append((old, canonical))
    else:
        raise ValueError(
            "Unsupported mapping JSON format. Use dict {old: canonical} or list of {old, canonical}."
        )

    out: List[MappingPair] = []
    for old_p, canonical_p in pairs:
        old_abs = _resolve_under_subset_or_abs(subset_root, old_p)
        canonical_abs = _resolve_under_subset_or_abs(subset_root, canonical_p)
        out.append(MappingPair(old_abs=old_abs, canonical_abs=canonical_abs))

    # de-dup
    seen = set()
    unique: List[MappingPair] = []
    for mp in out:
        k = (mp.old_abs, mp.canonical_abs)
        if k in seen:
            continue
        seen.add(k)
        unique.append(mp)
    return unique


def _load_certificate_lookup(
    certificate_jsonl: str,
    subset_root: str,
) -> Dict[Tuple[str, str], Dict[str, Any]]:
    lookup: Dict[Tuple[str, str], Dict[str, Any]] = {}
    with open(certificate_jsonl, "r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            row = json.loads(line)
            if not bool(row.get("eligible")):
                continue
            if row.get("reject_reason") not in (None, ""):
                continue
            old_path = row.get("old_asset") or row.get("old_usd")
            canonical_path = row.get("canonical_asset") or row.get("canonical_usd")
            if not isinstance(old_path, str) or not isinstance(canonical_path, str):
                continue
            old_abs = _resolve_under_subset_or_abs(subset_root, old_path)
            canonical_abs = _resolve_under_subset_or_abs(subset_root, canonical_path)
            lookup[(old_abs, canonical_abs)] = row
    return lookup


def _resolve_under_subset_or_abs(subset_root: str, p: str) -> str:
    if not p:
        return _norm_abs(p)

    s = p.replace("\\", "/")
    if os.path.isabs(s):
        return _norm_abs(s)

    s = s.lstrip("./")

    # Support common repo-relative forms, e.g.:
    # - GRScenes-test1/GRScenes_assets/... (repo-root relative)
    # - GRScenes_assets/... (subset-root relative)
    subset_root = _norm_abs(subset_root)
    subset_name = os.path.basename(subset_root.rstrip("/"))
    repo_root = os.path.dirname(subset_root)

    if s.startswith("GRScenes_assets/"):
        return _norm_abs(os.path.join(subset_root, s))

    if subset_name and s.startswith(subset_name + "/"):
        return _norm_abs(os.path.join(repo_root, s))

    # Default: treat as subset-root relative.
    return _norm_abs(os.path.join(subset_root, s))


def _category_root_from_asset_abs(asset_abs: str) -> str:
    return _norm_abs(os.path.dirname(os.path.dirname(os.path.dirname(asset_abs))))


def _uid_from_asset_abs(asset_abs: str) -> str:
    return os.path.basename(os.path.dirname(os.path.dirname(asset_abs)))


def _build_transitive_witness_inputs(
    old_abs: str,
    canonical_abs: str,
    cert_row: Dict[str, Any],
) -> Tuple[Dict[Tuple[str, str], str], List[str]]:
    witness_uids = cert_row.get("transitive_witness_uids") or []
    witness_modes = cert_row.get("transitive_witness_modes") or []
    if not isinstance(witness_uids, list) or len(witness_uids) < 2:
        raise ValueError("missing transitive_witness_uids")
    if (
        not isinstance(witness_modes, list)
        or len(witness_modes) != len(witness_uids) - 1
    ):
        raise ValueError("transitive witness modes do not match witness path")

    old_root = _category_root_from_asset_abs(old_abs)
    canonical_root = _category_root_from_asset_abs(canonical_abs)
    if old_root != canonical_root:
        raise ValueError("transitive witness spans multiple category roots")

    old_uid = _uid_from_asset_abs(old_abs)
    canonical_uid = _uid_from_asset_abs(canonical_abs)
    uid_to_path: Dict[str, str] = {
        old_uid: old_abs,
        canonical_uid: canonical_abs,
    }
    for uid in witness_uids:
        if not isinstance(uid, str):
            raise ValueError("transitive witness uid must be a string")
        uid_to_path.setdefault(
            uid, _norm_abs(os.path.join(old_root, uid, "usd", f"{uid}.usd"))
        )

    group_members = [uid_to_path[uid] for uid in witness_uids]
    mode_index: Dict[Tuple[str, str], str] = {}
    for lhs_uid, rhs_uid, mode in zip(witness_uids, witness_uids[1:], witness_modes):
        if not isinstance(mode, str):
            raise ValueError("transitive witness mode must be a string")
        mode_index[(lhs_uid, rhs_uid)] = mode
        mode_index[(rhs_uid, lhs_uid)] = mode
    return mode_index, group_members


def _resolve_ref_asset_path(base_dir: str, asset_path: str) -> str:
    if not asset_path:
        return ""
    s = asset_path.replace("\\", "/")
    if os.path.isabs(s):
        return _norm_abs(s)
    return _norm_abs(os.path.join(base_dir, s))


def _format_new_asset_path(base_dir: str, old_asset_path_raw: str, new_abs: str) -> str:
    """Keep relative/absolute style consistent with the original authored path."""
    if not old_asset_path_raw:
        return new_abs

    old_s = old_asset_path_raw.replace("\\", "/")
    if os.path.isabs(old_s):
        return _posix_relpath(new_abs)

    rel = os.path.relpath(new_abs, base_dir)
    return _posix_relpath(rel)


def _get_asset_internal_matrix(asset_usd_abs: str) -> Gf.Matrix4d:
    """Compute full internal transform from defaultPrim through Instance down to first mesh parent.

    This captures the complete transform chain that positions mesh geometry
    in the asset's local coordinate space, including intermediate prims
    like Group_00, Component_N, etc.

    The dedup compensation formula ``M_new_local = M_canon_internal⁻¹ * M_old_internal * M_old_local``
    requires ``M_internal`` to represent the same coordinate space transformation for both
    old and canonical assets.  By walking the chain to the first mesh's parent we capture
    the full positioning transform regardless of hierarchy depth.
    """
    # Import shared helpers (avoids duplicating chain-walk logic)
    import sys as _sys

    _scripts_dir = os.path.dirname(os.path.abspath(__file__))
    if _scripts_dir not in _sys.path:
        _sys.path.insert(0, _scripts_dir)
    from usd_xform_utils import get_local_matrix, get_chain_transform, find_all_meshes

    stage = Usd.Stage.Open(asset_usd_abs, load=Usd.Stage.LoadNone)
    if stage is None:
        raise RuntimeError(f"Failed to open asset USD: {asset_usd_abs}")

    default_prim = stage.GetDefaultPrim()
    if not default_prim or not default_prim.IsValid():
        children = [c for c in stage.GetPseudoRoot().GetChildren() if c and c.IsValid()]
        default_prim = children[0] if children else None

    if not default_prim or not default_prim.IsValid():
        return Gf.Matrix4d(1.0)

    # Look for "Instance" child under the default prim (e.g. /Root/Instance).
    instance_prim = None
    for child in default_prim.GetChildren():
        if child.GetName() == "Instance":
            instance_prim = child
            break

    if not instance_prim or not instance_prim.IsValid():
        return Gf.Matrix4d(1.0)

    meshes = find_all_meshes(instance_prim)
    if not meshes:
        # No mesh → fallback to Instance-only (preserves old behavior for meshless assets)
        return get_local_matrix(instance_prim)

    # Use first mesh's parent as the chain endpoint.
    # The chain: Instance → Group_00 → ... → mesh_parent
    # This gives M such that p_defaultPrim = p_mesh_local * M
    first_mesh = meshes[0]
    mesh_parent = first_mesh.GetParent()

    # get_chain_transform(ancestor, descendant) walks from ancestor's child to descendant inclusive
    if mesh_parent.GetPath() == instance_prim.GetPath():
        # Mesh is direct child of Instance → just Instance transform (common case)
        return get_local_matrix(instance_prim)

    return get_chain_transform(default_prim, mesh_parent)


def _ensure_matrix_xform(prim) -> None:
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return

    # Author a single xformOp:transform.
    prim.CreateAttribute(
        "xformOpOrder", Sdf.ValueTypeNames.TokenArray, custom=False
    ).Set(["xformOp:transform"])
    if not prim.HasAttribute("xformOp:transform"):
        prim.CreateAttribute(
            "xformOp:transform", Sdf.ValueTypeNames.Matrix4d, custom=False
        )


def _set_local_matrix(prim, m: Gf.Matrix4d) -> None:
    _ensure_matrix_xform(prim)
    prim.GetAttribute("xformOp:transform").Set(m)


def _iter_asset_values(v: Any) -> Iterable[Sdf.AssetPath]:
    if v is None:
        return []
    if isinstance(v, Sdf.AssetPath):
        return [v]
    if isinstance(v, (list, tuple)):
        out: List[Sdf.AssetPath] = []
        for item in v:
            if isinstance(item, Sdf.AssetPath):
                out.append(item)
        return out
    return []


def _rewrite_reference_listop(
    refs, base_dir: str, old_abs: str, new_abs: str
) -> Tuple[Any, int]:
    if not refs:
        return refs, 0

    changed = 0
    # NOTE: In some USD Python bindings, ListOp item lists returned by
    # `refs.explicitItems` etc are plain Python lists that are *copies*.
    # Mutating them via `.append()` does not affect the ListOp.
    # We must construct new item lists and assign them back.
    new_refs = Sdf.ReferenceListOp()

    # Preserve which bucket the original authored into when possible.
    # (GRScenes layouts commonly use Prepended Items.)
    if getattr(refs, "explicitItems", None):
        bucket = "explicitItems"
    elif getattr(refs, "prependedItems", None):
        bucket = "prependedItems"
    elif getattr(refs, "addedItems", None):
        bucket = "addedItems"
    else:
        bucket = "explicitItems"

    def _iter_items():
        try:
            return list(refs.GetAddedOrExplicitItems())
        except Exception:
            return []

    items = _iter_items()
    if not items:
        return refs, 0

    new_items: List[Sdf.Reference] = []
    for ref in items:
        ap = getattr(ref, "assetPath", "") or ""
        resolved = _resolve_ref_asset_path(base_dir, ap)
        if resolved == old_abs:
            new_ap = _format_new_asset_path(base_dir, ap, new_abs)
            new_ref = Sdf.Reference(
                assetPath=new_ap,
                primPath=getattr(ref, "primPath", Sdf.Path.emptyPath),
                layerOffset=getattr(ref, "layerOffset", None),
                customData=getattr(ref, "customData", None),
            )
            new_items.append(new_ref)
            changed += 1
        else:
            new_items.append(ref)

    # Assign back into the correct bucket.
    setattr(new_refs, bucket, new_items)

    return new_refs, changed


def _rewrite_payload_listop(
    payloads, base_dir: str, old_abs: str, new_abs: str
) -> Tuple[Any, int]:
    if not payloads:
        return payloads, 0

    changed = 0
    new_pls = Sdf.PayloadListOp()

    if getattr(payloads, "explicitItems", None):
        bucket = "explicitItems"
    elif getattr(payloads, "prependedItems", None):
        bucket = "prependedItems"
    elif getattr(payloads, "addedItems", None):
        bucket = "addedItems"
    else:
        bucket = "explicitItems"

    def _iter_items():
        try:
            return list(payloads.GetAddedOrExplicitItems())
        except Exception:
            return []

    items = _iter_items()
    if not items:
        return payloads, 0

    new_items: List[Sdf.Payload] = []
    for pl in items:
        ap = getattr(pl, "assetPath", "") or ""
        resolved = _resolve_ref_asset_path(base_dir, ap)
        if resolved == old_abs:
            new_ap = _format_new_asset_path(base_dir, ap, new_abs)
            new_pl = Sdf.Payload(
                assetPath=new_ap,
                primPath=getattr(pl, "primPath", Sdf.Path.emptyPath),
                layerOffset=getattr(pl, "layerOffset", None),
            )
            new_items.append(new_pl)
            changed += 1
        else:
            new_items.append(pl)

    setattr(new_pls, bucket, new_items)

    return new_pls, changed


def _rewrite_reference_listop_with_mapping(
    refs,
    base_dir: str,
    mapping_by_old_abs: Dict[str, str],
) -> Tuple[Any, int, List[Tuple[str, str]]]:
    """Rewrite references by resolving each authored ref and looking it up in a mapping dict.

    Returns: (new_refs, changed_count, changed_pairs[(old_abs, new_abs), ...])
    """
    if not refs:
        return refs, 0, []

    def _iter_items():
        try:
            return list(refs.GetAddedOrExplicitItems())
        except Exception:
            return []

    items = _iter_items()
    if not items:
        return refs, 0, []

    if getattr(refs, "explicitItems", None):
        bucket = "explicitItems"
    elif getattr(refs, "prependedItems", None):
        bucket = "prependedItems"
    elif getattr(refs, "addedItems", None):
        bucket = "addedItems"
    else:
        bucket = "explicitItems"

    changed = 0
    changed_pairs: List[Tuple[str, str]] = []
    new_refs = Sdf.ReferenceListOp()
    new_items: List[Sdf.Reference] = []

    for ref in items:
        ap = getattr(ref, "assetPath", "") or ""
        resolved = _resolve_ref_asset_path(base_dir, ap)
        new_abs = mapping_by_old_abs.get(resolved)
        if new_abs:
            new_ap = _format_new_asset_path(base_dir, ap, new_abs)
            new_ref = Sdf.Reference(
                assetPath=new_ap,
                primPath=getattr(ref, "primPath", Sdf.Path.emptyPath),
                layerOffset=getattr(ref, "layerOffset", None),
                customData=getattr(ref, "customData", None),
            )
            new_items.append(new_ref)
            changed += 1
            changed_pairs.append((resolved, new_abs))
        else:
            new_items.append(ref)

    setattr(new_refs, bucket, new_items)
    return new_refs, changed, changed_pairs


def _rewrite_payload_listop_with_mapping(
    payloads,
    base_dir: str,
    mapping_by_old_abs: Dict[str, str],
) -> Tuple[Any, int, List[Tuple[str, str]]]:
    if not payloads:
        return payloads, 0, []

    def _iter_items():
        try:
            return list(payloads.GetAddedOrExplicitItems())
        except Exception:
            return []

    items = _iter_items()
    if not items:
        return payloads, 0, []

    if getattr(payloads, "explicitItems", None):
        bucket = "explicitItems"
    elif getattr(payloads, "prependedItems", None):
        bucket = "prependedItems"
    elif getattr(payloads, "addedItems", None):
        bucket = "addedItems"
    else:
        bucket = "explicitItems"

    changed = 0
    changed_pairs: List[Tuple[str, str]] = []
    new_pls = Sdf.PayloadListOp()
    new_items: List[Sdf.Payload] = []

    for pl in items:
        ap = getattr(pl, "assetPath", "") or ""
        resolved = _resolve_ref_asset_path(base_dir, ap)
        new_abs = mapping_by_old_abs.get(resolved)
        if new_abs:
            new_ap = _format_new_asset_path(base_dir, ap, new_abs)
            new_pl = Sdf.Payload(
                assetPath=new_ap,
                primPath=getattr(pl, "primPath", Sdf.Path.emptyPath),
                layerOffset=getattr(pl, "layerOffset", None),
            )
            new_items.append(new_pl)
            changed += 1
            changed_pairs.append((resolved, new_abs))
        else:
            new_items.append(pl)

    setattr(new_pls, bucket, new_items)
    return new_pls, changed, changed_pairs


def _get_authored_payload_listop(prim):
    if not prim or not prim.IsValid() or not prim.HasAuthoredPayloads():
        return None
    try:
        for spec in prim.GetPrimStack():
            payload_list = getattr(spec, "payloadList", None)
            if payload_list is None:
                continue
            try:
                items = list(payload_list.GetAddedOrExplicitItems())
            except Exception:
                items = []
            if items:
                return payload_list
    except Exception:
        return None
    return None


def _set_payload_listop(prim, payloads) -> None:
    items = []
    try:
        items = list(payloads.GetAddedOrExplicitItems())
    except Exception:
        items = []
    payload_api = prim.GetPayloads()
    payload_api.ClearPayloads()
    for item in items:
        payload_api.AddPayload(
            getattr(item, "assetPath", "") or "",
            getattr(item, "primPath", Sdf.Path.emptyPath),
            getattr(item, "layerOffset", None),
        )


def _preview_asset_attr_mapping_changes(
    prim,
    base_dir: str,
    mapping_by_old_abs: Dict[str, str],
) -> List[Dict[str, Any]]:
    previews: List[Dict[str, Any]] = []
    for attr in prim.GetAttributes():
        t = attr.GetTypeName()
        if t not in (Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.AssetArray):
            continue

        try:
            v = attr.Get()
        except Exception:
            continue

        if isinstance(v, Sdf.AssetPath):
            resolved = _resolve_ref_asset_path(base_dir, v.path)
            new_abs = mapping_by_old_abs.get(resolved)
            if new_abs:
                previews.append(
                    {
                        "attr": attr.GetName(),
                        "old_abs": resolved,
                        "new_abs": new_abs,
                    }
                )
            continue

        if isinstance(v, (list, tuple)):
            for item in v:
                if not isinstance(item, Sdf.AssetPath):
                    continue
                resolved = _resolve_ref_asset_path(base_dir, item.path)
                new_abs = mapping_by_old_abs.get(resolved)
                if new_abs:
                    previews.append(
                        {
                            "attr": attr.GetName(),
                            "old_abs": resolved,
                            "new_abs": new_abs,
                        }
                    )
    return previews


def rewrite_layout(
    *,
    layout_usd: str,
    out_usd: Optional[str],
    subset_root: str,
    mapping_pairs: Sequence[MappingPair],
    apply_compensation: bool,
    set_instanceable: bool,
    dry_run: bool,
    report_out: Optional[str],
    max_preview: int,
    v_matrix_mode: str = "none",
    mode_reports_dir: Optional[str] = None,
    certificate_jsonl: Optional[str] = None,
    bbox_gated: bool = False,
    bbox_policy: str = "bbox_primary_rmse_observe",
) -> Dict[str, Any]:
    if _PXR_IMPORT_ERROR is not None:
        raise RuntimeError(f"pxr.Usd not available: {_PXR_IMPORT_ERROR}")

    layout_usd = _norm_abs(layout_usd)

    started_at = time.time()

    # --- V matrix setup ---
    _v_mode_index = None
    _certificate_lookup: Dict[Tuple[str, str], Dict[str, Any]] = {}
    _v_matrix_cache: Dict[Tuple[str, str], Tuple[Optional[Gf.Matrix4d], str]] = {}
    if v_matrix_mode == "auto" and apply_compensation:
        import importlib.util as _ilu

        _cvt_path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "compute_vertex_transform.py"
        )
        _spec = _ilu.spec_from_file_location("compute_vertex_transform", _cvt_path)
        _cvt = _ilu.module_from_spec(_spec)
        _spec.loader.exec_module(_cvt)
        if mode_reports_dir:
            _v_mode_index = _cvt.build_mode_index(mode_reports_dir)
        else:
            _v_mode_index = {}
        if certificate_jsonl:
            _certificate_lookup = _load_certificate_lookup(
                certificate_jsonl, subset_root
            )

    def _get_V_cached(old_abs: str, new_abs: str) -> Tuple[Optional[Gf.Matrix4d], str]:
        """Get V matrix and dedup mode for a pair, with caching.

        Returns (None, "aspect_ratio_rejected") when the pair fails the
        aspect-ratio guard — the caller must skip the reference rewrite
        entirely for these pairs.
        """
        if v_matrix_mode != "auto":
            return Gf.Matrix4d(1.0), "identity"
        key = (old_abs, new_abs)
        cached = _v_matrix_cache.get(key)
        if cached is not None:
            return cached
        mode = _cvt.determine_compensation_mode(old_abs, new_abs, _v_mode_index)
        if mode == "transitive":
            cert_row = _certificate_lookup.get(key)
            if cert_row is None:
                _v_matrix_cache[key] = (None, mode)
                return None, mode
            try:
                local_mode_index, group_members = _build_transitive_witness_inputs(
                    old_abs,
                    new_abs,
                    cert_row,
                )
                V = _cvt.compute_V_for_pair(
                    old_abs,
                    new_abs,
                    mode,
                    mode_index=local_mode_index,
                    group_members=group_members,
                )
            except RuntimeError as e:
                if "aspect ratio mismatch" in str(e):
                    _v_matrix_cache[key] = (None, "aspect_ratio_rejected")
                    return None, "aspect_ratio_rejected"
                V = None
            except Exception:
                V = None
            _v_matrix_cache[key] = (V, mode)
            return V, mode
        try:
            V = _cvt.compute_V_for_pair(
                old_abs, new_abs, mode, mode_index=_v_mode_index
            )
        except RuntimeError as e:
            if "aspect ratio mismatch" in str(e):
                _v_matrix_cache[key] = (None, "aspect_ratio_rejected")
                return None, "aspect_ratio_rejected"
            V = Gf.Matrix4d(1.0)
        except Exception:
            V = Gf.Matrix4d(1.0)
        _v_matrix_cache[key] = (V, mode)
        return V, mode

    # Copy-on-write unless writing in-place.
    # If dry-run, do NOT copy/write; just report the intended output path.
    planned_out_usd: Optional[str] = None
    target_usd = layout_usd
    if out_usd:
        out_usd = _norm_abs(out_usd)
        planned_out_usd = out_usd
        if not dry_run:
            os.makedirs(os.path.dirname(out_usd), exist_ok=True)
            # In-place: out_usd == layout_usd → skip copy to avoid truncating source
            if os.path.abspath(out_usd) != os.path.abspath(layout_usd):
                with open(layout_usd, "rb") as rf, open(out_usd, "wb") as wf:
                    wf.write(rf.read())
            target_usd = out_usd

    # Resolve relative reference paths from the directory of the file we are
    # actually opening/modifying.
    base_dir = os.path.dirname(target_usd)

    stage = Usd.Stage.Open(target_usd)
    if stage is None:
        raise RuntimeError(f"Failed to open stage: {target_usd}")

    # Lazily compute internal matrices only for assets involved in actual changes.
    asset_internal_cache: Dict[str, Gf.Matrix4d] = {}

    def _get_internal_cached(asset_abs: str) -> Gf.Matrix4d:
        m = asset_internal_cache.get(asset_abs)
        if m is None:
            m = _get_asset_internal_matrix(asset_abs)
            asset_internal_cache[asset_abs] = m
        return m

    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    changes: List[Dict[str, Any]] = []
    refs_changed = 0
    payloads_changed = 0
    asset_attrs_changed = 0
    xform_compensated = 0
    instanceable_set = 0
    reject_records = 0

    rewritten_prims: set[str] = set()

    mapping_by_old = {mp.old_abs: mp.canonical_abs for mp in mapping_pairs}

    def _record_reject(
        prim_path: str,
        reason: str,
        *,
        old_abs: Optional[str] = None,
        canonical_abs: Optional[str] = None,
        scene_shape: Optional[str] = None,
        details: Optional[Dict[str, Any]] = None,
    ) -> None:
        nonlocal reject_records
        reject_records += 1
        payload: Dict[str, Any] = {
            "prim": prim_path,
            "kind": "bbox_gated_reject",
            "reject_reason": reason,
            "bbox_policy": bbox_policy,
        }
        if old_abs is not None:
            payload["old_abs"] = old_abs
        if canonical_abs is not None:
            payload["canonical_abs"] = canonical_abs
        if scene_shape is not None:
            payload["scene_shape"] = scene_shape
        if details:
            payload.update(details)
        changes.append(payload)

    # Pre-filter: exclude pairs that fail the aspect-ratio guard.
    # This must happen before the traversal loop so that rejected pairs
    # never get their references rewritten.
    if v_matrix_mode == "auto" and apply_compensation:
        rejected_keys: List[str] = []
        for old_abs, canonical_abs in mapping_by_old.items():
            V, dedup_mode = _get_V_cached(old_abs, canonical_abs)
            if V is None and dedup_mode == "aspect_ratio_rejected":
                rejected_keys.append(old_abs)
                changes.append(
                    {
                        "prim": "(pre-filter)",
                        "kind": "aspect_ratio_rejected",
                        "old_abs": old_abs,
                        "canonical_abs": canonical_abs,
                        "dedup_mode": dedup_mode,
                    }
                )
        for k in rejected_keys:
            del mapping_by_old[k]

    for prim in stage.Traverse():
        if not prim or not prim.IsValid():
            continue

        prim_path_str = str(prim.GetPath())
        refs = prim.GetMetadata("references") if prim.HasAuthoredReferences() else None
        pls = _get_authored_payload_listop(prim)

        new_refs = refs
        ref_changed_pairs: List[Tuple[str, str]] = []
        if refs:
            new_refs, ref_count, ref_changed_pairs = (
                _rewrite_reference_listop_with_mapping(refs, base_dir, mapping_by_old)
            )
        else:
            ref_count = 0

        new_pls = pls
        payload_changed_pairs: List[Tuple[str, str]] = []
        if pls:
            new_pls, payload_count, payload_changed_pairs = (
                _rewrite_payload_listop_with_mapping(pls, base_dir, mapping_by_old)
            )
        else:
            payload_count = 0

        asset_attr_previews = _preview_asset_attr_mapping_changes(
            prim, base_dir, mapping_by_old
        )

        if bbox_gated and (ref_count or payload_count or asset_attr_previews):
            if payload_count:
                old_abs, new_abs = payload_changed_pairs[0]
                _record_reject(
                    prim_path_str,
                    "payload_rewrite_disabled_bbox_phase1",
                    old_abs=old_abs,
                    canonical_abs=new_abs,
                    scene_shape="payload",
                    details={
                        "changed_pairs": [
                            {"old_abs": a, "new_abs": b}
                            for a, b in payload_changed_pairs
                        ]
                    },
                )
                continue

            if asset_attr_previews:
                first = asset_attr_previews[0]
                _record_reject(
                    prim_path_str,
                    "asset_attr_rewrite_disabled_bbox_phase1",
                    old_abs=str(first["old_abs"]),
                    canonical_abs=str(first["new_abs"]),
                    scene_shape="asset_attr",
                    details={"attrs": asset_attr_previews},
                )
                continue

            uniq = sorted(set(ref_changed_pairs))
            if len(uniq) != 1:
                _record_reject(
                    prim_path_str,
                    "multi_ref_changed_prim_disabled_bbox_phase1",
                    scene_shape="multi_ref",
                    details={
                        "changed_pairs": [{"old_abs": a, "new_abs": b} for a, b in uniq]
                    },
                )
                continue

            old_abs, new_abs = uniq[0]
            if not apply_compensation:
                _record_reject(
                    prim_path_str,
                    "bbox_gated_requires_compensation",
                    old_abs=old_abs,
                    canonical_abs=new_abs,
                    scene_shape="references",
                )
                continue

            try:
                old_local = xform_cache.GetLocalTransformation(prim)
                if isinstance(old_local, tuple):
                    old_local = old_local[0]
                V, dedup_mode = _get_V_cached(old_abs, new_abs)
                if V is None:
                    _record_reject(
                        prim_path_str,
                        "unresolved_compensation_state",
                        old_abs=old_abs,
                        canonical_abs=new_abs,
                        scene_shape="references",
                    )
                    continue
                if dedup_mode not in (
                    "geom_only",
                    "identity",
                    "topo_filesize",
                    "shape_invariant",
                    "transitive",
                ):
                    _record_reject(
                        prim_path_str,
                        f"mode_not_enabled_{dedup_mode}",
                        old_abs=old_abs,
                        canonical_abs=new_abs,
                        scene_shape="references",
                    )
                    continue
                if dedup_mode in ("geom_only", "identity"):
                    old_internal = _get_internal_cached(old_abs)
                    canonical_internal = _get_internal_cached(new_abs)
                    new_local = (
                        canonical_internal.GetInverse() * old_internal * old_local
                    )
                else:
                    new_local = V * old_local
            except Exception as e:
                _record_reject(
                    prim_path_str,
                    "xform_preflight_failed",
                    old_abs=old_abs,
                    canonical_abs=new_abs,
                    scene_shape="references",
                    details={"error": str(e)},
                )
                continue

            prim.SetMetadata("references", new_refs)
            refs_changed += ref_count
            rewritten_prims.add(prim_path_str)
            for old_abs_item, new_abs_item in ref_changed_pairs:
                changes.append(
                    {
                        "prim": prim_path_str,
                        "kind": "references",
                        "old_abs": old_abs_item,
                        "new_abs": new_abs_item,
                        "count": 1,
                    }
                )
            _set_local_matrix(prim, new_local)
            xform_compensated += 1
            changes.append(
                {
                    "prim": prim_path_str,
                    "kind": "xform_compensation",
                    "old_asset_abs": old_abs,
                    "canonical_asset_abs": new_abs,
                    "v_matrix_mode": v_matrix_mode,
                    "dedup_mode": dedup_mode,
                }
            )
            continue

        # Legacy behavior
        if refs and ref_count:
            prim.SetMetadata("references", new_refs)
            refs_changed += ref_count
            rewritten_prims.add(prim_path_str)
            for old_abs, new_abs in ref_changed_pairs:
                changes.append(
                    {
                        "prim": prim_path_str,
                        "kind": "references",
                        "old_abs": old_abs,
                        "new_abs": new_abs,
                        "count": 1,
                    }
                )

            if apply_compensation:
                uniq = sorted(set(ref_changed_pairs))
                if len(uniq) == 1:
                    old_abs, new_abs = uniq[0]
                    try:
                        old_local = xform_cache.GetLocalTransformation(prim)
                        if isinstance(old_local, tuple):
                            old_local = old_local[0]
                        V, dedup_mode = _get_V_cached(old_abs, new_abs)
                        if dedup_mode in ("geom_only", "identity"):
                            old_internal = _get_internal_cached(old_abs)
                            canonical_internal = _get_internal_cached(new_abs)
                            new_local = (
                                canonical_internal.GetInverse()
                                * old_internal
                                * old_local
                            )
                        else:
                            new_local = V * old_local
                        _set_local_matrix(prim, new_local)
                        xform_compensated += 1
                        changes.append(
                            {
                                "prim": prim_path_str,
                                "kind": "xform_compensation",
                                "old_asset_abs": old_abs,
                                "canonical_asset_abs": new_abs,
                                "v_matrix_mode": v_matrix_mode,
                                "dedup_mode": dedup_mode,
                            }
                        )
                    except Exception as e:
                        changes.append(
                            {
                                "prim": prim_path_str,
                                "kind": "xform_compensation_error",
                                "old_asset_abs": old_abs,
                                "canonical_asset_abs": new_abs,
                                "error": str(e),
                            }
                        )
                else:
                    changes.append(
                        {
                            "prim": prim_path_str,
                            "kind": "xform_compensation_skipped_multi_ref",
                            "changed_pairs": [
                                {"old_abs": a, "new_abs": b} for a, b in uniq
                            ],
                        }
                    )

        if pls and payload_count:
            _set_payload_listop(prim, new_pls)
            payloads_changed += payload_count
            for old_abs, new_abs in payload_changed_pairs:
                changes.append(
                    {
                        "prim": prim_path_str,
                        "kind": "payloads",
                        "old_abs": old_abs,
                        "new_abs": new_abs,
                        "count": 1,
                    }
                )

        for attr in prim.GetAttributes():
            t = attr.GetTypeName()
            if t not in (Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.AssetArray):
                continue

            try:
                v = attr.Get()
            except Exception:
                continue

            if isinstance(v, Sdf.AssetPath):
                ap = v.path
                resolved = _resolve_ref_asset_path(base_dir, ap)
                if resolved in mapping_by_old:
                    new_abs = mapping_by_old[resolved]
                    new_ap = _format_new_asset_path(base_dir, ap, new_abs)
                    attr.Set(Sdf.AssetPath(new_ap))
                    asset_attrs_changed += 1
                    changes.append(
                        {
                            "prim": prim_path_str,
                            "kind": "asset_attr",
                            "attr": attr.GetName(),
                            "old_abs": resolved,
                            "new_abs": new_abs,
                        }
                    )
                continue

            if isinstance(v, (list, tuple)):
                did_any = False
                new_vals: List[Sdf.AssetPath] = []
                for item in v:
                    if not isinstance(item, Sdf.AssetPath):
                        new_vals.append(item)
                        continue
                    ap = item.path
                    resolved = _resolve_ref_asset_path(base_dir, ap)
                    if resolved in mapping_by_old:
                        new_abs = mapping_by_old[resolved]
                        new_ap = _format_new_asset_path(base_dir, ap, new_abs)
                        new_vals.append(Sdf.AssetPath(new_ap))
                        did_any = True
                        changes.append(
                            {
                                "prim": prim_path_str,
                                "kind": "asset_attr",
                                "attr": attr.GetName(),
                                "old_abs": resolved,
                                "new_abs": new_abs,
                            }
                        )
                    else:
                        new_vals.append(item)
                if did_any:
                    attr.Set(new_vals)
                    asset_attrs_changed += 1

    summary: Dict[str, Any] = {
        "layout_in": layout_usd,
        "layout_out": planned_out_usd or target_usd,
        "subset_root": subset_root,
        "dry_run": bool(dry_run),
        "apply_compensation": bool(apply_compensation),
        "set_instanceable": bool(set_instanceable),
        "elapsed_sec": max(0.0, time.time() - started_at),
        "asset_internal_matrices_computed": len(asset_internal_cache),
        "v_matrix_mode": v_matrix_mode,
        "v_matrices_computed": len(_v_matrix_cache),
        "bbox_gated": bool(bbox_gated),
        "bbox_policy": bbox_policy,
        "counts": {
            "refs_changed": refs_changed,
            "payloads_changed": payloads_changed,
            "asset_attrs_changed": asset_attrs_changed,
            "xform_compensated": xform_compensated,
            "instanceable_set": instanceable_set,
            "reject_records": reject_records,
            "change_records": len(changes),
        },
        "mapping_pairs": [
            {"old_abs": mp.old_abs, "canonical_abs": mp.canonical_abs}
            for mp in mapping_pairs
        ],
        "changes_preview": changes[: max_preview if max_preview > 0 else 0],
    }

    if report_out:
        report_out = _norm_abs(report_out)
        os.makedirs(os.path.dirname(report_out) or ".", exist_ok=True)
        with open(report_out, "w", encoding="utf-8") as f:
            json.dump(
                {"summary": summary, "changes": changes},
                f,
                indent=2,
                ensure_ascii=False,
            )

    if not dry_run:
        if set_instanceable and rewritten_prims:
            for p in sorted(rewritten_prims):
                prim = stage.GetPrimAtPath(Sdf.Path(p))
                if not prim or not prim.IsValid():
                    continue
                try:
                    prim.SetInstanceable(True)
                    instanceable_set += 1
                    changes.append(
                        {"prim": str(p), "kind": "set_instanceable", "value": True}
                    )
                except Exception as e:
                    changes.append(
                        {
                            "prim": str(p),
                            "kind": "set_instanceable_error",
                            "error": str(e),
                        }
                    )

            # Update counts post-apply
            summary["counts"]["instanceable_set"] = instanceable_set
            summary["counts"]["change_records"] = len(changes)

            if report_out:
                with open(report_out, "w", encoding="utf-8") as f:
                    json.dump(
                        {"summary": summary, "changes": changes},
                        f,
                        indent=2,
                        ensure_ascii=False,
                    )

        stage.GetRootLayer().Save()

    return summary


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Rewrite layout.usd asset references using a mapping, optionally applying Step 3B transform compensation.",
    )
    ap.add_argument("--layout-usd", required=True, help="Path to layout.usd")
    ap.add_argument(
        "--subset-root",
        default=None,
        help="Subset/package root that contains GRScenes_assets. If omitted, inferred from --layout-usd.",
    )
    ap.add_argument(
        "--mapping-json",
        required=True,
        help='Mapping JSON: either {"old":"canonical",...} or [{"old":...,"canonical":...}, ...]. Paths are subset-root-relative or absolute.',
    )
    ap.add_argument(
        "--out-usd",
        default=None,
        help="Optional output USD path. If omitted, edits the input in-place (use --dry-run first!).",
    )
    ap.add_argument(
        "--dry-run",
        action="store_true",
        help="Do not save; just report what would change.",
    )
    ap.add_argument(
        "--no-compensation",
        action="store_true",
        help="Only rewrite references; do NOT apply transform compensation.",
    )
    ap.add_argument(
        "--set-instanceable",
        action="store_true",
        help="After rewriting, set instanceable=true on prims whose references were rewritten (writes to output stage).",
    )
    ap.add_argument(
        "--report-out",
        default=None,
        help="Optional JSON report output path (summary + full change list).",
    )
    ap.add_argument(
        "--v-matrix-mode",
        choices=["none", "auto"],
        default="none",
        help="V matrix compensation mode: 'none' (legacy, V=Identity) or 'auto' (mode-dispatched V).",
    )
    ap.add_argument(
        "--mode-reports-dir",
        default=None,
        help="Directory containing geom_only/, topo_filesize/, shape_invariant/ dedup reports. Required when --v-matrix-mode=auto.",
    )
    ap.add_argument(
        "--bbox-gated",
        action="store_true",
        help="Enable fail-closed bbox-gated rewrite behavior.",
    )
    ap.add_argument(
        "--bbox-policy",
        choices=["bbox_primary_rmse_observe", "bbox_primary_rmse_harder"],
        default="bbox_primary_rmse_observe",
        help="BBox-gated policy label recorded in reports.",
    )
    ap.add_argument(
        "--preview",
        type=int,
        default=30,
        help="Max number of change records to print in stdout preview.",
    )

    args = ap.parse_args(argv)

    layout_usd = args.layout_usd
    subset_root = args.subset_root
    if subset_root is None:
        inferred = _find_subset_root_from_layout(layout_usd)
        if inferred is None:
            raise SystemExit(
                "Failed to infer --subset-root; please provide it explicitly"
            )
        subset_root = inferred

    mapping_pairs = _load_mapping(args.mapping_json, subset_root)

    summary = rewrite_layout(
        layout_usd=layout_usd,
        out_usd=args.out_usd,
        subset_root=subset_root,
        mapping_pairs=mapping_pairs,
        apply_compensation=(not args.no_compensation),
        set_instanceable=bool(args.set_instanceable),
        dry_run=bool(args.dry_run),
        report_out=args.report_out,
        max_preview=max(0, int(args.preview)),
        v_matrix_mode=args.v_matrix_mode,
        mode_reports_dir=args.mode_reports_dir,
        bbox_gated=bool(args.bbox_gated),
        bbox_policy=args.bbox_policy,
    )

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
