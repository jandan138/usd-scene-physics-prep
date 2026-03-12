#!/usr/bin/env python3
"""Rewrite asset references in a layout.usd and optionally apply transform compensation.

This script supports the workflow described in:
- docs/operations/asset_dedup_c1_scene_instancing_runbook.md (Step 3B)

Core idea:
- Replace references/payloads/asset-path attributes that point to an "old" asset USD with a "canonical" asset USD.
- If assets carry different internal alignment transforms, adjust the *layout prim* local transform so that the
  final world placement stays unchanged.

Mathematically (row-vector convention, p' = p * M):
- p_world = p_mesh * M_assetInternal * M_layout * M_parent_world
- After swapping old asset ref to canonical, world position must be unchanged:
  M_oldInternal * M_layout_old = M_canonicalInternal * M_layout_new
  => M_layout_new = inverse(M_canonicalInternal) * M_oldInternal * M_layout_old  (left-multiply)

Important limitations (intentional):
- We treat "asset internal" as the local transform on the Instance child of the asset stage's defaultPrim
  (/Root/Instance), where normalize_asset_transforms.py writes the alignment transform.
  Falls back to defaultPrim if no Instance child exists.
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
        raise ValueError("Unsupported mapping JSON format. Use dict {old: canonical} or list of {old, canonical}.")

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
    """Return local xform of asset's /Root/Instance as 'asset internal' matrix.

    normalize_asset_transforms.py writes the alignment transform (recenter +
    Y-up->Z-up rotation) on the Instance child of the defaultPrim.  We must
    read that prim -- NOT the defaultPrim itself, whose transform is typically
    identity.  Falls back to defaultPrim when no Instance child exists.
    """
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
    # This is where normalize_asset_transforms.py writes the alignment transform.
    instance_prim = None
    for child in default_prim.GetChildren():
        if child.GetName() == "Instance":
            instance_prim = child
            break

    # Use Instance prim if found, otherwise fall back to default prim
    target_prim = instance_prim if (instance_prim and instance_prim.IsValid()) else default_prim

    xf = UsdGeom.Xformable(target_prim)
    if not xf:
        return Gf.Matrix4d(1.0)

    res = xf.GetLocalTransformation(Usd.TimeCode.Default())
    if isinstance(res, tuple):
        return res[0]
    return res


def _ensure_matrix_xform(prim) -> None:
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return

    # Author a single xformOp:transform.
    prim.CreateAttribute("xformOpOrder", Sdf.ValueTypeNames.TokenArray, custom=False).Set(["xformOp:transform"])
    if not prim.HasAttribute("xformOp:transform"):
        prim.CreateAttribute("xformOp:transform", Sdf.ValueTypeNames.Matrix4d, custom=False)


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


def _rewrite_reference_listop(refs, base_dir: str, old_abs: str, new_abs: str) -> Tuple[Any, int]:
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


def _rewrite_payload_listop(payloads, base_dir: str, old_abs: str, new_abs: str) -> Tuple[Any, int]:
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
) -> Dict[str, Any]:
    if _PXR_IMPORT_ERROR is not None:
        raise RuntimeError(f"pxr.Usd not available: {_PXR_IMPORT_ERROR}")

    layout_usd = _norm_abs(layout_usd)

    started_at = time.time()

    # Copy-on-write unless writing in-place.
    # If dry-run, do NOT copy/write; just report the intended output path.
    planned_out_usd: Optional[str] = None
    target_usd = layout_usd
    if out_usd:
        out_usd = _norm_abs(out_usd)
        planned_out_usd = out_usd
        if not dry_run:
            os.makedirs(os.path.dirname(out_usd), exist_ok=True)
            # copy file only (layers referenced by relative paths are untouched)
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

    rewritten_prims: set[str] = set()

    mapping_by_old = {mp.old_abs: mp.canonical_abs for mp in mapping_pairs}

    for prim in stage.Traverse():
        if not prim or not prim.IsValid():
            continue

        prim_path_str = str(prim.GetPath())

        # 1) References
        if prim.HasAuthoredReferences():
            refs = prim.GetMetadata("references")
            if refs:
                new_refs, n, changed_pairs = _rewrite_reference_listop_with_mapping(refs, base_dir, mapping_by_old)
                if n:
                    prim.SetMetadata("references", new_refs)
                    refs_changed += n
                    rewritten_prims.add(prim_path_str)
                    for old_abs, new_abs in changed_pairs:
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
                        uniq = sorted(set(changed_pairs))
                        if len(uniq) == 1:
                            old_abs, new_abs = uniq[0]
                            # Compensate local transform so world placement is unchanged.
                            # Derivation (row-vector, p' = p * M):
                            #   M_old_inst * M_old_local = M_canon_inst * M_new_local
                            #   => M_new_local = M_canon_inst^{-1} * M_old_inst * M_old_local
                            try:
                                old_internal = _get_internal_cached(old_abs)
                                canonical_internal = _get_internal_cached(new_abs)
                                old_local = xform_cache.GetLocalTransformation(prim)
                                if isinstance(old_local, tuple):
                                    old_local = old_local[0]
                                new_local = canonical_internal.GetInverse() * old_internal * old_local
                                _set_local_matrix(prim, new_local)
                                xform_compensated += 1
                                changes.append(
                                    {
                                        "prim": prim_path_str,
                                        "kind": "xform_compensation",
                                        "old_asset_abs": old_abs,
                                        "canonical_asset_abs": new_abs,
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
                            # Safety: a prim with multiple different reference rewrites cannot be unambiguously compensated.
                            changes.append(
                                {
                                    "prim": prim_path_str,
                                    "kind": "xform_compensation_skipped_multi_ref",
                                    "changed_pairs": [{"old_abs": a, "new_abs": b} for a, b in uniq],
                                }
                            )

        # 2) Payloads
        if prim.HasAuthoredPayloads():
            pls = prim.GetMetadata("payloads")
            if pls:
                new_pls, n, changed_pairs = _rewrite_payload_listop_with_mapping(pls, base_dir, mapping_by_old)
                if n:
                    prim.SetMetadata("payloads", new_pls)
                    payloads_changed += n
                    for old_abs, new_abs in changed_pairs:
                        changes.append(
                            {
                                "prim": prim_path_str,
                                "kind": "payloads",
                                "old_abs": old_abs,
                                "new_abs": new_abs,
                                "count": 1,
                            }
                        )

        # 3) Asset-valued attributes (fallback)
        for attr in prim.GetAttributes():
            t = attr.GetTypeName()
            if t not in (Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.AssetArray):
                continue

            try:
                v = attr.Get()
            except Exception:
                continue

            # scalar asset path
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

            # array asset paths
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
        "counts": {
            "refs_changed": refs_changed,
            "payloads_changed": payloads_changed,
            "asset_attrs_changed": asset_attrs_changed,
            "xform_compensated": xform_compensated,
            "instanceable_set": instanceable_set,
            "change_records": len(changes),
        },
        "mapping_pairs": [{"old_abs": mp.old_abs, "canonical_abs": mp.canonical_abs} for mp in mapping_pairs],
        "changes_preview": changes[: max_preview if max_preview > 0 else 0],
    }

    if report_out:
        report_out = _norm_abs(report_out)
        os.makedirs(os.path.dirname(report_out) or ".", exist_ok=True)
        with open(report_out, "w", encoding="utf-8") as f:
            json.dump({"summary": summary, "changes": changes}, f, indent=2, ensure_ascii=False)

    if not dry_run:
        if set_instanceable and rewritten_prims:
            for p in sorted(rewritten_prims):
                prim = stage.GetPrimAtPath(Sdf.Path(p))
                if not prim or not prim.IsValid():
                    continue
                try:
                    prim.SetInstanceable(True)
                    instanceable_set += 1
                    changes.append({"prim": str(p), "kind": "set_instanceable", "value": True})
                except Exception as e:
                    changes.append({"prim": str(p), "kind": "set_instanceable_error", "error": str(e)})

            # Update counts post-apply
            summary["counts"]["instanceable_set"] = instanceable_set
            summary["counts"]["change_records"] = len(changes)

            if report_out:
                with open(report_out, "w", encoding="utf-8") as f:
                    json.dump({"summary": summary, "changes": changes}, f, indent=2, ensure_ascii=False)

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
            raise SystemExit("Failed to infer --subset-root; please provide it explicitly")
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
    )

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
