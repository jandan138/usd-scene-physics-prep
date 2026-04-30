"""Shared scan utility functions for C1 pipeline (pure-Python + pxr).

Functions in this module serve both the existing serial autorun path
(c1_bulk_step6_category_promote_scan_soft_delete.py) and the parallel
pipeline's Phase 2 mega-scan (c1_phase2_merge_scan_delete.py).

Pure-Python helpers:
  _write_json, _append_jsonl — file write utilities
  _parse_uid_from_report_style_asset_usd — UID extractor
  _abs_from_usd_ref, _abs_to_subset_rel, _normalize_mapping_key — path helpers
  _emit_progress — progress emitter
  _iter_usd_files — USD file enumerator with exclusion rules
  build_old_asset_path_set — build old asset path set from a mapping JSON
  build_combined_old_asset_path_set — build combined set from multiple mapping JSONs

pxr-dependent scanning helpers:
  _scan_stage_for_old_assets — single-stage asset reference scanner
  _scan_tree_pxr — full-tree scanner
"""

from __future__ import annotations

import json
import os
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Set

try:
    from pxr import Sdf, Usd  # type: ignore

    _PXR_ERR: Optional[Exception] = None
except Exception as e:  # pragma: no cover
    Sdf = None  # type: ignore
    Usd = None  # type: ignore
    _PXR_ERR = e


def _write_json(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def _append_jsonl(path: Path, payload: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "a", encoding="utf-8") as f:
        f.write(json.dumps(payload, ensure_ascii=False) + "\n")


def _parse_uid_from_report_style_asset_usd(p: str) -> Optional[str]:
    """Extract UID from asset USD path.

    Handles both relative and absolute paths by locating GRScenes_assets/ marker:
      .../GRScenes_assets/<cat>/<uid>/usd/<uid>.usd  ->  <uid>
    """
    try:
        parts = Path(p).parts
    except Exception:
        return None
    for i, part in enumerate(parts):
        if part == "GRScenes_assets" and i + 2 < len(parts):
            return parts[i + 2]
    if len(parts) >= 6:
        return parts[3]
    return None


def _abs_from_usd_ref(base_dir: str, asset_path: str) -> str:
    s = (asset_path or "").replace("\\", "/").strip()
    if s.startswith("@") and s.endswith("@"):
        s = s[1:-1]
    if not s:
        return ""
    if os.path.isabs(s):
        return os.path.abspath(s)
    return os.path.abspath(os.path.join(base_dir, s))


def _abs_to_subset_rel(abs_path: str, dataset_name: str) -> Optional[str]:
    p = (abs_path or "").replace("\\", "/")
    marker = f"/{dataset_name}/GRScenes_assets/"
    idx = p.find(marker)
    if idx >= 0:
        return p[idx + 1 + len(dataset_name) + 1 :]
    marker2 = "/GRScenes_assets/"
    idx2 = p.find(marker2)
    if idx2 >= 0:
        return p[idx2 + 1 :]
    return None


def _normalize_mapping_key(p: str, dataset_name: str) -> Optional[str]:
    s = (p or "").replace("\\", "/").strip()
    marker = f"{dataset_name}/GRScenes_assets/"
    if marker in s:
        return s[s.find(marker) + len(dataset_name) + 1 :].lstrip("/")
    marker2 = "GRScenes_assets/"
    if marker2 in s:
        return s[s.find(marker2) :].lstrip("/")
    return None


@dataclass
class ScanHit:
    file: str
    hit_assets: List[str]


def _scan_stage_for_old_assets(stage_path: Path, old_asset_usd_rel_set: Set[str], dataset_name: str) -> List[str]:
    """Return matched report-style asset USD paths referenced/payloaded/authored in this stage.

    Important: Match by full report-style asset USD path (mapping key), NOT just UID.
    UIDs are not guaranteed globally unique across categories, so UID-only matching can
    produce false positives and abort safe deletes.
    """
    if _PXR_ERR is not None:
        raise RuntimeError(f"pxr.Usd not available: {_PXR_ERR}")

    base_dir = str(stage_path.parent)
    stage = Usd.Stage.Open(str(stage_path), load=Usd.Stage.LoadNone)
    if stage is None:
        return []

    matched: Set[str] = set()

    for prim in stage.Traverse():
        if not prim or not prim.IsValid():
            continue

        if prim.HasAuthoredReferences():
            refs = prim.GetMetadata("references")
            if refs:
                try:
                    items = list(refs.GetAddedOrExplicitItems())
                except Exception:
                    items = []
                for r in items:
                    ap = str(getattr(r, "assetPath", "") or "")
                    abs_p = _abs_from_usd_ref(base_dir, ap)
                    rel = _abs_to_subset_rel(abs_p, dataset_name)
                    if not rel:
                        continue
                    if rel in old_asset_usd_rel_set:
                        matched.add(rel)

        if prim.HasAuthoredPayloads():
            pls = prim.GetMetadata("payloads")
            if pls:
                try:
                    items = list(pls.GetAddedOrExplicitItems())
                except Exception:
                    items = []
                for r in items:
                    ap = str(getattr(r, "assetPath", "") or "")
                    abs_p = _abs_from_usd_ref(base_dir, ap)
                    rel = _abs_to_subset_rel(abs_p, dataset_name)
                    if not rel:
                        continue
                    if rel in old_asset_usd_rel_set:
                        matched.add(rel)

        for attr in prim.GetAttributes():
            tn = attr.GetTypeName()
            if tn not in (Sdf.ValueTypeNames.Asset, Sdf.ValueTypeNames.AssetArray):
                continue
            try:
                v = attr.Get()
            except Exception:
                continue

            def handle_asset_path(asset_path: str) -> None:
                abs_p = _abs_from_usd_ref(base_dir, asset_path)
                rel = _abs_to_subset_rel(abs_p, dataset_name)
                if not rel:
                    return
                if rel in old_asset_usd_rel_set:
                    matched.add(rel)

            if isinstance(v, Sdf.AssetPath):
                handle_asset_path(v.path)
            elif isinstance(v, (list, tuple)):
                for vv in v:
                    if isinstance(vv, Sdf.AssetPath):
                        handle_asset_path(vv.path)

    return sorted(matched)


def _emit_progress(
    *,
    started_at: float,
    processed: int,
    total: int,
    hits: int,
    phase: str,
    progress_json: Optional[Path],
    progress_jsonl: Optional[Path],
    current_file: Optional[str] = None,
) -> None:
    elapsed = max(1e-9, time.time() - started_at)
    rate = processed / elapsed
    eta_sec: Optional[float]
    if total <= 0 or rate <= 0:
        eta_sec = None
    else:
        eta_sec = max(0.0, (total - processed) / rate)

    payload = {
        "phase": phase,
        "processed": processed,
        "total": total,
        "hits": hits,
        "current_file": current_file,
        "elapsed_sec": elapsed,
        "rate_files_per_sec": rate,
        "eta_sec": eta_sec,
        "timestamp_unix": int(time.time()),
    }

    if progress_jsonl:
        _append_jsonl(progress_jsonl, payload)
    if progress_json:
        _write_json(progress_json, payload)

    eta_msg = "?" if eta_sec is None else f"{eta_sec/60.0:.1f}m"
    cur = "" if not current_file else f" | cur={Path(current_file).name}"
    print(f"  {phase} {processed}/{total} | hits={hits} | rate={rate:.2f}/s | eta={eta_msg}{cur}", flush=True)


def _iter_usd_files(root: Path, *, exclude_dir_contains: Sequence[str]) -> List[Path]:
    out: List[Path] = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirpath_str = str(dirpath) + "/"
        if any(x and x in dirpath_str for x in exclude_dir_contains):
            continue
        for fn in filenames:
            if not fn.endswith(".usd"):
                continue
            if ".pre_" in fn:
                continue
            if ".c1_" in fn:
                continue
            if ".parallel_" in fn:
                continue
            if ".baseline." in fn:
                continue
            out.append(Path(dirpath) / fn)
    out.sort()
    return out


def _scan_tree_pxr(
    root: Path,
    *,
    old_asset_usd_rel_set: Set[str],
    dataset_name: str,
    exclude_dir_contains: Sequence[str],
    progress_every: int,
    progress_json: Optional[Path],
    progress_jsonl: Optional[Path],
) -> Dict[str, object]:
    started_at = time.time()
    print(f"Listing USD files under: {root.resolve()}", flush=True)
    _emit_progress(
        started_at=started_at,
        processed=0,
        total=0,
        hits=0,
        phase="discover",
        progress_json=progress_json,
        progress_jsonl=progress_jsonl,
    )

    files = _iter_usd_files(root, exclude_dir_contains=exclude_dir_contains)
    total = len(files)

    print(f"Scanning {total} USD files via pxr...", flush=True)
    _emit_progress(
        started_at=started_at,
        processed=0,
        total=total,
        hits=0,
        phase="scan",
        progress_json=progress_json,
        progress_jsonl=progress_jsonl,
    )

    hits: List[Dict[str, object]] = []

    for idx, p in enumerate(files, start=1):
        if idx == 1 or (progress_every and progress_every > 0 and idx % progress_every == 0):
            _emit_progress(
                started_at=started_at,
                processed=idx,
                total=total,
                hits=len(hits),
                phase="scan",
                progress_json=progress_json,
                progress_jsonl=progress_jsonl,
                current_file=str(p),
            )

        try:
            matched = _scan_stage_for_old_assets(p, old_asset_usd_rel_set, dataset_name)
        except Exception as e:
            hits.append({"file": str(p), "error": str(e)})
            continue

        if matched:
            hits.append({"file": str(p), "hit_assets": matched})

    _emit_progress(
        started_at=started_at,
        processed=total,
        total=total,
        hits=len(hits),
        phase="done",
        progress_json=progress_json,
        progress_jsonl=progress_jsonl,
    )

    return {"scanned_files": total, "hit_files": len(hits), "results": hits}


def build_old_asset_path_set(mapping_json_path: str, dataset_name: str) -> Set[str]:
    """Build set of old asset relative paths from a mapping JSON."""
    with open(mapping_json_path, encoding="utf-8") as f:
        mapping = json.load(f)
    result = set()
    for old_key in mapping.keys():
        norm = _normalize_mapping_key(old_key, dataset_name)
        if norm:
            result.add(norm)
    return result


def build_combined_old_asset_path_set(
    mapping_paths: List[str], dataset_name: str
) -> Set[str]:
    """Build combined set of old asset paths from multiple mapping JSONs."""
    result = set()
    for mp in mapping_paths:
        result |= build_old_asset_path_set(mp, dataset_name)
    return result
