#!/usr/bin/env python3
"""Bulk C1 Step 6 (category-scale): promote layouts, pxr-scan for old assets, then soft-delete.

Why
- Per-group Step 6 doesn't scale to thousands of groups.
- This script runs Step 6 once per *category batch* (or any large mapping), while keeping
  the same safety gates: backup -> promote -> scan hit=0 -> soft delete -> scan hit=0.

Inputs
- mapping JSON: {old_asset_usd_rel: canonical_asset_usd_rel, ...}
  Typically produced by scripts/c1_build_bulk_mapping_from_dedup_report.py.

Important
- Scanning uses pxr.Usd (NOT byte substring matching), because .usdc may not contain
  the full authored asset path as a contiguous byte string.

Outputs
- promote report
- post-promote scan reports (layouts + full tree)
- soft-delete report
- post-soft-delete layout scan report
- optional ledger JSONL append
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple

try:
    from pxr import Sdf, Usd  # type: ignore

    _PXR_ERR: Optional[Exception] = None
except Exception as e:  # pragma: no cover
    Sdf = None  # type: ignore
    Usd = None  # type: ignore
    _PXR_ERR = e


def _stamp_now() -> str:
    return time.strftime("%Y%m%d_%H%M%S", time.localtime())


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
    # Find 'GRScenes_assets' in parts and extract uid (2 positions after)
    for i, part in enumerate(parts):
        if part == "GRScenes_assets" and i + 2 < len(parts):
            return parts[i + 2]
    # Fallback for relative report-style paths
    if len(parts) >= 6:
        return parts[3]
    return None


def _abs_from_usd_ref(base_dir: str, asset_path: str) -> str:
    s = (asset_path or "").replace("\\", "/").strip()
    if s.startswith("@") and s.endswith("@"):  # Sdf.AssetPath string form
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

        # Asset attributes (rare but safe)
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
        dirpath_str = str(dirpath)
        if any(x and x in dirpath_str for x in exclude_dir_contains):
            continue
        for fn in filenames:
            if not fn.endswith(".usd"):
                continue
            # Backup / rollback artifacts are expected to keep old references.
            # They are safe to skip for the delete gate, because rollback requires restoring
            # both the layout backup and the moved asset directories.
            if ".pre_" in fn:
                continue
            # Intermediate C1 outputs are review artifacts (the promoted files are the originals).
            # Scanning them would produce false positives when they still point at old UIDs.
            if ".c1_" in fn:
                continue
            out.append(Path(dirpath) / fn)
    out.sort()
    return out


def _derive_output_suffix(out_name: str) -> str:
    """Given a layout out name like 'layout.c1_xxx.usd', return suffix like '.c1_xxx'."""
    bn = Path(out_name).name
    if not bn.endswith(".usd"):
        raise ValueError(f"out-name must end with .usd: {out_name}")
    stem = bn[: -len(".usd")]
    if not stem.startswith("layout"):
        raise ValueError(f"out-name must start with 'layout': {out_name}")
    suffix = stem[len("layout") :]
    if not suffix:
        raise ValueError(f"out-name must include a suffix after 'layout': {out_name}")
    return suffix


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


def main() -> int:
    ap = argparse.ArgumentParser(description="Bulk C1 Step6 (category-scale): promote layouts, scan, soft delete")
    ap.add_argument("--dataset-root", required=True)
    ap.add_argument("--bak-root", required=True)
    ap.add_argument("--mapping-json", required=True)
    ap.add_argument("--category", required=True)
    ap.add_argument("--group-label", required=True)
    ap.add_argument("--out-name", required=True, help="The generated layout filename to promote (next to layout.usd)")
    ap.add_argument(
        "--promote-scene-files",
        default="layout.usd,start_result_interaction.usd,start_result_navigation.usd",
        help="Comma-separated scene USD filenames to promote alongside layout.usd (if outputs exist)",
    )
    ap.add_argument("--layout-root", default=None)
    ap.add_argument("--report-dir", required=True)
    ap.add_argument("--ledger-jsonl", default=None)
    ap.add_argument("--audit-verdict-json", default=None, help="Required authoritative audit verdict for bbox-gated Step6.")
    ap.add_argument("--bbox-gated", action="store_true", help="Require authoritative audit verdict before promote/soft-delete.")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--progress-every", type=int, default=1000)
    ap.add_argument(
        "--exclude-dir-contains",
        action="append",
        default=["/_dedup_assets/"],
        help="Skip directories containing this substring (repeatable)",
    )
    args = ap.parse_args()

    if _PXR_ERR is not None:
        raise SystemExit(f"pxr.Usd not available: {_PXR_ERR}")

    dataset_root = Path(args.dataset_root)
    dataset_name = dataset_root.name
    bak_root = Path(args.bak_root)
    layout_root = Path(args.layout_root) if args.layout_root else (dataset_root / "GRScenes100")
    report_dir = Path(args.report_dir)
    report_dir.mkdir(parents=True, exist_ok=True)

    mapping_payload: Dict[str, str] = json.loads(Path(args.mapping_json).read_text())
    mapping: Dict[str, str] = {}
    for old_key, canonical_key in mapping_payload.items():
        old_rel = _normalize_mapping_key(old_key, dataset_name)
        canonical_rel = _normalize_mapping_key(canonical_key, dataset_name)
        if not old_rel or not canonical_rel:
            continue
        mapping[old_rel] = canonical_rel
    if not mapping:
        raise SystemExit("Empty mapping JSON")

    # Safety: exclude any "old" asset that is also a canonical target.
    # This prevents false scan hits from transitive canonical conflicts
    # (e.g., same asset appearing with different path forms in the report).
    canonical_set: Set[str] = set(mapping.values())
    conflicting = set(mapping.keys()) & canonical_set
    if conflicting:
        print(
            f"WARNING: {len(conflicting)} old assets are also canonical targets — "
            f"excluding from old set to avoid false scan hits",
            flush=True,
        )
        for k in conflicting:
            del mapping[k]

    old_asset_usd_rel_set: Set[str] = set(mapping.keys())
    old_uids = sorted({uid for k in mapping.keys() if (uid := _parse_uid_from_report_style_asset_usd(k))})
    canonical_uids = sorted({uid for v in mapping.values() if (uid := _parse_uid_from_report_style_asset_usd(v))})

    stamp = _stamp_now()

    promote_report_path = report_dir / "promote_to_layout_usd_report.json"
    post_promote_layout_scan_path = report_dir / "post_promote_layout_scan_pxr.json"
    post_promote_full_scan_path = report_dir / "post_promote_full_usd_scan_excluding_backups_pxr.json"
    soft_delete_report_path = report_dir / "soft_delete_old_assets_report.json"
    post_soft_delete_layout_scan_path = report_dir / "post_soft_delete_layout_scan_pxr.json"
    step6_gate_decision_path = report_dir / "step6_gate_decision.json"

    progress_json = report_dir / "progress.json"
    progress_jsonl = report_dir / "progress.jsonl"

    audit_verdict = None
    if args.bbox_gated or args.audit_verdict_json:
        if not args.audit_verdict_json:
            raise SystemExit("bbox-gated Step6 requires --audit-verdict-json")
        audit_verdict = json.loads(Path(args.audit_verdict_json).read_text(encoding="utf-8"))
        gate_decision = {
            "bbox_gated": True,
            "audit_verdict_json": str(Path(args.audit_verdict_json)),
            "audit_passed": bool(audit_verdict.get("passed")),
            "blocking_reason_counts": audit_verdict.get("blocking_reason_counts", {}),
            "ref_changed_fail_count": audit_verdict.get("ref_changed_fail_count", 0),
            "scenes_error": audit_verdict.get("scenes_error", 0),
            "total_no_mesh": audit_verdict.get("total_no_mesh", 0),
        }
        _write_json(step6_gate_decision_path, gate_decision)
        if not gate_decision["audit_passed"]:
            raise SystemExit(
                f"ABORT: authoritative audit failed; see {args.audit_verdict_json} and {step6_gate_decision_path}"
            )
    elif args.dry_run:
        _write_json(step6_gate_decision_path, {"bbox_gated": False, "audit_passed": None})

    # 1) Promote scene USDs (layout + derived outputs)
    layouts = sorted(layout_root.glob("**/layout.usd"))
    scene_files = [s.strip() for s in str(args.promote_scene_files).split(",") if s.strip()]
    if "layout.usd" not in scene_files:
        scene_files = ["layout.usd"] + scene_files

    layout_out_suffix = _derive_output_suffix(args.out_name)

    promote_items: List[Dict[str, object]] = []

    for lp in layouts:
        scene_dir = lp.parent
        for scene_fn in scene_files:
            src = scene_dir / scene_fn
            if not src.exists():
                continue

            if scene_fn == "layout.usd":
                outp = scene_dir / args.out_name
            else:
                base = scene_fn[: -len(".usd")] if scene_fn.endswith(".usd") else scene_fn
                outp = scene_dir / f"{base}{layout_out_suffix}.usd"

            if not outp.exists():
                continue

            base = src.name[: -len(".usd")] if src.name.endswith(".usd") else src.name
            backup = scene_dir / f"{base}.pre_{args.group_label}.{stamp}.usd"
            promote_items.append({"file": str(src), "out": str(outp), "backup": str(backup)})

    promote_report = {
        "group_label": args.group_label,
        "category": args.category,
        "stamp": stamp,
        "layouts_scanned": len(layouts),
        "layouts_to_promote": len([x for x in promote_items if str(x.get('file','')).endswith('/layout.usd')]),
        "files_to_promote": len(promote_items),
        "promote_scene_files": scene_files,
        "items": promote_items,
    }

    if args.dry_run:
        _write_json(promote_report_path, promote_report)
        print(f"DRYRUN promote items={len(promote_items)} wrote {promote_report_path}")
        return 0

    for it in promote_items:
        lp = Path(str(it["file"]))
        outp = Path(str(it["out"]))
        backup = Path(str(it["backup"]))
        shutil.copy2(lp, backup)
        shutil.copy2(outp, lp)

    _write_json(promote_report_path, promote_report)
    print(f"Promoted files={len(promote_items)} wrote {promote_report_path}")

    # 2) Post-promote scans (pxr)
    # 2a) layout-only scan (kept for backwards compatibility)
    layout_hits: List[Dict[str, object]] = []
    for lp in layouts:
        matched = _scan_stage_for_old_assets(lp, old_asset_usd_rel_set, dataset_name)
        if matched:
            layout_hits.append({"layout": str(lp), "hit_assets": matched})
    post_promote_layout_scan = {"scanned_layouts": len(layouts), "hit_layouts": len(layout_hits), "results": layout_hits}
    _write_json(post_promote_layout_scan_path, post_promote_layout_scan)

    # 2a2) promoted scene files scan (layout + derived)
    scene_hits: List[Dict[str, object]] = []
    for it in promote_items:
        fp = Path(str(it["file"]))
        matched = _scan_stage_for_old_assets(fp, old_asset_usd_rel_set, dataset_name)
        if matched:
            scene_hits.append({"file": str(fp), "hit_assets": matched})
    _write_json(report_dir / "post_promote_scene_files_scan_pxr.json", {"scanned_files": len(promote_items), "hit_files": len(scene_hits), "results": scene_hits})

    # 2b) full tree scan excluding backups
    full_scan = _scan_tree_pxr(
        dataset_root,
        old_asset_usd_rel_set=old_asset_usd_rel_set,
        dataset_name=dataset_name,
        exclude_dir_contains=list(args.exclude_dir_contains),
        progress_every=int(args.progress_every),
        progress_json=progress_json,
        progress_jsonl=progress_jsonl,
    )
    _write_json(post_promote_full_scan_path, full_scan)

    if int(full_scan.get("hit_files", 0)) != 0:
        raise SystemExit(
            f"ABORT: post-promote full scan hit_files={full_scan.get('hit_files')}; see {post_promote_full_scan_path}"
        )

    # 3) Soft delete old assets
    dedup_root = bak_root / "_dedup_assets" / f"{args.group_label}_{stamp}" / "GRScenes_assets" / args.category
    moved: List[Dict[str, object]] = []
    missing: List[str] = []

    for uid in old_uids:
        src_dir = dataset_root / "GRScenes_assets" / args.category / uid
        if not src_dir.exists():
            missing.append(str(src_dir))
            continue
        dst_dir = dedup_root / uid
        dst_dir.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(str(src_dir), str(dst_dir))
        moved.append({"src": str(src_dir), "dst": str(dst_dir)})

    soft_delete_report = {
        "group_label": args.group_label,
        "category": args.category,
        "stamp": stamp,
        "old_uids": old_uids,
        "canonical_uids": canonical_uids,
        "moved": moved,
        "missing": missing,
    }
    _write_json(soft_delete_report_path, soft_delete_report)

    # 4) Post soft-delete layout scan
    layout_hits2: List[Dict[str, object]] = []
    for lp in layouts:
        matched = _scan_stage_for_old_assets(lp, old_asset_usd_rel_set, dataset_name)
        if matched:
            layout_hits2.append({"layout": str(lp), "hit_assets": matched})
    post_soft = {"scanned_layouts": len(layouts), "hit_layouts": len(layout_hits2), "results": layout_hits2}
    _write_json(post_soft_delete_layout_scan_path, post_soft)

    if int(post_soft.get("hit_layouts", 0)) != 0:
        raise SystemExit(
            f"ABORT: post-soft-delete layout scan hit_layouts={post_soft.get('hit_layouts')}; see {post_soft_delete_layout_scan_path}"
        )

    # 5) Ledger
    if args.ledger_jsonl:
        entry = {
            "stamp": stamp,
            "group_label": args.group_label,
            "category": args.category,
            "mapping_json": args.mapping_json,
            "old_uids_count": len(old_uids),
            "canonical_uids_count": len(canonical_uids),
            "reports": {
                "promote": str(promote_report_path),
                "post_promote_layout_scan": str(post_promote_layout_scan_path),
                "post_promote_full_scan": str(post_promote_full_scan_path),
                "soft_delete": str(soft_delete_report_path),
                "post_soft_delete_layout_scan": str(post_soft_delete_layout_scan_path),
                "step6_gate_decision": str(step6_gate_decision_path),
            },
        }
        _append_jsonl(Path(args.ledger_jsonl), entry)

    print("DONE")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
