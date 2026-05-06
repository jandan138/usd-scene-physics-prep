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
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Set

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from scan_utils import (
    _PXR_ERR,
    _append_jsonl,
    _emit_progress,
    _iter_usd_files,
    _normalize_mapping_key,
    _parse_uid_from_report_style_asset_usd,
    _scan_stage_for_old_assets,
    _scan_tree_pxr,
    _write_json,
    build_old_asset_path_set,
)


def _stamp_now() -> str:
    return time.strftime("%Y%m%d_%H%M%S", time.localtime())


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
