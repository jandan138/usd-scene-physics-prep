#!/usr/bin/env python3
"""C1 Step 6 helper: promote layouts, scan references with progress, then soft-delete old assets.

This script packages what we've been doing manually into a repeatable, auditable Step 6.

It creates an evidence chain (reports) and a machine-readable ledger entry so you can:
- see progress during long scans
- avoid re-processing the same dedup group next time

Safety principles
- Always backup layout.usd before promoting.
- Never delete assets until post-promote scans confirm hit=0.
- Soft delete = move directories into *_bak for easy rollback.

Expected directory conventions
- layout outputs: layout.c1_<group_label>_dedup_v1.usd next to layout.usd
- assets: GRScenes-test1/GRScenes_assets/<category>/<uid>/usd/<uid>.usd

"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import time
from pathlib import Path
from typing import Dict, List, Optional, Sequence


def _stamp_now() -> str:
    return time.strftime("%Y%m%d_%H%M%S", time.localtime())


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
        progress_jsonl.parent.mkdir(parents=True, exist_ok=True)
        with open(progress_jsonl, "a", encoding="utf-8") as f:
            f.write(json.dumps(payload, ensure_ascii=False) + "\n")
    if progress_json:
        progress_json.parent.mkdir(parents=True, exist_ok=True)
        with open(progress_json, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)

    eta_msg = "?" if eta_sec is None else f"{eta_sec/60.0:.1f}m"
    cur = "" if not current_file else f" | cur={Path(current_file).name}"
    print(
        f"  {phase} {processed}/{total} | hits={hits} | rate={rate:.2f}/s | eta={eta_msg}{cur}",
        flush=True,
    )


def _patterns_from_mapping_json(mapping_json: Path, *, category: str) -> List[str]:
    mapping: Dict[str, str] = json.loads(mapping_json.read_text())
    patterns: List[str] = []
    for old_asset_usd in mapping.keys():
        p = Path(old_asset_usd)
        if len(p.parts) < 6:
            continue
        uid = p.parts[3]
        patterns.append(f"GRScenes_assets/{category}/{uid}/usd/{uid}.usd")
    return sorted(set(patterns))


def _uids_from_mapping_json(mapping_json: Path) -> Dict[str, object]:
    mapping: Dict[str, str] = json.loads(mapping_json.read_text())
    old_uids = sorted({Path(k).parts[3] for k in mapping.keys() if len(Path(k).parts) >= 6})
    if not mapping:
        raise SystemExit("Empty mapping JSON")
    canon = Path(next(iter(mapping.values())))
    if len(canon.parts) < 6:
        raise SystemExit("Cannot parse canonical asset uid from mapping values")
    canon_uid = canon.parts[3]
    if not all(len(Path(v).parts) >= 6 and Path(v).parts[3] == canon_uid for v in mapping.values()):
        raise SystemExit("Mapping values are not all the same canonical asset")
    return {
        "old_uids": old_uids,
        "canonical_uid": canon_uid,
        "canonical_asset_usd": next(iter(mapping.values())),
        "old_count": len(old_uids),
    }


def _scan_layouts(layout_root: Path, *, patterns: Sequence[str]) -> Dict[str, object]:
    layouts = sorted(layout_root.glob("**/layout.usd"))
    results: List[Dict[str, object]] = []
    for lp in layouts:
        try:
            data = lp.read_bytes()
        except Exception as e:
            results.append({"layout": str(lp), "error": str(e)})
            continue
        hits = [pat for pat in patterns if pat.encode("utf-8") in data]
        if hits:
            results.append({"layout": str(lp), "hit_paths": hits})
    return {
        "scanned_layouts": len(layouts),
        "hit_layouts": len(results),
        "results": results,
    }


def _scan_full_usd_tree(
    root: Path,
    *,
    patterns: Sequence[str],
    exclude_dir_contains: Sequence[str],
    exclude_name_contains: Sequence[str],
    progress_every: int,
    progress_json: Optional[Path],
    progress_jsonl: Optional[Path],
) -> Dict[str, object]:
    started_at = time.time()

    def iter_files() -> List[Path]:
        out: List[Path] = []
        for dirpath, dirnames, filenames in os.walk(root):
            dirpath_str = str(dirpath)
            if any(x and x in dirpath_str for x in exclude_dir_contains):
                continue
            for fn in filenames:
                if not fn.endswith(".usd"):
                    continue
                if any(x and x in fn for x in exclude_name_contains):
                    continue
                out.append(Path(dirpath) / fn)
        out.sort()
        return out

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

    files = iter_files()
    total = len(files)

    print(f"Scanning {total} USD files for old-asset paths...", flush=True)
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
        if idx == 1:
            _emit_progress(
                started_at=started_at,
                processed=1,
                total=total,
                hits=len(hits),
                phase="scan",
                progress_json=progress_json,
                progress_jsonl=progress_jsonl,
                current_file=str(p),
            )

        if progress_every and progress_every > 0 and idx % progress_every == 0:
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
            data = p.read_bytes()
        except Exception as e:
            hits.append({"file": str(p), "error": str(e)})
            continue

        matched = [pat for pat in patterns if pat.encode("utf-8") in data]
        if matched:
            hits.append({"file": str(p), "hit_paths": matched})

    _emit_progress(
        started_at=started_at,
        processed=total,
        total=total,
        hits=len(hits),
        phase="done",
        progress_json=progress_json,
        progress_jsonl=progress_jsonl,
    )

    return {
        "scanned_files": total,
        "hit_files": len(hits),
        "results": hits,
    }


def _append_ledger(ledger_jsonl: Path, entry: Dict[str, object]) -> None:
    ledger_jsonl.parent.mkdir(parents=True, exist_ok=True)
    with open(ledger_jsonl, "a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description="C1 Step6: promote layouts, scan, soft delete old assets")
    parser.add_argument("--dataset-root", required=True, help="Dataset root (e.g. GRScenes-test1)")
    parser.add_argument("--bak-root", required=True, help="Backup root (e.g. GRScenes-test1_bak)")

    parser.add_argument(
        "--layout-root",
        default=None,
        help="Layout root to search for layout.usd (default: <dataset-root>/GRScenes100)",
    )

    parser.add_argument("--group-label", required=True, help="Group label for naming backups/reports (e.g. sigb526)")
    parser.add_argument("--category", default="plant", help="Asset category (default: plant)")
    parser.add_argument("--mapping-json", required=True, help="Mapping JSON old_asset_usd -> canonical_asset_usd")

    parser.add_argument(
        "--layout-out-name",
        default=None,
        help="Generated output layout filename to promote (default: layout.c1_<group-label>_dedup_v1.usd)",
    )

    parser.add_argument(
        "--exclude-dir-contains",
        action="append",
        default=["/_dedup_assets/"],
        help="Skip any directory whose path contains this substring during full scan (repeatable)",
    )
    parser.add_argument(
        "--exclude-name-contains",
        action="append",
        default=[],
        help="Skip any *.usd filename containing this substring during full scan (repeatable)",
    )

    parser.add_argument(
        "--progress-every",
        type=int,
        default=1000,
        help="Emit scan progress every N files (default: 1000)",
    )

    parser.add_argument(
        "--report-dir",
        default=None,
        help="Report output directory (default: check_reports/c1_pilot/<group-label>_batch)",
    )

    parser.add_argument(
        "--ledger-jsonl",
        default="check_reports/c1_pilot/c1_dedup_ledger.jsonl",
        help="Append-only ledger JSONL path",
    )

    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print planned actions without modifying files",
    )

    args = parser.parse_args()

    stamp = _stamp_now()

    dataset_root = Path(args.dataset_root)
    bak_root = Path(args.bak_root)
    layout_root = Path(args.layout_root) if args.layout_root else dataset_root / "GRScenes100"

    mapping_json = Path(args.mapping_json)

    report_dir = Path(args.report_dir) if args.report_dir else Path("check_reports") / "c1_pilot" / f"{args.group_label}_batch"
    report_dir.mkdir(parents=True, exist_ok=True)

    patterns = _patterns_from_mapping_json(mapping_json, category=args.category)
    uids_info = _uids_from_mapping_json(mapping_json)

    layout_out_name = args.layout_out_name or f"layout.c1_{args.group_label}_dedup_v1.usd"

    promote_report = report_dir / "promote_to_layout_usd_report.json"
    post_promote_layout_scan = report_dir / "post_promote_layout_scan.json"
    post_promote_full_scan = report_dir / "post_promote_full_usd_scan_excluding_backups.json"
    post_soft_delete_layout_scan = report_dir / "post_soft_delete_layout_scan.json"
    soft_delete_report = report_dir / "soft_delete_old_assets_report.json"

    full_scan_progress_json = report_dir / "post_promote_full_usd_scan_progress.json"
    full_scan_progress_jsonl = report_dir / "post_promote_full_usd_scan_progress.jsonl"

    print(f"Step6 group={args.group_label} category={args.category} stamp={stamp}", flush=True)

    # 1) Promote layouts
    out_layouts = sorted(layout_root.glob(f"**/{layout_out_name}"))
    if not out_layouts:
        raise SystemExit(f"No generated layouts found: {layout_root}/**/{layout_out_name}")

    promote_ops: List[Dict[str, str]] = []
    for outp in out_layouts:
        scene_dir = outp.parent
        dst = scene_dir / "layout.usd"
        backup = scene_dir / f"layout.pre_c1_{args.group_label}.{stamp}.usd"
        if not dst.exists():
            raise SystemExit(f"Missing layout.usd: {dst}")
        if backup.exists():
            raise SystemExit(f"Backup already exists: {backup}")

        promote_ops.append(
            {
                "dir": str(scene_dir.resolve()),
                "backup": str(backup.resolve()),
                "promoted_from": str(outp.resolve()),
                "promoted_to": str(dst.resolve()),
            }
        )

        if not args.dry_run:
            shutil.copy2(dst, backup)
            os.replace(outp, dst)

    if not args.dry_run:
        promote_report.write_text(
            json.dumps({"stamp": stamp, "count": len(promote_ops), "ops": promote_ops}, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    print(f"Promote: {len(promote_ops)} layouts", flush=True)

    # 2) Post-promote scans
    layout_scan_payload = _scan_layouts(layout_root, patterns=patterns)
    if not args.dry_run:
        post_promote_layout_scan.write_text(json.dumps(layout_scan_payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"Post-promote layout scan hit={layout_scan_payload['hit_layouts']}", flush=True)

    # Ensure backups are excluded by default
    if not any("pre_c1_" in s for s in args.exclude_name_contains):
        args.exclude_name_contains.append(f"pre_c1_{args.group_label}")

    full_scan_payload = _scan_full_usd_tree(
        dataset_root,
        patterns=patterns,
        exclude_dir_contains=args.exclude_dir_contains,
        exclude_name_contains=args.exclude_name_contains,
        progress_every=int(args.progress_every),
        progress_json=None if args.dry_run else full_scan_progress_json,
        progress_jsonl=None if args.dry_run else full_scan_progress_jsonl,
    )

    if not args.dry_run:
        post_promote_full_scan.write_text(json.dumps(full_scan_payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    if int(full_scan_payload["hit_files"]) != 0 or int(layout_scan_payload["hit_layouts"]) != 0:
        raise SystemExit(
            f"Ref scan not clean (layout_hit={layout_scan_payload['hit_layouts']} full_hit={full_scan_payload['hit_files']}). "
            "Aborting before soft delete."
        )

    # 3) Soft delete old assets (move dirs)
    src_root = dataset_root / "GRScenes_assets" / args.category
    dest_root = bak_root / "_dedup_assets" / f"{args.group_label}_{stamp}" / "GRScenes_assets" / args.category

    moves: List[Dict[str, str]] = []
    if not args.dry_run:
        dest_root.mkdir(parents=True, exist_ok=True)

    for uid in uids_info["old_uids"]:  # type: ignore[index]
        if uid == uids_info["canonical_uid"]:
            continue
        src = src_root / uid
        dst = dest_root / uid
        if not src.exists():
            raise SystemExit(f"Missing src asset dir: {src}")
        if dst.exists():
            raise SystemExit(f"Dest already exists: {dst}")
        moves.append({"uid": str(uid), "from": str(src.resolve()), "to": str(dst.resolve())})
        if not args.dry_run:
            shutil.move(str(src), str(dst))

    if not args.dry_run:
        soft_delete_report.write_text(
            json.dumps(
                {
                    "stamp": stamp,
                    "count": len(moves),
                    "dest_root": str(dest_root.resolve()),
                    "moves": moves,
                },
                indent=2,
                ensure_ascii=False,
            )
            + "\n",
            encoding="utf-8",
        )

    print(f"Soft delete: moved {len(moves)} asset dirs", flush=True)

    # 4) Post-soft-delete layout scan
    post_soft_payload = _scan_layouts(layout_root, patterns=patterns)
    if not args.dry_run:
        post_soft_delete_layout_scan.write_text(json.dumps(post_soft_payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    if int(post_soft_payload["hit_layouts"]) != 0:
        raise SystemExit(f"Post-soft-delete layout scan hit={post_soft_payload['hit_layouts']} (expected 0)")

    # 5) Append ledger
    ledger_entry = {
        "stamp": stamp,
        "group_label": args.group_label,
        "category": args.category,
        "mapping_json": str(mapping_json),
        "canonical_asset_usd": uids_info["canonical_asset_usd"],
        "canonical_uid": uids_info["canonical_uid"],
        "old_asset_count": uids_info["old_count"],
        "promoted_layouts": len(promote_ops),
        "reports": {
            "promote_to_layout_usd_report": str(promote_report),
            "post_promote_layout_scan": str(post_promote_layout_scan),
            "post_promote_full_usd_scan_excluding_backups": str(post_promote_full_scan),
            "soft_delete_old_assets_report": str(soft_delete_report),
            "post_soft_delete_layout_scan": str(post_soft_delete_layout_scan),
        },
        "bak_dest_root": str(dest_root),
    }

    if not args.dry_run:
        _append_ledger(Path(args.ledger_jsonl), ledger_entry)

    print("DONE", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
