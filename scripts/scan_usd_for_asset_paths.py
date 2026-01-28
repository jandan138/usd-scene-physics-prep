#!/usr/bin/env python3
"""Scan USD files for references to a set of asset-path patterns, with progress output.

Why this exists
- Full-library scans can take a long time (tens of thousands of USD files).
- Without progress, it's easy to assume the process is stuck.

What it does
- Discovers candidate *.usd files under a root directory.
- Scans file bytes for any of the provided path patterns.
- Writes:
  - result JSON with hits
  - progress snapshot JSON (overwrite) + progress history JSONL (append)

Pattern matching strategy
- Match the *asset USD path substring* (e.g. "GRScenes_assets/plant/<uid>/usd/<uid>.usd").
- Avoid matching raw UID substrings, which can produce false positives due to prim names.

Typical usage (example)
  python scripts/scan_usd_for_asset_paths.py \
    --root GRScenes-test1 \
    --mapping-json check_reports/c1_pilot/pilot_mapping_plant_sigb526_full.json \
    --exclude-name-contains pre_c1_sigb526 \
    --out-json check_reports/c1_pilot/sigb526_batch/post_promote_full_usd_scan_excluding_backups.json

"""

from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence


def _stamp_now() -> str:
    return time.strftime("%Y%m%d_%H%M%S", time.localtime())


def _emit_progress(
    *,
    started_at: float,
    processed: int,
    total: int,
    hits: int,
    phase: str,
    progress_json: Optional[str],
    progress_jsonl: Optional[str],
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
        Path(progress_jsonl).parent.mkdir(parents=True, exist_ok=True)
        with open(progress_jsonl, "a", encoding="utf-8") as f:
            f.write(json.dumps(payload, ensure_ascii=False) + "\n")
    if progress_json:
        Path(progress_json).parent.mkdir(parents=True, exist_ok=True)
        with open(progress_json, "w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2, ensure_ascii=False)

    eta_msg = "?" if eta_sec is None else f"{eta_sec/60.0:.1f}m"
    cur = "" if not current_file else f" | cur={Path(current_file).name}"
    print(
        f"  {phase} {processed}/{total} | hits={hits} | rate={rate:.2f}/s | eta={eta_msg}{cur}",
        flush=True,
    )


def _iter_usd_files(root: Path, *, exclude_dir_contains: Sequence[str]) -> Iterable[Path]:
    for dirpath, dirnames, filenames in os.walk(root):
        dirpath_str = str(dirpath)
        if any(x and x in dirpath_str for x in exclude_dir_contains):
            continue
        for fn in filenames:
            if not fn.endswith(".usd"):
                continue
            yield Path(dirpath) / fn


def _patterns_from_mapping_json(mapping_json: Path, *, category: str) -> List[str]:
    mapping: Dict[str, str] = json.loads(mapping_json.read_text())
    patterns: List[str] = []
    for old_asset_usd in mapping.keys():
        p = Path(old_asset_usd)
        # Expect: GRScenes-test1/GRScenes_assets/<category>/<uid>/usd/<uid>.usd
        # Parts: 0 dataset, 1 GRScenes_assets, 2 category, 3 uid, 4 usd, 5 file
        if len(p.parts) < 6:
            continue
        uid = p.parts[3]
        patterns.append(f"GRScenes_assets/{category}/{uid}/usd/{uid}.usd")
    return sorted(set(patterns))


def main() -> int:
    parser = argparse.ArgumentParser(description="Scan USD files for old asset path patterns with progress output")
    parser.add_argument("--root", required=True, help="Root directory to scan (e.g. GRScenes-test1)")

    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--patterns-json", default=None, help="JSON file containing a list of string patterns")
    group.add_argument(
        "--mapping-json",
        default=None,
        help="Mapping JSON (old_asset_usd -> canonical_asset_usd); patterns are derived from mapping keys",
    )

    parser.add_argument(
        "--category",
        default="plant",
        help="Asset category used to derive patterns from mapping JSON (default: plant)",
    )

    parser.add_argument(
        "--exclude-dir-contains",
        action="append",
        default=["/_dedup_assets/"],
        help="Skip any directory whose path contains this substring (can be repeated)",
    )
    parser.add_argument(
        "--exclude-name-contains",
        action="append",
        default=[],
        help="Skip any *.usd file whose filename contains this substring (can be repeated)",
    )

    parser.add_argument(
        "--progress-every",
        type=int,
        default=1000,
        help="Emit progress every N files during scan (0 disables)",
    )
    parser.add_argument(
        "--progress-json",
        default=None,
        help="Progress snapshot JSON path (overwrite). If omitted, no snapshot is written.",
    )
    parser.add_argument(
        "--progress-jsonl",
        default=None,
        help="Progress history JSONL path (append). If omitted, no history is written.",
    )

    parser.add_argument(
        "--out-json",
        required=True,
        help="Output report JSON path",
    )

    args = parser.parse_args()

    started_at = time.time()
    root = Path(args.root)
    out_json = Path(args.out_json)

    if args.patterns_json:
        patterns = json.loads(Path(args.patterns_json).read_text())
        if not isinstance(patterns, list) or not all(isinstance(x, str) for x in patterns):
            raise SystemExit("--patterns-json must be a JSON list of strings")
        patterns = sorted(set(patterns))
    else:
        patterns = _patterns_from_mapping_json(Path(args.mapping_json), category=args.category)

    if not patterns:
        raise SystemExit("No patterns to scan for")

    print(f"Starting USD scan under: {root.resolve()}", flush=True)
    print(f"Patterns: {len(patterns)}", flush=True)

    # Discover candidates
    usd_files: List[Path] = []
    discover_every = 0
    if args.progress_every and args.progress_every > 0:
        discover_every = max(args.progress_every * 20, 20000)

    _emit_progress(
        started_at=started_at,
        processed=0,
        total=0,
        hits=0,
        phase="discover",
        progress_json=args.progress_json,
        progress_jsonl=args.progress_jsonl,
    )

    for i, p in enumerate(_iter_usd_files(root, exclude_dir_contains=args.exclude_dir_contains), start=1):
        if any(x and x in p.name for x in args.exclude_name_contains):
            continue
        usd_files.append(p)
        if discover_every and i % discover_every == 0:
            _emit_progress(
                started_at=started_at,
                processed=i,
                total=0,
                hits=0,
                phase="discover",
                progress_json=args.progress_json,
                progress_jsonl=args.progress_jsonl,
            )

    usd_files.sort()
    total = len(usd_files)

    print(f"Scanning {total} USD files...", flush=True)
    _emit_progress(
        started_at=started_at,
        processed=0,
        total=total,
        hits=0,
        phase="scan",
        progress_json=args.progress_json,
        progress_jsonl=args.progress_jsonl,
    )

    hits: List[Dict[str, object]] = []
    hit_count = 0

    for idx, p in enumerate(usd_files, start=1):
        if idx == 1:
            _emit_progress(
                started_at=started_at,
                processed=1,
                total=total,
                hits=hit_count,
                phase="scan",
                progress_json=args.progress_json,
                progress_jsonl=args.progress_jsonl,
                current_file=str(p),
            )

        if args.progress_every and args.progress_every > 0 and idx % args.progress_every == 0:
            _emit_progress(
                started_at=started_at,
                processed=idx,
                total=total,
                hits=hit_count,
                phase="scan",
                progress_json=args.progress_json,
                progress_jsonl=args.progress_jsonl,
                current_file=str(p),
            )

        try:
            data = p.read_bytes()
        except Exception as e:
            hits.append({"file": str(p), "error": str(e)})
            hit_count += 1
            continue

        matched = [pat for pat in patterns if pat.encode("utf-8") in data]
        if matched:
            hits.append({"file": str(p), "hit_paths": matched})
            hit_count += 1

    payload = {
        "stamp": _stamp_now(),
        "root": str(root),
        "patterns_count": len(patterns),
        "scanned_files": total,
        "hit_files": len(hits),
        "results": hits,
    }

    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    _emit_progress(
        started_at=started_at,
        processed=total,
        total=total,
        hits=len(hits),
        phase="done",
        progress_json=args.progress_json,
        progress_jsonl=args.progress_jsonl,
    )

    print(f"Wrote: {out_json}", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
