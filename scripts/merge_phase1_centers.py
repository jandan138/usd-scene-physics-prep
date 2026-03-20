#!/usr/bin/env python3
"""Merge per-category Phase 1 centers files into one flat directory.

Phase 2 normalization and audit helpers read `centers_*.json` from a single
directory. This helper collects those files from per-category Phase 1 report
directories and materializes them into one flat destination using either copies
or symlinks.

Usage:
    python scripts/merge_phase1_centers.py \
        --phase1-root check_reports/test0_full/<RUN_ID>/normalize/phase1 \
        --out-dir check_reports/test0_full/<RUN_ID>/normalize/phase1/centers_merged
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Merge per-category Phase 1 centers files into a flat directory",
    )
    parser.add_argument(
        "--phase1-root",
        required=True,
        help="Phase 1 report root containing one subdirectory per category",
    )
    parser.add_argument(
        "--out-dir",
        required=True,
        help="Destination directory for the flat centers_*.json bundle",
    )
    parser.add_argument(
        "--mode",
        choices=("copy", "symlink"),
        default="copy",
        help="Materialization mode for merged files (default: copy)",
    )
    return parser.parse_args()


def _discover_center_files(phase1_root: Path, out_dir: Path) -> list[Path]:
    center_files: list[Path] = []
    out_dir_resolved = out_dir.resolve(strict=False)

    for entry in sorted(phase1_root.iterdir(), key=lambda path: path.name):
        if not entry.is_dir():
            continue
        if entry.resolve(strict=False) == out_dir_resolved:
            continue
        center_files.extend(sorted(entry.glob("centers_*.json"), key=lambda path: path.name))

    return center_files


def _relative_symlink_target(src: Path, dst_dir: Path) -> str:
    return os.path.relpath(src, start=dst_dir)


def _materialize_file(src: Path, dst: Path, mode: str) -> None:
    if mode == "copy":
        shutil.copy2(src, dst)
        return
    if mode == "symlink":
        os.symlink(_relative_symlink_target(src, dst.parent), dst)
        return
    raise ValueError(f"unsupported mode: {mode}")


def main() -> int:
    args = parse_args()

    phase1_root = Path(args.phase1_root).resolve()
    out_dir = Path(args.out_dir).resolve()

    if not phase1_root.is_dir():
        print(f"ERROR: phase1 root does not exist or is not a directory: {phase1_root}", file=sys.stderr)
        return 1

    center_files = _discover_center_files(phase1_root, out_dir)
    if not center_files:
        print(f"ERROR: no centers_*.json files found under {phase1_root}", file=sys.stderr)
        return 1

    destination_names: dict[str, Path] = {}
    collisions: list[str] = []
    for src in center_files:
        existing = destination_names.get(src.name)
        if existing is not None:
            collisions.append(
                f"{src.name}: {existing.parent.name}/{existing.name} and {src.parent.name}/{src.name}"
            )
        destination_names[src.name] = src

    if collisions:
        print("ERROR: duplicate centers filenames found across category report dirs:", file=sys.stderr)
        for item in collisions:
            print(f"  - {item}", file=sys.stderr)
        return 1

    out_dir.mkdir(parents=True, exist_ok=True)

    preexisting_paths = [
        out_dir / src.name
        for src in center_files
        if (out_dir / src.name).exists() or (out_dir / src.name).is_symlink()
    ]
    summary_path = out_dir / "merge_summary.json"
    if summary_path.exists() or summary_path.is_symlink():
        preexisting_paths.append(summary_path)

    if preexisting_paths:
        print("ERROR: destination already contains merge outputs:", file=sys.stderr)
        for path in sorted(preexisting_paths):
            print(f"  - {path}", file=sys.stderr)
        return 1

    merged_names: list[str] = []
    for src in center_files:
        dst = out_dir / src.name
        _materialize_file(src=src, dst=dst, mode=args.mode)
        merged_names.append(src.name)

    scanned_dirs = [
        entry
        for entry in phase1_root.iterdir()
        if entry.is_dir() and entry.resolve(strict=False) != out_dir
    ]

    summary = {
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "src_root": str(phase1_root),
        "dst_root": str(out_dir),
        "mode": args.mode,
        "source_report_dirs_scanned": len(scanned_dirs),
        "merged_centers_files_count": len(merged_names),
        "merged_centers_files": merged_names,
    }
    with summary_path.open("w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
        f.write("\n")

    print(f"Merged {len(merged_names)} centers files into {out_dir}")
    print(f"Summary written: {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
