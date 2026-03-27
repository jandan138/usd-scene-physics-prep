#!/usr/bin/env python3
"""Restore old ground asset dirs from bak to main dataset for plane-fix re-run.

Background:
  c1_v3_vmatrix_fix (2026-03-20) soft-deleted old ground assets by moving them to
  GRScenes-test0-rebuilt-normalized_bak/_dedup_assets/c1_v3_vmatrix_fix_20260320_230116/GRScenes_assets/ground/

  For the shape-invariant plane-fix re-run (c1_v4_plane_fix) we must:
    1. Start from layout.pre_c1_normalize_only.*.usd (which has original old-asset refs)
    2. Re-apply dedup with the corrected compute_V_shape_invariant
    3. V matrix computation needs to open the old asset USD to extract vertices

  This script restores those old assets back to the main dataset so the re-run can read them.
  After the re-run, c1_autorun step6 will soft-delete them again (to a new bak subdir).

Usage:
  python scripts/restore_ground_assets_for_plane_fix.py [--dry-run] [--repo-root .]
"""
from __future__ import annotations

import argparse
import shutil
from pathlib import Path


BAK_SUBDIR = "_dedup_assets/c1_v3_vmatrix_fix_20260320_230116/GRScenes_assets/ground"
MAIN_SUBDIR = "GRScenes_assets/ground"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--repo-root", default=".", help="Repository root (default: cwd)")
    ap.add_argument(
        "--dataset-root",
        default="GRScenes-test0-rebuilt-normalized",
        help="Main dataset dir relative to repo root",
    )
    ap.add_argument(
        "--bak-root",
        default="GRScenes-test0-rebuilt-normalized_bak",
        help="Bak dir relative to repo root",
    )
    ap.add_argument("--dry-run", action="store_true", help="Print actions but do not move files")
    args = ap.parse_args()

    repo_root = Path(args.repo_root).resolve()
    src_base = repo_root / args.bak_root / BAK_SUBDIR
    dst_base = repo_root / args.dataset_root / MAIN_SUBDIR

    if not src_base.is_dir():
        raise SystemExit(f"Bak ground dir not found: {src_base}")

    dst_base.mkdir(parents=True, exist_ok=True)

    moved = 0
    skipped_exists = 0
    skipped_canonical = 0

    uid_dirs = sorted(src_base.iterdir())
    print(f"Found {len(uid_dirs)} uid dirs in bak ground dir")

    for uid_dir in uid_dirs:
        if not uid_dir.is_dir():
            continue
        dst = dst_base / uid_dir.name
        if dst.exists():
            # Already present in main dataset — this is the canonical asset, leave it alone
            skipped_canonical += 1
            continue
        print(f"  {'[DRY]' if args.dry_run else 'MOVE'} {uid_dir.name}")
        if not args.dry_run:
            shutil.move(str(uid_dir), str(dst))
        moved += 1

    print(
        f"\nSummary: moved={moved}  skipped_already_in_main={skipped_canonical}  "
        f"dry_run={args.dry_run}"
    )
    if args.dry_run:
        print("(dry-run: no files were moved)")


if __name__ == "__main__":
    main()
