#!/usr/bin/env python3
"""Restore a normalized dataset to its pre-dedup state.

Restores layout.usd files from pre_c1_normalize_only snapshots and copies
back removed asset directories from a backup root.

Usage:
    python scripts/restore_pre_dedup_working_tree.py \
        --dataset-root GRScenes-test0-rebuilt-normalized \
        --bak-root GRScenes-test0-rebuilt-normalized_bak/_dedup_assets_merged/GRScenes_assets \
        [--dry-run]
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import shutil
import sys
from concurrent.futures import ThreadPoolExecutor, as_completed


def _find_pre_c1_layout(scene_dir: str) -> str | None:
    """Return the most recent layout.pre_c1_normalize_only.*.usd in scene_dir."""
    pattern = os.path.join(scene_dir, "layout.pre_c1_normalize_only.*.usd")
    candidates = sorted(glob.glob(pattern))
    if not candidates:
        return None
    # Sorted lexicographically; last one is most recent by timestamp in name
    return candidates[-1]


def restore_layouts(
    dataset_root: str,
    scene_name: str,
    scene_category: str,
    dry_run: bool,
) -> dict:
    """Restore layout.usd from pre_c1_normalize_only snapshots."""
    scenes_dir = os.path.join(dataset_root, scene_name, scene_category)
    restored = 0
    missing = 0

    if not os.path.isdir(scenes_dir):
        print(f"[WARN] scenes dir does not exist: {scenes_dir}")
        return {"pre_c1_layouts_restored": 0, "pre_c1_layouts_missing": 0}

    scene_ids = sorted(os.listdir(scenes_dir))
    for sid in scene_ids:
        scene_dir = os.path.join(scenes_dir, sid)
        if not os.path.isdir(scene_dir):
            continue
        snapshot = _find_pre_c1_layout(scene_dir)
        if snapshot is None:
            missing += 1
            print(f"  [MISS] no pre_c1_normalize_only snapshot: {sid}")
            continue

        layout_dst = os.path.join(scene_dir, "layout.usd")
        if dry_run:
            print(f"  [DRY] would copy {os.path.basename(snapshot)} -> layout.usd  ({sid})")
        else:
            shutil.copy2(snapshot, layout_dst)
            print(f"  [OK] restored layout.usd from {os.path.basename(snapshot)}  ({sid})")
        restored += 1

    return {
        "pre_c1_layouts_restored": restored,
        "pre_c1_layouts_missing": missing,
    }


def _copy_asset_dir(src: str, dst: str) -> str:
    """Copy a single asset uid directory. Returns dst on success."""
    shutil.copytree(src, dst)
    return dst


def restore_assets(
    dataset_root: str,
    bak_root: str,
    asset_name: str,
    dry_run: bool,
    workers: int,
) -> dict:
    """Restore removed asset directories from backup."""
    target_assets = os.path.join(dataset_root, asset_name)
    bak_assets = bak_root  # already points to {bak_root} with {category}/{uid}/ inside

    assets_restored = 0
    assets_skipped = 0
    assets_conflict = 0
    copy_tasks = []  # (src, dst)

    if not os.path.isdir(bak_assets):
        print(f"[WARN] backup assets dir does not exist: {bak_assets}")
        return {
            "assets_restored": 0,
            "assets_skipped_already_exist": 0,
            "assets_conflict": 0,
        }

    categories = sorted(
        d for d in os.listdir(bak_assets)
        if os.path.isdir(os.path.join(bak_assets, d))
    )

    for cat in categories:
        bak_cat_dir = os.path.join(bak_assets, cat)
        tgt_cat_dir = os.path.join(target_assets, cat)
        uids = sorted(
            d for d in os.listdir(bak_cat_dir)
            if os.path.isdir(os.path.join(bak_cat_dir, d))
        )
        for uid in uids:
            bak_uid_dir = os.path.join(bak_cat_dir, uid)
            tgt_uid_dir = os.path.join(tgt_cat_dir, uid)

            if os.path.exists(tgt_uid_dir):
                # Check for conflict: compare USD file sizes
                conflict = _check_conflict(bak_uid_dir, tgt_uid_dir)
                if conflict:
                    assets_conflict += 1
                assets_skipped += 1
            else:
                copy_tasks.append((bak_uid_dir, tgt_uid_dir))

    if dry_run:
        for src, dst in copy_tasks:
            rel = os.path.relpath(dst, target_assets)
            print(f"  [DRY] would restore {rel}")
        assets_restored = len(copy_tasks)
    else:
        # Ensure category dirs exist
        needed_cats = set()
        for _, dst in copy_tasks:
            needed_cats.add(os.path.dirname(dst))
        for cat_dir in needed_cats:
            os.makedirs(cat_dir, exist_ok=True)

        # Parallel copy
        if workers <= 1 or len(copy_tasks) <= 1:
            for src, dst in copy_tasks:
                _copy_asset_dir(src, dst)
                assets_restored += 1
        else:
            with ThreadPoolExecutor(max_workers=workers) as pool:
                futs = {
                    pool.submit(_copy_asset_dir, src, dst): (src, dst)
                    for src, dst in copy_tasks
                }
                for fut in as_completed(futs):
                    src, dst = futs[fut]
                    try:
                        fut.result()
                        assets_restored += 1
                    except Exception as exc:
                        rel = os.path.relpath(dst, target_assets)
                        print(f"  [ERR] failed to restore {rel}: {exc}")

        print(f"  [OK] restored {assets_restored} asset dirs")

    return {
        "assets_restored": assets_restored,
        "assets_skipped_already_exist": assets_skipped,
        "assets_conflict": assets_conflict,
    }


def _check_conflict(bak_dir: str, tgt_dir: str) -> bool:
    """Check if the main USD file sizes differ between backup and target."""
    bak_usd = _find_main_usd(bak_dir)
    tgt_usd = _find_main_usd(tgt_dir)
    if bak_usd is None or tgt_usd is None:
        return False
    try:
        return os.path.getsize(bak_usd) != os.path.getsize(tgt_usd)
    except OSError:
        return False


def _find_main_usd(uid_dir: str) -> str | None:
    """Find the main .usd file in a uid asset directory (under usd/ subdir)."""
    usd_subdir = os.path.join(uid_dir, "usd")
    if os.path.isdir(usd_subdir):
        for f in os.listdir(usd_subdir):
            if f.endswith(".usd"):
                return os.path.join(usd_subdir, f)
    # Fallback: look directly in uid_dir
    for f in os.listdir(uid_dir):
        if f.endswith(".usd"):
            return os.path.join(uid_dir, f)
    return None


def count_asset_dirs(base: str) -> int:
    """Count uid-level directories under base/{category}/{uid}/."""
    count = 0
    if not os.path.isdir(base):
        return 0
    for cat in os.listdir(base):
        cat_dir = os.path.join(base, cat)
        if not os.path.isdir(cat_dir):
            continue
        for uid in os.listdir(cat_dir):
            if os.path.isdir(os.path.join(cat_dir, uid)):
                count += 1
    return count


def main():
    parser = argparse.ArgumentParser(
        description="Restore a normalized dataset to pre-dedup state."
    )
    parser.add_argument(
        "--dataset-root", required=True,
        help="Path to normalized dataset (e.g., GRScenes-test0-rebuilt-normalized)",
    )
    parser.add_argument(
        "--bak-root", required=True,
        help="Path to backup directory containing {category}/{uid}/ asset dirs",
    )
    parser.add_argument(
        "--asset-name", default="GRScenes_assets",
        help="Asset directory name (default: GRScenes_assets)",
    )
    parser.add_argument(
        "--scene-name", default="GRScenes100",
        help="Scene directory name (default: GRScenes100)",
    )
    parser.add_argument(
        "--scene-category", default="home",
        help="Scene category (default: home)",
    )
    parser.add_argument(
        "--out-checklist", default=None,
        help="Output checklist JSON path (default: {dataset_root}/restore_checklist.json)",
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="Report only, do not copy",
    )
    parser.add_argument(
        "--workers", type=int, default=4,
        help="Parallel copy workers (default: 4)",
    )
    args = parser.parse_args()

    dataset_root = os.path.abspath(args.dataset_root)
    bak_root = os.path.abspath(args.bak_root)
    out_checklist = args.out_checklist or os.path.join(
        dataset_root, "restore_checklist.json"
    )

    if not os.path.isdir(dataset_root):
        print(f"ERROR: dataset root does not exist: {dataset_root}")
        sys.exit(1)
    if not os.path.isdir(bak_root):
        print(f"ERROR: backup root does not exist: {bak_root}")
        sys.exit(1)

    mode = "DRY RUN" if args.dry_run else "LIVE"
    print(f"=== Restore Pre-Dedup Working Tree ({mode}) ===")
    print(f"  dataset_root: {dataset_root}")
    print(f"  bak_root:     {bak_root}")
    print(f"  asset_name:   {args.asset_name}")
    print(f"  scene_name:   {args.scene_name}")
    print()

    # Phase 1: Restore layouts
    print("[Phase 1] Restoring layout.usd from pre_c1_normalize_only snapshots...")
    layout_stats = restore_layouts(
        dataset_root, args.scene_name, args.scene_category, args.dry_run,
    )
    print(
        f"  layouts: restored={layout_stats['pre_c1_layouts_restored']}  "
        f"missing={layout_stats['pre_c1_layouts_missing']}"
    )
    print()

    # Phase 2: Restore assets
    print("[Phase 2] Restoring removed asset directories from backup...")
    asset_stats = restore_assets(
        dataset_root, bak_root, args.asset_name, args.dry_run, args.workers,
    )
    print(
        f"  assets: restored={asset_stats['assets_restored']}  "
        f"skipped={asset_stats['assets_skipped_already_exist']}  "
        f"conflict={asset_stats['assets_conflict']}"
    )
    print()

    # Phase 3: Final counts
    print("[Phase 3] Counting final asset directories...")
    target_assets = os.path.join(dataset_root, args.asset_name)
    final_count = count_asset_dirs(target_assets)
    expected_count = count_asset_dirs(bak_root)

    match = final_count >= expected_count and asset_stats["assets_conflict"] == 0

    checklist = {
        **layout_stats,
        **asset_stats,
        "final_asset_count": final_count,
        "expected_asset_count": expected_count,
        "match": match,
    }

    print(f"  final_asset_count:    {final_count}")
    print(f"  expected_asset_count: {expected_count}")
    print(f"  match:                {match}")
    print()

    if not args.dry_run:
        os.makedirs(os.path.dirname(os.path.abspath(out_checklist)), exist_ok=True)
        with open(out_checklist, "w") as f:
            json.dump(checklist, f, indent=2)
        print(f"Checklist written to: {out_checklist}")
    else:
        print("Checklist (dry run, not written):")
        print(json.dumps(checklist, indent=2))

    if not match:
        print("\n[WARN] Restore did not achieve full match. Check conflicts.")
        sys.exit(2)

    print("\n[OK] Restore complete.")


if __name__ == "__main__":
    main()
