#!/usr/bin/env python3
"""Copy annotation JSON, render PNGs, and GLB files from test1 into a subset.

For each asset in the subset, looks up the matching asset in the source
(test1) by MD5 hash and copies:
  - {hash}_annotation.json  (overwrites existing file/symlink)
  - front.png, back.png, left.png, right.png
  - glb/{hash}.glb

When ``--dedup-report`` is given, assets not found directly in the source
are looked up via their dedup group: if a sibling hash in the same group
exists in the source, its files are copied and the annotation JSON ``uid``
field is patched to the original hash.
"""

import argparse
import json
import logging
import os
import shutil
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

LOG = logging.getLogger(__name__)

# Categories that differ in name between subset (key) and test1 (value).
CATEGORY_RENAME = {
    "bathtub": "bath_tub",
    "bookshelf": "book_shelf",
    "dishwasher": "dish_washer",
    "nightstand": "night_stand",
    "Musical_instrument": "musical_instrument",
    "ceilinglight": "ceiling_light",
    "chestofdrawers": "chest_of_drawers",
    "coffeemaker": "coffee_maker",
    "electriccooker": "electric_cooker",
    "shoecabinet": "shoe_cabinet",
    "shoppingtrolley": "shopping_trolley",
    "sideboardcabinet": "sideboard_cabinet",
    "sofachair": "sofa_chair",
    "teatable": "tea_table",
    "trashcan": "trash_can",
    "tvstand": "tv_stand",
    "washingmachine": "washing_machine",
}

# Known nested directory to skip.
SKIP_CATEGORIES = {"GRScenes_assets"}

RENDER_PNGS = ["front.png", "back.png", "left.png", "right.png"]


def _load_dedup_lookup(report_path: Path) -> dict[str, list[tuple[str, str]]]:
    """Build hash → [(category, hash), ...] lookup from a dedup report JSON.

    Each dedup group lists ``usd_paths`` like
    ``GRScenes-test1/GRScenes_assets/CAT/HASH/usd/HASH.usd``.
    We parse category (parts[2]) and hash (parts[3]) from each path.

    Returns a dict mapping every member hash to the full list of members in
    its group (including itself).
    """
    LOG.info("Loading dedup report from %s (this may take a moment)...", report_path)
    with open(report_path) as f:
        data = json.load(f)

    lookup: dict[str, list[tuple[str, str]]] = {}
    for group in data.get("duplicates", []):
        members: list[tuple[str, str]] = []
        for usd_path in group.get("usd_paths", []):
            parts = usd_path.split("/")
            if len(parts) >= 4:
                members.append((parts[2], parts[3]))
        for _cat, h in members:
            lookup[h] = members

    LOG.info("Dedup lookup built: %d hashes in %d groups",
             len(lookup), len(data.get("duplicates", [])))
    return lookup


def _find_dedup_representative(
    asset_hash: str,
    cat_name: str,
    dedup_lookup: dict[str, list[tuple[str, str]]],
    source_root: Path,
) -> Path | None:
    """Find an existing source directory for a dedup sibling of *asset_hash*.

    Returns the source asset directory Path if found, else None.
    """
    members = dedup_lookup.get(asset_hash)
    if not members:
        return None
    for member_cat, member_hash in members:
        if member_hash == asset_hash:
            continue
        source_cat = CATEGORY_RENAME.get(member_cat, member_cat)
        candidate = source_root / source_cat / member_hash
        if candidate.is_dir():
            return candidate
    return None


def _enrich_one_dedup(
    subset_asset_dir: Path,
    source_asset_dir: Path,
    asset_hash: str,
    source_hash: str,
    skip_existing: bool,
    dry_run: bool,
) -> dict:
    """Enrich via dedup fallback: copy from *source_hash* dir, patch uid."""
    copied = 0
    missing_files = []

    # 1. Annotation JSON — copy from source hash, patch uid
    src_json = source_asset_dir / f"{source_hash}_annotation.json"
    dst_json = subset_asset_dir / f"{asset_hash}_annotation.json"
    if src_json.exists():
        if skip_existing and dst_json.exists() and not dst_json.is_symlink():
            pass  # skip
        elif dry_run:
            copied += 1
        else:
            if dst_json.is_symlink():
                dst_json.unlink()
            dst_json.parent.mkdir(parents=True, exist_ok=True)
            # Copy, then patch uid
            shutil.copy2(src_json, dst_json)
            try:
                ann = json.loads(dst_json.read_text())
                ann["uid"] = asset_hash
                dst_json.write_text(json.dumps(ann, indent=2, ensure_ascii=False))
            except (json.JSONDecodeError, KeyError):
                LOG.warning("Could not patch uid in %s", dst_json)
            copied += 1
    else:
        missing_files.append(f"{source_hash}_annotation.json")

    # 2. Render PNGs (same filenames, no hash in name)
    for png in RENDER_PNGS:
        src_png = source_asset_dir / png
        dst_png = subset_asset_dir / png
        if _copy_file(src_png, dst_png, skip_existing, dry_run):
            copied += 1
        elif not src_png.exists():
            missing_files.append(png)

    # 3. GLB — source has source_hash.glb, destination needs asset_hash.glb
    src_glb = source_asset_dir / "glb" / f"{source_hash}.glb"
    dst_glb = subset_asset_dir / "glb" / f"{asset_hash}.glb"
    if src_glb.exists():
        if skip_existing and dst_glb.exists() and not dst_glb.is_symlink():
            pass
        elif dry_run:
            copied += 1
        else:
            if dst_glb.is_symlink():
                dst_glb.unlink()
            dst_glb.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src_glb, dst_glb)
            copied += 1
    else:
        missing_files.append(f"glb/{source_hash}.glb")

    return {"copied": copied, "missing_files": missing_files}


def _copy_file(src: Path, dst: Path, skip_existing: bool, dry_run: bool) -> bool:
    """Copy a single file. Returns True if copied (or would copy in dry-run)."""
    if not src.exists():
        return False
    if skip_existing and dst.exists() and not dst.is_symlink():
        return False
    if dry_run:
        return True
    # Remove symlinks so we write a real file.
    if dst.is_symlink():
        dst.unlink()
    dst.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(src, dst)
    return True


def _enrich_one(
    subset_asset_dir: Path,
    source_asset_dir: Path,
    asset_hash: str,
    skip_existing: bool,
    dry_run: bool,
) -> dict:
    """Enrich a single asset. Returns a result dict."""
    copied = 0
    missing_files = []

    # 1. Annotation JSON
    src_json = source_asset_dir / f"{asset_hash}_annotation.json"
    dst_json = subset_asset_dir / f"{asset_hash}_annotation.json"
    if _copy_file(src_json, dst_json, skip_existing, dry_run):
        copied += 1
    elif not src_json.exists():
        missing_files.append(src_json.name)

    # 2. Render PNGs
    for png in RENDER_PNGS:
        src_png = source_asset_dir / png
        dst_png = subset_asset_dir / png
        if _copy_file(src_png, dst_png, skip_existing, dry_run):
            copied += 1
        elif not src_png.exists():
            missing_files.append(png)

    # 3. GLB
    src_glb = source_asset_dir / "glb" / f"{asset_hash}.glb"
    dst_glb = subset_asset_dir / "glb" / f"{asset_hash}.glb"
    if _copy_file(src_glb, dst_glb, skip_existing, dry_run):
        copied += 1
    elif not src_glb.exists():
        missing_files.append(f"glb/{asset_hash}.glb")

    return {"copied": copied, "missing_files": missing_files}


def run(
    subset_root: Path,
    source_root: Path,
    dry_run: bool,
    workers: int,
    skip_existing: bool,
    dedup_report: Path | None = None,
):
    # Load dedup lookup if provided.
    dedup_lookup: dict[str, list[tuple[str, str]]] = {}
    if dedup_report:
        dedup_lookup = _load_dedup_lookup(dedup_report)

    direct_matched = 0
    dedup_fallback = 0
    skipped_no_source = 0
    errors = 0
    total_copied = 0
    skipped_hashes = []

    # Task tuples: (subset_dir, source_dir, asset_hash, source_hash_or_None)
    # source_hash is None for direct matches, set for dedup fallback.
    tasks: list[tuple[Path, Path, str, str | None]] = []

    for category_dir in sorted(subset_root.iterdir()):
        if not category_dir.is_dir():
            continue
        cat_name = category_dir.name
        if cat_name in SKIP_CATEGORIES:
            LOG.info("Skipping nested directory: %s", cat_name)
            continue
        source_cat = CATEGORY_RENAME.get(cat_name, cat_name)
        source_cat_dir = source_root / source_cat
        for asset_dir in sorted(category_dir.iterdir()):
            if not asset_dir.is_dir():
                continue
            asset_hash = asset_dir.name
            src_asset_dir = source_cat_dir / asset_hash
            if src_asset_dir.is_dir():
                # Direct match.
                tasks.append((asset_dir, src_asset_dir, asset_hash, None))
            elif dedup_lookup:
                # Try dedup fallback.
                rep_dir = _find_dedup_representative(
                    asset_hash, cat_name, dedup_lookup, source_root,
                )
                if rep_dir is not None:
                    tasks.append((asset_dir, rep_dir, asset_hash, rep_dir.name))
                else:
                    skipped_no_source += 1
                    skipped_hashes.append(f"{cat_name}/{asset_hash}")
            else:
                skipped_no_source += 1
                skipped_hashes.append(f"{cat_name}/{asset_hash}")

    direct_count = sum(1 for t in tasks if t[3] is None)
    dedup_count = sum(1 for t in tasks if t[3] is not None)
    LOG.info(
        "Found %d assets to enrich (direct=%d, dedup_fallback=%d), %d missing from source",
        len(tasks), direct_count, dedup_count, skipped_no_source,
    )

    def _worker(args):
        subset_dir, src_dir, h, src_hash = args
        try:
            if src_hash is None:
                return {
                    **_enrich_one(subset_dir, src_dir, h, skip_existing, dry_run),
                    "mode": "direct",
                }
            else:
                return {
                    **_enrich_one_dedup(subset_dir, src_dir, h, src_hash, skip_existing, dry_run),
                    "mode": "dedup",
                }
        except Exception as exc:
            LOG.error("Error enriching %s: %s", subset_dir, exc)
            return {"error": str(exc)}

    with ThreadPoolExecutor(max_workers=workers) as pool:
        futures = {pool.submit(_worker, t): t for t in tasks}
        for fut in as_completed(futures):
            result = fut.result()
            if "error" in result:
                errors += 1
            else:
                if result["mode"] == "direct":
                    direct_matched += 1
                else:
                    dedup_fallback += 1
                total_copied += result["copied"]
                if result["missing_files"]:
                    task_info = futures[fut]
                    LOG.debug(
                        "Missing files in source %s: %s",
                        task_info[1],
                        result["missing_files"],
                    )

    action = "Would copy" if dry_run else "Copied"
    print(f"\n{'=== DRY RUN ===' if dry_run else '=== DONE ==='}")
    print(f"Direct matched:          {direct_matched}")
    print(f"Dedup fallback:          {dedup_fallback}")
    print(f"{action} files:          {total_copied}")
    print(f"Skipped (no source):     {skipped_no_source}")
    print(f"Errors:                  {errors}")

    if skipped_hashes and LOG.isEnabledFor(logging.DEBUG):
        print("\nSkipped asset hashes (first 20):")
        for h in skipped_hashes[:20]:
            print(f"  {h}")
        if len(skipped_hashes) > 20:
            print(f"  ... and {len(skipped_hashes) - 20} more")


def main():
    parser = argparse.ArgumentParser(
        description="Enrich subset assets with annotation, PNGs, and GLB from test1."
    )
    parser.add_argument(
        "--subset-root",
        type=Path,
        required=True,
        help="Path to subset GRScenes_assets directory",
    )
    parser.add_argument(
        "--source-root",
        type=Path,
        required=True,
        help="Path to test1 GRScenes_assets directory",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Report what would be done without copying",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=8,
        help="Number of parallel copy threads (default: 8)",
    )
    parser.add_argument(
        "--skip-existing",
        action="store_true",
        help="Skip files that already exist in subset (default: overwrite)",
    )
    parser.add_argument(
        "--dedup-report",
        type=Path,
        default=None,
        help="Path to dedup report JSON for fallback lookup of deduplicated assets",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Enable debug logging",
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(levelname)s %(message)s",
    )

    if not args.subset_root.is_dir():
        parser.error(f"Subset root not found: {args.subset_root}")
    if not args.source_root.is_dir():
        parser.error(f"Source root not found: {args.source_root}")
    if args.dedup_report and not args.dedup_report.is_file():
        parser.error(f"Dedup report not found: {args.dedup_report}")

    run(
        subset_root=args.subset_root,
        source_root=args.source_root,
        dry_run=args.dry_run,
        workers=args.workers,
        skip_existing=args.skip_existing,
        dedup_report=args.dedup_report,
    )


if __name__ == "__main__":
    main()
