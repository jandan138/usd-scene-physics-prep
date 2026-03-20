#!/usr/bin/env python3
"""
Batch-fix absolute MDL/texture paths in normalized asset USDs.

Rewrites all absolute paths pointing to the original dataset's Material/mdl/
directory into relative paths (e.g., ../../../../Material/mdl/xxx.mdl),
making the normalized dataset portable.

Usage:
    python scripts/fix_normalized_mdl_paths.py \
        --dataset-root /path/to/GRScenes-test1-normalized \
        [--materials-dir /path/to/GRScenes-test1-normalized/Material/mdl] \
        [--category bottle] \
        [--dry-run] \
        [--workers 8]
"""

import argparse
import glob
import json
import logging
import os
import sys
import time
from collections import defaultdict
from multiprocessing import Pool
from typing import Dict, List, Optional, Tuple

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger(__name__)


def _find_asset_usds(dataset_root: str, category: Optional[str] = None) -> List[str]:
    """Find all asset USD files under GRScenes_assets/."""
    assets_root = os.path.join(dataset_root, "GRScenes_assets")
    if category:
        pattern = os.path.join(assets_root, category, "*/usd/*.usd")
    else:
        pattern = os.path.join(assets_root, "*/*/usd/*.usd")
    return sorted(glob.glob(pattern))


def _extract_category(usd_path: str) -> str:
    """Extract category from asset path: .../GRScenes_assets/<category>/<hash>/usd/<hash>.usd"""
    parts = usd_path.replace("\\", "/").split("/")
    try:
        idx = parts.index("usd")
        return parts[idx - 2] if idx >= 2 else "unknown"
    except ValueError:
        return "unknown"


def _rewrite_single_asset(args: Tuple[str, str, bool]) -> Dict:
    """Rewrite MDL paths for a single asset USD. Worker function for Pool."""
    usd_path, materials_dir, dry_run = args
    category = _extract_category(usd_path)
    result = {
        "path": usd_path,
        "category": category,
        "updated": 0,
        "error": None,
    }

    try:
        from pxr import Usd, Sdf

        stage = Usd.Stage.Open(usd_path)
        if stage is None:
            result["error"] = "Failed to open stage"
            return result

        root = stage.GetRootLayer()
        usd_dir = os.path.dirname(usd_path)
        materials_base = os.path.relpath(materials_dir, usd_dir)
        updated = 0

        for prim in stage.Traverse():
            for attr in prim.GetAttributes():
                t = attr.GetTypeName()

                if t == Sdf.ValueTypeNames.Asset:
                    val = attr.Get()
                    if not isinstance(val, Sdf.AssetPath) or not val.path:
                        continue
                    newp = _rewrite_path(val.path, materials_base)
                    if newp and newp != val.path:
                        if not dry_run:
                            attr.Set(Sdf.AssetPath(newp))
                        updated += 1

                elif t == Sdf.ValueTypeNames.AssetArray:
                    val = attr.Get()
                    if not val:
                        continue
                    changed = False
                    new_vals = []
                    for ap in val:
                        if not isinstance(ap, Sdf.AssetPath) or not ap.path:
                            new_vals.append(ap)
                            continue
                        newp = _rewrite_path(ap.path, materials_base)
                        if newp and newp != ap.path:
                            changed = True
                            new_vals.append(Sdf.AssetPath(newp))
                        else:
                            new_vals.append(ap)
                    if changed:
                        if not dry_run:
                            attr.Set(new_vals)
                        updated += 1

                elif t in (Sdf.ValueTypeNames.String, Sdf.ValueTypeNames.Token):
                    val = attr.Get()
                    if isinstance(val, str) and "::.::Materials::" in val:
                        name = val.split("::.::Materials::", 1)[1]
                        newp = _posix_join(materials_base, name + ".mdl")
                        if not dry_run:
                            attr.Set(Sdf.AssetPath(newp))
                        updated += 1

        if updated > 0 and not dry_run:
            root.Export(usd_path)

        result["updated"] = updated

    except Exception as e:
        result["error"] = str(e)

    return result


def _posix_join(base: str, rest: str) -> str:
    """Join paths in POSIX style, normalize Textures -> textures."""
    path = os.path.join(base, rest).replace("\\", "/")
    if "/Textures/" in path:
        path = path.replace("/Textures/", "/textures/")
    return path


def _rewrite_path(path: str, materials_base: str) -> Optional[str]:
    """Rewrite a single asset path if it references Material/mdl.
    Returns the new path, or None if no rewrite needed."""

    # Skip URLs (e.g., twinbru S3)
    if path.startswith("http://") or path.startswith("https://"):
        return None

    # Texture paths: extract filename after /textures/ or /Textures/
    tex_idx = path.lower().rfind("/textures/")
    if tex_idx >= 0:
        filename = path[tex_idx + len("/textures/"):]
        return _posix_join(materials_base, "textures/" + filename)

    # MDL paths: find Material/mdl/ or Materials/ or Material/ prefix
    idx = -1
    prefix_len = 0
    if "Material/mdl/" in path:
        idx = path.find("Material/mdl/")
        prefix_len = len("Material/mdl/")
    elif "Materials/" in path:
        idx = path.find("Materials/")
        prefix_len = len("Materials/")
    elif "Material/" in path:
        idx = path.find("Material/")
        prefix_len = len("Material/")

    if idx >= 0:
        rest = path[idx + prefix_len:]
        if rest.startswith("mdl/"):
            rest = rest[4:]
        return _posix_join(materials_base, rest)

    return None


def main():
    parser = argparse.ArgumentParser(
        description="Fix absolute MDL/texture paths in normalized asset USDs"
    )
    parser.add_argument(
        "--dataset-root",
        required=True,
        help="Root of normalized dataset (e.g., GRScenes-test1-normalized/)",
    )
    parser.add_argument(
        "--materials-dir",
        default=None,
        help="Absolute path to Material/mdl dir. Default: {dataset-root}/Material/mdl",
    )
    parser.add_argument(
        "--category",
        default=None,
        help="Process only this category (e.g., bottle). Default: all categories.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Scan and report without modifying files.",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=8,
        help="Number of parallel workers (default: 8).",
    )
    parser.add_argument(
        "--report-dir",
        default="check_reports",
        help="Directory to save the JSON report (default: check_reports).",
    )
    args = parser.parse_args()

    dataset_root = os.path.abspath(args.dataset_root)
    materials_dir = args.materials_dir or os.path.join(dataset_root, "Material", "mdl")
    materials_dir = os.path.abspath(materials_dir)

    if not os.path.isdir(dataset_root):
        log.error("Dataset root not found: %s", dataset_root)
        sys.exit(1)

    if not os.path.isdir(materials_dir):
        log.warning(
            "Materials dir not found: %s -- relative paths will be computed but "
            "won't resolve until the directory exists (create symlink or copy).",
            materials_dir,
        )

    # Find assets
    usd_paths = _find_asset_usds(dataset_root, args.category)
    log.info(
        "Found %d asset USDs%s",
        len(usd_paths),
        f" (category={args.category})" if args.category else "",
    )
    if not usd_paths:
        log.warning("No assets found. Check --dataset-root and --category.")
        sys.exit(0)

    if args.dry_run:
        log.info("DRY RUN -- no files will be modified.")

    # Process in parallel
    work_items = [(p, materials_dir, args.dry_run) for p in usd_paths]
    t0 = time.time()
    total = len(usd_paths)

    workers = min(args.workers, len(usd_paths))
    results: List[Dict] = []

    # Progress tracking state
    _prog_fixed = 0
    _prog_errors = 0
    # Track per-category completion for mid-run summaries
    _cat_expected: Dict[str, int] = defaultdict(int)
    for p in usd_paths:
        _cat_expected[_extract_category(p)] += 1
    _cat_done: Dict[str, int] = defaultdict(int)
    _cat_fixed: Dict[str, int] = defaultdict(int)
    _cat_attrs: Dict[str, int] = defaultdict(int)
    _cat_errs: Dict[str, int] = defaultdict(int)
    _cat_reported = set()

    def _update_progress(done: int, r: Dict) -> None:
        nonlocal _prog_fixed, _prog_errors
        if r["error"]:
            _prog_errors += 1
            _cat_errs[r["category"]] += 1
        if r["updated"] > 0:
            _prog_fixed += 1
            _cat_fixed[r["category"]] += 1
            _cat_attrs[r["category"]] += r["updated"]
        _cat_done[r["category"]] += 1

        # Print progress bar every 50 assets or at the end
        if done % 50 == 0 or done == total:
            elapsed = time.time() - t0
            rate = done / elapsed if elapsed > 0 else 0
            eta = (total - done) / rate if rate > 0 else 0
            pct = done * 100.0 / total
            bar_width = 30
            filled = int(bar_width * done / total)
            bar = "=" * filled
            if filled < bar_width:
                bar += ">"
                bar += " " * (bar_width - filled - 1)
            else:
                bar = "=" * bar_width
            line = (
                f"\r[{bar}] {done}/{total} ({pct:.1f}%) | "
                f"{rate:.0f} assets/s | ETA: {eta:.0f}s | "
                f"Fixed: {_prog_fixed} | Errors: {_prog_errors}"
            )
            print(line, end="", flush=True, file=sys.stderr)

        # Check if a category just completed; print summary line
        cat = r["category"]
        if cat not in _cat_reported and _cat_done[cat] == _cat_expected[cat]:
            _cat_reported.add(cat)
            # Move to new line before printing category summary
            print(file=sys.stderr)
            cf = _cat_fixed.get(cat, 0)
            ca = _cat_attrs.get(cat, 0)
            ce = _cat_errs.get(cat, 0)
            log.info(
                "  [done] %-25s  %d assets, %d fixed (%d attrs), %d errors",
                cat, _cat_done[cat], cf, ca, ce,
            )

    if workers <= 1:
        for i, w in enumerate(work_items):
            r = _rewrite_single_asset(w)
            results.append(r)
            _update_progress(i + 1, r)
    else:
        with Pool(workers) as pool:
            for i, r in enumerate(pool.imap_unordered(_rewrite_single_asset, work_items, chunksize=64)):
                results.append(r)
                _update_progress(i + 1, r)

    # Ensure we move past the progress line
    print(file=sys.stderr)
    elapsed = time.time() - t0

    # Aggregate per-category stats
    cat_stats = defaultdict(lambda: {"total": 0, "updated": 0, "errors": 0, "attrs_fixed": 0})
    errors = []
    total_assets_with_updates = 0
    total_attrs = 0

    for r in results:
        cat = r["category"]
        cat_stats[cat]["total"] += 1
        if r["error"]:
            cat_stats[cat]["errors"] += 1
            errors.append({"path": r["path"], "error": r["error"]})
        elif r["updated"] > 0:
            cat_stats[cat]["updated"] += 1
            cat_stats[cat]["attrs_fixed"] += r["updated"]
            total_assets_with_updates += 1
            total_attrs += r["updated"]

    # Print summary
    mode_label = "DRY-RUN" if args.dry_run else "REWRITE"
    log.info("=" * 60)
    log.info("[%s] Complete in %.1fs", mode_label, elapsed)
    log.info("  Assets processed      : %d", len(results))
    log.info("  Assets with rewrites  : %d", total_assets_with_updates)
    log.info("  Assets with 0 rewrites: %d", len(results) - total_assets_with_updates - len(errors))
    log.info("  Assets with errors    : %d", len(errors))
    log.info("  Total attrs rewritten : %d", total_attrs)

    # Per-category breakdown
    print()
    print(f"{'Category':<30} {'Total':>8} {'Fixed':>8} {'Attrs':>8} {'Errors':>8}")
    print("-" * 66)
    for cat in sorted(cat_stats.keys()):
        s = cat_stats[cat]
        print(f"{cat:<30} {s['total']:>8} {s['updated']:>8} {s['attrs_fixed']:>8} {s['errors']:>8}")

    if errors:
        print(f"\nErrors ({len(errors)}):")
        for e in errors:
            print(f"  {e['path']}: {e['error']}")

    # Save JSON report
    report = {
        "mode": mode_label.lower(),
        "dataset_root": dataset_root,
        "materials_dir": materials_dir,
        "total_assets": len(results),
        "assets_with_updates": total_assets_with_updates,
        "total_attrs_updated": total_attrs,
        "total_errors": len(errors),
        "elapsed_seconds": round(elapsed, 1),
        "per_category": {k: dict(v) for k, v in cat_stats.items()},
        "errors": errors,
    }
    report_dir = os.path.abspath(args.report_dir)
    os.makedirs(report_dir, exist_ok=True)
    report_path = os.path.join(report_dir, "mdl_path_fix_report.json")
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    log.info("Report saved to: %s", report_path)


if __name__ == "__main__":
    main()
