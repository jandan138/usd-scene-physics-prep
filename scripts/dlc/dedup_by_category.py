#!/usr/bin/env python3
"""DLC chunk wrapper: split asset categories across workers and run dedup report.

Each DLC worker gets a subset of the 79 asset categories (sorted by name),
then calls report_asset_mesh_dedup.py for each category sequentially.

Usage (inside DLC container via run_task.sh dedup mode):
    python scripts/dlc/dedup_by_category.py \
        --chunk-id 0 --chunk-total 10 \
        --assets-root GRScenes-test1/GRScenes_assets \
        --out-dir check_reports/dedup_tolerance \
        --merge-tolerance 0.005 --float-quantize-eps 1e-2

Local test (single category, chunk 0 of 79):
    ./scripts/isaac_python.sh scripts/dlc/dedup_by_category.py \
        --chunk-id 0 --chunk-total 79 \
        --assets-root GRScenes-test1/GRScenes_assets \
        --out-dir /tmp/dedup_test \
        --merge-tolerance 0.005 --float-quantize-eps 1e-2
"""

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path


def get_categories(assets_root: str) -> list[str]:
    """Return sorted list of category directory names."""
    cats = []
    for entry in os.scandir(assets_root):
        if entry.is_dir():
            cats.append(entry.name)
    return sorted(cats)


def chunk_slice(items: list, chunk_id: int, chunk_total: int) -> list:
    """Return the slice of items assigned to this chunk."""
    n = len(items)
    base = n // chunk_total
    remainder = n % chunk_total
    # First 'remainder' chunks get (base+1) items, rest get base
    if chunk_id < remainder:
        start = chunk_id * (base + 1)
        end = start + base + 1
    else:
        start = remainder * (base + 1) + (chunk_id - remainder) * base
        end = start + base
    return items[start:end]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="DLC chunk wrapper: run dedup report per category"
    )
    parser.add_argument("--chunk-id", type=int, required=True)
    parser.add_argument("--chunk-total", type=int, required=True)
    parser.add_argument(
        "--assets-root", required=True,
        help="Assets root (e.g. GRScenes-test1/GRScenes_assets)",
    )
    parser.add_argument(
        "--out-dir", default="check_reports/dedup_tolerance",
        help="Output root; reports go to <out-dir>/<category>/",
    )
    parser.add_argument("--merge-tolerance", type=float, default=0.005)
    parser.add_argument("--float-quantize-eps", type=float, default=1e-2)
    parser.add_argument("--progress-every", type=int, default=100)
    args = parser.parse_args()

    assets_root = os.path.abspath(args.assets_root)
    if not os.path.isdir(assets_root):
        print(f"ERROR: assets-root not found: {assets_root}", file=sys.stderr)
        return 1

    all_cats = get_categories(assets_root)
    my_cats = chunk_slice(all_cats, args.chunk_id, args.chunk_total)

    print(
        f"Chunk {args.chunk_id}/{args.chunk_total}: "
        f"{len(my_cats)} categories out of {len(all_cats)} total",
        flush=True,
    )
    print(f"  Categories: {my_cats}", flush=True)

    if not my_cats:
        print("No categories assigned to this chunk. Done.", flush=True)
        return 0

    # Locate report_asset_mesh_dedup.py relative to this file
    repo_root = Path(__file__).resolve().parents[2]
    dedup_script = repo_root / "scripts" / "report_asset_mesh_dedup.py"

    failed = []
    for i, cat in enumerate(my_cats):
        cat_root = os.path.join(assets_root, cat)
        cat_out = os.path.join(args.out_dir, cat)

        print(
            f"\n[{i+1}/{len(my_cats)}] Processing category: {cat}",
            flush=True,
        )
        t0 = time.time()

        cmd = [
            sys.executable,
            str(dedup_script),
            "--assets-root", cat_root,
            "--out-dir", cat_out,
            "--dataset", cat,
            "--float-quantize-eps", str(args.float_quantize_eps),
            "--merge-tolerance", str(args.merge_tolerance),
            "--progress-every", str(args.progress_every),
        ]

        try:
            subprocess.run(cmd, check=True)
            elapsed = time.time() - t0
            print(f"  Done: {cat} ({elapsed:.1f}s)", flush=True)
        except subprocess.CalledProcessError as exc:
            elapsed = time.time() - t0
            print(
                f"  FAILED: {cat} (exit {exc.returncode}, {elapsed:.1f}s)",
                file=sys.stderr, flush=True,
            )
            failed.append(cat)

    print(f"\n=== Chunk {args.chunk_id} summary ===", flush=True)
    print(f"  Processed: {len(my_cats)}", flush=True)
    print(f"  Failed: {len(failed)} {failed}", flush=True)

    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
