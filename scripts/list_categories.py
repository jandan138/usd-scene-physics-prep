#!/usr/bin/env python3
# List all categories and their asset (uid) counts under GRScenes_assets.

import argparse
import os
import sys


def main():
    parser = argparse.ArgumentParser(
        description="List categories and asset counts under GRScenes_assets."
    )
    parser.add_argument(
        "--assets-root",
        default="GRScenes-test1/GRScenes_assets",
        help="Path to GRScenes_assets directory (default: GRScenes-test1/GRScenes_assets)",
    )
    args = parser.parse_args()

    assets_root = args.assets_root
    if not os.path.isabs(assets_root):
        # Resolve relative to the current working directory
        assets_root = os.path.join(os.getcwd(), assets_root)

    if not os.path.isdir(assets_root):
        print(f"Error: assets-root does not exist or is not a directory: {assets_root}", file=sys.stderr)
        sys.exit(1)

    # Collect categories: first-level subdirectories under assets_root
    categories = {}
    for entry in os.scandir(assets_root):
        if entry.is_dir():
            category = entry.name
            # Count uids: first-level subdirectories under the category directory
            uid_count = sum(
                1 for uid_entry in os.scandir(entry.path) if uid_entry.is_dir()
            )
            categories[category] = uid_count

    # Sort alphabetically by category name
    total_assets = 0
    for category in sorted(categories):
        count = categories[category]
        total_assets += count
        print(f"{category}: {count} assets")

    total_categories = len(categories)
    print(f"Total: {total_categories} categories, {total_assets} assets")


if __name__ == "__main__":
    main()
