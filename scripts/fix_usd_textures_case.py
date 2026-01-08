"""
Script to fix 'Textures' -> 'textures' case sensitivity issues in existing USD files.
This script scans a directory for USD files and uses the rewrite_usd_mdl_paths logic
(which now includes the case fix) to update them.
"""

import os
import argparse
import sys

# Ensure the project root is in sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
if project_root not in sys.path:
    sys.path.append(project_root)

from specs_normalizer.utils.mdl_rewrite import rewrite_usd_mdl_paths_inplace

def main():
    parser = argparse.ArgumentParser(description="Fix Textures case in USD files")
    parser.add_argument("--root", required=True, help="Root directory to scan for USD files")
    parser.add_argument("--materials-dir", required=True, help="Path to the Materials directory (used for relative path calc)")
    args = parser.parse_args()

    root_dir = args.root
    materials_dir = args.materials_dir
    
    print(f"Scanning {root_dir} for USD files...")
    
    count = 0
    fixed_count = 0
    
    # First pass: count total files
    total_files = 0
    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if file.endswith(".usd"):
                total_files += 1

    print(f"Found {total_files} USD files to process.")

    for root, dirs, files in os.walk(root_dir):
        for file in files:
            if file.endswith(".usd"):
                usd_path = os.path.join(root, file)
                count += 1
                if count % 100 == 0:
                    print(f"Processing {count}/{total_files}...")
                try:
                    # We use inplace rewrite which will trigger _posix_join and apply the Textures->textures fix
                    res = rewrite_usd_mdl_paths_inplace(
                        usd_path=usd_path,
                        materials_dir=materials_dir,
                        use_relative=True,
                        backup=False, # No backup to save space/time for this fix
                        rewrite_module_refs=True
                    )
                    if int(res["updated"]) > 0:
                        # print(f"Fixed {res['updated']} paths in: {usd_path}")
                        fixed_count += 1
                except Exception as e:
                    print(f"Error processing {usd_path}: {e}")

    print(f"Done. Fixed {fixed_count} out of {total_files} USD files.")

if __name__ == "__main__":
    main()
