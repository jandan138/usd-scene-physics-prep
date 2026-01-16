#!/usr/bin/env python3

import argparse
import json
import os
from typing import Dict, Iterable, List, Tuple


_USD_EXTS = (".usd", ".usda", ".usdc")


def iter_usd_dirs(root: str) -> Iterable[str]:
    for dirpath, dirnames, filenames in os.walk(root):
        # Skip the centralized material library itself
        if os.path.normpath(dirpath).endswith(os.path.join("Material", "mdl", "textures")):
            dirnames[:] = []
            continue

        # Heuristic: USD directories are named 'usd' and contain usd files
        if os.path.basename(dirpath) != "usd":
            continue
        if any(fn.lower().endswith(_USD_EXTS) for fn in filenames):
            yield dirpath


def compute_rel_target(usd_dir: str, textures_root: str) -> str:
    rel = os.path.relpath(textures_root, usd_dir)
    return rel.replace(os.sep, "/")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=(
            "Create per-USD 'textures' symlinks pointing to centralized Material/mdl/textures. "
            "This is optional but useful so opening a USD inside its folder can resolve textures." 
        )
    )
    ap.add_argument(
        "--root",
        required=True,
        help="Dataset root (contains Material/mdl/textures and GRScenes_assets/...)",
    )
    ap.add_argument("--dry-run", action="store_true", help="Do not modify filesystem")
    ap.add_argument(
        "--force",
        action="store_true",
        help="Replace existing symlink if it points elsewhere (won't delete real directories)",
    )
    ap.add_argument(
        "--quiet",
        action="store_true",
        help="Only print final JSON summary",
    )

    args = ap.parse_args()

    root = os.path.abspath(args.root)
    textures_root = os.path.join(root, "Material", "mdl", "textures")
    if not os.path.isdir(textures_root):
        print(f"[ERROR] Missing centralized textures dir: {textures_root}")
        return 2

    created = 0
    fixed = 0
    skipped = 0
    warnings: List[Dict[str, str]] = []
    scanned = 0

    for usd_dir in sorted(iter_usd_dirs(root)):
        scanned += 1
        link_path = os.path.join(usd_dir, "textures")
        desired = compute_rel_target(usd_dir, textures_root)

        if os.path.lexists(link_path):
            if os.path.islink(link_path):
                try:
                    cur = os.readlink(link_path)
                except OSError:
                    cur = ""

                if cur == desired:
                    skipped += 1
                    continue

                if not args.force:
                    skipped += 1
                    warnings.append(
                        {
                            "usd_dir": usd_dir,
                            "action": "skip_existing_symlink",
                            "current": cur,
                            "desired": desired,
                        }
                    )
                    continue

                if not args.dry_run:
                    os.unlink(link_path)
                    os.symlink(desired, link_path)
                fixed += 1
                if not args.quiet:
                    print(f"[fixed] {link_path} -> {desired}")
                continue

            # Exists but not a symlink: do not delete automatically.
            skipped += 1
            warnings.append(
                {
                    "usd_dir": usd_dir,
                    "action": "skip_existing_non_symlink",
                    "path": link_path,
                }
            )
            continue

        if not args.dry_run:
            os.symlink(desired, link_path)
        created += 1
        if not args.quiet:
            print(f"[created] {link_path} -> {desired}")

    summary = {
        "root": root,
        "textures_root": textures_root,
        "scanned_usd_dirs": scanned,
        "created": created,
        "fixed": fixed,
        "skipped": skipped,
        "warnings": warnings[:50],
        "warnings_truncated": len(warnings) > 50,
        "dry_run": bool(args.dry_run),
        "force": bool(args.force),
    }

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
