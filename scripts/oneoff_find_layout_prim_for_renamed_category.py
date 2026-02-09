#!/usr/bin/env python3
"""Find an example layout.usd prim that references a renamed GRScenes_assets category.

Usage:
  ./scripts/isaac_python.sh scripts/oneoff_find_layout_prim_for_renamed_category.py \
    --subset-root GRScenes-test1 --category night_stand

Prints:
  - layout path
  - prim path in that layout
  - kind (reference/payload)
  - assetPath (should contain GRScenes_assets/<category>/)

This is a one-off helper for manual spot-checking.
"""

from __future__ import annotations

import argparse
import os

from pxr import Usd


def _iter_layouts(scenes_root: str):
    for dirpath, _dirnames, filenames in os.walk(scenes_root):
        for fn in filenames:
            if fn == "layout.usd":
                yield os.path.join(dirpath, fn)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--subset-root", required=True)
    ap.add_argument(
        "--category",
        required=True,
        help="Target category name under GRScenes_assets (e.g. night_stand)",
    )
    ap.add_argument(
        "--print-asset-default-prim",
        action="store_true",
        help="Also open the referenced asset stage and print its default prim path.",
    )
    args = ap.parse_args()

    scenes_root = os.path.join(args.subset_root, "GRScenes100")
    needle = f"GRScenes_assets/{args.category}/"

    layouts = sorted(_iter_layouts(scenes_root))
    for layout in layouts:
        stage = Usd.Stage.Open(layout)
        if stage is None:
            continue

        for prim in stage.Traverse():
            if not prim:
                continue

            refs = prim.GetMetadata("references")
            if refs:
                for ref in refs.GetAddedOrExplicitItems():
                    asset_path = getattr(ref, "assetPath", "") or ""
                    if needle in asset_path:
                        print("layout:", layout)
                        print("prim_path:", prim.GetPath())
                        print("kind: reference")
                        print("assetPath:", asset_path)
                        if args.print_asset_default_prim:
                            asset_abs = os.path.normpath(os.path.join(os.path.dirname(layout), asset_path))
                            asset_stage = Usd.Stage.Open(asset_abs)
                            print("asset_abs:", asset_abs)
                            if asset_stage is None:
                                print("asset_default_prim:", None)
                            else:
                                dp = asset_stage.GetDefaultPrim()
                                print("asset_default_prim:", dp.GetPath() if dp else None)
                        return 0

            payload = prim.GetMetadata("payload")
            if payload:
                for pl in payload.GetAddedOrExplicitItems():
                    asset_path = getattr(pl, "assetPath", "") or ""
                    if needle in asset_path:
                        print("layout:", layout)
                        print("prim_path:", prim.GetPath())
                        print("kind: payload")
                        print("assetPath:", asset_path)
                        if args.print_asset_default_prim:
                            asset_abs = os.path.normpath(os.path.join(os.path.dirname(layout), asset_path))
                            asset_stage = Usd.Stage.Open(asset_abs)
                            print("asset_abs:", asset_abs)
                            if asset_stage is None:
                                print("asset_default_prim:", None)
                            else:
                                dp = asset_stage.GetDefaultPrim()
                                print("asset_default_prim:", dp.GetPath() if dp else None)
                        return 0

    print("NO_MATCH")
    print("needle:", needle)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
