#!/usr/bin/env python3
"""Compare desk prim transforms pre vs post dedup."""

import json
import os
from pxr import Usd, UsdGeom, Sdf, Gf

PROJECT = "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep"
SCENE_ID = "MV7J6NIKTKJZ2AABAAAAADA8_usd"
LAYOUT_DIR = f"{PROJECT}/GRScenes-test0-rebuilt-normalized/GRScenes100/home/{SCENE_ID}"
MODEL_PRIM_PATH = "/Root/Meshes/Animation/desk/model_64a72e14d9b7e061a46e6fc802452525_0"
TARGET_HASH = "64a72e14d9b7e061a46e6fc802452525"
CANON_HASH = "08331d5e4c3ca970076b9c0f0a02d799"


def get_full_xform(stage, prim_path):
    """Get the xformOp:transform or composed local transform."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        return None, None
    xformable = UsdGeom.Xformable(prim)
    ops = xformable.GetOrderedXformOps()
    op_dict = {}
    for op in ops:
        name = op.GetOpName()
        val = op.Get()
        if isinstance(val, Gf.Matrix4d):
            op_dict[name] = val
        elif isinstance(val, (Gf.Vec3d, Gf.Vec3f)):
            op_dict[name] = list(val)
        elif isinstance(val, (Gf.Quatd, Gf.Quatf)):
            op_dict[name] = [val.GetReal()] + list(val.GetImaginary())
        else:
            op_dict[name] = val

    local_xform = xformable.GetLocalTransformation()
    return op_dict, local_xform


def get_ref_asset(stage, prim_path):
    """Get the reference asset path on a prim."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        return None
    for spec in prim.GetPrimStack():
        for ref in spec.referenceList.prependedItems:
            return ref.assetPath
        for ref in spec.referenceList.appendedItems:
            return ref.assetPath
    return None


def mat4_to_list(m):
    return [[m[r][c] for c in range(4)] for r in range(4)]


def print_layout_info(label, layout_path):
    print(f"\n--- {label} ---")
    print(f"  File: {layout_path}")
    if not os.path.exists(layout_path):
        print("  NOT FOUND")
        return

    stage = Usd.Stage.Open(layout_path, Usd.Stage.LoadNone)
    if not stage:
        print("  FAILED TO OPEN")
        return

    ref_asset = get_ref_asset(stage, MODEL_PRIM_PATH)
    print(f"  Reference asset: {ref_asset}")

    ops, local_xform = get_full_xform(stage, MODEL_PRIM_PATH)
    if ops:
        print(f"  XformOps:")
        for k, v in ops.items():
            if isinstance(v, Gf.Matrix4d):
                print(f"    {k}:")
                for row in mat4_to_list(v):
                    print(f"      {row}")
            else:
                print(f"    {k}: {v}")
    if local_xform:
        print(f"  Local transform (composed):")
        for row in mat4_to_list(local_xform):
            print(f"    {row}")

    # Check Instance child xform
    instance_path = MODEL_PRIM_PATH + "/Instance"
    ops2, local2 = get_full_xform(stage, instance_path)
    if ops2:
        print(f"  Instance child XformOps:")
        for k, v in ops2.items():
            if isinstance(v, Gf.Matrix4d):
                print(f"    {k}:")
                for row in mat4_to_list(v):
                    print(f"      {row}")
            else:
                print(f"    {k}: {v}")


# Compare layouts
layouts = [
    ("CURRENT", f"{LAYOUT_DIR}/layout.usd"),
    ("PRE-C1-NORMALIZE-ONLY", f"{LAYOUT_DIR}/layout.pre_c1_normalize_only.20260315_chain_fix_v1.usd"),
    ("PRE-V2-VMATRIX (desk run)", f"{LAYOUT_DIR}/layout.pre_c1_v2_vmatrix.20260319_172707.usd"),
]

for label, path in layouts:
    print_layout_info(label, path)

# Also check the two asset USD files to get their internal transforms
print(f"\n{'='*60}")
print("ASSET INTERNAL TRANSFORMS")
print(f"{'='*60}")

for hash_val, label in [(TARGET_HASH, "OLD (duplicate)"), (CANON_HASH, "CANONICAL")]:
    asset_path = f"{PROJECT}/GRScenes-test0-rebuilt-normalized/GRScenes_assets/desk/{hash_val}/usd/{hash_val}.usd"
    print(f"\n--- {label}: {hash_val} ---")
    print(f"  Path: {asset_path}")
    if not os.path.exists(asset_path):
        print("  NOT FOUND (may have been deleted by dedup)")
        # Check backup
        bak_dir = f"{PROJECT}/GRScenes-test0-rebuilt-normalized_bak/_dedup_assets"
        import glob
        matches = glob.glob(f"{bak_dir}/**/desk/{hash_val}/**/*.usd", recursive=True)
        if matches:
            asset_path = matches[0]
            print(f"  Found in backup: {asset_path}")
        else:
            continue

    stage = Usd.Stage.Open(asset_path, Usd.Stage.LoadNone)
    if not stage:
        print("  FAILED TO OPEN")
        continue

    # Find Instance prim
    for prim in stage.TraverseAll():
        path_str = str(prim.GetPath())
        if "Instance" in path_str and prim.GetTypeName() == "Xform":
            ops, local = get_full_xform(stage, path_str)
            if ops:
                print(f"  {path_str} XformOps:")
                for k, v in ops.items():
                    if isinstance(v, Gf.Matrix4d):
                        print(f"    {k}:")
                        for row in mat4_to_list(v):
                            print(f"      {row}")
                    else:
                        print(f"    {k}: {v}")
            break  # Only first Instance

    # Also check root prim
    root = stage.GetDefaultPrim()
    if root:
        ops, local = get_full_xform(stage, str(root.GetPath()))
        if ops:
            print(f"  Root ({root.GetPath()}) XformOps:")
            for k, v in ops.items():
                if isinstance(v, Gf.Matrix4d):
                    print(f"    {k}:")
                    for row in mat4_to_list(v):
                        print(f"      {row}")
                else:
                    print(f"    {k}: {v}")

print("\nDone.")
