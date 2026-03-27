#!/usr/bin/env python3
"""Inspect desk prim in layout USD for dedup investigation."""

import json
import os
from pxr import Usd, UsdGeom, Sdf, Gf

PROJECT = "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep"
SCENE_ID = "MV7J6NIKTKJZ2AABAAAAADA8_usd"
LAYOUT_DIR = f"{PROJECT}/GRScenes-test0-rebuilt-normalized/GRScenes100/home/{SCENE_ID}"
TARGET_PRIM = "/Root/Meshes/Animation/desk/model_64a72e14d9b7e061a46e6fc802452525_0/Instance/Group_00/Component_9"
TARGET_HASH = "64a72e14d9b7e061a46e6fc802452525"
CANON_HASH = "08331d5e4c3ca970076b9c0f0a02d799"


def get_xform_info(prim):
    """Extract xformOp values from a prim."""
    if not prim:
        return None
    xformable = UsdGeom.Xformable(prim)
    ops = xformable.GetOrderedXformOps()
    result = {}
    for op in ops:
        name = op.GetOpName()
        val = op.Get()
        if val is not None:
            if isinstance(val, (Gf.Vec3d, Gf.Vec3f)):
                result[name] = list(val)
            elif isinstance(val, (Gf.Quatd, Gf.Quatf)):
                result[name] = [val.GetReal()] + list(val.GetImaginary())
            elif isinstance(val, Gf.Matrix4d):
                result[name] = [[val[r][c] for c in range(4)] for r in range(4)]
            else:
                result[name] = str(val)
    return result


def get_references(prim):
    """Get reference info from a prim."""
    refs = []
    prim_spec = prim.GetPrimStack()
    for spec in prim_spec:
        ref_list = spec.referenceList
        for ref in ref_list.prependedItems:
            refs.append({
                "asset": ref.assetPath,
                "prim_path": str(ref.primPath) if ref.primPath else None
            })
        for ref in ref_list.appendedItems:
            refs.append({
                "asset": ref.assetPath,
                "prim_path": str(ref.primPath) if ref.primPath else None
            })
    return refs


def inspect_layout(layout_path, label):
    """Inspect a single layout file."""
    print(f"\n{'='*80}")
    print(f"LAYOUT: {label}")
    print(f"Path: {layout_path}")
    print(f"{'='*80}")

    if not os.path.exists(layout_path):
        print("  FILE NOT FOUND")
        return

    stage = Usd.Stage.Open(layout_path)
    if not stage:
        print("  FAILED TO OPEN")
        return

    # Walk from target prim up to model hash level
    target = stage.GetPrimAtPath(TARGET_PRIM)
    if not target:
        print(f"  Target prim NOT FOUND: {TARGET_PRIM}")
        # Try to find the model hash ancestor
        model_path = f"/Root/Meshes/Animation/desk/model_{TARGET_HASH}_0"
        model_prim = stage.GetPrimAtPath(model_path)
        if model_prim:
            print(f"  But model-level prim EXISTS: {model_path}")
            print(f"  Children: {[str(c.GetPath()) for c in model_prim.GetAllChildren()]}")
        else:
            print(f"  Model-level prim also NOT FOUND: {model_path}")
            # Search for any desk prim with target hash
            desk_path = "/Root/Meshes/Animation/desk"
            desk_prim = stage.GetPrimAtPath(desk_path)
            if desk_prim:
                print(f"  Desk prims found:")
                for child in desk_prim.GetAllChildren():
                    p = str(child.GetPath())
                    if TARGET_HASH in p or CANON_HASH in p:
                        print(f"    {p}")
        return

    # Walk ancestors from target up
    path = TARGET_PRIM
    parts = path.split("/")
    for i in range(len(parts), 1, -1):
        ancestor_path = "/".join(parts[:i])
        if not ancestor_path:
            continue
        prim = stage.GetPrimAtPath(ancestor_path)
        if not prim:
            continue

        print(f"\n  Prim: {ancestor_path}")
        print(f"    Type: {prim.GetTypeName()}")

        # References
        refs = get_references(prim)
        if refs:
            print(f"    References:")
            for r in refs:
                print(f"      asset={r['asset']}, primPath={r['prim_path']}")

        # Xform
        xform = get_xform_info(prim)
        if xform:
            print(f"    XformOps:")
            for k, v in xform.items():
                print(f"      {k}: {v}")

        # Stop at a reasonable ancestor
        if "model_" in ancestor_path.split("/")[-1]:
            break

    # Also check for the canonical hash reference
    print(f"\n  --- Searching for canonical hash {CANON_HASH} ---")
    found_canon = False
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if CANON_HASH in p:
            print(f"  Found canonical hash prim: {p}")
            found_canon = True
            refs = get_references(prim)
            if refs:
                for r in refs:
                    print(f"    ref: asset={r['asset']}, primPath={r['prim_path']}")
            xform = get_xform_info(prim)
            if xform:
                for k, v in xform.items():
                    print(f"    {k}: {v}")
    if not found_canon:
        print("  No prims with canonical hash found")


# ============================================================
# 1. Current layout
# ============================================================
inspect_layout(f"{LAYOUT_DIR}/layout.usd", "CURRENT (layout.usd)")

# ============================================================
# 2. Pre-dedup layouts (earliest snapshots)
# ============================================================
# The normalize-only snapshot is the earliest pre-dedup state
pre_normalize = f"{LAYOUT_DIR}/layout.pre_c1_normalize_only.20260315_chain_fix_v1.usd"
inspect_layout(pre_normalize, "PRE-DEDUP (pre_c1_normalize_only)")

# First union_3way snapshot
pre_union = f"{LAYOUT_DIR}/layout.pre_c1_test0_rebuilt_union_3way.20260315_162636.usd"
inspect_layout(pre_union, "PRE-UNION-3WAY (earliest)")

# V2 vmatrix desk snapshot - find the one closest to desk run (17:18)
pre_v2_desk = f"{LAYOUT_DIR}/layout.pre_c1_v2_vmatrix.20260319_172707.usd"
inspect_layout(pre_v2_desk, "PRE-V2-VMATRIX (desk run timestamp 20260319_172707)")

# ============================================================
# 3. C1 report details
# ============================================================
print(f"\n{'='*80}")
print("C1 REPORT DETAILS")
print(f"{'='*80}")

# geom_only mapping
mapping_file = f"{PROJECT}/check_reports/test0_rebuilt_dedup/c1_bulk_v2/desk_geom_only_mapping.json"
with open(mapping_file) as f:
    mapping = json.load(f)

key = f"GRScenes-test0-rebuilt-normalized/GRScenes_assets/desk/{TARGET_HASH}/usd/{TARGET_HASH}.usd"
if key in mapping:
    print(f"\n  Mapping entry found (geom_only_mapping.json):")
    print(f"    Source: {key}")
    print(f"    Target: {mapping[key]}")
else:
    print(f"\n  Hash {TARGET_HASH} NOT in geom_only_mapping")

# Check stats
stats_file = f"{PROJECT}/check_reports/test0_rebuilt_dedup/c1_bulk_v2/desk_geom_only_mapping.stats.json"
with open(stats_file) as f:
    stats = json.load(f)
print(f"\n  desk_geom_only_mapping.stats: {json.dumps(stats, indent=4)}")

# Check step6 promote report for our scene
promote_file = f"{PROJECT}/check_reports/test0_rebuilt_dedup/c1_bulk_v2/desk_bulk_step6_v1/promote_to_layout_usd_report.json"
with open(promote_file) as f:
    promote = json.load(f)
for item in promote.get("items", []):
    if SCENE_ID in item.get("file", ""):
        print(f"\n  Promote report entry for {SCENE_ID}:")
        print(f"    {json.dumps(item, indent=4)}")

# ============================================================
# 4. V matrix details from topo_filesize report
# ============================================================
print(f"\n{'='*80}")
print("V MATRIX / DEDUP MODE DETAILS")
print(f"{'='*80}")

# The pair was in topo_filesize mode
print(f"\n  Dedup mode: topo_filesize (sig: topo_filesize_merge_4)")
print(f"  Pair: {TARGET_HASH} -> {CANON_HASH}")
print(f"  (canonical = {CANON_HASH}, duplicate = {TARGET_HASH})")

# Check if there's a V matrix computed for this pair
# Look in the c1_bulk_v2 batch report
batch_dir = f"{PROJECT}/check_reports/test0_rebuilt_dedup/c1_bulk_v2/desk_bulk_batch_v1"
if os.path.isdir(batch_dir):
    print(f"\n  Batch dir contents: {os.listdir(batch_dir)}")
    for fn in os.listdir(batch_dir):
        fp = os.path.join(batch_dir, fn)
        if fn.endswith('.json'):
            with open(fp) as f:
                data = json.load(f)
            content = json.dumps(data)
            if TARGET_HASH in content:
                print(f"\n  Found target in {fn}:")
                if isinstance(data, list):
                    for entry in data:
                        if TARGET_HASH in json.dumps(entry):
                            print(f"    {json.dumps(entry, indent=4)[:2000]}")
                elif isinstance(data, dict):
                    for k, v in data.items():
                        if TARGET_HASH in k or TARGET_HASH in json.dumps(v):
                            print(f"    {k}: {json.dumps(v, indent=4)[:2000]}")

# Check the post_promote layout scan
post_promote_file = f"{PROJECT}/check_reports/test0_rebuilt_dedup/c1_bulk_v2/desk_bulk_step6_v1/post_promote_layout_scan_pxr.json"
if os.path.exists(post_promote_file):
    with open(post_promote_file) as f:
        post_data = json.load(f)
    content = json.dumps(post_data)
    if TARGET_HASH in content or CANON_HASH in content:
        print(f"\n  post_promote_layout_scan entries with target/canon hash:")
        if isinstance(post_data, dict):
            for k, v in post_data.items():
                if TARGET_HASH in json.dumps(v) or CANON_HASH in json.dumps(v):
                    if TARGET_HASH in k or CANON_HASH in k or TARGET_HASH in json.dumps(v)[:200]:
                        print(f"    {k}: {json.dumps(v, indent=4)[:1000]}")

print("\n\nDone.")
