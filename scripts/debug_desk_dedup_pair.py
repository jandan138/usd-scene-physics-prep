#!/usr/bin/env python3
"""Debug desk dedup pair: 64a72e14 (old) -> 08331d5e (canonical).

Diagnoses compensation bug by comparing vertices, internal transforms,
Procrustes alignment, and the actual layout transform.
"""

import sys
import os
import json
import numpy as np

# Add project root to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pxr import Gf, Usd, UsdGeom

from scripts.compute_vertex_transform import (
    extract_instance_space_vertices,
    procrustes_full,
    _get_local_matrix,
    _get_chain_transform,
    _ensure_pxr,
)

np.set_printoptions(precision=8, suppress=True, linewidth=120)

OLD_ASSET = (
    "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/"
    "GRScenes-test0-rebuilt-normalized_bak/_dedup_assets/"
    "c1_v2_vmatrix_20260319_172707/GRScenes_assets/desk/"
    "64a72e14d9b7e061a46e6fc802452525/usd/"
    "64a72e14d9b7e061a46e6fc802452525.usd"
)

CANON_ASSET = (
    "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/"
    "GRScenes-test0-rebuilt-normalized/GRScenes_assets/desk/"
    "08331d5e4c3ca970076b9c0f0a02d799/usd/"
    "08331d5e4c3ca970076b9c0f0a02d799.usd"
)

LAYOUT_USD = (
    "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/"
    "GRScenes-test0-rebuilt-normalized/GRScenes100/home/"
    "MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd"
)

PRIM_PATH = "/Root/Meshes/Animation/desk/model_64a72e14d9b7e061a46e6fc802452525_0"

# Also check the backup layout
BACKUP_LAYOUT = (
    "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/"
    "GRScenes-test0-rebuilt-normalized_bak/_dedup_layouts/"
    "c1_v2_vmatrix_20260319_172707/GRScenes100/home/"
    "MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd"
)


def gf_to_np(m):
    """Gf.Matrix4d -> np.ndarray(4x4)."""
    return np.array([[m[i][j] for j in range(4)] for i in range(4)], dtype=np.float64)


def np_to_gf(m):
    """np.ndarray(4x4) -> Gf.Matrix4d."""
    gf = Gf.Matrix4d()
    for i in range(4):
        for j in range(4):
            gf[i][j] = float(m[i][j])
    return gf


def print_xform_ops(stage, prim_path, label):
    """Print all xformOps on a prim."""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        print(f"  [{label}] Prim not found: {prim_path}")
        return
    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    print(f"  [{label}] xformOps on {prim_path}:")
    for op in ops:
        print(f"    {op.GetOpName()} = {op.Get()}")
    local_mat = _get_local_matrix(prim)
    print(f"    -> local matrix:\n{gf_to_np(local_mat)}")


def inspect_asset_structure(usd_path, label):
    """Print prim hierarchy and transforms."""
    print(f"\n{'='*60}")
    print(f"ASSET: {label}")
    print(f"Path: {usd_path}")
    print(f"{'='*60}")

    stage = Usd.Stage.Open(usd_path, load=Usd.Stage.LoadNone)
    if not stage:
        print("  FAILED to open stage")
        return None

    default_prim = stage.GetDefaultPrim()
    print(f"  DefaultPrim: {default_prim.GetPath()}")

    # Print hierarchy
    mesh_count = 0
    total_verts = 0
    first_mesh_chain = None
    for prim in Usd.PrimRange(default_prim):
        depth = len(str(prim.GetPath()).split("/")) - len(str(default_prim.GetPath()).split("/"))
        indent = "  " * (depth + 1)
        xf = UsdGeom.Xformable(prim)
        ops = xf.GetOrderedXformOps() if xf else []
        mesh = UsdGeom.Mesh(prim)
        is_mesh = bool(mesh and mesh.GetPointsAttr() and mesh.GetPointsAttr().Get())
        vcount = len(mesh.GetPointsAttr().Get()) if is_mesh else 0

        ops_summary = ", ".join(op.GetOpName() for op in ops) if ops else "none"
        mesh_tag = f" [MESH: {vcount} verts]" if is_mesh else ""
        print(f"{indent}{prim.GetPath().name} ({prim.GetTypeName()}) xformOps=[{ops_summary}]{mesh_tag}")

        if is_mesh:
            mesh_count += 1
            total_verts += vcount
            if first_mesh_chain is None:
                chain_mat = _get_chain_transform(default_prim, prim)
                first_mesh_chain = gf_to_np(chain_mat)

        # Print xformOp values for non-identity transforms
        local_mat = _get_local_matrix(prim)
        local_np = gf_to_np(local_mat)
        if not np.allclose(local_np, np.eye(4), atol=1e-6):
            for op in ops:
                val = op.Get()
                print(f"{indent}  {op.GetOpName()} = {val}")

    print(f"\n  Total meshes: {mesh_count}, total vertices: {total_verts}")
    if first_mesh_chain is not None:
        print(f"  Chain transform (root->first_mesh):\n{first_mesh_chain}")

    return stage


def get_internal_matrix(usd_path):
    """Get the Instance prim's local transform (the 'internal' matrix)."""
    stage = Usd.Stage.Open(usd_path, load=Usd.Stage.LoadNone)
    if not stage:
        return None
    dp = stage.GetDefaultPrim()
    if not dp:
        return None
    # The 'Instance' child typically holds the internal transform
    for child in dp.GetChildren():
        if child.GetName() == "Instance":
            return gf_to_np(_get_local_matrix(child))
    # If no Instance child, the defaultPrim itself may have transforms
    return gf_to_np(_get_local_matrix(dp))


def main():
    _ensure_pxr()

    # ---- Task 1: Vertex extraction and comparison ----
    print("\n" + "#"*70)
    print("# TASK 1: Vertex extraction and Procrustes alignment")
    print("#"*70)

    pts_old = extract_instance_space_vertices(OLD_ASSET)
    pts_canon = extract_instance_space_vertices(CANON_ASSET)

    print(f"\nOld asset vertices: {pts_old.shape if pts_old is not None else 'None'}")
    print(f"Canon asset vertices: {pts_canon.shape if pts_canon is not None else 'None'}")

    if pts_old is not None and pts_canon is not None:
        print(f"\nFirst 10 vertices (OLD):\n{pts_old[:10]}")
        print(f"\nFirst 10 vertices (CANON):\n{pts_canon[:10]}")

        # Bounding boxes
        bb_old_min, bb_old_max = pts_old.min(axis=0), pts_old.max(axis=0)
        bb_canon_min, bb_canon_max = pts_canon.min(axis=0), pts_canon.max(axis=0)
        print(f"\nOLD bbox: min={bb_old_min}, max={bb_old_max}, size={bb_old_max - bb_old_min}")
        print(f"CANON bbox: min={bb_canon_min}, max={bb_canon_max}, size={bb_canon_max - bb_canon_min}")

        # Check vertex count match
        if pts_old.shape[0] == pts_canon.shape[0]:
            print(f"\nVertex count MATCHES: {pts_old.shape[0]}")
            # Direct vertex difference
            diff = pts_old - pts_canon
            print(f"Max vertex difference: {np.abs(diff).max():.8f}")
            print(f"Mean vertex difference: {np.abs(diff).mean():.8f}")
        else:
            print(f"\nVertex count MISMATCH: old={pts_old.shape[0]}, canon={pts_canon.shape[0]}")

        # Procrustes alignment: V such that pts_old ≈ pts_canon * V
        print("\n--- Procrustes alignment (pts_canon -> pts_old) ---")
        V = procrustes_full(pts_canon, pts_old)
        V_np = gf_to_np(V) if hasattr(V, '__getitem__') and not isinstance(V, np.ndarray) else np.array(V)
        # If it returned a Gf.Matrix4d
        if isinstance(V, Gf.Matrix4d):
            V_np = gf_to_np(V)
        print(f"V matrix (4x4):\n{V_np}")

        # Extract rotation, scale, translation from V
        R = V_np[:3, :3]
        t = V_np[3, :3]
        scale = np.linalg.norm(R, axis=0)
        R_normalized = R / scale
        print(f"\nV rotation part (3x3):\n{R}")
        print(f"V scale factors: {scale}")
        print(f"V translation: {t}")
        print(f"det(R_3x3) = {np.linalg.det(R):.8f}")
        print(f"det(R_normalized) = {np.linalg.det(R_normalized):.8f}")

        # Verify: pts_canon * V ≈ pts_old
        ones = np.ones((pts_canon.shape[0], 1))
        pts_canon_h = np.hstack([pts_canon, ones])
        pts_canon_transformed = pts_canon_h @ V_np
        residual = pts_canon_transformed[:, :3] - pts_old
        rmse = np.sqrt(np.mean(residual**2))
        max_err = np.abs(residual).max()
        print(f"\nVerification (pts_canon * V vs pts_old):")
        print(f"  RMSE: {rmse:.10f}")
        print(f"  Max error: {max_err:.10f}")

    # ---- Task 2: Internal transforms ----
    print("\n" + "#"*70)
    print("# TASK 2: Internal transforms of each asset")
    print("#"*70)

    stage_old = inspect_asset_structure(OLD_ASSET, "OLD (64a72e14)")
    stage_canon = inspect_asset_structure(CANON_ASSET, "CANON (08331d5e)")

    M_old_internal = get_internal_matrix(OLD_ASSET)
    M_canon_internal = get_internal_matrix(CANON_ASSET)
    print(f"\nM_old_internal:\n{M_old_internal}")
    print(f"\nM_canon_internal:\n{M_canon_internal}")

    # ---- Task 3: Layout transforms ----
    print("\n" + "#"*70)
    print("# TASK 3: Layout transforms (old vs current)")
    print("#"*70)

    # Check backup layout for original transform
    if os.path.exists(BACKUP_LAYOUT):
        print(f"\nBackup layout: {BACKUP_LAYOUT}")
        stage_bak = Usd.Stage.Open(BACKUP_LAYOUT, load=Usd.Stage.LoadNone)
        if stage_bak:
            prim_bak = stage_bak.GetPrimAtPath(PRIM_PATH)
            if prim_bak and prim_bak.IsValid():
                M_old_local = gf_to_np(_get_local_matrix(prim_bak))
                print(f"M_old_local (from backup layout):\n{M_old_local}")
                print_xform_ops(stage_bak, PRIM_PATH, "BACKUP")

                # Check what the reference points to
                refs = prim_bak.GetReferences()
                print(f"\n  References on backup prim:")
                # Use composition to find reference
                for ref in prim_bak.GetPrimStack():
                    print(f"    layer: {ref.layer.identifier}")
            else:
                print(f"  Prim {PRIM_PATH} not found in backup layout")
    else:
        print(f"\nBackup layout not found: {BACKUP_LAYOUT}")
        M_old_local = None

    # Current layout
    print(f"\nCurrent layout: {LAYOUT_USD}")
    stage_cur = Usd.Stage.Open(LAYOUT_USD, load=Usd.Stage.LoadNone)
    if stage_cur:
        prim_cur = stage_cur.GetPrimAtPath(PRIM_PATH)
        if prim_cur and prim_cur.IsValid():
            M_cur_local = gf_to_np(_get_local_matrix(prim_cur))
            print(f"M_current_local (from current layout):\n{M_cur_local}")
            print_xform_ops(stage_cur, PRIM_PATH, "CURRENT")
        else:
            print(f"  Prim {PRIM_PATH} not found in current layout")
            M_cur_local = None

    # ---- Task 3b: Compute expected M_new_local ----
    print("\n" + "#"*70)
    print("# TASK 3b: Compute expected M_new_local")
    print("#"*70)

    if M_old_local is not None and M_canon_internal is not None and M_old_internal is not None:
        # V matrix from Procrustes
        print(f"\nV (from Procrustes):\n{V_np}")

        # M_canon_internal^-1
        M_canon_inv = np.linalg.inv(M_canon_internal)
        print(f"\nM_canon_internal^-1:\n{M_canon_inv}")

        # M_new_local = M_canon_internal^-1 * V * M_old_internal * M_old_local
        # Row-vector convention: composition is left-to-right
        M_expected = M_canon_inv @ V_np @ M_old_internal @ M_old_local
        print(f"\nExpected M_new_local = M_canon_inv @ V @ M_old_internal @ M_old_local:")
        print(f"{M_expected}")

        if M_cur_local is not None:
            diff = M_expected - M_cur_local
            print(f"\nDiff (expected - current):\n{diff}")
            print(f"Max diff: {np.abs(diff).max():.10f}")

        # Also try the other multiply order (column-vector convention)
        M_expected_col = M_old_local @ M_old_internal @ V_np @ M_canon_inv
        print(f"\nAlternate order M_old_local @ M_old_internal @ V @ M_canon_inv:")
        print(f"{M_expected_col}")
        if M_cur_local is not None:
            diff2 = M_expected_col - M_cur_local
            print(f"Max diff: {np.abs(diff2).max():.10f}")

        # World-space check: what position does the desk end up at?
        # Old world = M_old_internal * M_old_local (vertices already in instance space)
        # Actually: world_pts = pts_instance @ M_internal @ M_local
        # For old: world = pts_old_instance @ M_old_internal @ M_old_local
        # For new: world = pts_canon_instance @ M_canon_internal @ M_new_local
        print("\n--- World-space vertex check ---")
        if pts_old is not None and pts_canon is not None:
            ones_old = np.ones((pts_old.shape[0], 1))
            ones_canon = np.ones((pts_canon.shape[0], 1))

            # Old world position
            pts_old_h = np.hstack([pts_old, ones_old])
            world_old = (pts_old_h @ M_old_internal @ M_old_local)[:, :3]

            # New world position with CURRENT layout transform
            pts_canon_h = np.hstack([pts_canon, ones_canon])
            if M_cur_local is not None:
                world_new_cur = (pts_canon_h @ M_canon_internal @ M_cur_local)[:, :3]
                world_diff_cur = world_new_cur - world_old
                print(f"World displacement (current transform):")
                print(f"  Max: {np.abs(world_diff_cur).max():.8f}")
                print(f"  Mean: {np.abs(world_diff_cur).mean():.8f}")
                print(f"  RMSE: {np.sqrt(np.mean(world_diff_cur**2)):.8f}")

            # New world position with EXPECTED layout transform
            world_new_exp = (pts_canon_h @ M_canon_internal @ M_expected)[:, :3]
            world_diff_exp = world_new_exp - world_old
            print(f"World displacement (expected transform):")
            print(f"  Max: {np.abs(world_diff_exp).max():.8f}")
            print(f"  Mean: {np.abs(world_diff_exp).mean():.8f}")
            print(f"  RMSE: {np.sqrt(np.mean(world_diff_exp**2)):.8f}")

            # BBox comparison
            print(f"\nOld world bbox: {world_old.min(0)} .. {world_old.max(0)}")
            if M_cur_local is not None:
                print(f"New world bbox (current): {world_new_cur.min(0)} .. {world_new_cur.max(0)}")
            print(f"New world bbox (expected): {world_new_exp.min(0)} .. {world_new_exp.max(0)}")

    print("\nDone.")


if __name__ == "__main__":
    main()
