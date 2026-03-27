#!/usr/bin/env python3
"""Debug desk dedup pair v2: compare pre-dedup vs post-dedup layout transforms.

Uses the pre-C1 v2_vmatrix snapshot taken just before desk processing.
"""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pxr import Gf, Usd, UsdGeom

from scripts.compute_vertex_transform import (
    extract_instance_space_vertices,
    procrustes_full,
    _get_local_matrix,
    _get_chain_transform,
    _ensure_pxr,
)

np.set_printoptions(precision=10, suppress=True, linewidth=140)

BASE = "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep"

OLD_ASSET = (
    f"{BASE}/GRScenes-test0-rebuilt-normalized_bak/_dedup_assets/"
    "c1_v2_vmatrix_20260319_172707/GRScenes_assets/desk/"
    "64a72e14d9b7e061a46e6fc802452525/usd/64a72e14d9b7e061a46e6fc802452525.usd"
)
CANON_ASSET = (
    f"{BASE}/GRScenes-test0-rebuilt-normalized/GRScenes_assets/desk/"
    "08331d5e4c3ca970076b9c0f0a02d799/usd/08331d5e4c3ca970076b9c0f0a02d799.usd"
)

LAYOUT_DIR = (
    f"{BASE}/GRScenes-test0-rebuilt-normalized/GRScenes100/home/"
    "MV7J6NIKTKJZ2AABAAAAADA8_usd"
)
PRE_DEDUP_LAYOUT = f"{LAYOUT_DIR}/layout.pre_c1_v2_vmatrix.20260319_172707.usd"
POST_DEDUP_LAYOUT = f"{LAYOUT_DIR}/layout.usd"

PRIM_PATH = "/Root/Meshes/Animation/desk/model_64a72e14d9b7e061a46e6fc802452525_0"


def gf_to_np(m):
    return np.array([[m[i][j] for j in range(4)] for i in range(4)], dtype=np.float64)


def get_prim_transform(layout_path, prim_path):
    """Get the local transform and reference path of a prim in a layout."""
    stage = Usd.Stage.Open(layout_path, load=Usd.Stage.LoadNone)
    if not stage:
        print(f"  FAILED to open: {layout_path}")
        return None, None
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        print(f"  Prim not found: {prim_path}")
        return None, None

    local_mat = gf_to_np(_get_local_matrix(prim))

    # Get xformOps
    xf = UsdGeom.Xformable(prim)
    ops = xf.GetOrderedXformOps()
    print(f"  xformOps:")
    for op in ops:
        print(f"    {op.GetOpName()} = {op.Get()}")

    # Get references
    stack = prim.GetPrimStack()
    for spec in stack:
        refs = spec.referenceList
        if refs:
            for ref in refs.GetAddedOrExplicitItems():
                print(f"  Reference: {ref.assetPath}")

    return local_mat, stage


def get_internal_matrix(usd_path):
    """Get Instance prim's local transform."""
    stage = Usd.Stage.Open(usd_path, load=Usd.Stage.LoadNone)
    if not stage:
        return np.eye(4)
    dp = stage.GetDefaultPrim()
    if not dp:
        return np.eye(4)
    for child in dp.GetChildren():
        if child.GetName() == "Instance":
            return gf_to_np(_get_local_matrix(child))
    return gf_to_np(_get_local_matrix(dp))


def main():
    _ensure_pxr()

    print("="*80)
    print("DESK DEDUP PAIR DIAGNOSTIC V2")
    print("="*80)
    print(f"Old asset:     64a72e14d9b7e061a46e6fc802452525")
    print(f"Canon asset:   08331d5e4c3ca970076b9c0f0a02d799")
    print(f"Dedup mode:    topo_filesize")
    print(f"Layout scene:  MV7J6NIKTKJZ2AABAAAAADA8_usd")
    print(f"Layout prim:   {PRIM_PATH}")

    # ---- Vertex extraction ----
    print("\n" + "-"*80)
    print("1. VERTEX EXTRACTION & PROCRUSTES")
    print("-"*80)

    pts_old = extract_instance_space_vertices(OLD_ASSET)
    pts_canon = extract_instance_space_vertices(CANON_ASSET)
    print(f"Old vertices: {pts_old.shape}")
    print(f"Canon vertices: {pts_canon.shape}")

    bb_old = (pts_old.min(0), pts_old.max(0))
    bb_canon = (pts_canon.min(0), pts_canon.max(0))
    print(f"\nOLD bbox:   min={bb_old[0]}, max={bb_old[1]}")
    print(f"            size={bb_old[1]-bb_old[0]}")
    print(f"CANON bbox: min={bb_canon[0]}, max={bb_canon[1]}")
    print(f"            size={bb_canon[1]-bb_canon[0]}")

    # Key observation: X and Z axes are swapped between old and canon
    print(f"\nOLD   X-range: [{bb_old[0][0]:.4f}, {bb_old[1][0]:.4f}] width={bb_old[1][0]-bb_old[0][0]:.4f}")
    print(f"CANON Z-range: [{bb_canon[0][2]:.4f}, {bb_canon[1][2]:.4f}] width={bb_canon[1][2]-bb_canon[0][2]:.4f}")
    print(f"OLD   Z-range: [{bb_old[0][2]:.4f}, {bb_old[1][2]:.4f}] width={bb_old[1][2]-bb_old[0][2]:.4f}")
    print(f"CANON X-range: [{bb_canon[0][0]:.4f}, {bb_canon[1][0]:.4f}] width={bb_canon[1][0]-bb_canon[0][0]:.4f}")
    print("=> Canon is ~90 deg rotated from Old around Y axis")

    # Procrustes
    V_gf = procrustes_full(pts_canon, pts_old)
    V = gf_to_np(V_gf)
    print(f"\nV matrix (Procrustes: pts_old = pts_canon * V):\n{V}")

    # Decompose V rotation
    R = V[:3,:3]
    det_R = np.linalg.det(R)
    print(f"det(R_3x3) = {det_R:.10f}")

    # What rotation angle does this represent?
    trace_R = np.trace(R)
    angle_rad = np.arccos(np.clip((trace_R - 1) / 2, -1, 1))
    angle_deg = np.degrees(angle_rad)
    print(f"Rotation angle: {angle_deg:.4f} degrees")
    print(f"Translation: {V[3,:3]}")

    # Verify
    ones = np.ones((pts_canon.shape[0], 1))
    pts_canon_h = np.hstack([pts_canon, ones])
    pts_transformed = (pts_canon_h @ V)[:,:3]
    rmse = np.sqrt(np.mean((pts_transformed - pts_old)**2))
    print(f"Verification RMSE: {rmse:.10f}")

    # ---- Internal matrices ----
    print("\n" + "-"*80)
    print("2. INTERNAL MATRICES")
    print("-"*80)

    M_old_int = get_internal_matrix(OLD_ASSET)
    M_canon_int = get_internal_matrix(CANON_ASSET)
    print(f"M_old_internal:\n{M_old_int}")
    print(f"\nM_canon_internal:\n{M_canon_int}")
    print(f"\nM_canon_internal == M_old_internal? {np.allclose(M_canon_int, M_old_int)}")

    M_canon_inv = np.linalg.inv(M_canon_int)
    print(f"\nM_canon_internal^-1:\n{M_canon_inv}")

    # ---- Layout transforms ----
    print("\n" + "-"*80)
    print("3. LAYOUT TRANSFORMS")
    print("-"*80)

    print(f"\n--- Pre-dedup layout (snapshot before desk category) ---")
    M_old_local, _ = get_prim_transform(PRE_DEDUP_LAYOUT, PRIM_PATH)
    if M_old_local is not None:
        print(f"M_old_local:\n{M_old_local}")

    print(f"\n--- Post-dedup layout (current) ---")
    M_cur_local, _ = get_prim_transform(POST_DEDUP_LAYOUT, PRIM_PATH)
    if M_cur_local is not None:
        print(f"M_cur_local (current):\n{M_cur_local}")

    # ---- Expected compensation ----
    print("\n" + "-"*80)
    print("4. COMPENSATION FORMULA VERIFICATION")
    print("-"*80)

    if M_old_local is not None:
        # Formula: M_new_local = M_canon_inv @ V @ M_old_internal @ M_old_local
        M_expected = M_canon_inv @ V @ M_old_int @ M_old_local
        print(f"\nM_expected = M_canon_inv @ V @ M_old_int @ M_old_local:")
        print(f"{M_expected}")

        if M_cur_local is not None:
            diff = M_expected - M_cur_local
            print(f"\nDiff (expected - current):\n{diff}")
            print(f"Max diff: {np.abs(diff).max():.12f}")
            match = np.allclose(M_expected, M_cur_local, atol=1e-4)
            print(f"Match (atol=1e-4)? {match}")

        # ---- World-space verification ----
        print("\n" + "-"*80)
        print("5. WORLD-SPACE VERIFICATION")
        print("-"*80)

        # Old world = pts_old_instance @ M_old_internal @ M_old_local
        ones_o = np.ones((pts_old.shape[0], 1))
        pts_old_h = np.hstack([pts_old, ones_o])
        world_old = (pts_old_h @ M_old_int @ M_old_local)[:,:3]

        # New world (current) = pts_canon_instance @ M_canon_internal @ M_cur_local
        ones_c = np.ones((pts_canon.shape[0], 1))
        pts_canon_h = np.hstack([pts_canon, ones_c])

        if M_cur_local is not None:
            world_new_cur = (pts_canon_h @ M_canon_int @ M_cur_local)[:,:3]
            disp_cur = world_new_cur - world_old
            print(f"\nWorld displacement (current transform):")
            print(f"  Max: {np.abs(disp_cur).max():.8f}")
            print(f"  Mean: {np.abs(disp_cur).mean():.8f}")
            print(f"  RMSE: {np.sqrt(np.mean(disp_cur**2)):.8f}")

            print(f"\nOld world bbox:     {world_old.min(0)} .. {world_old.max(0)}")
            print(f"New world bbox (cur): {world_new_cur.min(0)} .. {world_new_cur.max(0)}")

            # Center displacement
            old_center = (world_old.min(0) + world_old.max(0)) / 2
            new_center = (world_new_cur.min(0) + world_new_cur.max(0)) / 2
            print(f"\nOld center:  {old_center}")
            print(f"New center:  {new_center}")
            print(f"Center displacement: {new_center - old_center}")
            print(f"Center distance: {np.linalg.norm(new_center - old_center):.8f}")

        # New world (expected)
        world_new_exp = (pts_canon_h @ M_canon_int @ M_expected)[:,:3]
        disp_exp = world_new_exp - world_old
        print(f"\nWorld displacement (expected transform):")
        print(f"  Max: {np.abs(disp_exp).max():.8f}")
        print(f"  Mean: {np.abs(disp_exp).mean():.8f}")
        print(f"  RMSE: {np.sqrt(np.mean(disp_exp**2)):.8f}")

        # ---- Check if V*M_old_int commutes issue ----
        print("\n" + "-"*80)
        print("6. ALTERNATIVE FORMULA CHECKS")
        print("-"*80)

        # Since M_old_int = M_canon_int = 0.1*I, they cancel:
        # M_expected = 10*I @ V @ 0.1*I @ M_old_local = V @ M_old_local
        M_simple = V @ M_old_local
        print(f"\nSimplified: V @ M_old_local (since M_canon_inv @ ... @ M_old_int cancels):")
        print(f"{M_simple}")
        print(f"Same as M_expected? {np.allclose(M_simple, M_expected)}")

        # What if the code used the wrong multiply order?
        # Try: M_old_local @ M_old_int @ V @ M_canon_inv
        M_alt1 = M_old_local @ M_old_int @ V @ M_canon_inv
        print(f"\nAlt order: M_old_local @ M_old_int @ V @ M_canon_inv:")
        print(f"{M_alt1}")
        if M_cur_local is not None:
            print(f"Match current? {np.allclose(M_alt1, M_cur_local, atol=1e-4)}")

        # Try: M_old_local @ V
        M_alt2 = M_old_local @ V
        print(f"\nAlt: M_old_local @ V:")
        print(f"{M_alt2}")
        if M_cur_local is not None:
            print(f"Match current? {np.allclose(M_alt2, M_cur_local, atol=1e-4)}")

        # What about V^-1 instead of V?
        V_inv = np.linalg.inv(V)
        M_alt3 = V_inv @ M_old_local
        print(f"\nAlt: V_inv @ M_old_local:")
        print(f"{M_alt3}")
        if M_cur_local is not None:
            print(f"Match current? {np.allclose(M_alt3, M_cur_local, atol=1e-4)}")

        M_alt4 = M_old_local @ V_inv
        print(f"\nAlt: M_old_local @ V_inv:")
        print(f"{M_alt4}")
        if M_cur_local is not None:
            print(f"Match current? {np.allclose(M_alt4, M_cur_local, atol=1e-4)}")

    print("\nDone.")


if __name__ == "__main__":
    main()
