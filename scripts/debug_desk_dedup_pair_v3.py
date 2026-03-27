#!/usr/bin/env python3
"""Debug desk dedup: verify Gf.Matrix4d multiplication vs numpy."""

import sys
import os
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from pxr import Gf, Usd, UsdGeom

from scripts.compute_vertex_transform import (
    extract_instance_space_vertices,
    procrustes_full,
    compute_V_for_pair,
    determine_compensation_mode,
    build_mode_index,
    numpy_to_gf_matrix4d,
    _get_local_matrix,
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

PRE_DEDUP_LAYOUT = (
    f"{BASE}/GRScenes-test0-rebuilt-normalized/GRScenes100/home/"
    "MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.pre_c1_v2_vmatrix.20260319_172707.usd"
)
PRIM_PATH = "/Root/Meshes/Animation/desk/model_64a72e14d9b7e061a46e6fc802452525_0"


def gf_to_np(m):
    return np.array([[m[i][j] for j in range(4)] for i in range(4)], dtype=np.float64)


def main():
    _ensure_pxr()

    # 1. Get the V matrix exactly as compute_V_for_pair would
    print("="*80)
    print("Reproducing exact V computation from compute_V_for_pair")
    print("="*80)

    # First check what mode is determined
    mode_index = build_mode_index("check_reports/test0_rebuilt_dedup")
    mode = determine_compensation_mode(OLD_ASSET, CANON_ASSET, mode_index)
    print(f"Determined mode: {mode}")

    V_gf = compute_V_for_pair(OLD_ASSET, CANON_ASSET, mode, mode_index=mode_index)
    V_np = gf_to_np(V_gf)
    print(f"\nV (Gf.Matrix4d -> numpy):\n{V_np}")

    # Also compute V directly via procrustes_full
    pts_canon = extract_instance_space_vertices(CANON_ASSET)
    pts_old = extract_instance_space_vertices(OLD_ASSET)
    V_np_direct = procrustes_full(pts_canon, pts_old)
    print(f"\nV (direct procrustes_full -> numpy):\n{V_np_direct}")

    # Check numpy_to_gf_matrix4d round-trip
    V_gf_roundtrip = numpy_to_gf_matrix4d(V_np_direct)
    V_np_roundtrip = gf_to_np(V_gf_roundtrip)
    print(f"\nV (numpy -> Gf -> numpy roundtrip):\n{V_np_roundtrip}")
    print(f"Roundtrip matches? {np.allclose(V_np_direct, V_np_roundtrip)}")

    # 2. Get internal matrices as Gf.Matrix4d
    stage_old = Usd.Stage.Open(OLD_ASSET, load=Usd.Stage.LoadNone)
    dp_old = stage_old.GetDefaultPrim()
    instance_old = stage_old.GetPrimAtPath(dp_old.GetPath().AppendChild("Instance"))
    M_old_int_gf = _get_local_matrix(instance_old)

    stage_canon = Usd.Stage.Open(CANON_ASSET, load=Usd.Stage.LoadNone)
    dp_canon = stage_canon.GetDefaultPrim()
    instance_canon = stage_canon.GetPrimAtPath(dp_canon.GetPath().AppendChild("Instance"))
    M_canon_int_gf = _get_local_matrix(instance_canon)

    print(f"\nM_old_int (Gf):\n{gf_to_np(M_old_int_gf)}")
    print(f"M_canon_int (Gf):\n{gf_to_np(M_canon_int_gf)}")

    # 3. Get old local transform
    stage_pre = Usd.Stage.Open(PRE_DEDUP_LAYOUT, load=Usd.Stage.LoadNone)
    prim_pre = stage_pre.GetPrimAtPath(PRIM_PATH)
    M_old_local_gf = _get_local_matrix(prim_pre)
    print(f"\nM_old_local (Gf):\n{gf_to_np(M_old_local_gf)}")

    # 4. Compute new_local using EXACT same code as line 606
    M_canon_inv_gf = M_canon_int_gf.GetInverse()
    print(f"\nM_canon_int.GetInverse() (Gf):\n{gf_to_np(M_canon_inv_gf)}")

    # Line 606: new_local = canonical_internal.GetInverse() * V * old_internal * old_local
    new_local_gf = M_canon_inv_gf * V_gf * M_old_int_gf * M_old_local_gf
    print(f"\nnew_local (Gf, line 606 formula):\n{gf_to_np(new_local_gf)}")

    # 5. Now compute using numpy
    M_canon_inv_np = np.linalg.inv(gf_to_np(M_canon_int_gf))
    M_old_int_np = gf_to_np(M_old_int_gf)
    M_old_local_np = gf_to_np(M_old_local_gf)

    new_local_np = M_canon_inv_np @ V_np @ M_old_int_np @ M_old_local_np
    print(f"\nnew_local (numpy, @ operator):\n{new_local_np}")

    print(f"\nGf result == numpy result? {np.allclose(gf_to_np(new_local_gf), new_local_np, atol=1e-6)}")

    # 6. Verify: does Gf.Matrix4d multiplication do A*B = A@B or A*B = B@A?
    print("\n" + "="*80)
    print("Gf.Matrix4d multiplication convention test")
    print("="*80)

    A = Gf.Matrix4d()
    A[0][0] = 2.0; A[1][1] = 3.0; A[2][2] = 4.0; A[3][3] = 1.0
    B = Gf.Matrix4d()
    B[3][0] = 10.0  # translation
    B[0][0] = 1.0; B[1][1] = 1.0; B[2][2] = 1.0; B[3][3] = 1.0

    C_gf = A * B
    A_np = gf_to_np(A)
    B_np = gf_to_np(B)
    C_np_AB = A_np @ B_np
    C_np_BA = B_np @ A_np

    print(f"A:\n{A_np}")
    print(f"B:\n{B_np}")
    print(f"A*B (Gf):\n{gf_to_np(C_gf)}")
    print(f"A@B (numpy):\n{C_np_AB}")
    print(f"B@A (numpy):\n{C_np_BA}")
    print(f"Gf A*B == numpy A@B? {np.allclose(gf_to_np(C_gf), C_np_AB)}")
    print(f"Gf A*B == numpy B@A? {np.allclose(gf_to_np(C_gf), C_np_BA)}")

    # 7. Verify world-space displacement
    print("\n" + "="*80)
    print("World-space displacement check")
    print("="*80)

    ones = np.ones((pts_old.shape[0], 1))

    # Old world: pts_old @ M_old_int @ M_old_local
    world_old = (np.hstack([pts_old, ones]) @ M_old_int_np @ M_old_local_np)[:,:3]

    # New world with computed compensation
    new_local_final = gf_to_np(new_local_gf)
    M_canon_int_np = gf_to_np(M_canon_int_gf)
    world_new = (np.hstack([pts_canon, ones]) @ M_canon_int_np @ new_local_final)[:,:3]

    disp = world_new - world_old
    print(f"Max displacement: {np.abs(disp).max():.10f}")
    print(f"RMSE displacement: {np.sqrt(np.mean(disp**2)):.10f}")

    # 8. What does the CURRENT layout.usd have?
    CUR_LAYOUT = f"{BASE}/GRScenes-test0-rebuilt-normalized/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd"
    stage_cur = Usd.Stage.Open(CUR_LAYOUT, load=Usd.Stage.LoadNone)
    prim_cur = stage_cur.GetPrimAtPath(PRIM_PATH)
    M_cur_gf = _get_local_matrix(prim_cur)
    M_cur_np = gf_to_np(M_cur_gf)
    print(f"\nCurrent layout M_local:\n{M_cur_np}")
    print(f"\nComputed new_local:\n{new_local_final}")
    print(f"\nMatch? {np.allclose(M_cur_np, new_local_final, atol=1e-4)}")

    if not np.allclose(M_cur_np, new_local_final, atol=1e-4):
        print(f"\nDiff:\n{M_cur_np - new_local_final}")
        print(f"\nThe current layout was NOT produced by our formula!")
        print(f"Current matches M_old_local? {np.allclose(M_cur_np, M_old_local_np, atol=1e-4)}")

        # Check if current was produced with V=Identity
        new_local_no_V = gf_to_np(M_canon_inv_gf * Gf.Matrix4d(1.0) * M_old_int_gf * M_old_local_gf)
        print(f"\nnew_local with V=Identity:\n{new_local_no_V}")
        print(f"Current matches V=Identity formula? {np.allclose(M_cur_np, new_local_no_V, atol=1e-4)}")

    print("\nDone.")


if __name__ == "__main__":
    main()
