#!/usr/bin/env python3
"""Test Gf.Matrix4d element assignment."""
from pxr import Gf
import numpy as np

np.set_printoptions(precision=10, suppress=True)

# Test 1: Simple assignment via [i][j]
m = Gf.Matrix4d()
m[0][0] = 0.0
m[0][2] = -1.0
m[1][1] = 1.0
m[2][0] = 1.0
m[2][2] = 0.0
m[3][3] = 1.0

print("After [i][j] assignment:")
for i in range(4):
    print(f"  Row {i}: {[m[i][j] for j in range(4)]}")

# Test 2: numpy_to_gf_matrix4d exactly as in the code
V_np = np.array([
    [ 0.0, 0.0, -1.0, 0.0],
    [ 0.0, 1.0,  0.0, 0.0],
    [ 1.0, 0.0,  0.0, 0.0],
    [ 0.0, 0.0,  0.0, 1.0],
], dtype=np.float64)

gf_m = Gf.Matrix4d()
for i in range(4):
    for j in range(4):
        gf_m[i][j] = float(V_np[i][j])

print(f"\nInput numpy:\n{V_np}")
print(f"\nGf.Matrix4d after assignment:")
for i in range(4):
    print(f"  Row {i}: {[gf_m[i][j] for j in range(4)]}")

# Test 3: Use SetRow instead
gf_m2 = Gf.Matrix4d()
for i in range(4):
    gf_m2.SetRow(i, Gf.Vec4d(*V_np[i].tolist()))

print(f"\nGf.Matrix4d after SetRow:")
for i in range(4):
    print(f"  Row {i}: {[gf_m2[i][j] for j in range(4)]}")

# Test 4: Constructor
gf_m3 = Gf.Matrix4d(
    V_np[0][0], V_np[0][1], V_np[0][2], V_np[0][3],
    V_np[1][0], V_np[1][1], V_np[1][2], V_np[1][3],
    V_np[2][0], V_np[2][1], V_np[2][2], V_np[2][3],
    V_np[3][0], V_np[3][1], V_np[3][2], V_np[3][3],
)
print(f"\nGf.Matrix4d via constructor:")
for i in range(4):
    print(f"  Row {i}: {[gf_m3[i][j] for j in range(4)]}")

# Test 5: Verify m[i][j] = val actually persists
m2 = Gf.Matrix4d(1.0)  # identity
print(f"\nIdentity: {[m2[0][j] for j in range(4)]}")
m2[0][1] = 99.0
print(f"After m2[0][1] = 99.0: {[m2[0][j] for j in range(4)]}")
print(f"m2[0][1] = {m2[0][1]}")
