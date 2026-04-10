---
title: "Ground Displaced Prims Analysis (v4 Verification)"
code_reference: scripts/placement_pairwise_compare.py
created_at: 2026-03-23
updated_at: 2026-03-23
maintainer: categorize-agent
status: complete
---

# Ground Displaced Prims Analysis

## IMPORTANT CORRECTION (2026-03-23)

**The "displacement" metric in `placement_pairwise_compare.py` measures LOCAL transform
position difference, NOT world-space vertex error.** Task #1 repro confirmed that for the
worst pair (disp=84.39), actual world centroid displacement is **0.000000**. The V matrix
compensation is correct — the local transform changes are expected and intentional.

The pair structure analysis below remains useful for understanding dedup group composition,
but the displacement values should NOT be interpreted as errors.

## Context

After v4 plane fix + dedup V-matrix compensation, the ground-only pairwise comparison
(`v4_verification/ground_only_pairwise_compare.json`) reports **1961 ref_changed prims
with local transform displacement > 0.01** out of 10,388 total ground prims across 99
scenes. These local transform changes are expected consequences of V matrix compensation.

All 4,243 ref_same prims have zero displacement (confirmed correct).

## Data Completeness Note

The v4 JSON stores only `top_20_worst` prims per scene. Of the 1961 displaced prims,
**1549 (79%)** are captured in these top-20 lists; 412 prims from 56 scenes with >20
displaced prims are truncated. The truncated prims are the lowest-displacement ones
per scene, so the analysis captures all high-displacement patterns.

## Key Finding: 100% of displacement comes from shape_invariant dedup

| Dedup Mode | Displaced Prims | Unique Pairs |
|------------|---------------:|-------------:|
| shape_invariant + union | 1,407 | 1,407 |
| union only (transitive) | 139 | 139 |
| topo_filesize + union | 3 | 3 |
| geom_only | **0** | 0 |
| **Total** | **1,549** | **1,549** |

- **Zero displaced prims from geom_only dedup.** geom_only (Identity V) is fully safe.
- **3 prims from topo_filesize** (negligible).
- **139 prims from union-only** = transitive chain merging. These pairs are in the same
  union group but different shape_invariant groups (137 of 139 have both UIDs in SI but
  in different SI groups). Their V matrix was composed through BFS path accumulation.
- The remaining **1,407 prims** are direct shape_invariant matches.

## Pair Structure

Every (canonical_uid, old_uid) pair appears in exactly **1 scene** (count=1 per pair).
This means each ground prim references a unique old asset that was deduplicated to a
canonical.

- **1,549 unique pairs** from the extracted data
- **268 unique canonical UIDs** (many-to-one: ~5.8 old UIDs per canonical on average)
- **1,549 unique old UIDs** (each old UID displaced in exactly one scene)

### Top 10 Canonical UIDs by Displaced Prim Count

| Canonical UID | Displaced Prims | Union Group Size |
|---------------|----------------:|-----------------:|
| 00bc09c21472... | 90 | 91 |
| 0167b1e927a1... | 80 | 85 |
| 00c6fa7c590d... | 71 | 81 |
| 0262452aa846... | 67 | 76 |
| 002f910efb92... | 53 | 58 |
| 0886699250674... | 47 | 52 |
| 04410d87ca0e... | 44 | 47 |
| 0d7dbcb947ab... | 43 | 66 |
| 0126e8d5ec0f... | 40 | 45 |
| 023625e705ae... | 38 | 43 |

The displaced prim count per canonical roughly tracks the union group size, confirming
that larger dedup groups produce more displaced prims.

## Displacement Distribution

| Range | Prims | Unique Pairs | Unique Canonicals |
|-------|------:|-------------:|------------------:|
| 0.01 - 0.1 | 36 | 36 | 32 |
| 0.1 - 1.0 | 296 | 296 | 140 |
| 1.0 - 10.0 | **1,158** | 1,158 | 156 |
| 10.0+ | 59 | 59 | 29 |
| **Total** | **1,549** | 1,549 | 268 |

**75% of displaced prims** have displacement in the 1.0-10.0 range. The worst cases
(10+) are concentrated in 29 canonical UIDs.

### Top 20 Pairs by Max Displacement

| # | Canonical UID | Old UID | Disp | Verts |
|---|---------------|---------|-----:|------:|
| 1 | 2d975f167aa3... | f47c41fd54ce... | 84.39 | 7 |
| 2 | 2d975f167aa3... | 7e61b818af58... | 84.39 | 7 |
| 3 | 8f88a917ee6e... | fc1dcfdba7da... | 83.43 | 5 |
| 4 | 009f55b20962... | 904cf7ba92fc... | 61.92 | 7 |
| 5 | 009f55b20962... | a8d73e9bbd71... | 61.92 | 7 |
| 6 | a101a60a4aba... | aeca355b17b5... | 60.83 | 7 |
| 7 | 27a8274f7c63... | 776414ef49cd... | 58.23 | 5 |
| 8 | 050c33bb6b5b... | 127262baaf1d... | 57.78 | 7 |
| 9 | 161401cc8de9... | 465a1a7317a1... | 50.59 | 7 |
| 10 | 31c663f42c20... | 7ff9211ac0ef... | 44.07 | 7 |
| 11 | 0d9521620261... | 5ed9ad762282... | 43.51 | 7 |
| 12 | 161401cc8de9... | fc32d135cbd3... | 42.41 | 7 |
| 13 | 009f55b20962... | 5ee6ef182f74... | 41.44 | 7 |
| 14 | 2d975f167aa3... | 3913304dfd5c... | 41.06 | 7 |
| 15 | 2d975f167aa3... | dc25dbd9572a... | 40.07 | 7 |
| 16 | 8b99078d8357... | f57f48851603... | 38.88 | 5 |
| 17 | 161401cc8de9... | ffb0b3b4bfee... | 37.33 | 7 |
| 18 | 009f55b20962... | 0f232dba97c0... | 35.03 | 7 |
| 19 | 8b99078d8357... | e339948df737... | 34.87 | 5 |
| 20 | 24d31209566536... | af2e563e7408... | 34.58 | 7 |

The worst cases (>30 displacement) recur across a few canonical UIDs:
- `2d975f167aa3...` appears 4 times in top 20
- `009f55b20962...` appears 4 times
- `161401cc8de9...` appears 3 times

## Vertex Count Distribution

| Vertices | Prims | Mean Disp | Max Disp |
|----------|------:|----------:|---------:|
| 4 | 60 | 2.83 | 10.80 |
| 5 | 80 | 6.15 | 83.43 |
| 6 | **1,267** | 2.46 | 9.75 |
| 7 | 125 | 12.91 | 84.39 |
| 8 | 11 | 0.71 | 1.51 |
| 14-110 | 6 | varies | 2.32 |

**82% of displaced prims have 6 vertices** (typical rectangular ground planes split
into 2 triangles = 4 verts + 2 shared). These have moderate displacement (mean 2.46,
max 9.75).

The worst displacements (>30) come from **5-vertex and 7-vertex** ground prims —
irregular polygon shapes where the shape_invariant V matrix (bbox denormalization +
Procrustes alignment) introduces the most error.

## Transitive Chain Prims (Union-Only)

139 prims are displaced because of **transitive dedup merging** — the canonical and
old UIDs are in the same union group but different shape_invariant groups. Their
V matrix was composed through BFS path accumulation:

- 137 of 139 have both UIDs in shape_invariant but in different SI groups
- 5 have only the canonical in SI
- Displacement range: 0.15 - 26.91 (mean 3.93)

These transitive V matrices accumulate error from multiple Procrustes alignments.

## Interpretation (CORRECTED)

The local transform displacement is **not an error** — it is the expected result of
V matrix compensation. When a prim's reference is changed from old_uid to canonical_uid,
the local transform is adjusted by V^{-1} so that the world-space vertex positions
remain unchanged. Task #1 repro confirmed world centroid displacement = 0.000000 for
the worst case (local disp = 84.39).

The large local transform changes for shape_invariant pairs reflect the geometric
difference between the old and canonical assets (which can differ in position/scale
in asset-local space). geom_only pairs have V=Identity, so no local transform change
is needed.
