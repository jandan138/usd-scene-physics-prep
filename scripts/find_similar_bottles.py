#!/usr/bin/env python3
"""
Find all bottle assets with the same topology signature as target bottles
(779 vertices, 3 meshes) and compare their bounding box aspect ratios.

Usage:
    python scripts/find_similar_bottles.py
"""

import os
import sys
import json
import time
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed

# ---------------------------------------------------------------------------
# USD setup
# ---------------------------------------------------------------------------
try:
    from pxr import Usd, UsdGeom, Gf
except ImportError:
    sys.exit("ERROR: pxr (usd-core) not available. Install via: pip install usd-core")

BOTTLE_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "GRScenes-test1-normalized", "GRScenes_assets", "bottle",
)

TARGETS = [
    "7861bdaa89323558eb8046679f567498",
    "79088d12b87f6758da805eb64c8a3582",
    "79090fe893611281c78d1c237c2f5b64",
]

DEDUP_REPORT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "check_reports", "normalized_dedup_tolerance", "bottle",
    "bottle_asset_mesh_dedup_geom_only.json",
)


def analyze_asset(asset_hash: str) -> dict:
    """Load a bottle USD and compute geometry stats."""
    usd_dir = os.path.join(BOTTLE_DIR, asset_hash, "usd")
    usd_path = os.path.join(usd_dir, f"{asset_hash}.usd")

    result = {
        "hash": asset_hash,
        "total_verts": 0,
        "total_faces": 0,
        "mesh_count": 0,
        "bbox_min": None,
        "bbox_max": None,
        "bbox_dims": None,
        "error": None,
    }

    if not os.path.isfile(usd_path):
        result["error"] = "usd file not found"
        return result

    try:
        stage = Usd.Stage.Open(usd_path)
        if not stage:
            result["error"] = "failed to open stage"
            return result

        all_min = Gf.Vec3d(float("inf"), float("inf"), float("inf"))
        all_max = Gf.Vec3d(float("-inf"), float("-inf"), float("-inf"))

        for prim in stage.Traverse():
            if not prim.IsA(UsdGeom.Mesh):
                continue
            mesh = UsdGeom.Mesh(prim)
            result["mesh_count"] += 1

            points = mesh.GetPointsAttr().Get()
            if points:
                result["total_verts"] += len(points)
                for p in points:
                    for i in range(3):
                        if p[i] < all_min[i]:
                            all_min[i] = p[i]
                        if p[i] > all_max[i]:
                            all_max[i] = p[i]

            face_counts = mesh.GetFaceVertexCountsAttr().Get()
            if face_counts:
                result["total_faces"] += len(face_counts)

        if result["mesh_count"] > 0:
            result["bbox_min"] = list(all_min)
            result["bbox_max"] = list(all_max)
            dims = [all_max[i] - all_min[i] for i in range(3)]
            result["bbox_dims"] = dims

    except Exception as e:
        result["error"] = str(e)

    return result


def main():
    # Enumerate all bottle assets
    if not os.path.isdir(BOTTLE_DIR):
        sys.exit(f"ERROR: Bottle directory not found: {BOTTLE_DIR}")

    all_hashes = sorted([
        d for d in os.listdir(BOTTLE_DIR)
        if os.path.isdir(os.path.join(BOTTLE_DIR, d))
    ])
    print(f"Total bottle assets found: {len(all_hashes)}")

    # -----------------------------------------------------------------------
    # Step 1: Analyze all assets in parallel
    # -----------------------------------------------------------------------
    print("\nAnalyzing all bottle assets (parallel)...")
    t0 = time.time()
    results = []
    errors = []

    with ProcessPoolExecutor(max_workers=16) as pool:
        futures = {pool.submit(analyze_asset, h): h for h in all_hashes}
        done_count = 0
        for fut in as_completed(futures):
            done_count += 1
            if done_count % 200 == 0:
                print(f"  ... {done_count}/{len(all_hashes)}")
            r = fut.result()
            if r["error"]:
                errors.append(r)
            else:
                results.append(r)

    elapsed = time.time() - t0
    print(f"Done in {elapsed:.1f}s. Successful: {len(results)}, Errors: {len(errors)}")

    if errors:
        print(f"\nFirst 5 errors:")
        for e in errors[:5]:
            print(f"  {e['hash']}: {e['error']}")

    # -----------------------------------------------------------------------
    # Step 2: Overall topology distribution
    # -----------------------------------------------------------------------
    topo_groups = defaultdict(list)
    for r in results:
        key = (r["total_verts"], r["mesh_count"])
        topo_groups[key].append(r)

    # Sort by count descending
    sorted_topos = sorted(topo_groups.items(), key=lambda x: -len(x[1]))

    print(f"\n{'='*70}")
    print("TOPOLOGY DISTRIBUTION (top 20 by count)")
    print(f"{'='*70}")
    print(f"{'Verts':>8} {'Meshes':>6} {'Count':>6}")
    print(f"{'-'*8} {'-'*6} {'-'*6}")
    for (verts, meshes), group in sorted_topos[:20]:
        marker = " <-- TARGET" if verts == 779 and meshes == 3 else ""
        print(f"{verts:>8} {meshes:>6} {len(group):>6}{marker}")

    # -----------------------------------------------------------------------
    # Step 3: Filter to 779 verts, 3 meshes
    # -----------------------------------------------------------------------
    target_key = (779, 3)
    matched = topo_groups.get(target_key, [])
    print(f"\n{'='*70}")
    print(f"BOTTLES WITH EXACT TOPOLOGY (779 verts, 3 meshes): {len(matched)}")
    print(f"{'='*70}")

    if not matched:
        print("No matches found.")
        return

    # Compute aspect ratios and sort
    for r in matched:
        dims = r["bbox_dims"]
        if dims:
            sorted_dims = sorted(dims, reverse=True)
            # Aspect ratios: longest/middle, longest/shortest
            if sorted_dims[2] > 1e-9:
                r["aspect_lm"] = sorted_dims[0] / sorted_dims[1] if sorted_dims[1] > 1e-9 else float("inf")
                r["aspect_ls"] = sorted_dims[0] / sorted_dims[2]
            else:
                r["aspect_lm"] = float("inf")
                r["aspect_ls"] = float("inf")
            r["sorted_dims"] = sorted_dims
        else:
            r["aspect_lm"] = None
            r["aspect_ls"] = None
            r["sorted_dims"] = None

    # Sort by total_faces then hash for stable output
    matched.sort(key=lambda r: (r["total_faces"], r["hash"]))

    # Sub-group by face count
    face_groups = defaultdict(list)
    for r in matched:
        face_groups[r["total_faces"]].append(r)

    print(f"\nSub-grouping by face count:")
    for fc, group in sorted(face_groups.items()):
        print(f"  {fc} faces: {len(group)} assets")

    # Print target bottles first
    print(f"\n--- Target bottles ---")
    print(f"{'Hash':>34} {'Verts':>6} {'Faces':>6} {'Meshes':>6} {'DimX':>8} {'DimY':>8} {'DimZ':>8} {'AR_LM':>7} {'AR_LS':>7}")
    for r in matched:
        if r["hash"] in TARGETS:
            dims = r["bbox_dims"] or [0, 0, 0]
            print(f"{r['hash']:>34} {r['total_verts']:>6} {r['total_faces']:>6} {r['mesh_count']:>6} "
                  f"{dims[0]:>8.4f} {dims[1]:>8.4f} {dims[2]:>8.4f} "
                  f"{r['aspect_lm'] or 0:>7.3f} {r['aspect_ls'] or 0:>7.3f}")

    print(f"\n--- All {len(matched)} matched bottles ---")
    print(f"{'Hash':>34} {'Verts':>6} {'Faces':>6} {'Meshes':>6} {'DimX':>8} {'DimY':>8} {'DimZ':>8} {'AR_LM':>7} {'AR_LS':>7}")
    for r in matched:
        dims = r["bbox_dims"] or [0, 0, 0]
        marker = " *" if r["hash"] in TARGETS else ""
        print(f"{r['hash']:>34} {r['total_verts']:>6} {r['total_faces']:>6} {r['mesh_count']:>6} "
              f"{dims[0]:>8.4f} {dims[1]:>8.4f} {dims[2]:>8.4f} "
              f"{r['aspect_lm'] or 0:>7.3f} {r['aspect_ls'] or 0:>7.3f}{marker}")

    # -----------------------------------------------------------------------
    # Step 4: Cluster matched bottles by similar aspect ratios
    # -----------------------------------------------------------------------
    print(f"\n{'='*70}")
    print("ASPECT RATIO CLUSTERING (tolerance=0.1)")
    print(f"{'='*70}")

    # Simple greedy clustering by aspect ratio similarity
    unassigned = list(matched)
    clusters = []
    AR_TOL = 0.1

    while unassigned:
        seed = unassigned.pop(0)
        cluster = [seed]
        remaining = []
        for r in unassigned:
            if (seed["aspect_lm"] is not None and r["aspect_lm"] is not None and
                abs(seed["aspect_lm"] - r["aspect_lm"]) < AR_TOL and
                abs(seed["aspect_ls"] - r["aspect_ls"]) < AR_TOL):
                cluster.append(r)
            else:
                remaining.append(r)
        unassigned = remaining
        clusters.append(cluster)

    clusters.sort(key=lambda c: -len(c))
    for i, cl in enumerate(clusters):
        target_in = [r["hash"] for r in cl if r["hash"] in TARGETS]
        marker = f"  <<< contains targets: {target_in}" if target_in else ""
        print(f"\nCluster {i} ({len(cl)} assets){marker}:")
        for r in cl[:10]:
            dims = r["sorted_dims"] or [0, 0, 0]
            t = " *" if r["hash"] in TARGETS else ""
            print(f"  {r['hash']} faces={r['total_faces']} dims=({dims[0]:.4f}, {dims[1]:.4f}, {dims[2]:.4f}) "
                  f"AR=({r['aspect_lm']:.3f}, {r['aspect_ls']:.3f}){t}")
        if len(cl) > 10:
            print(f"  ... and {len(cl)-10} more")

    # -----------------------------------------------------------------------
    # Step 5: Check dedup report for grouping
    # -----------------------------------------------------------------------
    print(f"\n{'='*70}")
    print("DEDUP REPORT CHECK")
    print(f"{'='*70}")

    matched_hashes = {r["hash"] for r in matched}

    if os.path.isfile(DEDUP_REPORT):
        with open(DEDUP_REPORT) as f:
            dedup = json.load(f)

        # Check which matched bottles appear in any dedup group
        grouped_hashes = set()
        for group in dedup["duplicates"]:
            group_hashes = set()
            for p in group["usd_paths"]:
                h = p.split("/")[-3]
                group_hashes.add(h)

            overlap = group_hashes & matched_hashes
            if overlap:
                print(f"\nDedup group (sig={group['sig']}, count={group['count']}):")
                print(f"  Matched bottles in group: {sorted(overlap)}")
                grouped_hashes |= overlap

        ungrouped = matched_hashes - grouped_hashes
        print(f"\n{len(grouped_hashes)}/{len(matched_hashes)} matched bottles appear in dedup groups")
        print(f"{len(ungrouped)} matched bottles are NOT in any dedup group (treated as unique)")

        # Check if any of the targets share the same dedup group
        target_sigs = {}
        for a in dedup["assets"]:
            h = a["usd_path"].split("/")[-3]
            if h in matched_hashes:
                target_sigs[h] = a.get("asset_geom_sig", "?")

        # Group by geom_sig
        sig_groups = defaultdict(list)
        for h, sig in target_sigs.items():
            sig_groups[sig].append(h)

        multi_sig = {s: hs for s, hs in sig_groups.items() if len(hs) > 1}
        if multi_sig:
            print(f"\nMatched bottles sharing same geom_sig:")
            for sig, hs in multi_sig.items():
                target_in = [h for h in hs if h in TARGETS]
                print(f"  sig={sig[:16]}...: {len(hs)} assets" +
                      (f" (includes targets: {target_in})" if target_in else ""))
        else:
            print(f"\nAll {len(target_sigs)} matched bottles have UNIQUE geom signatures")
            print("(The dedup system treats each as a distinct model)")
    else:
        print(f"Dedup report not found: {DEDUP_REPORT}")

    # -----------------------------------------------------------------------
    # Step 6: Summary
    # -----------------------------------------------------------------------
    print(f"\n{'='*70}")
    print("SUMMARY")
    print(f"{'='*70}")
    print(f"Total bottle assets scanned: {len(results)}")
    print(f"Assets with topology (779 verts, 3 meshes): {len(matched)}")
    print(f"Distinct face counts in matched set: {sorted(face_groups.keys())}")
    print(f"Aspect ratio clusters: {len(clusters)}")
    for i, cl in enumerate(clusters[:5]):
        target_in = [r['hash'] for r in cl if r['hash'] in TARGETS]
        print(f"  Cluster {i}: {len(cl)} assets" +
              (f" (targets: {target_in})" if target_in else ""))


if __name__ == "__main__":
    main()
