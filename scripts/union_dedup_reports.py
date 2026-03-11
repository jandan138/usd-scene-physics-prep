#!/usr/bin/env python3
"""Union merge of geom_only and shape_invariant dedup reports for the same category.

Computes the UNION of removable asset sets from both modes, producing a merged
report that maximizes deduplication coverage.

Usage:
    # Single category
    python scripts/union_dedup_reports.py \
      --geom-only check_reports/normalized_v2_dedup/bottle/bottle_asset_mesh_dedup_geom_only.json \
      --shape-invariant check_reports/shape_invariant_test/bottle/bottle_asset_mesh_dedup_shape_invariant.json \
      --output check_reports/union_merged/bottle/bottle_union_merged.json

    # Batch mode
    python scripts/union_dedup_reports.py \
      --batch \
      --geom-dir check_reports/normalized_v2_dedup/ \
      --shape-dir check_reports/shape_invariant_test/ \
      --output-dir check_reports/union_merged/ \
      --summary check_reports/union_merged/summary.json
"""

import argparse
import glob
import json
import os
import time


def normalize_path(p):
    """Strip leading ./ and normalize to consistent relative form."""
    if p.startswith("./"):
        p = p[2:]
    return p


def extract_removable_and_groups(report):
    """Extract removable set and group mappings from a dedup report.

    Returns:
        removable: set of normalized paths that are removable
        asset_to_group: dict mapping each asset path to its group index
        groups: list of (representative_path, [member_paths]) tuples
    """
    removable = set()
    asset_to_group = {}
    groups = []

    for i, g in enumerate(report.get("duplicates", [])):
        paths = [normalize_path(p) for p in g["usd_paths"]]
        representative = paths[0]
        groups.append((representative, paths))
        for p in paths:
            asset_to_group[p] = i
        for p in paths[1:]:
            removable.add(p)

    return removable, asset_to_group, groups


class UnionFind:
    """Simple union-find for merging groups."""

    def __init__(self):
        self._parent = {}

    def find(self, x):
        if x not in self._parent:
            self._parent[x] = x
        while self._parent[x] != x:
            self._parent[x] = self._parent[self._parent[x]]
            x = self._parent[x]
        return x

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self._parent[rb] = ra


def union_merge(geom_report, shape_report):
    """Merge geom_only and shape_invariant reports by union of removable sets.

    Uses union-find to merge groups that share assets across the two modes.
    For each shape_invariant group, we link its representative with every member,
    then also link with the geom_only group representative of shared members.
    Finally, we collect connected components and pick a non-removable representative.
    """
    geom_rem, geom_a2g, geom_groups = extract_removable_and_groups(geom_report)
    shape_rem, shape_a2g, shape_groups = extract_removable_and_groups(shape_report)

    union_removable = geom_rem | shape_rem
    shape_only_adds = shape_rem - geom_rem

    # Build a union-find over all assets that appear in any duplicate group.
    uf = UnionFind()

    # Add all geom_only groups: link all members to their representative.
    for rep, members in geom_groups:
        for m in members:
            uf.union(rep, m)

    # For each shape-only add, link it into its shape_invariant group's
    # representative. This may merge two previously separate geom groups
    # if the shape group spans them.
    for path in shape_only_adds:
        if path not in shape_a2g:
            continue
        shape_gidx = shape_a2g[path]
        shape_rep = shape_groups[shape_gidx][0]
        # Link the shape-only add to the shape representative
        uf.union(shape_rep, path)

    # Collect connected components
    components = {}  # root -> list of members
    all_grouped = set()
    for _, members in geom_groups:
        for m in members:
            all_grouped.add(m)
    for path in shape_only_adds:
        all_grouped.add(path)
        # Also add the shape representative so it's in the component
        if path in shape_a2g:
            shape_rep = shape_groups[shape_a2g[path]][0]
            all_grouped.add(shape_rep)

    for asset in all_grouped:
        root = uf.find(asset)
        components.setdefault(root, []).append(asset)

    # For each component, pick a representative (first non-removable asset),
    # then all others that are in union_removable become removable.
    output_duplicates = []
    actual_removable = set()

    for root, members in components.items():
        if len(members) < 2:
            continue

        # Separate into non-removable (keepers) and removable
        keepers = [m for m in members if m not in union_removable]
        removables = [m for m in members if m in union_removable]

        if not removables:
            # No removable assets in this component — skip
            continue

        if not keepers:
            # All members are removable; pick one as representative (keep it)
            # Prefer geom_only representatives
            chosen = None
            for m in members:
                if m in geom_a2g:
                    gidx = geom_a2g[m]
                    if geom_groups[gidx][0] == m:
                        chosen = m
                        break
            if chosen is None:
                chosen = members[0]
            keepers = [chosen]
            removables = [m for m in members if m != chosen]

        # Build group: representative first, then removables, then other keepers
        rep = keepers[0]
        group_paths = [rep] + sorted(removables) + sorted(keepers[1:])

        output_duplicates.append({
            "sig": "",
            "count": len(group_paths),
            "usd_paths": group_paths,
        })
        for p in removables:
            actual_removable.add(p)

    # Sort groups for deterministic output, then number them
    output_duplicates.sort(key=lambda g: g["usd_paths"][0])
    for i, g in enumerate(output_duplicates):
        g["sig"] = f"union_merge_{i}"

    # Validation: no asset is both representative and removable
    representatives = {g["usd_paths"][0] for g in output_duplicates}
    conflicts = representatives & actual_removable
    if conflicts:
        print(f"WARNING: {len(conflicts)} assets are both representative and removable!")
        for c in sorted(conflicts)[:5]:
            print(f"  - {c}")

    # Build meta
    geom_meta = geom_report.get("meta", {})
    meta = {
        "dataset": geom_meta.get("dataset", ""),
        "mode": "union_merged",
        "source_modes": ["geom_only", "shape_invariant"],
        "geom_only_removable": len(geom_rem),
        "shape_invariant_removable": len(shape_rem),
        "union_removable": len(actual_removable),
        "geom_only_unique_adds": len(geom_rem - shape_rem),
        "shape_invariant_unique_adds": len(shape_only_adds),
        "asset_usd_count": geom_meta.get("asset_usd_count", 0),
        "duplicate_group_count": len(output_duplicates),
        "generated_at_unix": int(time.time()),
    }

    result = {
        "meta": meta,
        "duplicates": output_duplicates,
        "assets": geom_report.get("assets", []),
        "errors": geom_report.get("errors", []) + shape_report.get("errors", []),
    }

    return result


def run_single(geom_path, shape_path, output_path):
    """Run union merge for a single category pair."""
    with open(geom_path) as f:
        geom_report = json.load(f)
    with open(shape_path) as f:
        shape_report = json.load(f)

    result = union_merge(geom_report, shape_report)

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w") as f:
        json.dump(result, f, indent=2)

    meta = result["meta"]
    total = meta["asset_usd_count"]
    removable = meta["union_removable"]
    pct = removable / total * 100 if total else 0
    print(f"[{meta['dataset']}] union removable: {removable}/{total} ({pct:.1f}%)")
    print(f"  geom_only: {meta['geom_only_removable']}, "
          f"shape_invariant: {meta['shape_invariant_removable']}, "
          f"shape-only adds: {meta['shape_invariant_unique_adds']}")
    print(f"  groups: {meta['duplicate_group_count']}")
    print(f"  output: {output_path}")

    return result


def run_batch(geom_dir, shape_dir, output_dir, summary_path=None):
    """Run union merge for all matching categories across two directories."""
    # Find shape_invariant reports
    shape_pattern = os.path.join(shape_dir, "*", "*_asset_mesh_dedup_shape_invariant.json")
    shape_files = sorted(glob.glob(shape_pattern))

    if not shape_files:
        print(f"No shape_invariant reports found matching {shape_pattern}")
        return

    summary = {"categories": {}, "totals": {}}
    total_geom = 0
    total_shape = 0
    total_union = 0
    total_assets = 0

    for shape_path in shape_files:
        category = os.path.basename(os.path.dirname(shape_path))
        geom_path = os.path.join(
            geom_dir, category,
            f"{category}_asset_mesh_dedup_geom_only.json"
        )
        if not os.path.exists(geom_path):
            print(f"[{category}] SKIP: no geom_only report at {geom_path}")
            continue

        output_path = os.path.join(
            output_dir, category,
            f"{category}_union_merged.json"
        )

        print(f"\n--- {category} ---")
        result = run_single(geom_path, shape_path, output_path)
        meta = result["meta"]

        summary["categories"][category] = {
            "asset_usd_count": meta["asset_usd_count"],
            "geom_only_removable": meta["geom_only_removable"],
            "shape_invariant_removable": meta["shape_invariant_removable"],
            "union_removable": meta["union_removable"],
            "shape_invariant_unique_adds": meta["shape_invariant_unique_adds"],
            "duplicate_group_count": meta["duplicate_group_count"],
        }
        total_geom += meta["geom_only_removable"]
        total_shape += meta["shape_invariant_removable"]
        total_union += meta["union_removable"]
        total_assets += meta["asset_usd_count"]

    summary["totals"] = {
        "categories_processed": len(summary["categories"]),
        "total_assets": total_assets,
        "total_geom_only_removable": total_geom,
        "total_shape_invariant_removable": total_shape,
        "total_union_removable": total_union,
        "total_shape_only_adds": total_union - total_geom,
        "union_dedup_rate": f"{total_union / total_assets * 100:.1f}%" if total_assets else "0%",
    }

    print(f"\n=== BATCH SUMMARY ===")
    print(f"Categories: {len(summary['categories'])}")
    print(f"Total assets: {total_assets}")
    print(f"Geom-only removable: {total_geom}")
    print(f"Shape-invariant removable: {total_shape}")
    print(f"Union removable: {total_union} (shape-only adds: {total_union - total_geom})")
    if total_assets:
        print(f"Union dedup rate: {total_union / total_assets * 100:.1f}%")

    if summary_path:
        os.makedirs(os.path.dirname(summary_path) or ".", exist_ok=True)
        with open(summary_path, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"Summary written to {summary_path}")

    return summary


def main():
    parser = argparse.ArgumentParser(
        description="Union merge geom_only + shape_invariant dedup reports"
    )
    parser.add_argument("--batch", action="store_true",
                        help="Batch mode: process all matching categories")

    # Single mode
    parser.add_argument("--geom-only", help="Path to geom_only report JSON")
    parser.add_argument("--shape-invariant", help="Path to shape_invariant report JSON")
    parser.add_argument("--output", help="Output path for merged report")

    # Batch mode
    parser.add_argument("--geom-dir", help="Directory containing geom_only reports")
    parser.add_argument("--shape-dir", help="Directory containing shape_invariant reports")
    parser.add_argument("--output-dir", help="Output directory for merged reports")
    parser.add_argument("--summary", help="Path for batch summary JSON")

    args = parser.parse_args()

    if args.batch:
        if not all([args.geom_dir, args.shape_dir, args.output_dir]):
            parser.error("--batch requires --geom-dir, --shape-dir, and --output-dir")
        run_batch(args.geom_dir, args.shape_dir, args.output_dir, args.summary)
    else:
        if not all([args.geom_only, args.shape_invariant, args.output]):
            parser.error("Single mode requires --geom-only, --shape-invariant, and --output")
        run_single(args.geom_only, args.shape_invariant, args.output)


if __name__ == "__main__":
    main()
