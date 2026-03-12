#!/usr/bin/env python3
"""Union merge of dedup reports for the same category.

Computes the UNION of removable asset sets from multiple modes, producing a merged
report that maximizes deduplication coverage.

Usage:
    # Single category (2-way: geom_only + shape_invariant)
    python scripts/union_dedup_reports.py \
      --geom-only check_reports/normalized_v2_dedup/bottle/bottle_asset_mesh_dedup_geom_only.json \
      --shape-invariant check_reports/shape_invariant_test/bottle/bottle_asset_mesh_dedup_shape_invariant.json \
      --output check_reports/union_merged/bottle/bottle_union_merged.json

    # N-way merge (arbitrary number of reports)
    python scripts/union_dedup_reports.py \
      --reports report_a.json report_b.json report_c.json \
      --output check_reports/union_merged/bottle/bottle_union_merged.json

    # Batch mode
    python scripts/union_dedup_reports.py \
      --batch \
      --geom-dir check_reports/normalized_v2_dedup/ \
      --shape-dir check_reports/shape_invariant_test/ \
      [--topo-dir check_reports/topo_filesize/] \
      --output-dir check_reports/union_merged/ \
      --summary check_reports/union_merged/summary.json
"""

import argparse
import glob
import json
import os
import time
from typing import List


def normalize_path(p):
    """Normalize asset path to consistent relative report-style form.

    Handles absolute paths by extracting the dataset-relative portion starting
    from the dataset name (detected via GRScenes_assets marker).  Also strips
    leading './' for plain relative paths.

    This is critical for union merge correctness: geom_only reports typically use
    relative paths while shape_invariant reports may use absolute paths.  Without
    normalization the union-find treats the same physical asset as two different
    strings, causing transitive canonical conflicts in downstream C1 pipelines.
    """
    p = p.replace("\\", "/").strip()
    if p.startswith("./"):
        p = p[2:]

    # If the path is absolute and contains a known dataset marker, convert to
    # relative report-style: <dataset_name>/GRScenes_assets/<cat>/<uid>/...
    if "/" in p:
        marker = "/GRScenes_assets/"
        idx = p.find(marker)
        if idx >= 0:
            # Walk backwards from the marker to find the dataset directory name.
            # The dataset name is the path component immediately before GRScenes_assets.
            prefix = p[:idx]
            # Find last '/' in prefix to get dataset dir name
            slash = prefix.rfind("/")
            if slash >= 0:
                dataset_name = prefix[slash + 1:]
            else:
                dataset_name = prefix
            if dataset_name:
                p = dataset_name + marker + p[idx + len(marker):]

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

    # Link ALL shape_invariant group members (not just shape_only_adds).
    # This ensures full transitive closure: if a shape group spans multiple
    # geom groups, the union-find merges them into one connected component.
    for rep, members in shape_groups:
        for m in members:
            uf.union(rep, m)

    # Collect connected components
    components = {}  # root -> list of members
    all_grouped = set()
    for _, members in geom_groups:
        all_grouped.update(members)
    for _, members in shape_groups:
        all_grouped.update(members)

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


def union_merge_n(reports: List[dict]) -> dict:
    """Merge N dedup reports by union of all groups via union-find.

    Each report contributes its duplicate groups to a shared union-find.
    Connected components form the merged groups.
    """
    uf = UnionFind()
    all_grouped = set()
    all_removable = set()

    # Track per-mode stats
    mode_stats = {}
    asset_count = 0
    all_errors = []
    all_assets = []

    for report in reports:
        mode = report.get("meta", {}).get("mode", "unknown")
        _, _, groups = extract_removable_and_groups(report)

        removable_count = 0
        for rep, members in groups:
            for m in members:
                uf.union(rep, m)
                all_grouped.add(m)
            for m in members[1:]:
                all_removable.add(m)
                removable_count += 1

        mode_stats[mode] = {
            "removable": removable_count,
            "group_count": len(groups),
        }

        meta = report.get("meta", {})
        if meta.get("asset_usd_count", 0) > asset_count:
            asset_count = meta["asset_usd_count"]
            all_assets = report.get("assets", [])
        all_errors.extend(report.get("errors", []))

    # Collect connected components
    components = {}
    for asset in all_grouped:
        root = uf.find(asset)
        components.setdefault(root, []).append(asset)

    # Build output groups
    output_duplicates = []
    actual_removable = set()

    for root, members in components.items():
        if len(members) < 2:
            continue

        keepers = [m for m in members if m not in all_removable]
        removables = [m for m in members if m in all_removable]

        if not removables:
            continue

        if not keepers:
            chosen = members[0]
            keepers = [chosen]
            removables = [m for m in members if m != chosen]

        rep = keepers[0]
        group_paths = [rep] + sorted(removables) + sorted(keepers[1:])
        output_duplicates.append({
            "sig": "",
            "count": len(group_paths),
            "usd_paths": group_paths,
        })
        for p in removables:
            actual_removable.add(p)

    output_duplicates.sort(key=lambda g: g["usd_paths"][0])
    for i, g in enumerate(output_duplicates):
        g["sig"] = f"union_merge_{i}"

    # Validation
    representatives = {g["usd_paths"][0] for g in output_duplicates}
    conflicts = representatives & actual_removable
    if conflicts:
        print(f"WARNING: {len(conflicts)} assets are both representative and removable!")
        for c in sorted(conflicts)[:5]:
            print(f"  - {c}")

    meta = {
        "dataset": "",
        "mode": "union_merged",
        "source_modes": list(mode_stats.keys()),
        "per_mode_stats": mode_stats,
        "union_removable": len(actual_removable),
        "asset_usd_count": asset_count,
        "duplicate_group_count": len(output_duplicates),
        "generated_at_unix": int(time.time()),
    }

    # Try to get dataset from first report
    for report in reports:
        ds = report.get("meta", {}).get("dataset", "")
        if ds:
            meta["dataset"] = ds
            break

    return {
        "meta": meta,
        "duplicates": output_duplicates,
        "assets": all_assets,
        "errors": all_errors,
    }


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


def run_batch(geom_dir, shape_dir, output_dir, summary_path=None, topo_dir=None):
    """Run union merge for all matching categories across directories.

    When topo_dir is provided and a topo_filesize report exists for a category,
    uses union_merge_n() to merge all available reports (geom_only, shape_invariant,
    topo_filesize) instead of the 2-way union_merge().
    """
    # Find shape_invariant reports
    shape_pattern = os.path.join(shape_dir, "*", "*_asset_mesh_dedup_shape_invariant.json")
    shape_files = sorted(glob.glob(shape_pattern))

    if not shape_files:
        print(f"No shape_invariant reports found matching {shape_pattern}")
        return

    summary = {"categories": {}, "totals": {}}
    total_union = 0
    total_assets = 0
    per_mode_totals = {}

    for sf in shape_files:
        category = os.path.basename(os.path.dirname(sf))
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

        # Check for topo_filesize report
        topo_path = None
        if topo_dir:
            topo_path = os.path.join(
                topo_dir, category,
                f"{category}_asset_mesh_dedup_topo_filesize.json"
            )
            if not os.path.exists(topo_path):
                topo_path = None

        print(f"\n--- {category} ---")

        if topo_path:
            # N-way merge with all available reports
            report_paths = [geom_path, sf, topo_path]
            reports = []
            for rp in report_paths:
                with open(rp) as f:
                    reports.append(json.load(f))
            result = union_merge_n(reports)
            os.makedirs(os.path.dirname(output_path), exist_ok=True)
            with open(output_path, "w") as f:
                json.dump(result, f, indent=2)
            meta = result["meta"]
            total = meta["asset_usd_count"]
            removable = meta["union_removable"]
            pct = removable / total * 100 if total else 0
            print(f"[{meta['dataset']}] N-way union removable: {removable}/{total} ({pct:.1f}%)")
            print(f"  Source modes: {meta['source_modes']}")
            print(f"  Groups: {meta['duplicate_group_count']}")
        else:
            # 2-way merge (legacy path)
            result = run_single(geom_path, sf, output_path)

        meta = result["meta"]
        cat_stats = {
            "asset_usd_count": meta["asset_usd_count"],
            "union_removable": meta["union_removable"],
            "duplicate_group_count": meta["duplicate_group_count"],
            "source_modes": meta.get("source_modes", ["geom_only", "shape_invariant"]),
        }

        # Include per-mode stats if available
        if "per_mode_stats" in meta:
            cat_stats["per_mode_stats"] = meta["per_mode_stats"]
        else:
            cat_stats["geom_only_removable"] = meta.get("geom_only_removable", 0)
            cat_stats["shape_invariant_removable"] = meta.get("shape_invariant_removable", 0)

        summary["categories"][category] = cat_stats
        total_union += meta["union_removable"]
        total_assets += meta["asset_usd_count"]

        # Accumulate per-mode totals
        if "per_mode_stats" in meta:
            for mode, stats in meta["per_mode_stats"].items():
                per_mode_totals.setdefault(mode, 0)
                per_mode_totals[mode] += stats["removable"]
        else:
            per_mode_totals.setdefault("geom_only", 0)
            per_mode_totals["geom_only"] += meta.get("geom_only_removable", 0)
            per_mode_totals.setdefault("shape_invariant", 0)
            per_mode_totals["shape_invariant"] += meta.get("shape_invariant_removable", 0)

    summary["totals"] = {
        "categories_processed": len(summary["categories"]),
        "total_assets": total_assets,
        "total_union_removable": total_union,
        "per_mode_removable": per_mode_totals,
        "union_dedup_rate": f"{total_union / total_assets * 100:.1f}%" if total_assets else "0%",
    }

    print(f"\n=== BATCH SUMMARY ===")
    print(f"Categories: {len(summary['categories'])}")
    print(f"Total assets: {total_assets}")
    for mode, count in sorted(per_mode_totals.items()):
        print(f"  {mode} removable: {count}")
    print(f"Union removable: {total_union}")
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
        description="Union merge dedup reports (2-way or N-way)"
    )
    parser.add_argument("--batch", action="store_true",
                        help="Batch mode: process all matching categories")

    # Single mode (2-way legacy)
    parser.add_argument("--geom-only", help="Path to geom_only report JSON")
    parser.add_argument("--shape-invariant", help="Path to shape_invariant report JSON")
    parser.add_argument("--output", help="Output path for merged report")

    # N-way merge
    parser.add_argument("--reports", nargs="+",
                        help="List of report JSON paths for N-way merge "
                             "(alternative to --geom-only/--shape-invariant)")

    # Batch mode
    parser.add_argument("--geom-dir", help="Directory containing geom_only reports")
    parser.add_argument("--shape-dir", help="Directory containing shape_invariant reports")
    parser.add_argument("--topo-dir", help="Directory containing topo_filesize reports (optional)")
    parser.add_argument("--output-dir", help="Output directory for merged reports")
    parser.add_argument("--summary", help="Path for batch summary JSON")

    args = parser.parse_args()

    if args.batch:
        if not all([args.geom_dir, args.shape_dir, args.output_dir]):
            parser.error("--batch requires --geom-dir, --shape-dir, and --output-dir")
        run_batch(args.geom_dir, args.shape_dir, args.output_dir, args.summary,
                  topo_dir=args.topo_dir)
    elif args.reports:
        if not args.output:
            parser.error("--reports requires --output")
        reports = []
        for rp in args.reports:
            with open(rp) as f:
                reports.append(json.load(f))
        result = union_merge_n(reports)
        os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
        with open(args.output, "w") as f:
            json.dump(result, f, indent=2)
        meta = result["meta"]
        total = meta["asset_usd_count"]
        removable = meta["union_removable"]
        pct = removable / total * 100 if total else 0
        print(f"[{meta['dataset']}] N-way union removable: {removable}/{total} ({pct:.1f}%)")
        print(f"  Source modes: {meta['source_modes']}")
        print(f"  Groups: {meta['duplicate_group_count']}")
        print(f"  Output: {args.output}")
    else:
        if not all([args.geom_only, args.shape_invariant, args.output]):
            parser.error("Single mode requires --geom-only, --shape-invariant, and --output")
        run_single(args.geom_only, args.shape_invariant, args.output)


if __name__ == "__main__":
    main()
