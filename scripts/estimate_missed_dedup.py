#!/usr/bin/env python3
"""Estimate assets missed by the current dedup algorithm across all categories.

Reads existing dedup reports (geom_only mode) and analyzes topology classes
to find assets that share topology (same vertex_count, face_count, mesh_count
per mesh) but were NOT grouped together by the current algorithm.

These represent potential duplicates missed due to:
- Vertex reordering (same mesh, different vertex order)
- Scale/position differences beyond merge tolerance
- Float quantization boundary effects

Output:
- check_reports/normalized_v2_dedup/cross_category_impact_estimate.json
- check_reports/normalized_v2_dedup/cross_category_impact_estimate.md
"""

import json
import os
import sys
from collections import defaultdict
from pathlib import Path

REPORTS_DIR = Path("check_reports/normalized_v2_dedup")
ASSETS_DIR = Path("GRScenes-test1-normalized/GRScenes_assets")
OUTPUT_JSON = REPORTS_DIR / "cross_category_impact_estimate.json"
OUTPUT_MD = REPORTS_DIR / "cross_category_impact_estimate.md"


def load_category_report(category: str):
    """Load the geom_only dedup report for a category."""
    report_path = REPORTS_DIR / category / f"{category}_asset_mesh_dedup_geom_only.json"
    if not report_path.exists():
        return None
    with open(report_path) as f:
        return json.load(f)


def build_grouped_set(report):
    """Build set of asset hashes that are in a dedup group (duplicates)."""
    grouped = set()
    for dup_group in report.get("duplicates", []):
        for usd_path in dup_group.get("usd_paths", []):
            # Extract hash from path like .../bottle/<hash>/usd/<hash>.usd
            parts = usd_path.split("/")
            # Find the hash — it's the directory name before /usd/
            for i, p in enumerate(parts):
                if p == "usd" and i > 0:
                    grouped.add(parts[i - 1])
                    break
    return grouped


def build_topology_key(asset_entry):
    """Build a topology key from an asset's mesh info.

    Returns a tuple of (mesh_count, sorted list of (vertex_count, face_count) per mesh).
    We sort per-mesh tuples to be order-invariant.
    """
    meshes = asset_entry.get("meshes", [])
    mesh_count = len(meshes)
    per_mesh = sorted((m["vertex_count"], m["face_count"]) for m in meshes)
    return (mesh_count, tuple(per_mesh))


def extract_hash(usd_path):
    """Extract asset hash from usd_path."""
    parts = usd_path.split("/")
    for i, p in enumerate(parts):
        if p == "usd" and i > 0:
            return parts[i - 1]
    return None


def analyze_category(category: str):
    """Analyze a single category for missed dedup candidates."""
    report = load_category_report(category)
    if report is None:
        return None

    assets = report.get("assets", [])
    duplicates = report.get("duplicates", [])
    meta = report.get("meta", {})

    total_assets = len(assets)
    if total_assets == 0:
        return None

    # Build set of grouped asset hashes
    grouped_hashes = build_grouped_set(report)

    # Also build a mapping: asset_hash -> dedup_group_sig
    hash_to_group = {}
    for dup_group in duplicates:
        sig = dup_group.get("sig", "")
        for usd_path in dup_group.get("usd_paths", []):
            h = extract_hash(usd_path)
            if h:
                hash_to_group[h] = sig

    # Use the existing asset_topo_sig from the report for topology grouping
    topo_groups = defaultdict(list)
    for asset in assets:
        topo_sig = asset.get("asset_topo_sig", "")
        asset_hash = extract_hash(asset.get("usd_path", ""))
        if not asset_hash or not topo_sig:
            continue

        # Also compute a simpler topology key for cross-validation
        topo_key = build_topology_key(asset)

        topo_groups[topo_sig].append({
            "hash": asset_hash,
            "topo_key": topo_key,
            "geom_sig": asset.get("asset_geom_sig", ""),
            "is_grouped": asset_hash in grouped_hashes,
            "group_sig": hash_to_group.get(asset_hash),
        })

    # Analyze each topology class
    topo_class_stats = []
    total_ungrouped_in_multi = 0
    total_in_multi_topo = 0

    for topo_sig, members in topo_groups.items():
        if len(members) < 2:
            continue

        grouped_count = sum(1 for m in members if m["is_grouped"])
        ungrouped_count = sum(1 for m in members if not m["is_grouped"])

        # Count distinct geom_sigs (these are distinct vertex arrangements)
        distinct_geom_sigs = len(set(m["geom_sig"] for m in members))

        # Count distinct dedup groups
        distinct_groups = len(set(m["group_sig"] for m in members if m["group_sig"]))

        total_in_multi_topo += len(members)
        total_ungrouped_in_multi += ungrouped_count

        topo_class_stats.append({
            "topo_sig": topo_sig,
            "total_members": len(members),
            "grouped_count": grouped_count,
            "ungrouped_count": ungrouped_count,
            "distinct_geom_sigs": distinct_geom_sigs,
            "distinct_dedup_groups": distinct_groups,
            "topo_key": str(members[0]["topo_key"]),
        })

    # Sort by ungrouped count desc
    topo_class_stats.sort(key=lambda x: x["ungrouped_count"], reverse=True)

    # Count assets that are ungrouped AND in a topo class with >=2 distinct geom sigs
    # (these are the real "missed" candidates — same topology, different vertex arrangement)
    missed_candidates = 0
    fragmented_groups = 0
    for tc in topo_class_stats:
        if tc["distinct_geom_sigs"] > 1:
            # Multiple distinct geometry signatures means potential duplicates
            # that differ only in vertex order/scale
            missed_candidates += tc["ungrouped_count"]
            if tc["distinct_dedup_groups"] > 1 or (tc["distinct_dedup_groups"] >= 1 and tc["ungrouped_count"] > 0):
                fragmented_groups += 1

    return {
        "category": category,
        "total_assets": total_assets,
        "existing_dedup_groups": len(duplicates),
        "existing_removable": sum(g.get("count", len(g.get("usd_paths", []))) - 1 for g in duplicates),
        "topo_classes_with_multi": len(topo_class_stats),
        "total_in_multi_topo": total_in_multi_topo,
        "ungrouped_in_multi_topo": total_ungrouped_in_multi,
        "missed_candidates": missed_candidates,
        "fragmented_topo_groups": fragmented_groups,
        "top_topo_classes": topo_class_stats[:5],
    }


def main():
    os.chdir(Path(__file__).resolve().parent.parent)

    categories = sorted([
        d.name for d in ASSETS_DIR.iterdir()
        if d.is_dir()
    ])

    print(f"Analyzing {len(categories)} categories...")

    results = []
    total_assets = 0
    total_existing_removable = 0
    total_missed = 0
    total_ungrouped_multi = 0
    total_fragmented = 0
    errors = []

    for i, cat in enumerate(categories):
        try:
            result = analyze_category(cat)
            if result is None:
                errors.append(f"{cat}: no report found or empty")
                continue
            results.append(result)
            total_assets += result["total_assets"]
            total_existing_removable += result["existing_removable"]
            total_missed += result["missed_candidates"]
            total_ungrouped_multi += result["ungrouped_in_multi_topo"]
            total_fragmented += result["fragmented_topo_groups"]
            if (i + 1) % 10 == 0:
                print(f"  [{i+1}/{len(categories)}] processed {cat}")
        except Exception as e:
            errors.append(f"{cat}: {e}")
            import traceback
            traceback.print_exc()

    # Sort by missed candidates
    results.sort(key=lambda x: x["missed_candidates"], reverse=True)

    # Global top 20 topology classes
    all_topo_classes = []
    for r in results:
        for tc in r.get("top_topo_classes", []):
            tc_copy = dict(tc)
            tc_copy["category"] = r["category"]
            all_topo_classes.append(tc_copy)

    # Re-collect ALL topo classes (not just top 5 per category) for global ranking
    all_topo_classes_full = []
    for cat in categories:
        report = load_category_report(cat)
        if not report:
            continue
        assets = report.get("assets", [])
        grouped_hashes = build_grouped_set(report)
        hash_to_group = {}
        for dup_group in report.get("duplicates", []):
            sig = dup_group.get("sig", "")
            for usd_path in dup_group.get("usd_paths", []):
                h = extract_hash(usd_path)
                if h:
                    hash_to_group[h] = sig

        topo_groups = defaultdict(list)
        for asset in assets:
            topo_sig = asset.get("asset_topo_sig", "")
            asset_hash = extract_hash(asset.get("usd_path", ""))
            if not asset_hash or not topo_sig:
                continue
            topo_groups[topo_sig].append({
                "hash": asset_hash,
                "geom_sig": asset.get("asset_geom_sig", ""),
                "is_grouped": asset_hash in grouped_hashes,
                "group_sig": hash_to_group.get(asset_hash),
                "topo_key": build_topology_key(asset),
            })

        for topo_sig, members in topo_groups.items():
            if len(members) < 2:
                continue
            grouped_count = sum(1 for m in members if m["is_grouped"])
            ungrouped_count = sum(1 for m in members if not m["is_grouped"])
            distinct_geom_sigs = len(set(m["geom_sig"] for m in members))
            distinct_groups = len(set(m["group_sig"] for m in members if m["group_sig"]))

            if distinct_geom_sigs > 1 and ungrouped_count > 0:
                all_topo_classes_full.append({
                    "category": cat,
                    "topo_sig": topo_sig,
                    "total_members": len(members),
                    "grouped_count": grouped_count,
                    "ungrouped_count": ungrouped_count,
                    "distinct_geom_sigs": distinct_geom_sigs,
                    "distinct_dedup_groups": distinct_groups,
                    "topo_key": str(members[0]["topo_key"]),
                })

    all_topo_classes_full.sort(key=lambda x: x["ungrouped_count"], reverse=True)
    top20_topo = all_topo_classes_full[:20]

    # Broader estimate: group by topo_key (vertex/face counts) instead of topo_sig
    # This captures assets with same mesh structure but different face index ordering
    broader_by_category = {}
    for cat in categories:
        report = load_category_report(cat)
        if not report:
            continue
        assets = report.get("assets", [])
        grouped_hashes = build_grouped_set(report)

        count_groups = defaultdict(list)
        for asset in assets:
            asset_hash = extract_hash(asset.get("usd_path", ""))
            if not asset_hash:
                continue
            topo_key = build_topology_key(asset)
            count_groups[topo_key].append({
                "hash": asset_hash,
                "is_grouped": asset_hash in grouped_hashes,
                "topo_sig": asset.get("asset_topo_sig", ""),
            })

        broader_missed = 0
        broader_classes = 0
        broader_missed_nontrivial = 0
        for topo_key, members in count_groups.items():
            if len(members) < 2:
                continue
            distinct_topo_sigs = len(set(m["topo_sig"] for m in members))
            if distinct_topo_sigs < 2:
                continue  # Only one topo_sig — already handled by narrow estimate
            ungrouped = sum(1 for m in members if not m["is_grouped"])
            # These are in same count class but different topo_sig — potential
            # face-reordered duplicates not caught by topo_sig grouping
            broader_missed += ungrouped
            broader_classes += 1
            # Non-trivial: total vertex count > 100
            total_verts = sum(vc for vc, fc in topo_key[1])
            if total_verts > 100:
                broader_missed_nontrivial += ungrouped

        broader_by_category[cat] = {
            "broader_missed": broader_missed,
            "broader_missed_nontrivial": broader_missed_nontrivial,
            "broader_classes": broader_classes,
        }

    total_broader_missed = sum(v["broader_missed"] for v in broader_by_category.values())
    total_broader_nontrivial = sum(v["broader_missed_nontrivial"] for v in broader_by_category.values())

    # Summary
    summary = {
        "total_categories": len(results),
        "total_assets": total_assets,
        "existing_removable_duplicates": total_existing_removable,
        "existing_dedup_rate_pct": round(total_existing_removable / total_assets * 100, 1) if total_assets else 0,
        "ungrouped_in_multi_topology_classes": total_ungrouped_multi,
        "estimated_missed_candidates": total_missed,
        "estimated_additional_dedup_rate_pct": round(total_missed / total_assets * 100, 1) if total_assets else 0,
        "combined_potential_dedup_rate_pct": round((total_existing_removable + total_missed) / total_assets * 100, 1) if total_assets else 0,
        "fragmented_topology_groups": total_fragmented,
        "broader_estimate_cross_topo_sig": total_broader_missed,
        "broader_nontrivial_cross_topo_sig": total_broader_nontrivial,
        "broader_combined_pct": round((total_existing_removable + total_missed + total_broader_missed) / total_assets * 100, 1) if total_assets else 0,
        "broader_nontrivial_combined_pct": round((total_existing_removable + total_missed + total_broader_nontrivial) / total_assets * 100, 1) if total_assets else 0,
        "per_category": [{
            "category": r["category"],
            "total_assets": r["total_assets"],
            "existing_removable": r["existing_removable"],
            "missed_candidates": r["missed_candidates"],
            "ungrouped_in_multi_topo": r["ungrouped_in_multi_topo"],
            "topo_classes_with_multi": r["topo_classes_with_multi"],
            "fragmented_topo_groups": r["fragmented_topo_groups"],
            "broader_missed": broader_by_category.get(r["category"], {}).get("broader_missed", 0),
        } for r in results],
        "top_20_topology_classes_by_ungrouped": top20_topo,
        "errors": errors,
    }

    # Write JSON
    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    with open(OUTPUT_JSON, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\nJSON written to {OUTPUT_JSON}")

    # Write MD
    md_lines = []
    md_lines.append("# Cross-Category Dedup Impact Estimate")
    md_lines.append("")
    md_lines.append("Estimate of assets potentially missed by the current dedup algorithm")
    md_lines.append("due to vertex reordering, scale differences, or float noise beyond merge tolerance.")
    md_lines.append("")
    md_lines.append("## Summary")
    md_lines.append("")
    md_lines.append(f"| Metric | Value |")
    md_lines.append(f"|--------|-------|")
    md_lines.append(f"| Categories analyzed | {summary['total_categories']} |")
    md_lines.append(f"| Total assets | {summary['total_assets']:,} |")
    md_lines.append(f"| Existing removable duplicates | {summary['existing_removable_duplicates']:,} ({summary['existing_dedup_rate_pct']}%) |")
    md_lines.append(f"| Ungrouped in multi-topology classes | {summary['ungrouped_in_multi_topology_classes']:,} |")
    md_lines.append(f"| **Estimated missed candidates** | **{summary['estimated_missed_candidates']:,}** ({summary['estimated_additional_dedup_rate_pct']}%) |")
    md_lines.append(f"| Combined potential dedup rate | {summary['combined_potential_dedup_rate_pct']}% |")
    md_lines.append(f"| Fragmented topology groups | {summary['fragmented_topology_groups']:,} |")
    md_lines.append(f"| Broader estimate (cross topo_sig, same counts) | {summary['broader_estimate_cross_topo_sig']:,} |")
    md_lines.append(f"| Broader non-trivial (>100 verts) | {summary['broader_nontrivial_cross_topo_sig']:,} |")
    md_lines.append(f"| Broader combined potential dedup rate | {summary['broader_combined_pct']}% |")
    md_lines.append(f"| Broader non-trivial combined rate | {summary['broader_nontrivial_combined_pct']}% |")
    md_lines.append("")
    md_lines.append("**Interpretation**: 'Missed candidates' are assets sharing identical topology")
    md_lines.append("(same mesh count, vertex count, face count per mesh) with other assets,")
    md_lines.append("but having different geometry signatures. These likely differ only in")
    md_lines.append("vertex ordering, minor scale, or float noise beyond the current merge tolerance.")
    md_lines.append("")

    md_lines.append("## Top Categories by Missed Candidates")
    md_lines.append("")
    md_lines.append("| Category | Total | Existing Dedup | Missed | Fragmented Groups |")
    md_lines.append("|----------|-------|---------------|--------|-------------------|")
    for r in results[:25]:
        if r["missed_candidates"] == 0:
            continue
        md_lines.append(
            f"| {r['category']} | {r['total_assets']:,} | {r['existing_removable']:,} | "
            f"{r['missed_candidates']:,} | {r['fragmented_topo_groups']} |"
        )
    md_lines.append("")

    md_lines.append("## Top 20 Topology Classes by Ungrouped Assets")
    md_lines.append("")
    md_lines.append("| # | Category | Members | Grouped | Ungrouped | Distinct Geom Sigs | Dedup Groups | Topology |")
    md_lines.append("|---|----------|---------|---------|-----------|-------------------|--------------|----------|")
    for i, tc in enumerate(top20_topo):
        # Shorten topo_key for display
        topo_display = tc["topo_key"]
        if len(topo_display) > 60:
            topo_display = topo_display[:57] + "..."
        md_lines.append(
            f"| {i+1} | {tc['category']} | {tc['total_members']} | {tc['grouped_count']} | "
            f"{tc['ungrouped_count']} | {tc['distinct_geom_sigs']} | "
            f"{tc['distinct_dedup_groups']} | `{topo_display}` |"
        )
    md_lines.append("")

    md_lines.append("## Per-Category Detail")
    md_lines.append("")
    md_lines.append("| Category | Total | Existing Removable | Missed | Ungrouped Multi-Topo | Multi-Topo Classes | Fragmented |")
    md_lines.append("|----------|-------|-------------------|--------|---------------------|-------------------|------------|")
    for r in sorted(results, key=lambda x: x["category"]):
        md_lines.append(
            f"| {r['category']} | {r['total_assets']:,} | {r['existing_removable']:,} | "
            f"{r['missed_candidates']:,} | {r['ungrouped_in_multi_topo']:,} | "
            f"{r['topo_classes_with_multi']:,} | {r['fragmented_topo_groups']} |"
        )
    md_lines.append("")

    if errors:
        md_lines.append("## Errors")
        md_lines.append("")
        for e in errors:
            md_lines.append(f"- {e}")
        md_lines.append("")

    with open(OUTPUT_MD, "w") as f:
        f.write("\n".join(md_lines))
    print(f"MD written to {OUTPUT_MD}")

    # Print quick summary
    print(f"\n{'='*60}")
    print(f"SUMMARY")
    print(f"{'='*60}")
    print(f"Total assets:                    {total_assets:,}")
    print(f"Existing removable:              {total_existing_removable:,} ({summary['existing_dedup_rate_pct']}%)")
    print(f"Estimated missed candidates:     {total_missed:,} ({summary['estimated_additional_dedup_rate_pct']}%)")
    print(f"Combined potential dedup rate:    {summary['combined_potential_dedup_rate_pct']}%")
    print(f"Fragmented topology groups:      {total_fragmented}")
    print(f"Broader (cross topo_sig):        {total_broader_missed:,} additional (all)")
    print(f"Broader non-trivial (>100v):     {total_broader_nontrivial:,} additional")
    print(f"Broader combined dedup rate:      {summary['broader_combined_pct']}% (all) / {summary['broader_nontrivial_combined_pct']}% (non-trivial)")
    print(f"\nTop 5 categories by missed:")
    for r in results[:5]:
        print(f"  {r['category']:20s}  {r['missed_candidates']:5d} missed / {r['total_assets']:5d} total")


if __name__ == "__main__":
    main()
