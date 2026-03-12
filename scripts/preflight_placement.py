#!/usr/bin/env python3
"""Preflight validation for Plan C dedup placement patch.

READ-ONLY script -- never modifies any USD or data files.

Validates 6 metrics before running patch_dedup_placement.py:
1. Earliest pre_c1 backup selection per scene
2. Old-ref locatability rate (backup or live)
3. Mapping coverage rate (changed refs covered by consolidated mapping)
4. Current-ref consistency rate (current refs match mapping canonical)
5. Canonical live-member resolution rate
6. /Root/Instance anomaly bucketing

Run via: ./scripts/isaac_python.sh scripts/preflight_placement.py [options]
"""

from __future__ import annotations

import argparse
import fnmatch
import json
import logging
import os
import re
import sys
import time
from collections import defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

try:
    from pxr import Gf, Sdf, Usd, UsdGeom

    _PXR_AVAILABLE = True
except Exception:
    _PXR_AVAILABLE = False

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("preflight")

PASS_THRESHOLD = 0.95


# ---------------------------------------------------------------------------
# Utility helpers
# ---------------------------------------------------------------------------

def _norm(p: str) -> str:
    """Normalize a path to absolute POSIX form."""
    return os.path.normpath(os.path.abspath(p)).replace("\\", "/")


def _resolve_mapping_path(subset_root: str, p: str) -> str:
    """Resolve a mapping path (relative or absolute) to absolute."""
    p = p.replace("\\", "/")
    if os.path.isabs(p):
        return _norm(p)
    p = p.lstrip("./")

    # "GRScenes-test1-normalized/GRScenes_assets/..." -> join with parent of subset_root
    subset_name = os.path.basename(subset_root.rstrip("/"))
    if subset_name and p.startswith(subset_name + "/"):
        return _norm(os.path.join(os.path.dirname(subset_root), p))
    # "GRScenes_assets/..." -> join with subset_root
    if p.startswith("GRScenes_assets/"):
        return _norm(os.path.join(subset_root, p))
    return _norm(os.path.join(subset_root, p))


def _resolve_ref_asset_path(base_dir: str, asset_path: str) -> str:
    """Resolve a USD reference asset path to absolute."""
    if not asset_path:
        return ""
    s = asset_path.replace("\\", "/")
    if os.path.isabs(s):
        return _norm(s)
    return _norm(os.path.join(base_dir, s))


# ---------------------------------------------------------------------------
# Phase 0: Load consolidated mapping
# ---------------------------------------------------------------------------

def load_consolidated_mapping(mapping_dir: str, subset_root: str) -> Dict[str, str]:
    """Load all *_geom_only_mapping.json files into {old_abs: canonical_abs}."""
    mapping: Dict[str, str] = {}
    mapping_dir = _norm(mapping_dir)

    for fname in sorted(os.listdir(mapping_dir)):
        if not fname.endswith("_geom_only_mapping.json"):
            continue
        fpath = os.path.join(mapping_dir, fname)
        try:
            with open(fpath, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            log.warning("Failed to load mapping %s: %s", fpath, e)
            continue

        if not isinstance(data, dict):
            continue
        for old_p, canon_p in data.items():
            if not isinstance(old_p, str) or not isinstance(canon_p, str):
                continue
            old_abs = _resolve_mapping_path(subset_root, old_p)
            canon_abs = _resolve_mapping_path(subset_root, canon_p)
            if old_abs != canon_abs:
                mapping[old_abs] = canon_abs

    log.info("Loaded consolidated mapping: %d old->canonical pairs from %s", len(mapping), mapping_dir)
    return mapping


# ---------------------------------------------------------------------------
# Phase 0b: Build old-asset locator from _dedup_assets/ backups
# ---------------------------------------------------------------------------

def build_old_asset_locator(bak_root: str, subset_root: str) -> Dict[str, str]:
    """Build {original_abs_path: backup_abs_path} from _dedup_assets/ backup dirs.

    Backup structure: <bak_root>/_dedup_assets/<batch_label>/GRScenes_assets/<category>/<hash>/usd/<hash>.usd
    Original would be: <subset_root>/GRScenes_assets/<category>/<hash>/usd/<hash>.usd
    """
    locator: Dict[str, str] = {}
    dedup_dir = os.path.join(_norm(bak_root), "_dedup_assets")
    if not os.path.isdir(dedup_dir):
        log.warning("_dedup_assets dir not found: %s", dedup_dir)
        return locator

    subset_root = _norm(subset_root)

    for batch_label in sorted(os.listdir(dedup_dir)):
        batch_dir = os.path.join(dedup_dir, batch_label)
        assets_dir = os.path.join(batch_dir, "GRScenes_assets")
        if not os.path.isdir(assets_dir):
            continue
        for category in os.listdir(assets_dir):
            cat_dir = os.path.join(assets_dir, category)
            if not os.path.isdir(cat_dir):
                continue
            for hash_dir in os.listdir(cat_dir):
                usd_dir = os.path.join(cat_dir, hash_dir, "usd")
                if not os.path.isdir(usd_dir):
                    continue
                for usd_file in os.listdir(usd_dir):
                    if not usd_file.endswith(".usd"):
                        continue
                    backup_path = _norm(os.path.join(usd_dir, usd_file))
                    # Reconstruct original path
                    original_path = _norm(os.path.join(
                        subset_root, "GRScenes_assets", category, hash_dir, "usd", usd_file
                    ))
                    locator[original_path] = backup_path

    log.info("Built old-asset locator: %d entries from %s", len(locator), dedup_dir)
    return locator


# ---------------------------------------------------------------------------
# Scene discovery
# ---------------------------------------------------------------------------

def discover_scenes(dataset_root: str, scene_filter: Optional[str] = None) -> List[str]:
    """Return sorted list of scene directory absolute paths."""
    scenes_root = os.path.join(_norm(dataset_root), "GRScenes100", "home")
    if not os.path.isdir(scenes_root):
        log.error("Scenes root not found: %s", scenes_root)
        return []

    scene_dirs = []
    for d in sorted(os.listdir(scenes_root)):
        full = os.path.join(scenes_root, d)
        if not os.path.isdir(full):
            continue
        if scene_filter and not fnmatch.fnmatch(d, scene_filter):
            continue
        scene_dirs.append(full)

    log.info("Discovered %d scene(s) %s", len(scene_dirs),
             f"(filter={scene_filter})" if scene_filter else "")
    return scene_dirs


# ---------------------------------------------------------------------------
# Metric 1: Earliest pre_c1 selection
# ---------------------------------------------------------------------------

def find_earliest_pre_c1(scene_dir: str) -> Optional[Tuple[str, str]]:
    """Find the earliest .pre_c1_*.usd file. Returns (filename, timestamp) or None."""
    pre_c1_files = []
    for f in os.listdir(scene_dir):
        if f.startswith("layout.pre_c1_") and f.endswith(".usd"):
            pre_c1_files.append(f)

    if not pre_c1_files:
        return None

    # Sort by the timestamp portion: layout.pre_c1_<label>.<timestamp>.usd
    # Timestamps are like 20260311_140031
    def extract_timestamp(fname: str) -> str:
        # pattern: layout.pre_c1_<label>.<timestamp>.usd
        m = re.search(r'\.(\d{8}_\d{6})\.usd$', fname)
        return m.group(1) if m else "99999999_999999"

    pre_c1_files.sort(key=extract_timestamp)
    earliest = pre_c1_files[0]
    ts = extract_timestamp(earliest)
    return earliest, ts


# ---------------------------------------------------------------------------
# Reading references from a USD layout file (Sdf layer inspection)
# ---------------------------------------------------------------------------

def read_layout_refs(layout_path: str) -> Dict[str, str]:
    """Read all prim references from a layout USD.

    Returns {prim_path: resolved_abs_asset_path} for prims with references.
    Uses Sdf layer inspection like the rewrite script.
    """
    refs: Dict[str, str] = {}
    base_dir = os.path.dirname(layout_path)

    try:
        stage = Usd.Stage.Open(layout_path, load=Usd.Stage.LoadNone)
    except Exception as e:
        log.warning("Failed to open %s: %s", layout_path, e)
        return refs

    if stage is None:
        return refs

    for prim in stage.Traverse():
        if not prim or not prim.IsValid():
            continue

        prim_path = str(prim.GetPath())

        # Check references
        if prim.HasAuthoredReferences():
            ref_meta = prim.GetMetadata("references")
            if ref_meta:
                items = _get_ref_items(ref_meta)
                for ref_item in items:
                    ap = getattr(ref_item, "assetPath", "") or ""
                    if ap:
                        resolved = _resolve_ref_asset_path(base_dir, ap)
                        refs[prim_path] = resolved
                        break  # take first reference per prim

        # Check payloads as fallback
        if prim_path not in refs and prim.HasAuthoredPayloads():
            pl_meta = prim.GetMetadata("payloads")
            if pl_meta:
                items = _get_payload_items(pl_meta)
                for pl_item in items:
                    ap = getattr(pl_item, "assetPath", "") or ""
                    if ap:
                        resolved = _resolve_ref_asset_path(base_dir, ap)
                        refs[prim_path] = resolved
                        break

    return refs


def _get_ref_items(refs) -> list:
    """Extract reference items from a ListOp."""
    try:
        return list(refs.GetAddedOrExplicitItems())
    except Exception:
        return []


def _get_payload_items(pls) -> list:
    """Extract payload items from a ListOp."""
    try:
        return list(pls.GetAddedOrExplicitItems())
    except Exception:
        return []


# ---------------------------------------------------------------------------
# Metric 6: /Root/Instance anomaly bucketing
# ---------------------------------------------------------------------------

def check_root_instance(asset_usd_path: str) -> str:
    """Check /Root/Instance prim status.

    Returns one of:
      'has_instance_with_transform' - Instance prim exists with non-identity transform
      'has_instance_identity'       - Instance prim exists with identity transform
      'no_instance'                 - No Instance prim found
    """
    try:
        stage = Usd.Stage.Open(asset_usd_path, load=Usd.Stage.LoadNone)
    except Exception:
        return "no_instance"

    if stage is None:
        return "no_instance"

    default_prim = stage.GetDefaultPrim()
    if not default_prim or not default_prim.IsValid():
        children = [c for c in stage.GetPseudoRoot().GetChildren() if c and c.IsValid()]
        default_prim = children[0] if children else None

    if not default_prim or not default_prim.IsValid():
        return "no_instance"

    instance_prim = None
    for child in default_prim.GetChildren():
        if child.GetName() == "Instance":
            instance_prim = child
            break

    if not instance_prim or not instance_prim.IsValid():
        return "no_instance"

    xf = UsdGeom.Xformable(instance_prim)
    if not xf:
        return "has_instance_identity"

    try:
        res = xf.GetLocalTransformation(Usd.TimeCode.Default())
        m = res[0] if isinstance(res, tuple) else res
    except Exception:
        return "has_instance_identity"

    # Check if identity (within tolerance)
    identity = Gf.Matrix4d(1.0)
    is_identity = True
    for i in range(4):
        for j in range(4):
            if abs(m[i][j] - identity[i][j]) > 1e-6:
                is_identity = False
                break
        if not is_identity:
            break

    return "has_instance_identity" if is_identity else "has_instance_with_transform"


# ---------------------------------------------------------------------------
# Main preflight logic
# ---------------------------------------------------------------------------

def run_preflight(
    dataset_root: str,
    bak_root: str,
    mapping_dir: str,
    report_dir: str,
    scene_filter: Optional[str] = None,
) -> Dict[str, Any]:
    """Run all 6 preflight checks and return the report dict."""

    if not _PXR_AVAILABLE:
        log.error("pxr not available. Run via ./scripts/isaac_python.sh")
        return {"error": "pxr not available"}

    dataset_root = _norm(dataset_root)
    bak_root = _norm(bak_root)
    mapping_dir = _norm(mapping_dir)
    report_dir = _norm(report_dir)
    os.makedirs(report_dir, exist_ok=True)

    t0 = time.time()

    # --- Phase 0: Load data ---
    consolidated_mapping = load_consolidated_mapping(mapping_dir, dataset_root)
    old_asset_locator = build_old_asset_locator(bak_root, dataset_root)

    # Build reverse mapping: canonical_abs -> [old_abs, ...]
    canonical_to_olds: Dict[str, List[str]] = defaultdict(list)
    for old_abs, canon_abs in consolidated_mapping.items():
        canonical_to_olds[canon_abs].append(old_abs)

    # All unique canonical assets
    all_canonical_assets: Set[str] = set(consolidated_mapping.values())

    # --- Discover scenes ---
    scene_dirs = discover_scenes(dataset_root, scene_filter)
    if not scene_dirs:
        log.error("No scenes found!")
        return {"error": "No scenes found"}

    # --- Metric 1: Earliest pre_c1 selection ---
    log.info("=== Metric 1: Earliest pre_c1 selection ===")
    metric1_results: Dict[str, Any] = {}
    scenes_with_pre_c1 = 0
    scenes_without_pre_c1 = 0

    for scene_dir in scene_dirs:
        scene_id = os.path.basename(scene_dir)
        result = find_earliest_pre_c1(scene_dir)
        if result:
            metric1_results[scene_id] = {"file": result[0], "timestamp": result[1]}
            scenes_with_pre_c1 += 1
        else:
            metric1_results[scene_id] = {"file": None, "timestamp": None}
            scenes_without_pre_c1 += 1
            log.warning("  No pre_c1 backup found for scene %s", scene_id)

    m1_rate = scenes_with_pre_c1 / max(1, len(scene_dirs))
    log.info("  Result: %d/%d scenes have pre_c1 backups (%.1f%%)",
             scenes_with_pre_c1, len(scene_dirs), m1_rate * 100)

    # --- Metrics 2-4: Per-scene reference analysis ---
    log.info("=== Metrics 2-4: Reference analysis ===")

    # Aggregated counters
    total_old_refs = 0
    old_refs_locatable = 0
    total_changed_refs = 0
    changed_refs_in_mapping = 0
    total_mapping_covered = 0
    current_ref_consistent = 0

    per_scene_metrics: Dict[str, Any] = {}
    all_old_refs_seen: Set[str] = set()
    all_canonical_seen: Set[str] = set()

    for scene_dir in scene_dirs:
        scene_id = os.path.basename(scene_dir)
        pre_c1_info = metric1_results.get(scene_id, {})
        pre_c1_file = pre_c1_info.get("file")

        if not pre_c1_file:
            per_scene_metrics[scene_id] = {"skipped": "no_pre_c1_backup"}
            continue

        pre_c1_path = os.path.join(scene_dir, pre_c1_file)
        current_layout = os.path.join(scene_dir, "layout.usd")

        if not os.path.isfile(pre_c1_path):
            per_scene_metrics[scene_id] = {"skipped": "pre_c1_file_missing"}
            continue
        if not os.path.isfile(current_layout):
            per_scene_metrics[scene_id] = {"skipped": "current_layout_missing"}
            continue

        # Read refs from both files
        orig_refs = read_layout_refs(pre_c1_path)
        current_refs = read_layout_refs(current_layout)

        scene_old_refs = 0
        scene_locatable = 0
        scene_changed = 0
        scene_in_mapping = 0
        scene_consistent = 0

        for prim_path, orig_ref in orig_refs.items():
            if not orig_ref:
                continue

            all_old_refs_seen.add(orig_ref)
            scene_old_refs += 1
            total_old_refs += 1

            # Metric 2: Is old ref locatable?
            if os.path.isfile(orig_ref) or orig_ref in old_asset_locator:
                scene_locatable += 1
                old_refs_locatable += 1

            # Check if ref changed
            current_ref = current_refs.get(prim_path, "")
            if orig_ref != current_ref and current_ref:
                scene_changed += 1
                total_changed_refs += 1

                # Metric 3: Mapping coverage
                if orig_ref in consolidated_mapping:
                    scene_in_mapping += 1
                    changed_refs_in_mapping += 1
                    total_mapping_covered += 1

                    # Metric 4: Current-ref consistency
                    expected_canonical = consolidated_mapping[orig_ref]
                    all_canonical_seen.add(expected_canonical)
                    if _norm(current_ref) == _norm(expected_canonical):
                        scene_consistent += 1
                        current_ref_consistent += 1

        per_scene_metrics[scene_id] = {
            "total_orig_refs": scene_old_refs,
            "locatable": scene_locatable,
            "changed": scene_changed,
            "in_mapping": scene_in_mapping,
            "consistent": scene_consistent,
        }

    m2_rate = old_refs_locatable / max(1, total_old_refs)
    m3_rate = changed_refs_in_mapping / max(1, total_changed_refs)
    m4_rate = current_ref_consistent / max(1, total_mapping_covered)

    log.info("  Metric 2 (old-ref locatability): %d/%d (%.1f%%)",
             old_refs_locatable, total_old_refs, m2_rate * 100)
    log.info("  Metric 3 (mapping coverage): %d/%d (%.1f%%)",
             changed_refs_in_mapping, total_changed_refs, m3_rate * 100)
    log.info("  Metric 4 (current-ref consistency): %d/%d (%.1f%%)",
             current_ref_consistent, total_mapping_covered, m4_rate * 100)

    # --- Metric 5: Canonical live-member resolution rate ---
    log.info("=== Metric 5: Canonical live-member resolution ===")
    canonical_live = 0
    canonical_missing: List[str] = []

    for canon_abs in sorted(all_canonical_assets):
        if os.path.isfile(canon_abs):
            canonical_live += 1
        else:
            canonical_missing.append(canon_abs)

    m5_rate = canonical_live / max(1, len(all_canonical_assets))
    log.info("  Result: %d/%d canonical assets live (%.1f%%), %d missing",
             canonical_live, len(all_canonical_assets), m5_rate * 100,
             len(canonical_missing))

    # --- Metric 6: /Root/Instance anomaly bucketing ---
    log.info("=== Metric 6: /Root/Instance anomaly bucketing ===")

    # Sample canonical assets that are actually referenced
    assets_to_check: Set[str] = set()
    # Add all canonical assets (they will be used for compensation)
    for canon_abs in all_canonical_assets:
        if os.path.isfile(canon_abs):
            assets_to_check.add(canon_abs)
    # Add all old assets that are locatable (either live or backup)
    for old_abs in list(all_old_refs_seen)[:500]:  # cap at 500 for speed
        if os.path.isfile(old_abs):
            assets_to_check.add(old_abs)
        elif old_abs in old_asset_locator:
            assets_to_check.add(old_asset_locator[old_abs])

    instance_buckets = {
        "has_instance_with_transform": 0,
        "has_instance_identity": 0,
        "no_instance": 0,
    }
    instance_anomalies: List[Dict[str, str]] = []

    checked = 0
    for asset_path in sorted(assets_to_check):
        bucket = check_root_instance(asset_path)
        instance_buckets[bucket] += 1
        if bucket == "no_instance":
            instance_anomalies.append({"asset": asset_path, "bucket": bucket})
        checked += 1
        if checked % 500 == 0:
            log.info("  Checked %d/%d assets...", checked, len(assets_to_check))

    log.info("  Checked %d assets:", checked)
    for bucket, count in instance_buckets.items():
        log.info("    %s: %d", bucket, count)

    # --- Assemble report ---
    elapsed = time.time() - t0

    report = {
        "timestamp": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "elapsed_sec": round(elapsed, 1),
        "dataset_root": dataset_root,
        "bak_root": bak_root,
        "mapping_dir": mapping_dir,
        "scene_count": len(scene_dirs),
        "consolidated_mapping_size": len(consolidated_mapping),
        "old_asset_locator_size": len(old_asset_locator),
        "metrics": {
            "m1_pre_c1_selection": {
                "scenes_with_backup": scenes_with_pre_c1,
                "scenes_without_backup": scenes_without_pre_c1,
                "rate": round(m1_rate, 4),
                "per_scene": metric1_results,
            },
            "m2_old_ref_locatability": {
                "locatable": old_refs_locatable,
                "total": total_old_refs,
                "rate": round(m2_rate, 4),
            },
            "m3_mapping_coverage": {
                "covered": changed_refs_in_mapping,
                "total_changed": total_changed_refs,
                "rate": round(m3_rate, 4),
            },
            "m4_current_ref_consistency": {
                "consistent": current_ref_consistent,
                "total_covered": total_mapping_covered,
                "rate": round(m4_rate, 4),
            },
            "m5_canonical_live_resolution": {
                "live": canonical_live,
                "total": len(all_canonical_assets),
                "rate": round(m5_rate, 4),
                "missing": canonical_missing[:50],  # cap for readability
                "missing_count": len(canonical_missing),
            },
            "m6_root_instance_bucketing": {
                "checked": checked,
                "buckets": instance_buckets,
                "no_instance_samples": instance_anomalies[:30],
            },
        },
        "per_scene_details": per_scene_metrics,
    }

    # --- Write report ---
    report_path = os.path.join(report_dir, "preflight_report.json")
    os.makedirs(report_dir, exist_ok=True)
    with open(report_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)
    log.info("Report written to %s", report_path)

    # --- Print summary ---
    print("\n" + "=" * 70)
    print("PREFLIGHT PLACEMENT VALIDATION SUMMARY")
    print("=" * 70)
    print(f"Scenes: {len(scene_dirs)}")
    print(f"Consolidated mapping: {len(consolidated_mapping)} pairs")
    print(f"Old-asset locator: {len(old_asset_locator)} entries")
    print()

    rates = {
        "M1 Pre-C1 backup selection": m1_rate,
        "M2 Old-ref locatability": m2_rate,
        "M3 Mapping coverage": m3_rate,
        "M4 Current-ref consistency": m4_rate,
        "M5 Canonical live resolution": m5_rate,
    }

    all_pass = True
    for label, rate in rates.items():
        status = "PASS" if rate >= PASS_THRESHOLD else "FAIL"
        if rate < PASS_THRESHOLD:
            all_pass = False
        print(f"  {label}: {rate:.1%}  [{status}]")

    print()
    print(f"M6 /Root/Instance bucketing (checked {checked} assets):")
    for bucket, count in instance_buckets.items():
        print(f"  {bucket}: {count}")

    print()
    if all_pass:
        print("OVERALL: PASS -- all rates above {:.0%} threshold".format(PASS_THRESHOLD))
    else:
        print("OVERALL: FAIL -- one or more rates below {:.0%} threshold".format(PASS_THRESHOLD))

    print(f"\nElapsed: {elapsed:.1f}s")
    print(f"Report: {report_path}")
    print("=" * 70)

    return report


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Preflight validation for Plan C dedup placement patch."
    )
    parser.add_argument(
        "--dataset-root",
        default="./GRScenes-test1-normalized",
        help="Path to normalized dataset root (default: ./GRScenes-test1-normalized)",
    )
    parser.add_argument(
        "--bak-root",
        default="./GRScenes-test1-normalized_bak",
        help="Path to backup root (default: ./GRScenes-test1-normalized_bak)",
    )
    parser.add_argument(
        "--mapping-dir",
        default="./check_reports/c1_bulk/",
        help="Directory containing *_geom_only_mapping.json files (default: ./check_reports/c1_bulk/)",
    )
    parser.add_argument(
        "--report-dir",
        default="./check_reports/preflight/",
        help="Directory for report output (default: ./check_reports/preflight/)",
    )
    parser.add_argument(
        "--scene-filter",
        default=None,
        help="Optional glob pattern to filter scene dirs (e.g. 'MV7J6*')",
    )

    args = parser.parse_args()

    report = run_preflight(
        dataset_root=args.dataset_root,
        bak_root=args.bak_root,
        mapping_dir=args.mapping_dir,
        report_dir=args.report_dir,
        scene_filter=args.scene_filter,
    )

    if "error" in report:
        return 1

    # Check pass/fail
    metrics = report.get("metrics", {})
    for key in ["m1_pre_c1_selection", "m2_old_ref_locatability",
                "m3_mapping_coverage", "m4_current_ref_consistency",
                "m5_canonical_live_resolution"]:
        rate = metrics.get(key, {}).get("rate", 0.0)
        if rate < PASS_THRESHOLD:
            return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
