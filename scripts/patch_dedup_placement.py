#!/usr/bin/env python3
"""Patch dedup placement displacement (Plan C).

After C1 dedup replaced old asset references with canonical ones, the transform
compensation was a no-op due to two bugs:
  1. _get_asset_internal_matrix() read /Root (identity) instead of /Root/Instance
  2. Matrix multiply order was wrong (right-multiply instead of left-multiply)

This script retroactively applies the correct compensation by reading the original
asset transforms from backup and computing:
    M_new_local = M_canon_inst^{-1} * M_old_inst * M_old_local

It reads the pre-C1 layout backups to determine which prims had their references
changed, then patches the current layout.usd in-place (with backup).

Usage:
    ./scripts/isaac_python.sh scripts/patch_dedup_placement.py \\
        --dataset-root ./GRScenes-test1-normalized \\
        --bak-root ./GRScenes-test1-normalized_bak \\
        --mapping-dir ./check_reports/c1_bulk/ \\
        [--dry-run] [--workers 8] [--scene-filter "MV7J6*"] \\
        [--report-dir ./check_reports/placement_patch/]
"""

from __future__ import annotations

import argparse
import fnmatch
import hashlib
import json
import logging
import os
import shutil
import sys
import tempfile
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

try:
    from pxr import Gf, Sdf, Usd, UsdGeom
except ImportError as e:
    print(f"Error: pxr (USD) not available: {e}", file=sys.stderr)
    print("Run via: ./scripts/isaac_python.sh scripts/patch_dedup_placement.py", file=sys.stderr)
    sys.exit(1)

logger = logging.getLogger("patch_placement")

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
PATCH_VERSION = "1.0"
IDENTITY_TOLERANCE = 1e-6


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _norm_abs(p: str) -> str:
    """Normalize a path to absolute with forward slashes."""
    return os.path.normpath(os.path.abspath(p.replace("\\", "/")))


def _sha256_file(path: str) -> str:
    """Compute SHA-256 hex digest of a file."""
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def _is_near_identity(m: "Gf.Matrix4d", tol: float = IDENTITY_TOLERANCE) -> bool:
    """Check if a 4x4 matrix is approximately identity."""
    identity = Gf.Matrix4d(1.0)
    for r in range(4):
        for c in range(4):
            if abs(m[r][c] - identity[r][c]) > tol:
                return False
    return True


def _resolve_ref_asset_path(base_dir: str, asset_path: str) -> str:
    """Resolve a possibly-relative asset path to absolute."""
    if not asset_path:
        return ""
    s = asset_path.replace("\\", "/")
    if os.path.isabs(s):
        return _norm_abs(s)
    return _norm_abs(os.path.join(base_dir, s))


def _ensure_matrix_xform(prim) -> None:
    """Ensure prim has a single xformOp:transform op."""
    prim.CreateAttribute(
        "xformOpOrder", Sdf.ValueTypeNames.TokenArray, custom=False
    ).Set(["xformOp:transform"])
    if not prim.HasAttribute("xformOp:transform"):
        prim.CreateAttribute(
            "xformOp:transform", Sdf.ValueTypeNames.Matrix4d, custom=False
        )


def _set_local_matrix(prim, m: "Gf.Matrix4d") -> None:
    """Write a single xformOp:transform matrix on a prim."""
    _ensure_matrix_xform(prim)
    prim.GetAttribute("xformOp:transform").Set(m)


def _get_local_matrix(prim) -> "Gf.Matrix4d":
    """Return the local transform matrix of a prim."""
    xf = UsdGeom.Xformable(prim)
    if not xf:
        return Gf.Matrix4d(1.0)
    res = xf.GetLocalTransformation(Usd.TimeCode.Default())
    if isinstance(res, tuple):
        return res[0]
    return res


# ---------------------------------------------------------------------------
# Phase 0: Data structure building
# ---------------------------------------------------------------------------

def load_consolidated_mapping(
    mapping_dir: str, dataset_root: str
) -> Dict[str, str]:
    """Load all *_geom_only_mapping.json files into {old_abs: canonical_abs}.

    Mapping files use relative paths like:
        GRScenes-test1-normalized/GRScenes_assets/...
    We convert them to absolute by joining with the parent of dataset_root.
    """
    mapping_dir = _norm_abs(mapping_dir)
    dataset_root = _norm_abs(dataset_root)
    repo_root = os.path.dirname(dataset_root)  # parent of GRScenes-test1-normalized

    consolidated: Dict[str, str] = {}
    conflicts = 0

    for fname in sorted(os.listdir(mapping_dir)):
        if not fname.endswith("_geom_only_mapping.json"):
            continue
        fpath = os.path.join(mapping_dir, fname)
        with open(fpath, "r", encoding="utf-8") as f:
            data = json.load(f)
        if not isinstance(data, dict):
            logger.warning("Unexpected format in %s, skipping", fname)
            continue
        for old_rel, canon_rel in data.items():
            old_abs = _norm_abs(os.path.join(repo_root, old_rel))
            canon_abs = _norm_abs(os.path.join(repo_root, canon_rel))
            if old_abs in consolidated and consolidated[old_abs] != canon_abs:
                logger.error(
                    "Mapping conflict for %s: %s vs %s",
                    old_abs, consolidated[old_abs], canon_abs,
                )
                conflicts += 1
            consolidated[old_abs] = canon_abs

    logger.info(
        "Loaded %d mapping entries from %s (%d conflicts)",
        len(consolidated), mapping_dir, conflicts,
    )
    return consolidated


def build_old_asset_locator(bak_root: str) -> Dict[str, str]:
    """Scan _dedup_assets/ backup dirs to build {original_abs_path: backup_abs_path}.

    Backup dirs have structure:
        _dedup_assets/c1_<cat>_bulk_<ts>/GRScenes_assets/<cat>/<uid>/usd/<uid>.usd
    The original abs path is reconstructed from the directory name.
    """
    bak_root = _norm_abs(bak_root)
    dedup_dir = os.path.join(bak_root, "_dedup_assets")
    locator: Dict[str, str] = {}

    if not os.path.isdir(dedup_dir):
        logger.warning("Backup dedup dir not found: %s", dedup_dir)
        return locator

    for batch_name in sorted(os.listdir(dedup_dir)):
        batch_dir = os.path.join(dedup_dir, batch_name)
        assets_dir = os.path.join(batch_dir, "GRScenes_assets")
        if not os.path.isdir(assets_dir):
            continue
        for cat in os.listdir(assets_dir):
            cat_dir = os.path.join(assets_dir, cat)
            if not os.path.isdir(cat_dir):
                continue
            for uid in os.listdir(cat_dir):
                usd_path = os.path.join(cat_dir, uid, "usd", f"{uid}.usd")
                if os.path.isfile(usd_path):
                    # The original path was under the dataset root
                    # We need to reconstruct it. The dataset_root is the parent
                    # of bak_root's sibling. We store it relative to GRScenes_assets.
                    # The caller will need to resolve with dataset_root.
                    # Store as: GRScenes_assets/cat/uid/usd/uid.usd relative key
                    # Actually, store the original abs path as key.
                    # We don't know dataset_root here, so store rel_key -> backup_path.
                    rel_key = f"GRScenes_assets/{cat}/{uid}/usd/{uid}.usd"
                    locator[rel_key] = usd_path

    logger.info("Built old_asset_locator with %d entries from %s", len(locator), dedup_dir)
    return locator


def _resolve_locator_key(asset_abs: str, dataset_root: str) -> str:
    """Convert an absolute asset path to a locator key (GRScenes_assets/...)."""
    dataset_root = _norm_abs(dataset_root)
    if asset_abs.startswith(dataset_root + "/"):
        return asset_abs[len(dataset_root) + 1:]
    # Try to extract GRScenes_assets/ portion
    marker = "/GRScenes_assets/"
    idx = asset_abs.find(marker)
    if idx >= 0:
        return asset_abs[idx + 1:]  # strip leading /
    return ""


# ---------------------------------------------------------------------------
# Asset Instance Matrix Reading
# ---------------------------------------------------------------------------

def _get_root_instance_matrix(
    asset_usd_abs: str,
    old_asset_locator: Dict[str, str],
    dataset_root: str,
    cache: Dict[str, "Gf.Matrix4d"],
) -> "Gf.Matrix4d":
    """Read asset's /Root/Instance local transform. Cache results by asset path.

    Checks live path first, then backup locator.
    Falls back to identity if no Instance child found.
    """
    if asset_usd_abs in cache:
        return cache[asset_usd_abs]

    # Determine actual file path
    actual_path: Optional[str] = None
    if os.path.isfile(asset_usd_abs):
        actual_path = asset_usd_abs
    else:
        rel_key = _resolve_locator_key(asset_usd_abs, dataset_root)
        if rel_key and rel_key in old_asset_locator:
            actual_path = old_asset_locator[rel_key]

    if actual_path is None:
        raise FileNotFoundError(f"Asset not found live or in backup: {asset_usd_abs}")

    stage = Usd.Stage.Open(actual_path, load=Usd.Stage.LoadNone)
    if stage is None:
        raise RuntimeError(f"Failed to open asset USD: {actual_path}")

    # Find defaultPrim
    default_prim = stage.GetDefaultPrim()
    if not default_prim or not default_prim.IsValid():
        children = [c for c in stage.GetPseudoRoot().GetChildren() if c and c.IsValid()]
        default_prim = children[0] if children else None

    if not default_prim or not default_prim.IsValid():
        cache[asset_usd_abs] = Gf.Matrix4d(1.0)
        return Gf.Matrix4d(1.0)

    # Look for "Instance" child under the default prim
    instance_prim = None
    for child in default_prim.GetChildren():
        if child.GetName() == "Instance":
            instance_prim = child
            break

    target_prim = instance_prim if (instance_prim and instance_prim.IsValid()) else default_prim

    xf = UsdGeom.Xformable(target_prim)
    if not xf:
        cache[asset_usd_abs] = Gf.Matrix4d(1.0)
        return Gf.Matrix4d(1.0)

    res = xf.GetLocalTransformation(Usd.TimeCode.Default())
    m = res[0] if isinstance(res, tuple) else res
    cache[asset_usd_abs] = m
    return m


# ---------------------------------------------------------------------------
# Reference extraction from layout USD
# ---------------------------------------------------------------------------

def extract_prim_refs(layout_usd: str) -> Dict[str, str]:
    """Open a layout USD and extract {prim_path: resolved_ref_abs} for prims with references.

    Only the first reference asset path is captured per prim.
    """
    stage = Usd.Stage.Open(layout_usd, load=Usd.Stage.LoadNone)
    if stage is None:
        raise RuntimeError(f"Failed to open layout: {layout_usd}")

    base_dir = os.path.dirname(os.path.abspath(layout_usd))
    result: Dict[str, str] = {}

    for prim in stage.Traverse():
        if not prim or not prim.IsValid():
            continue
        if not prim.HasAuthoredReferences():
            continue
        refs = prim.GetMetadata("references")
        if not refs:
            continue
        try:
            items = list(refs.GetAddedOrExplicitItems())
        except Exception:
            continue
        if not items:
            continue
        # Take the first reference's asset path
        ap = getattr(items[0], "assetPath", "") or ""
        if ap:
            resolved = _resolve_ref_asset_path(base_dir, ap)
            result[str(prim.GetPath())] = resolved

    return result


# ---------------------------------------------------------------------------
# Scene finding helpers
# ---------------------------------------------------------------------------

def find_earliest_pre_c1(scene_dir: str) -> Optional[str]:
    """Find the earliest .pre_c1_*.usd backup in a scene directory.

    Sort by filename (which includes timestamp) and pick the first.
    """
    scene_path = Path(scene_dir)
    backups = sorted(scene_path.glob("layout.pre_c1_*.usd"))
    if not backups:
        return None
    return str(backups[0])


def discover_scenes(dataset_root: str, scene_filter: Optional[str] = None) -> List[str]:
    """Discover all scene directories under GRScenes100/.

    Returns list of absolute scene directory paths.
    """
    dataset_root = _norm_abs(dataset_root)
    scenes_root = os.path.join(dataset_root, "GRScenes100")
    result: List[str] = []

    if not os.path.isdir(scenes_root):
        logger.error("Scenes root not found: %s", scenes_root)
        return result

    for scene_type in sorted(os.listdir(scenes_root)):
        type_dir = os.path.join(scenes_root, scene_type)
        if not os.path.isdir(type_dir):
            continue
        for scene_id in sorted(os.listdir(type_dir)):
            scene_dir = os.path.join(type_dir, scene_id)
            layout = os.path.join(scene_dir, "layout.usd")
            if not os.path.isfile(layout):
                continue
            if scene_filter and not fnmatch.fnmatch(scene_id, scene_filter):
                continue
            result.append(scene_dir)

    return result


# ---------------------------------------------------------------------------
# Per-scene patch logic
# ---------------------------------------------------------------------------

def patch_single_scene(
    scene_dir: str,
    consolidated_mapping: Dict[str, str],
    old_asset_locator: Dict[str, str],
    dataset_root: str,
    dry_run: bool,
    report_dir: str,
) -> Dict[str, Any]:
    """Patch a single scene's layout.usd to compensate for dedup displacement.

    Returns a per-scene report dict.
    """
    dataset_root = _norm_abs(dataset_root)
    scene_dir = _norm_abs(scene_dir)
    scene_id = os.path.basename(scene_dir)
    layout_path = os.path.join(scene_dir, "layout.usd")

    report: Dict[str, Any] = {
        "scene": scene_id,
        "patched": 0,
        "skipped_same_ref": 0,
        "skipped_not_in_mapping": 0,
        "skipped_identity_delta": 0,
        "errors": 0,
        "entries": [],
    }

    # --- Idempotency: restore from first pre_patch backup if exists ---
    pre_patch_backups = sorted(Path(scene_dir).glob("layout.pre_patch.*.usd"))
    if pre_patch_backups:
        first_backup = str(pre_patch_backups[0])
        if not dry_run:
            shutil.copy2(first_backup, layout_path)
            logger.info("[%s] Restored from pre_patch backup: %s", scene_id, os.path.basename(first_backup))

    # --- 1a. Find earliest .pre_c1 backup ---
    earliest_pre_c1 = find_earliest_pre_c1(scene_dir)
    if earliest_pre_c1 is None:
        report["errors"] += 1
        report["entries"].append({
            "action": "error",
            "detail": "No .pre_c1_*.usd backup found",
        })
        return report

    report["earliest_pre_c1"] = os.path.basename(earliest_pre_c1)

    # --- 1b. Extract original refs from pre_c1 backup ---
    try:
        orig_refs = extract_prim_refs(earliest_pre_c1)
    except Exception as e:
        report["errors"] += 1
        report["entries"].append({
            "action": "error",
            "detail": f"Failed to read pre_c1 backup: {e}",
        })
        return report

    # --- 1c. Open current layout ---
    if not os.path.isfile(layout_path):
        report["errors"] += 1
        report["entries"].append({
            "action": "error",
            "detail": "layout.usd not found",
        })
        return report

    # --- Compute input hash before any modification ---
    input_hash = _sha256_file(layout_path)
    report["input_hash"] = f"sha256:{input_hash}"

    # --- 1d. Backup layout.usd ---
    if not dry_run and not pre_patch_backups:
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup_name = f"layout.pre_patch.{ts}.usd"
        backup_path = os.path.join(scene_dir, backup_name)
        shutil.copy2(layout_path, backup_path)

    # Open the stage for editing
    stage = Usd.Stage.Open(layout_path)
    if stage is None:
        report["errors"] += 1
        report["entries"].append({
            "action": "error",
            "detail": "Failed to open layout.usd",
        })
        return report

    base_dir = os.path.dirname(layout_path)
    xform_cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    instance_matrix_cache: Dict[str, Gf.Matrix4d] = {}
    any_changed = False

    # --- 1e. Process each prim ---
    for prim in stage.Traverse():
        if not prim or not prim.IsValid():
            continue
        if not prim.HasAuthoredReferences():
            continue

        prim_path_str = str(prim.GetPath())

        # Get current reference
        refs = prim.GetMetadata("references")
        if not refs:
            continue
        try:
            items = list(refs.GetAddedOrExplicitItems())
        except Exception:
            continue
        if not items:
            continue

        current_ap = getattr(items[0], "assetPath", "") or ""
        if not current_ap:
            continue
        current_ref = _resolve_ref_asset_path(base_dir, current_ap)

        # Get original ref from pre_c1
        orig_ref = orig_refs.get(prim_path_str)

        # SKIP if prim not present in original layout (new prim? unlikely)
        if orig_ref is None:
            continue

        # SKIP if ref unchanged (not deduped)
        if orig_ref == current_ref:
            report["skipped_same_ref"] += 1
            continue

        # SKIP if orig_ref not in consolidated mapping
        if orig_ref not in consolidated_mapping:
            report["skipped_not_in_mapping"] += 1
            report["entries"].append({
                "prim": prim_path_str,
                "action": "skipped_not_in_mapping",
                "orig_ref": orig_ref,
                "current_ref": current_ref,
            })
            continue

        expected_canonical = consolidated_mapping[orig_ref]

        # FAIL-CLOSED: current ref should match the canonical from mapping
        if current_ref != expected_canonical:
            report["errors"] += 1
            report["entries"].append({
                "prim": prim_path_str,
                "action": "error_ref_mismatch",
                "detail": f"current_ref={current_ref} != mapping_canonical={expected_canonical}",
                "orig_ref": orig_ref,
            })
            continue

        # --- Read M_old_inst from backup ---
        try:
            M_old_inst = _get_root_instance_matrix(
                orig_ref, old_asset_locator, dataset_root, instance_matrix_cache
            )
        except (FileNotFoundError, RuntimeError) as e:
            report["errors"] += 1
            report["entries"].append({
                "prim": prim_path_str,
                "action": "error_old_asset_not_found",
                "detail": str(e),
                "orig_ref": orig_ref,
            })
            continue

        # --- Read M_canon_inst from live dataset ---
        try:
            M_canon_inst = _get_root_instance_matrix(
                current_ref, old_asset_locator, dataset_root, instance_matrix_cache
            )
        except (FileNotFoundError, RuntimeError) as e:
            report["errors"] += 1
            report["entries"].append({
                "prim": prim_path_str,
                "action": "error_canonical_asset_not_found",
                "detail": str(e),
                "orig_ref": orig_ref,
                "current_ref": current_ref,
            })
            continue

        # --- Check if compensation needed ---
        delta = M_canon_inst.GetInverse() * M_old_inst
        if _is_near_identity(delta):
            report["skipped_identity_delta"] += 1
            continue

        # --- Compute compensated transform ---
        old_local = xform_cache.GetLocalTransformation(prim)
        if isinstance(old_local, tuple):
            old_local = old_local[0]

        new_local = M_canon_inst.GetInverse() * M_old_inst * old_local

        # --- Write ---
        if not dry_run:
            _set_local_matrix(prim, new_local)
            any_changed = True

        report["patched"] += 1
        report["entries"].append({
            "prim": prim_path_str,
            "action": "compensated",
            "orig_ref": orig_ref,
            "current_ref": current_ref,
        })

    # --- 1f. Atomic save ---
    if not dry_run and any_changed:
        tmp_path = layout_path + ".tmp"
        stage.GetRootLayer().Export(tmp_path)
        os.rename(tmp_path, layout_path)

    # --- 1g. Write per-scene report ---
    if report_dir:
        os.makedirs(report_dir, exist_ok=True)
        report_path = os.path.join(report_dir, f"{scene_id}_patch_report.json")
        with open(report_path, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)

    return report


# ---------------------------------------------------------------------------
# Parallel wrapper for ProcessPoolExecutor
# ---------------------------------------------------------------------------

def _patch_scene_worker(args_tuple):
    """Worker function for ProcessPoolExecutor."""
    (scene_dir, consolidated_mapping, old_asset_locator,
     dataset_root, dry_run, report_dir) = args_tuple
    try:
        return patch_single_scene(
            scene_dir, consolidated_mapping, old_asset_locator,
            dataset_root, dry_run, report_dir,
        )
    except Exception as e:
        scene_id = os.path.basename(scene_dir)
        return {
            "scene": scene_id,
            "patched": 0,
            "skipped_same_ref": 0,
            "skipped_not_in_mapping": 0,
            "skipped_identity_delta": 0,
            "errors": 1,
            "entries": [{"action": "error", "detail": f"Worker exception: {e}"}],
        }


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = argparse.ArgumentParser(
        description="Patch dedup placement displacement (Plan C).",
    )
    ap.add_argument(
        "--dataset-root", default="./GRScenes-test1-normalized",
        help="GRScenes-test1-normalized root directory",
    )
    ap.add_argument(
        "--bak-root", default="./GRScenes-test1-normalized_bak",
        help="Backup root directory",
    )
    ap.add_argument(
        "--mapping-dir", default="./check_reports/c1_bulk/",
        help="C1 mapping directory containing *_geom_only_mapping.json files",
    )
    ap.add_argument("--dry-run", action="store_true", help="Report only, do not modify files")
    ap.add_argument("--workers", type=int, default=8, help="Parallel workers (default: 8)")
    ap.add_argument(
        "--report-dir", default="./check_reports/placement_patch/",
        help="Report output directory",
    )
    ap.add_argument(
        "--scene-filter", default=None,
        help="Optional glob pattern to filter scene IDs (e.g. 'MV7J6*')",
    )

    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )

    dataset_root = _norm_abs(args.dataset_root)
    bak_root = _norm_abs(args.bak_root)
    mapping_dir = _norm_abs(args.mapping_dir)
    report_dir = _norm_abs(args.report_dir)
    os.makedirs(report_dir, exist_ok=True)

    run_id = f"patch_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    logger.info("Run ID: %s", run_id)
    logger.info("Dataset root: %s", dataset_root)
    logger.info("Backup root: %s", bak_root)
    logger.info("Mapping dir: %s", mapping_dir)
    logger.info("Dry run: %s", args.dry_run)
    logger.info("Workers: %d", args.workers)
    if args.scene_filter:
        logger.info("Scene filter: %s", args.scene_filter)

    t0 = time.time()

    # --- Phase 0a: Load consolidated mapping ---
    consolidated_mapping = load_consolidated_mapping(mapping_dir, dataset_root)
    if not consolidated_mapping:
        logger.error("No mapping entries loaded. Aborting.")
        return 1

    # --- Phase 0b: Build old_asset_locator ---
    old_asset_locator = build_old_asset_locator(bak_root)

    # --- Discover scenes ---
    scenes = discover_scenes(dataset_root, args.scene_filter)
    if not scenes:
        logger.error("No scenes found. Check --dataset-root and --scene-filter.")
        return 1
    logger.info("Found %d scenes to process", len(scenes))

    # --- Phase 1: Process scenes ---
    all_reports: List[Dict[str, Any]] = []

    if args.workers <= 1:
        # Sequential
        for i, scene_dir in enumerate(scenes):
            scene_id = os.path.basename(scene_dir)
            logger.info("[%d/%d] Processing %s", i + 1, len(scenes), scene_id)
            r = patch_single_scene(
                scene_dir, consolidated_mapping, old_asset_locator,
                dataset_root, args.dry_run, report_dir,
            )
            all_reports.append(r)
            if r["patched"] > 0 or r["errors"] > 0:
                logger.info(
                    "  %s: patched=%d, skipped_same=%d, skipped_identity=%d, errors=%d",
                    scene_id, r["patched"], r["skipped_same_ref"],
                    r["skipped_identity_delta"], r["errors"],
                )
    else:
        # Parallel
        work_args = [
            (scene_dir, consolidated_mapping, old_asset_locator,
             dataset_root, args.dry_run, report_dir)
            for scene_dir in scenes
        ]
        with ProcessPoolExecutor(max_workers=args.workers) as executor:
            futures = {
                executor.submit(_patch_scene_worker, wa): wa[0]
                for wa in work_args
            }
            done_count = 0
            for future in as_completed(futures):
                done_count += 1
                scene_dir = futures[future]
                scene_id = os.path.basename(scene_dir)
                try:
                    r = future.result()
                except Exception as e:
                    r = {
                        "scene": scene_id,
                        "patched": 0, "skipped_same_ref": 0,
                        "skipped_not_in_mapping": 0,
                        "skipped_identity_delta": 0,
                        "errors": 1,
                        "entries": [{"action": "error", "detail": str(e)}],
                    }
                all_reports.append(r)
                if r["patched"] > 0 or r["errors"] > 0:
                    logger.info(
                        "[%d/%d] %s: patched=%d, errors=%d",
                        done_count, len(scenes), scene_id,
                        r["patched"], r["errors"],
                    )

    elapsed = time.time() - t0

    # --- Summary ---
    total_patched = sum(r["patched"] for r in all_reports)
    total_skipped_same = sum(r["skipped_same_ref"] for r in all_reports)
    total_skipped_mapping = sum(r.get("skipped_not_in_mapping", 0) for r in all_reports)
    total_skipped_identity = sum(r["skipped_identity_delta"] for r in all_reports)
    total_errors = sum(r["errors"] for r in all_reports)

    logger.info("=" * 60)
    logger.info("SUMMARY")
    logger.info("  Scenes processed: %d", len(all_reports))
    logger.info("  Prims patched: %d", total_patched)
    logger.info("  Skipped (same ref): %d", total_skipped_same)
    logger.info("  Skipped (not in mapping): %d", total_skipped_mapping)
    logger.info("  Skipped (identity delta): %d", total_skipped_identity)
    logger.info("  Errors: %d", total_errors)
    logger.info("  Elapsed: %.1fs", elapsed)
    logger.info("  Dry run: %s", args.dry_run)
    logger.info("=" * 60)

    # --- Write patch_summary.json ---
    summary = {
        "total_scenes": len(all_reports),
        "total_prims_patched": total_patched,
        "total_skipped_same_ref": total_skipped_same,
        "total_skipped_not_in_mapping": total_skipped_mapping,
        "total_skipped_identity_delta": total_skipped_identity,
        "total_errors": total_errors,
        "elapsed_sec": round(elapsed, 2),
        "dry_run": args.dry_run,
    }
    summary_path = os.path.join(report_dir, "patch_summary.json")
    with open(summary_path, "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    logger.info("Summary written to: %s", summary_path)

    # --- Write manifest ---
    manifest = {
        "run_id": run_id,
        "patch_version": PATCH_VERSION,
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "dry_run": args.dry_run,
        "scenes": [
            {
                "scene_id": r["scene"],
                "earliest_pre_c1": r.get("earliest_pre_c1", ""),
                "input_hash": r.get("input_hash", ""),
                "patched": r["patched"],
                "skipped": r["skipped_same_ref"] + r.get("skipped_not_in_mapping", 0) + r["skipped_identity_delta"],
                "errors": r["errors"],
            }
            for r in sorted(all_reports, key=lambda x: x["scene"])
        ],
        "totals": {
            "scenes": len(all_reports),
            "patched": total_patched,
            "skipped": total_skipped_same + total_skipped_mapping + total_skipped_identity,
            "errors": total_errors,
        },
    }
    manifest_path = os.path.join(report_dir, f"manifest_{run_id}.json")
    with open(manifest_path, "w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2, ensure_ascii=False)
    logger.info("Manifest written to: %s", manifest_path)

    return 1 if total_errors > 0 else 0


if __name__ == "__main__":
    sys.exit(main())
