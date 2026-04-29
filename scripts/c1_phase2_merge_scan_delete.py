#!/usr/bin/env python3
"""Phase 2: Merge all category sidecars + mega-scan + soft-delete.

Usage:
  c1_phase2_merge_scan_delete.py \
    --dataset-root <path> --bak-root <path> --c1-bulk-dir <path> \
    --bbox-policy <policy> --out-version v1 --group-label <label> \
    [--categories-file <path>]
"""
import argparse
import json
import logging
import os
import shutil
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Set, Tuple

log = logging.getLogger(__name__)


def _is_pxr_available() -> Tuple[bool, str]:
    try:
        from pxr import Usd  # noqa: F401
        return True, ""
    except ImportError as e:
        return False, str(e)


def _setup_scripts_path() -> str:
    _script_dir = os.path.dirname(os.path.abspath(__file__))
    if _script_dir not in sys.path:
        sys.path.insert(0, _script_dir)
    return _script_dir


def _load_rewrite_module():
    _script_dir = _setup_scripts_path()
    import importlib.util
    mod_path = os.path.join(_script_dir, "rewrite_layout_asset_refs_with_compensation.py")
    spec = importlib.util.spec_from_file_location(
        "rewrite_layout_asset_refs_with_compensation", mod_path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Failed to load module spec from: {mod_path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def _load_mapping_from_dict(mapping: Dict[str, str], subset_root: str):
    """Convert {old: canonical} dict to list of MappingPair objects."""
    rl = _load_rewrite_module()
    pairs = []
    for old_key, canonical_key in mapping.items():
        old_abs = rl._resolve_under_subset_or_abs(subset_root, old_key)
        canonical_abs = rl._resolve_under_subset_or_abs(subset_root, canonical_key)
        pairs.append(rl.MappingPair(old_abs=old_abs, canonical_abs=canonical_abs))
    return pairs


def _combined_merge(
    dataset_root: Path,
    combined_mapping: Dict[str, str],
    scene_files: List[str],
):
    """Apply combined mapping to all scene files in one pass.

    Uses pxr.Usd — must be run in Isaac Sim environment.
    """
    pxr_ok, pxr_err = _is_pxr_available()
    if not pxr_ok:
        raise SystemExit(f"pxr.Usd not available — must run in Isaac Sim environment: {pxr_err}")

    rl = _load_rewrite_module()

    layout_files = sorted(dataset_root.glob("GRScenes100/**/layout.usd"))
    for layout_path in layout_files:
        scene_dir = layout_path.parent
        for sf in scene_files:
            scene_path = scene_dir / sf
            if not scene_path.exists():
                continue

            # Backup baseline if not already backed up
            backup = scene_path.with_suffix(".baseline.usd")
            if not backup.exists():
                shutil.copy2(str(scene_path), str(backup))
                log.info("Backed up %s -> %s", scene_path.name, backup.name)

            mapping_pairs = _load_mapping_from_dict(
                combined_mapping, str(dataset_root)
            )
            rl.rewrite_layout(
                layout_usd=str(scene_path),
                out_usd=str(scene_path),  # in-place
                subset_root=str(dataset_root),
                mapping_pairs=mapping_pairs,
                apply_compensation=True,
                set_instanceable=True,
                dry_run=False,
                report_out=None,
                max_preview=0,
                v_matrix_mode="none",
            )
            log.info("Merged %s", scene_path.relative_to(dataset_root))


def _sequential_merge(
    dataset_root: Path,
    c1_bulk: Path,
    policy: str,
    version: str,
    categories: List[str],
    group_label: str,
    scene_files: List[str],
):
    """Fallback: apply categories one by one via in-place rewrite_layout calls.

    Uses the same rewrite_layout approach as _combined_merge, but processes
    each category's mapping file iteratively.  This avoids the sidecar-
    vs-original dead-end where mega-scan reads the unmodified layout.usd.
    """
    pxr_ok, pxr_err = _is_pxr_available()
    if not pxr_ok:
        raise SystemExit(
            f"pxr.Usd not available — must run in Isaac Sim environment: {pxr_err}"
        )

    rl = _load_rewrite_module()

    for cat in categories:
        mapping_path = (
            c1_bulk / f"{cat}_{policy}_{version}" / "01_cert" / "filtered_mapping.json"
        )
        if not mapping_path.exists():
            log.warning("Skipping %s: mapping not found", cat)
            continue

        mapping = json.loads(mapping_path.read_text())
        pairs = _load_mapping_from_dict(mapping, str(dataset_root))

        if not pairs:
            log.warning("Skipping %s: mapping is empty", cat)
            continue

        layout_files = sorted(dataset_root.glob("GRScenes100/**/layout.usd"))
        for layout_path in layout_files:
            scene_dir = layout_path.parent
            for sf in scene_files:
                scene_path = scene_dir / sf
                if not scene_path.exists():
                    continue

                backup = scene_path.with_suffix(".baseline.usd")
                if not backup.exists():
                    shutil.copy2(str(scene_path), str(backup))
                    log.info("Backed up %s -> %s", scene_path.name, backup.name)

                rl.rewrite_layout(
                    layout_usd=str(scene_path),
                    out_usd=str(scene_path),  # in-place
                    subset_root=str(dataset_root),
                    mapping_pairs=pairs,
                    apply_compensation=True,
                    set_instanceable=True,
                    dry_run=False,
                    report_out=None,
                    max_preview=0,
                    v_matrix_mode="none",  # safe default for sequential fallback
                )
                log.info("Merged %s (cat=%s)", scene_path.relative_to(dataset_root), cat)


def _mega_soft_delete(
    dataset_root: Path,
    bak_root: Path,
    cat_mappings: List[Tuple[str, Dict[str, str]]],
    stamp: str,
) -> List[str]:
    """Move old asset directories to bak for all categories at once."""
    _setup_scripts_path()
    from scan_utils import _normalize_mapping_key

    errors: List[str] = []
    bak_dest = bak_root / "_dedup_assets" / f"phase2_combined_{stamp}"

    for cat, mapping in cat_mappings:
        for old_key in mapping.keys():
            rel = _normalize_mapping_key(old_key, dataset_root.name)
            if not rel:
                continue
            parts = Path(rel).parts
            if len(parts) < 3 or parts[0] != "GRScenes_assets":
                continue
            uid = parts[2]
            src_dir = dataset_root / "GRScenes_assets" / cat / uid
            if not src_dir.exists():
                continue
            dst_dir = bak_dest / "GRScenes_assets" / cat / uid
            try:
                dst_dir.parent.mkdir(parents=True, exist_ok=True)
                shutil.move(str(src_dir), str(dst_dir))
                log.info("  moved %s/%s -> bak", cat, uid)
            except Exception as e:
                errors.append(f"{cat}/{uid}: {e}")
    return errors


def _write_merge_report(c1_bulk: Path, status: str, **extra):
    report = {
        "phase": "phase2",
        "status": status,
        "timestamp": datetime.now().isoformat(),
        **extra,
    }
    (c1_bulk / "phase2_merge_report.json").write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )


def _discover_categories(c1_bulk: Path, policy: str, version: str) -> List[str]:
    """Discover category names from c1_bulk directory subdirs."""
    cats: List[str] = []
    suffix = f"_{policy}_{version}"
    for d in sorted(c1_bulk.iterdir()):
        if not d.is_dir():
            continue
        name = d.name
        if name.endswith(suffix):
            cat = name[:-len(suffix)]
            cats.append(cat)
    return cats


def main() -> int:
    ap = argparse.ArgumentParser(description="C1 Phase 2: merge+scan+delete")
    ap.add_argument("--dataset-root", required=True)
    ap.add_argument("--bak-root", required=True)
    ap.add_argument("--c1-bulk-dir", required=True)
    ap.add_argument("--bbox-policy", default="bbox_primary_rmse_observe")
    ap.add_argument("--out-version", default="v1")
    ap.add_argument("--group-label", required=True)
    ap.add_argument(
        "--categories-file",
        default=None,
        help="JSON list of category names (default: discover from c1_bulk)",
    )
    ap.add_argument(
        "--scene-files",
        default="layout.usd,start_result_interaction.usd,"
                "start_result_navigation.usd",
    )
    args = ap.parse_args()

    dataset_root = Path(args.dataset_root)
    bak_root = Path(args.bak_root)
    c1_bulk = Path(args.c1_bulk_dir)
    policy = args.bbox_policy
    version = args.out_version
    group_label = args.group_label
    scene_files = [s.strip() for s in args.scene_files.split(",")]

    _setup_scripts_path()
    from c1_parallel_merge import (
        gate_check_phase1,
        merge_category_mappings,
        discover_category_mappings,
    )

    # ---- Gate Check ----
    print("[Phase2] Gate check: verifying all Phase 1 jobs...", flush=True)
    if args.categories_file:
        categories = json.loads(Path(args.categories_file).read_text(encoding="utf-8"))
    else:
        categories = _discover_categories(c1_bulk, policy, version)

    print(f"[Phase2] {len(categories)} categories to process", flush=True)
    ok, failed = gate_check_phase1(c1_bulk, policy, version, categories)
    if not ok:
        print("[Phase2] GATE FAILED — some Phase 1 jobs incomplete:", flush=True)
        for f in failed:
            print(f"  - {f}", flush=True)
        _write_merge_report(c1_bulk, "gate_failed", failed_categories=failed)
        return 1
    print("[Phase2] Gate PASSED — all Phase 1 jobs completed with audit OK",
          flush=True)

    # ---- Merge ----
    print("[Phase2] Loading category mappings...", flush=True)
    t0 = time.time()

    cat_mappings = discover_category_mappings(c1_bulk, policy, version, categories)
    print(f"[Phase2] Loaded {len(cat_mappings)} category mappings", flush=True)

    combined_mapping, conflicts = merge_category_mappings(cat_mappings)
    if not combined_mapping:
        print("[Phase2] ERROR: combined mapping is empty — nothing to merge",
              flush=True)
        _write_merge_report(c1_bulk, "empty_mapping")
        return 1

    if conflicts:
        print(f"[Phase2] WARNING: {len(conflicts)} conflicts detected",
              flush=True)
        for c in conflicts[:10]:
            print(f"  Conflict: {c['old_key']} — "
                  f"{c['existing']['target']} vs {c['conflicting']['target']}",
                  flush=True)
        # Fall back to sequential merge
        print("[Phase2] Falling back to sequential merge...", flush=True)
        _sequential_merge(
            dataset_root, c1_bulk, policy, version, categories,
            group_label, scene_files,
        )
    else:
        print("[Phase2] No conflicts — using combined mapping "
              f"({len(combined_mapping)} pairs)", flush=True)
        _combined_merge(
            dataset_root, combined_mapping, scene_files,
        )

    merge_elapsed = time.time() - t0
    print(f"[Phase2] Merge complete ({merge_elapsed:.1f}s)", flush=True)

    # ---- Mega-Scan ----
    print("[Phase2] Mega-scan: scanning for old asset references...", flush=True)
    t1 = time.time()

    _setup_scripts_path()
    from scan_utils import (
        build_combined_old_asset_path_set,
        _scan_tree_pxr,
    )

    mapping_paths = [
        str(c1_bulk / f"{cat}_{policy}_{version}" / "01_cert"
             / "filtered_mapping.json")
        for cat, _ in cat_mappings
    ]
    old_asset_set = build_combined_old_asset_path_set(
        mapping_paths, dataset_root.name
    )
    print(f"[Phase2] Checking for {len(old_asset_set)} old asset paths",
          flush=True)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    scan_result = _scan_tree_pxr(
        dataset_root,
        old_asset_usd_rel_set=old_asset_set,
        dataset_name=dataset_root.name,
        exclude_dir_contains=["/_dedup_assets/"],
        progress_every=5000,
        progress_json=c1_bulk / f"phase2_mega_scan_{stamp}.json",
        progress_jsonl=c1_bulk / f"phase2_mega_scan_{stamp}.jsonl",
    )

    hit_files = scan_result.get("hit_files", 0)
    scan_elapsed = time.time() - t1
    print(f"[Phase2] Mega-scan: {hit_files} hit files out of "
          f"{scan_result.get('scanned_files', 0)} scanned "
          f"({scan_elapsed:.1f}s)", flush=True)

    if hit_files > 0:
        print("[Phase2] MEGA-SCAN FAILED — old asset references still exist",
              flush=True)
        _write_merge_report(
            c1_bulk, "scan_failed",
            scan_result=scan_result,
            merge_elapsed=merge_elapsed,
            scan_elapsed=scan_elapsed,
        )
        return 1
    print("[Phase2] Mega-scan PASSED — no old asset references", flush=True)

    # ---- Soft-Delete ----
    print("[Phase2] Soft-deleting old assets...", flush=True)
    t2 = time.time()

    delete_errors = _mega_soft_delete(
        dataset_root, bak_root, cat_mappings, stamp,
    )
    delete_elapsed = time.time() - t2
    print(f"[Phase2] Soft-delete complete ({delete_elapsed:.1f}s), "
          f"{len(delete_errors)} errors", flush=True)

    # ---- Post-Delete Scan ----
    print("[Phase2] Post-delete scan...", flush=True)

    post_result = _scan_tree_pxr(
        dataset_root,
        old_asset_usd_rel_set=old_asset_set,
        dataset_name=dataset_root.name,
        exclude_dir_contains=["/_dedup_assets/"],
        progress_every=5000,
        progress_json=c1_bulk / f"phase2_post_delete_scan_{stamp}.json",
        progress_jsonl=c1_bulk / f"phase2_post_delete_scan_{stamp}.jsonl",
    )

    post_hits = post_result.get("hit_files", 0)
    print(f"[Phase2] Post-delete scan: {post_hits} hit files", flush=True)

    total_elapsed = time.time() - t0
    _write_merge_report(
        c1_bulk, "complete",
        categories_processed=len(cat_mappings),
        merged_pairs=len(combined_mapping),
        conflicts=len(conflicts),
        merge_elapsed=merge_elapsed,
        scan_elapsed=scan_elapsed,
        delete_elapsed=delete_elapsed,
        total_elapsed=total_elapsed,
        scanned_files=scan_result.get("scanned_files", 0),
        hit_files=hit_files,
        post_hits=post_hits,
        delete_errors=len(delete_errors),
    )

    return 0 if (hit_files == 0 and post_hits == 0) else 1


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    sys.exit(main())
