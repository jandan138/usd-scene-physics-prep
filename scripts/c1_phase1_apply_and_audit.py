#!/usr/bin/env python3
"""Phase 1: Per-category apply+audit entry point for parallel DLC jobs.

Processes exactly ONE category: builds mapping, applies rewrites to all
3 scene files (layout.usd, start_result_interaction.usd,
start_result_navigation.usd), runs placement audit, and writes a
phase1_done.json completion marker.

Usage:
  c1_phase1_apply_and_audit.py \
    --category <cat> --dataset-root <path> --bak-root <path> \
    --report <path> --c1-bulk-dir <path> \
    --group-label <label> --bbox-gated --bbox-policy <policy> \
    --dedup-mode geom_only --v-matrix-mode auto --out-version v1
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path


def main():
    ap = argparse.ArgumentParser(description="C1 Phase 1: apply+audit for one category")
    ap.add_argument("--category", required=True)
    ap.add_argument("--dataset-root", required=True)
    ap.add_argument("--bak-root", required=True)
    ap.add_argument("--report", required=True, help="Unified dedup report JSON")
    ap.add_argument("--c1-bulk-dir", required=True)
    ap.add_argument("--group-label", required=True)
    ap.add_argument("--bbox-gated", action="store_true")
    ap.add_argument("--bbox-policy", default="bbox_primary_rmse_observe")
    ap.add_argument("--dedup-mode", default="geom_only")
    ap.add_argument("--v-matrix-mode", default="auto")
    ap.add_argument("--out-version", default="v1")
    ap.add_argument("--scene-files",
                    default="layout.usd,start_result_interaction.usd,"
                            "start_result_navigation.usd")
    args = ap.parse_args()

    category = args.category
    dataset_root = Path(args.dataset_root)
    bak_root = Path(args.bak_root)
    report_path = Path(args.report)
    c1_bulk_dir = Path(args.c1_bulk_dir)
    group_label = args.group_label

    # Category output directory
    cat_dir = c1_bulk_dir / f"{category}_{args.bbox_policy}_{args.out_version}"
    report_dir = cat_dir
    report_dir.mkdir(parents=True, exist_ok=True)

    # Step1 marker
    step1_dir = cat_dir / "01_cert"
    mapping_json = step1_dir / "filtered_mapping.json"

    script_dir = Path(__file__).resolve().parent

    # ---- Step 1: Build mapping ----
    print(f"[Phase1:{category}] Step 1: Building mapping...", flush=True)
    t0 = time.time()

    build_cmd = [
        sys.executable,
        str(script_dir / "c1_build_bulk_mapping_from_dedup_report.py"),
        "--report", str(report_path),
        "--category", category,
        "--c1-bulk-dir", str(c1_bulk_dir),
        "--dedup-mode", args.dedup_mode,
    ]
    if args.bbox_gated:
        build_cmd += ["--bbox-gated", "--bbox-policy", args.bbox_policy]
    build_cmd += ["--out-dir", str(step1_dir)]

    r = subprocess.run(build_cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(f"[Phase1:{category}] Step 1 FAILED", flush=True)
        print(r.stderr, flush=True)
        _write_phase1_done(cat_dir, "failed",
                          f"Step1 build_mapping failed: {r.stderr[:500]}")
        return 1

    if not mapping_json.exists():
        print(f"[Phase1:{category}] No mapping pairs (mapping_pairs_0)", flush=True)
        _write_phase1_done(cat_dir, "ok", audit_passed=True,
                          note="no mapping pairs")
        return 0

    # ---- Step 2: Apply rewrites ----
    print(f"[Phase1:{category}] Step 2: Applying rewrites...", flush=True)
    t1 = time.time()

    out_name = f"layout.parallel_{group_label}.{category}_{args.bbox_policy}_{args.out_version}.usd"
    apply_cmd = [
        sys.executable,
        str(script_dir / "c1_bulk_apply_layout_dedup.py"),
        "--mapping-json", str(mapping_json),
        "--dataset-root", str(dataset_root),
        "--bak-root", str(bak_root),
        "--report-dir", str(report_dir),
        "--out-name", out_name,
        "--bbox-gated" if args.bbox_gated else "--no-bbox-gated",
    ]
    if args.bbox_gated:
        apply_cmd += ["--bbox-policy", args.bbox_policy]
    apply_cmd += [
        "--v-matrix-mode", args.v_matrix_mode,
        "--scene-files", args.scene_files,
    ]

    r = subprocess.run(apply_cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(f"[Phase1:{category}] Step 2 FAILED", flush=True)
        print(r.stderr, flush=True)
        _write_phase1_done(cat_dir, "failed",
                          f"Step2 apply failed: {r.stderr[:500]}")
        return 1

    apply_elapsed = time.time() - t1
    print(f"[Phase1:{category}] Step 2 OK ({apply_elapsed:.1f}s)", flush=True)

    # ---- Step 3: Audit ----
    print(f"[Phase1:{category}] Step 3: Running audit...", flush=True)
    t2 = time.time()

    audit_dir = cat_dir / "03_audit"
    layout_root = dataset_root / "GRScenes100"

    audit_cmd = [
        sys.executable,
        str(script_dir / "placement_pairwise_compare.py"),
        "--layout-root", str(layout_root),
        "--dataset-root", str(dataset_root),
        "--subset-dirs", f"GRScenes100:{str(layout_root)}",
        "--report-dir", str(audit_dir),
        "--out-name", out_name,
        "--mode", "audit",
        "--output-basename", "placement_pairwise_compare",
    ]
    if args.bbox_gated:
        audit_cmd += [
            "--bbox-gated",
            "--bbox-policy", args.bbox_policy,
        ]

    r = subprocess.run(audit_cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(f"[Phase1:{category}] Step 3 FAILED", flush=True)
        print(r.stderr, flush=True)
        _write_phase1_done(cat_dir, "failed",
                          f"Step3 audit failed: {r.stderr[:500]}")
        return 1

    audit_elapsed = time.time() - t2

    # Check audit verdict
    verdict_json = audit_dir / "audit_verdict.json"
    audit_passed = False
    if verdict_json.exists():
        try:
            verdict = json.loads(verdict_json.read_text())
            audit_passed = bool(verdict.get("passed", False))
        except Exception:
            pass

    print(f"[Phase1:{category}] Step 3 {'PASSED' if audit_passed else 'FAILED'} "
          f"({audit_elapsed:.1f}s)", flush=True)

    total_elapsed = time.time() - t0
    _write_phase1_done(
        cat_dir,
        "ok" if audit_passed else "failed",
        audit_passed=audit_passed,
        elapsed_sec=total_elapsed,
        note=f"ok" if audit_passed else "audit verdict not passed",
        mapping_json=str(mapping_json),
        out_name=out_name,
    )
    return 0 if audit_passed else 1


def _write_phase1_done(
    cat_dir: Path,
    status: str,
    audit_passed: bool = False,
    elapsed_sec: float = 0.0,
    note: str = "",
    **extra,
):
    payload = {
        "status": status,
        "audit_passed": audit_passed,
        "elapsed_sec": elapsed_sec,
        "note": note,
        **extra,
    }
    cat_dir.mkdir(parents=True, exist_ok=True)
    (cat_dir / "phase1_done.json").write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )


if __name__ == "__main__":
    sys.exit(main())
