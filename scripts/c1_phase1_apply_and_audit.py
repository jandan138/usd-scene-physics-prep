#!/usr/bin/env python3
"""Phase 1: Per-category apply+audit entry point for parallel DLC jobs.

Processes exactly ONE category: builds mapping, applies rewrites to all
scene files (layout.usd, start_result_interaction.usd,
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


REPO_ROOT = Path(__file__).resolve().parents[1]
ISAAC_PY = REPO_ROOT / "scripts" / "isaac_python.sh"


def _changed_scene_ids_from_batch_summary(
    batch_summary_path: Path, dataset_root: Path
) -> list[str]:
    if not batch_summary_path.exists():
        return []
    try:
        payload = json.loads(batch_summary_path.read_text(encoding="utf-8"))
    except Exception:
        return []

    changed: set[str] = set()
    base = dataset_root / "GRScenes100"
    for item in payload.get("per_layout", []):
        if item.get("scene_file") != "layout.usd":
            continue
        counts = item.get("counts") or {}
        if not any(
            int(counts.get(k, 0))
            for k in ("refs_changed", "payloads_changed", "asset_attrs_changed")
        ):
            continue
        layout_in = item.get("layout_in")
        if not isinstance(layout_in, str):
            continue
        try:
            rel = Path(layout_in).resolve().relative_to(base.resolve())
        except Exception:
            continue
        if len(rel.parts) >= 3:
            changed.add(f"{rel.parts[0]}/{rel.parts[1]}")
    return sorted(changed)


def main():
    ap = argparse.ArgumentParser(description="C1 Phase 1: apply+audit for one category")
    ap.add_argument("--category", required=True)
    ap.add_argument("--dataset-root", required=True)
    ap.add_argument("--bak-root", required=True)
    ap.add_argument("--report", required=True, help="Unified dedup report JSON")
    ap.add_argument("--c1-bulk-dir", required=True)
    ap.add_argument("--group-label", required=True)
    ap.add_argument("--bbox-gated", action="store_true")
    ap.add_argument("--bbox-policy", default="bbox_primary_rmse_observe",
                    choices=["bbox_primary_rmse_observe", "bbox_primary_rmse_harder"])
    ap.add_argument("--dedup-mode", default="geom_only",
                    choices=["geom_only", "shape_invariant", "topo_filesize", "transitive"])
    ap.add_argument("--v-matrix-mode", default="auto",
                    choices=["none", "auto"])
    ap.add_argument("--out-version", default="v1")
    ap.add_argument("--scene-files",
                    default="layout.usd,start_result_interaction.usd,"
                            "start_result_navigation.usd")
    ap.add_argument("--mode-reports-dir", default=None,
                    help="Directory containing dedup mode reports "
                         "(required when --bbox-gated)")
    ap.add_argument("--input-layout-name", default=None)
    ap.add_argument("--set-instanceable", action="store_true", default=True)
    ap.add_argument("--no-set-instanceable", dest="set_instanceable",
                    action="store_false")
    ap.add_argument("--baseline-root", default=None)
    ap.add_argument("--eps-bbox", type=float, default=0.01)
    ap.add_argument("--eps-pos", type=float, default=0.01)
    ap.add_argument("--eps-angle", type=float, default=1.0)
    ap.add_argument("--eps-geom", type=float, default=0.01)
    args = ap.parse_args()

    category = args.category
    dataset_root = Path(args.dataset_root).resolve()
    report_path = Path(args.report).resolve()
    c1_bulk_dir = Path(args.c1_bulk_dir).resolve()
    group_label = args.group_label

    # Category output directory structure (matches autorun BBoxCategoryPlan)
    cat_dir = c1_bulk_dir / f"{category}_{args.bbox_policy}_{args.out_version}"
    cert_dir = cat_dir / "01_cert"
    apply_dir = cat_dir / "02_apply"
    audit_dir = cat_dir / "03_audit"

    # Paths matching autorun BBoxCategoryPlan fields
    mapping_json = cert_dir / "filtered_mapping.json"
    mapping_stats_json = cert_dir / "filtered_mapping.stats.json"
    certificate_jsonl = cert_dir / "pair_certificates.jsonl"
    certificate_summary_json = cert_dir / "pair_certificate_summary.json"
    certified_graph_json = cert_dir / "certified_graph.json"
    reject_ledger_jsonl = apply_dir / "rewrite_reject_ledger.jsonl"
    audit_report_json = audit_dir / "placement_pairwise_compare.json"
    audit_verdict_json = audit_dir / "audit_verdict.json"
    out_name = f"layout.{group_label}_{category}_{args.bbox_policy}_{args.out_version}.usd"

    script_dir = Path(__file__).resolve().parent

    if not ISAAC_PY.exists():
        raise FileNotFoundError(f"Missing Isaac wrapper: {ISAAC_PY}")

    # ---- Step 1: Build mapping (cert step) ----
    # Exact args matching c1_autorun_categories.py _run_bbox_gated cert build
    print(f"[Phase1:{category}] Step 1: Building mapping...", flush=True)
    t0 = time.time()

    build_cmd = [
        str(ISAAC_PY),
        str(script_dir / "c1_build_bulk_mapping_from_dedup_report.py"),
        "--report", str(report_path),
        "--dataset-root", str(dataset_root),
        "--category", category,
        "--out-mapping-json", str(mapping_json),
        "--out-stats-json", str(mapping_stats_json),
    ]
    if args.bbox_gated:
        build_cmd += [
            "--bbox-gated",
            "--bbox-policy", args.bbox_policy,
            "--dedup-mode", args.dedup_mode,
            "--out-certificate-jsonl", str(certificate_jsonl),
            "--out-certificate-summary-json", str(certificate_summary_json),
            "--out-certified-graph-json", str(certified_graph_json),
        ]
    if args.mode_reports_dir:
        build_cmd += ["--mode-reports-dir", args.mode_reports_dir]

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
    # Exact args matching c1_autorun_categories.py _run_bbox_gated bulk apply
    print(f"[Phase1:{category}] Step 2: Applying rewrites...", flush=True)
    t1 = time.time()

    apply_cmd = [
        str(ISAAC_PY),
        str(script_dir / "c1_bulk_apply_layout_dedup.py"),
        "--dataset-root", str(dataset_root),
        "--mapping-json", str(mapping_json),
    ]
    if args.bbox_gated:
        apply_cmd += [
            "--mapping-stats-json", str(mapping_stats_json),
            "--certificate-jsonl", str(certificate_jsonl),
        ]
    apply_cmd += [
        "--group-label", group_label,
        "--out-name", out_name,
        "--scene-files", args.scene_files,
        "--report-dir", str(apply_dir),
    ]
    if args.bbox_gated:
        apply_cmd += [
            "--bbox-gated",
            "--bbox-policy", args.bbox_policy,
            "--reject-ledger-jsonl", str(reject_ledger_jsonl),
        ]
    apply_cmd += [
        "--v-matrix-mode", args.v_matrix_mode,
    ]
    if args.baseline_root:
        apply_cmd += ["--baseline-root", str(args.baseline_root)]
    if args.mode_reports_dir:
        apply_cmd += ["--mode-reports-dir", args.mode_reports_dir]
    if args.input_layout_name:
        apply_cmd += ["--input-layout-name", args.input_layout_name]
    if args.set_instanceable:
        apply_cmd += ["--set-instanceable"]

    r = subprocess.run(apply_cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print(f"[Phase1:{category}] Step 2 FAILED", flush=True)
        print(r.stderr, flush=True)
        _write_phase1_done(cat_dir, "failed",
                           f"Step2 apply failed: {r.stderr[:500]}")
        return 1

    apply_elapsed = time.time() - t1
    print(f"[Phase1:{category}] Step 2 OK ({apply_elapsed:.1f}s)", flush=True)

    # ---- Get changed scene IDs from batch summary ----
    changed_scene_ids = _changed_scene_ids_from_batch_summary(
        apply_dir / "batch_summary.json", dataset_root
    )
    changed_scene_list_json = apply_dir / "changed_scene_ids.json"
    changed_scene_list_json.parent.mkdir(parents=True, exist_ok=True)
    changed_scene_list_json.write_text(
        json.dumps(changed_scene_ids, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    # ---- Step 3: Audit ----
    # Exact args matching c1_autorun_categories.py _build_bbox_audit_cmd
    print(f"[Phase1:{category}] Step 3: Running audit...", flush=True)
    t2 = time.time()

    if changed_scene_ids:
        audit_cmd = [
            str(ISAAC_PY),
            str(script_dir / "placement_pairwise_compare.py"),
            "--left-root", str(dataset_root),
            "--right-root", str(dataset_root),
            "--left-mode", "current",
            "--right-mode", "current",
            "--right-layout-name", out_name,
            "--label", f"{category}_{args.bbox_policy}",
            "--out", str(audit_report_json),
            "--verdict-out", str(audit_verdict_json),
            "--scene-list-json", str(changed_scene_list_json),
            "--certificate-jsonl", str(certificate_jsonl),
            "--bbox-policy", args.bbox_policy,
            "--eps-bbox", str(args.eps_bbox),
            "--eps-pos", str(args.eps_pos),
            "--eps-angle", str(args.eps_angle),
            "--eps-geom", str(args.eps_geom),
            "--allow-no-mesh",
        ]
        if args.mode_reports_dir:
            audit_cmd += ["--mode-reports-dir", args.mode_reports_dir]

        r = subprocess.run(audit_cmd, capture_output=True, text=True)
        if r.returncode != 0:
            print(f"[Phase1:{category}] Step 3 FAILED", flush=True)
            print(r.stderr, flush=True)
            _write_phase1_done(cat_dir, "failed",
                               f"Step3 audit failed: {r.stderr[:500]}")
            return 1
    else:
        # No changed layouts → write empty aggregate
        # (matches autorun no_changed_layouts fallback)
        empty_aggregate = {
            "scenes_ok": 0,
            "scenes_error": 0,
            "total_common_prims": 0,
            "total_compared": 0,
            "total_no_mesh": 0,
            "total_only_in_left": 0,
            "total_only_in_right": 0,
            "total_ref_changed": 0,
            "total_ref_same": 0,
            "ref_changed_hard_fail_count": 0,
            "displaced_breakdown": {
                "gt_0.01": 0,
                "gt_0.1": 0,
                "gt_1.0": 0,
                "gt_10.0": 0,
            },
            "max_per_mesh_breakdown": {
                "gt_0.01": 0,
                "gt_0.1": 0,
                "gt_1.0": 0,
                "gt_10.0": 0,
            },
            "vertex_rmse_breakdown": {
                "gt_0.01": 0,
                "gt_0.1": 0,
                "gt_1.0": 0,
                "gt_10.0": 0,
            },
            "total_vertex_rmse_count": 0,
            "ref_changed_breakdown": {
                "gt_0.01": 0,
                "gt_0.1": 0,
                "gt_1.0": 0,
                "gt_10.0": 0,
            },
            "ref_same_breakdown": {
                "gt_0.01": 0,
                "gt_0.1": 0,
                "gt_1.0": 0,
                "gt_10.0": 0,
            },
            "blocking_reason_counts": {},
            "category_maxima": {},
            "global_top_50_worst": [],
            "scene_errors": [],
        }
        audit_dir.mkdir(parents=True, exist_ok=True)
        audit_report_json.write_text(
            json.dumps(
                {
                    "label": f"{category}_{args.bbox_policy}",
                    "bbox_policy": args.bbox_policy,
                    "changed_scene_ids": changed_scene_ids,
                    "aggregate": empty_aggregate,
                    "per_scene": [],
                    "skipped_reason": "no_changed_layouts",
                },
                indent=2,
                ensure_ascii=False,
            )
            + "\n",
            encoding="utf-8",
        )
        audit_verdict_json.write_text(
            json.dumps(
                {
                    "label": f"{category}_{args.bbox_policy}",
                    "policy": args.bbox_policy,
                    "thresholds": {
                        "eps_bbox": args.eps_bbox,
                        "eps_pos": args.eps_pos,
                        "eps_angle": args.eps_angle,
                        "eps_geom": args.eps_geom,
                    },
                    "passed": True,
                    "compared_scope_complete": True,
                    "scenes_error": 0,
                    "total_no_mesh": 0,
                    "ref_changed_hard_fail_count": 0,
                    "blocking_reason_counts": {},
                    "skipped_reason": "no_changed_layouts",
                },
                indent=2,
                ensure_ascii=False,
            )
            + "\n",
            encoding="utf-8",
        )

    audit_elapsed = time.time() - t2

    # Check audit verdict
    audit_passed = False
    if audit_verdict_json.exists():
        try:
            verdict = json.loads(audit_verdict_json.read_text())
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
        note="ok" if audit_passed else "audit verdict not passed",
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
