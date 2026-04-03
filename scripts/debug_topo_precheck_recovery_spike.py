#!/usr/bin/env python3
"""Spike: topo cert rejects (bbox_precheck_failed_topo_filesize) — can NN rescue?

Reads pair_certificates.jsonl, filters topo bbox precheck failures, loads vertices,
recomputes baseline Procrustes bbox vs NN-path bbox and NN quality metrics.

Does NOT modify compute_vertex_transform.py. Use to size policy relaxations.

Example:
  python3 scripts/debug_topo_precheck_recovery_spike.py \\
    --cert-jsonl check_reports/test0_rebuilt_dedup/c1_bulk_v8_shuffle_fix/\\
bottle_bbox_primary_rmse_observe_v1/01_cert/pair_certificates.jsonl \\
    --dataset-root GRScenes-test0-rebuilt-normalize-prededup \\
    --out-csv check_reports/test0_rebuilt_dedup/topo_precheck_recovery_spike.csv
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import sys
import time
from typing import Any, Dict, List, Optional

import numpy as np

# Repo root on path for scripts.compute_vertex_transform
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from scipy.spatial import cKDTree

from scripts.compute_vertex_transform import (  # noqa: E402
    _bbox_delta_max_abs,
    _normalize_to_unit_bbox,
    _nn_procrustes_in_normalized_space,
    extract_instance_space_vertices,
    procrustes_full,
)

REJECT = "bbox_precheck_failed_topo_filesize"
CLOSE_GATE = 95.0
ACCEPT_CUR = 0.01
PRECHECK = 0.5


def _load_rows(path: str) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    with open(path, encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            o = json.loads(line)
            if o.get("reject_reason") == REJECT:
                rows.append(o)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--cert-jsonl", required=True)
    ap.add_argument("--dataset-root", required=True)
    ap.add_argument("--out-csv", required=True)
    ap.add_argument("--max-pairs", type=int, default=0, help="0 = all")
    args = ap.parse_args()

    root = os.path.abspath(args.dataset_root)
    rows = _load_rows(args.cert_jsonl)
    if args.max_pairs > 0:
        rows = rows[: args.max_pairs]

    os.makedirs(os.path.dirname(os.path.abspath(args.out_csv)), exist_ok=True)

    fieldnames = [
        "old_asset",
        "canonical_asset",
        "cert_max_abs",
        "n_verts",
        "baseline_bbox",
        "nn_close_pct",
        "nn_unique_ratio",
        "nn_mean_dist_norm",
        "nn_bbox",
        "load_error",
        "wall_time_s",
    ]

    summary = {
        "total_rows": len(rows),
        "ok": 0,
        "load_fail": 0,
        "vtx_mismatch": 0,
        "nn_close_le_95": 0,
        "nn_close_gt_95": 0,
        "nn_bbox_le_001": 0,
        "nn_bbox_le_015": 0,
        "nn_bbox_le_050": 0,
        "would_pass_precheck_if_nn_v": 0,
        "cur_code_should_accept": 0,
    }

    with open(args.out_csv, "w", newline="", encoding="utf-8") as out:
        w = csv.DictWriter(out, fieldnames=fieldnames)
        w.writeheader()

        for o in rows:
            t0 = time.perf_counter()
            rec: Dict[str, Any] = {k: "" for k in fieldnames}
            rec["old_asset"] = o.get("old_asset", "")
            rec["canonical_asset"] = o.get("canonical_asset", "")
            bd = o.get("bbox_delta") or {}
            rec["cert_max_abs"] = bd.get("max_abs", "")

            load_error = ""
            try:
                c_path = os.path.join(root, o["canonical_asset"])
                o_path = os.path.join(root, o["old_asset"])
                pts_c = extract_instance_space_vertices(c_path)
                pts_o = extract_instance_space_vertices(o_path)
            except Exception as exc:
                load_error = str(exc)[:200]
                rec["load_error"] = load_error
                rec["wall_time_s"] = f"{time.perf_counter() - t0:.4f}"
                w.writerow(rec)
                summary["load_fail"] += 1
                continue

            if len(pts_c) != len(pts_o):
                rec["load_error"] = f"vtx_mismatch {len(pts_c)} vs {len(pts_o)}"
                rec["n_verts"] = f"{len(pts_c)}/{len(pts_o)}"
                rec["wall_time_s"] = f"{time.perf_counter() - t0:.4f}"
                w.writerow(rec)
                summary["vtx_mismatch"] += 1
                continue

            n = len(pts_c)
            rec["n_verts"] = str(n)

            Vb = procrustes_full(pts_c, pts_o)
            bb = _bbox_delta_max_abs(Vb, pts_c, pts_o)
            rec["baseline_bbox"] = f"{bb:.8f}"

            try:
                Vn, nn_close, nn_uq = _nn_procrustes_in_normalized_space(pts_c, pts_o)
            except Exception as exc:
                rec["load_error"] = f"nn_failed:{exc}"[:200]
                rec["wall_time_s"] = f"{time.perf_counter() - t0:.4f}"
                w.writerow(rec)
                summary["load_fail"] += 1
                continue

            nb = _bbox_delta_max_abs(Vn, pts_c, pts_o)
            rec["nn_bbox"] = f"{nb:.8f}"
            rec["nn_close_pct"] = f"{nn_close:.4f}"
            rec["nn_unique_ratio"] = f"{nn_uq:.6f}"

            c_norm, _, _ = _normalize_to_unit_bbox(pts_c)
            o_norm, _, _ = _normalize_to_unit_bbox(pts_o)
            dists, _ = cKDTree(o_norm).query(c_norm)
            rec["nn_mean_dist_norm"] = f"{float(np.mean(dists)):.8f}"

            rec["wall_time_s"] = f"{time.perf_counter() - t0:.4f}"
            w.writerow(rec)
            summary["ok"] += 1

            if nn_close <= CLOSE_GATE:
                summary["nn_close_le_95"] += 1
            else:
                summary["nn_close_gt_95"] += 1
            if nb <= 0.01:
                summary["nn_bbox_le_001"] += 1
            if nb <= 0.15:
                summary["nn_bbox_le_015"] += 1
            if nb <= PRECHECK:
                summary["nn_bbox_le_050"] += 1
                summary["would_pass_precheck_if_nn_v"] += 1
            if nn_close > CLOSE_GATE and nb <= ACCEPT_CUR:
                summary["cur_code_should_accept"] += 1

    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
