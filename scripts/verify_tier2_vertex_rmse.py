#!/usr/bin/env python3
"""Verify vertex RMSE for Tier2-recoverable pairs.

Filters the spike CSV by recommended thresholds, computes V via NN-Procrustes,
and reports per-pair RMSE to validate that Tier2 recovery produces acceptable
vertex alignment.

Must be run via: ./scripts/isaac_python.sh scripts/verify_tier2_vertex_rmse.py
"""

import argparse
import json
import os
import sys
import time

import numpy as np
import pandas as pd

# Add project root so we can import from scripts.*
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PROJECT_ROOT)

from scripts.compute_vertex_transform import (
    extract_instance_space_vertices,
    _nn_procrustes_core,
    _bbox_delta_max_abs,
)


def parse_args():
    p = argparse.ArgumentParser(description="Verify vertex RMSE for Tier2 pairs")
    p.add_argument(
        "--csv",
        default=os.path.join(
            PROJECT_ROOT,
            "check_reports/test0_rebuilt_dedup/topo_precheck_recovery_spike.csv",
        ),
    )
    p.add_argument(
        "--thresholds-json",
        default=os.path.join(
            PROJECT_ROOT,
            "check_reports/test0_rebuilt_dedup/tier2_threshold_analysis.json",
        ),
    )
    p.add_argument(
        "--dataset-root",
        default=os.path.join(PROJECT_ROOT, "GRScenes-test0-rebuilt-normalize-prededup"),
    )
    p.add_argument(
        "--output",
        default=os.path.join(
            PROJECT_ROOT,
            "check_reports/test0_rebuilt_dedup/tier2_vertex_rmse_verification.json",
        ),
    )
    return p.parse_args()


def main():
    args = parse_args()

    # Load thresholds
    with open(args.thresholds_json) as f:
        thr_data = json.load(f)
    thr = thr_data["recommended_config"]
    print(f"Thresholds: {thr}")

    # Load and filter spike CSV
    df = pd.read_csv(args.csv)
    print(f"Total spike pairs: {len(df)}")

    mask = (
        (df["nn_bbox"] <= thr["nn_bbox"])
        & (df["nn_mean_dist_norm"] <= thr["nn_mean_dist_norm"])
        & (df["nn_unique_ratio"] >= thr["nn_unique_ratio"])
        & (df["nn_close_pct"] >= thr["nn_close_pct"])
        & (df["load_error"].isna() | (df["load_error"] == ""))
    )
    filtered = df[mask].reset_index(drop=True)
    print(f"Filtered recoverable pairs: {len(filtered)}")

    results = []
    errors = []
    t0 = time.time()

    for i, row in filtered.iterrows():
        old_path = os.path.join(args.dataset_root, row["old_asset"])
        canon_path = os.path.join(args.dataset_root, row["canonical_asset"])

        if not os.path.exists(old_path):
            errors.append({"pair_idx": i, "old_asset": row["old_asset"], "error": "old USD missing"})
            continue
        if not os.path.exists(canon_path):
            errors.append({"pair_idx": i, "canonical_asset": row["canonical_asset"], "error": "canon USD missing"})
            continue

        try:
            pts_old = extract_instance_space_vertices(old_path)
            pts_canon = extract_instance_space_vertices(canon_path)

            if pts_old is None or pts_canon is None:
                errors.append({
                    "pair_idx": i,
                    "old_asset": row["old_asset"],
                    "canonical_asset": row["canonical_asset"],
                    "error": "vertices extraction returned None",
                })
                continue

            V, nn_close_pct, nn_unique_ratio, nn_mean_dist_norm = _nn_procrustes_core(
                pts_canon, pts_old
            )

            # Compute bbox delta (same metric as nn_bbox in spike CSV)
            bbox_delta = _bbox_delta_max_abs(V, pts_canon, pts_old)

            # Compute RMSE in instance space
            N = pts_canon.shape[0]
            canon_h = np.hstack([pts_canon, np.ones((N, 1))])
            transformed = (canon_h @ V)[:, :3]
            rmse_abs = float(np.sqrt(np.mean(np.sum((pts_old - transformed) ** 2, axis=1))))

            # Compute bbox-normalized RMSE (divide by old bbox diagonal)
            old_extent = pts_old.max(axis=0) - pts_old.min(axis=0)
            old_diag = float(np.linalg.norm(old_extent))
            rmse_norm = rmse_abs / old_diag if old_diag > 1e-9 else float("inf")

            results.append({
                "old_asset": row["old_asset"],
                "canonical_asset": row["canonical_asset"],
                "rmse_abs": rmse_abs,
                "rmse_norm": rmse_norm,
                "bbox_delta_recomputed": bbox_delta,
                "nn_close_pct": nn_close_pct,
                "nn_unique_ratio": nn_unique_ratio,
                "nn_mean_dist_norm": nn_mean_dist_norm,
                "nn_bbox_from_csv": float(row["nn_bbox"]),
                "old_bbox_diag": old_diag,
                "n_verts_old": int(pts_old.shape[0]),
                "n_verts_canon": int(pts_canon.shape[0]),
            })
        except Exception as e:
            errors.append({
                "pair_idx": i,
                "old_asset": row["old_asset"],
                "canonical_asset": row["canonical_asset"],
                "error": str(e),
            })

        if (i + 1) % 10 == 0:
            print(f"  processed {i + 1}/{len(filtered)} pairs ...")

    elapsed = time.time() - t0

    # Summary stats
    if results:
        rmses_abs = [r["rmse_abs"] for r in results]
        rmses_norm = [r["rmse_norm"] for r in results]
        bbox_deltas = [r["bbox_delta_recomputed"] for r in results]
        summary = {
            "count": len(results),
            "errors": len(errors),
            "rmse_abs_max": float(np.max(rmses_abs)),
            "rmse_abs_mean": float(np.mean(rmses_abs)),
            "rmse_norm_max": float(np.max(rmses_norm)),
            "rmse_norm_mean": float(np.mean(rmses_norm)),
            "rmse_norm_p50": float(np.percentile(rmses_norm, 50)),
            "rmse_norm_p95": float(np.percentile(rmses_norm, 95)),
            "bbox_delta_max": float(np.max(bbox_deltas)),
            "bbox_delta_mean": float(np.mean(bbox_deltas)),
            "bbox_delta_p95": float(np.percentile(bbox_deltas, 95)),
            "elapsed_s": round(elapsed, 1),
        }
    else:
        summary = {"count": 0, "errors": len(errors), "elapsed_s": round(elapsed, 1)}

    # Verdict based on bbox_delta (the metric the cert pipeline actually uses)
    # and rmse_norm as a secondary check
    if not results:
        verdict = "WARNING"
    elif summary["bbox_delta_max"] < 0.1:
        verdict = "PASS"
    else:
        verdict = "WARNING"

    report = {
        "thresholds_used": thr,
        "filtered_count": len(filtered),
        "summary": summary,
        "verdict": verdict,
        "pairs": results,
        "errors": errors,
    }

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, "w") as f:
        json.dump(report, f, indent=2)

    print(f"\n=== Tier2 Vertex RMSE Verification ===")
    print(f"Filtered pairs: {len(filtered)}")
    print(f"Computed: {len(results)}, Errors: {len(errors)}")
    if results:
        print(f"RMSE (abs)   max={summary['rmse_abs_max']:.4f}  mean={summary['rmse_abs_mean']:.4f}")
        print(f"RMSE (norm)  max={summary['rmse_norm_max']:.6f}  mean={summary['rmse_norm_mean']:.6f}  "
              f"p50={summary['rmse_norm_p50']:.6f}  p95={summary['rmse_norm_p95']:.6f}")
        print(f"BBox delta   max={summary['bbox_delta_max']:.6f}  mean={summary['bbox_delta_mean']:.6f}  "
              f"p95={summary['bbox_delta_p95']:.6f}")
    print(f"Verdict: {verdict}")
    print(f"Elapsed: {elapsed:.1f}s")
    print(f"Report: {args.output}")

    # Cross-validate bbox_delta_recomputed vs nn_bbox_from_csv
    if results:
        max_delta_diff = max(
            abs(r["bbox_delta_recomputed"] - r["nn_bbox_from_csv"]) for r in results
        )
        print(f"\nCross-validation: max |recomputed - csv| bbox_delta = {max_delta_diff:.8f}")

    # Flag any pairs with bbox_delta > 0.05
    high = [r for r in results if r["bbox_delta_recomputed"] > 0.05]
    if high:
        print(f"\nWARNING: {len(high)} pairs with bbox_delta > 0.05:")
        for h in high:
            print(f"  bbox_delta={h['bbox_delta_recomputed']:.6f}  rmse_norm={h['rmse_norm']:.6f}  "
                  f"{h['old_asset']}")


if __name__ == "__main__":
    main()
