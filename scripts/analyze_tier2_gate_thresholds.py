#!/usr/bin/env python3
"""Offline threshold analysis for topo_filesize Tier-2 gate recovery.

Sweeps candidate threshold combinations over the spike CSV of 297 rejected
topo_filesize dedup pairs and reports:
  1. Parameter sweep ranked by recovered count
  2. Baseline-bbox bucketing for the recommended config
  3. Per-canonical grouping with flags
  4. Low close_pct pairs for the recommended config
"""

import argparse
import json
import sys
from itertools import product
from pathlib import Path

import numpy as np
import pandas as pd

DEFAULT_CSV = "check_reports/test0_rebuilt_dedup/topo_precheck_recovery_spike.csv"
DEFAULT_OUT = "check_reports/test0_rebuilt_dedup/tier2_threshold_analysis.json"

# Recommended config
REC = dict(nn_bbox=0.05, nn_mean_dist_norm=0.02, nn_unique_ratio=0.5, nn_close_pct=10)


def load(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    # Ensure numeric
    for c in ["nn_bbox", "nn_unique_ratio", "nn_mean_dist_norm", "nn_close_pct", "baseline_bbox"]:
        df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


def passes(df: pd.DataFrame, bb: float, ur: float, md: float, cp: float) -> pd.DataFrame:
    mask = (
        (df["nn_bbox"] <= bb)
        & (df["nn_unique_ratio"] >= ur)
        & (df["nn_mean_dist_norm"] <= md)
        & (df["nn_close_pct"] >= cp)
    )
    return df[mask]


def pct(s: pd.Series, q: float):
    return float(np.nanpercentile(s, q)) if len(s) > 0 else None


def sweep(df: pd.DataFrame):
    bb_vals = [0.01, 0.05, 0.10, 0.15, 0.50]
    ur_vals = [0.3, 0.5, 0.7, 0.9]
    md_vals = [0.01, 0.02, 0.05]
    cp_vals = [0, 10, 15, 20]

    rows = []
    for bb, ur, md, cp in product(bb_vals, ur_vals, md_vals, cp_vals):
        sub = passes(df, bb, ur, md, cp)
        n = len(sub)
        rows.append(dict(
            nn_bbox_le=bb, nn_unique_ratio_ge=ur,
            nn_mean_dist_norm_le=md, nn_close_pct_ge=cp,
            recovered=n,
            nn_bbox_p50=pct(sub["nn_bbox"], 50),
            nn_bbox_p95=pct(sub["nn_bbox"], 95),
            nn_bbox_max=float(sub["nn_bbox"].max()) if n else None,
        ))
    result = sorted(rows, key=lambda r: -r["recovered"])
    return result


def print_sweep(rows):
    print("=" * 100)
    print("PARAMETER SWEEP — Top 20 by recovered count")
    print("=" * 100)
    fmt = "{:>10} {:>14} {:>18} {:>14} {:>9} {:>10} {:>10} {:>10}"
    print(fmt.format("nn_bbox<=", "nn_uniq_r>=", "nn_mean_dist<=", "nn_close>=",
                      "recovered", "bb_p50", "bb_p95", "bb_max"))
    print("-" * 100)
    for r in rows[:20]:
        is_rec = " <-- REC" if (
            r["nn_bbox_le"] == REC["nn_bbox"]
            and r["nn_unique_ratio_ge"] == REC["nn_unique_ratio"]
            and r["nn_mean_dist_norm_le"] == REC["nn_mean_dist_norm"]
            and r["nn_close_pct_ge"] == REC["nn_close_pct"]
        ) else ""
        print(fmt.format(
            r["nn_bbox_le"], r["nn_unique_ratio_ge"],
            r["nn_mean_dist_norm_le"], r["nn_close_pct_ge"],
            r["recovered"],
            f'{r["nn_bbox_p50"]:.5f}' if r["nn_bbox_p50"] is not None else "-",
            f'{r["nn_bbox_p95"]:.5f}' if r["nn_bbox_p95"] is not None else "-",
            f'{r["nn_bbox_max"]:.5f}' if r["nn_bbox_max"] is not None else "-",
        ) + is_rec)

    # Print recommended row explicitly if not in top 20
    rec_row = next((r for r in rows if (
        r["nn_bbox_le"] == REC["nn_bbox"]
        and r["nn_unique_ratio_ge"] == REC["nn_unique_ratio"]
        and r["nn_mean_dist_norm_le"] == REC["nn_mean_dist_norm"]
        and r["nn_close_pct_ge"] == REC["nn_close_pct"]
    )), None)
    if rec_row:
        rank = next(i for i, r in enumerate(rows) if r is rec_row) + 1
        print(f"\nRecommended config rank: #{rank}, recovered: {rec_row['recovered']}/{len(rows)}")


def bbox_buckets(df: pd.DataFrame):
    sub = passes(df, REC["nn_bbox"], REC["nn_unique_ratio"],
                 REC["nn_mean_dist_norm"], REC["nn_close_pct"])
    bins = [0, 1, 5, 10, float("inf")]
    labels = ["<1", "1-5", "5-10", ">10"]
    sub = sub.copy()
    sub["bucket"] = pd.cut(sub["baseline_bbox"], bins=bins, labels=labels, right=False)
    rows = []
    print("\n" + "=" * 80)
    print(f"BASELINE_BBOX BUCKETS (recommended config, {len(sub)} recovered)")
    print("=" * 80)
    fmt = "{:>10} {:>8} {:>12} {:>12} {:>12}"
    print(fmt.format("bucket", "count", "bb_p50", "bb_p95", "bb_max"))
    print("-" * 80)
    for label in labels:
        g = sub[sub["bucket"] == label]
        n = len(g)
        row = dict(
            bucket=label, count=n,
            nn_bbox_p50=pct(g["nn_bbox"], 50),
            nn_bbox_p95=pct(g["nn_bbox"], 95),
            nn_bbox_max=float(g["nn_bbox"].max()) if n else None,
        )
        rows.append(row)
        print(fmt.format(
            label, n,
            f'{row["nn_bbox_p50"]:.5f}' if row["nn_bbox_p50"] is not None else "-",
            f'{row["nn_bbox_p95"]:.5f}' if row["nn_bbox_p95"] is not None else "-",
            f'{row["nn_bbox_max"]:.5f}' if row["nn_bbox_max"] is not None else "-",
        ))
    return rows


def per_canonical(df: pd.DataFrame):
    sub = passes(df, REC["nn_bbox"], REC["nn_unique_ratio"],
                 REC["nn_mean_dist_norm"], REC["nn_close_pct"])
    grouped = sub.groupby("canonical_asset").agg(
        count=("nn_bbox", "size"),
        nn_bbox_median=("nn_bbox", "median"),
        nn_bbox_max=("nn_bbox", "max"),
    ).reset_index().sort_values("count", ascending=False)

    rows = []
    print("\n" + "=" * 100)
    print(f"PER-CANONICAL GROUPING (recommended config, {len(grouped)} canonicals)")
    print("=" * 100)
    fmt = "{:>5} {:>10} {:>12} {:>60}"
    print(fmt.format("count", "bb_median", "bb_max", "canonical_asset"))
    print("-" * 100)
    for _, r in grouped.iterrows():
        flag = ""
        if r["count"] > 5:
            flag += " [HIGH_COUNT]"
        if r["nn_bbox_max"] > 0.04:
            flag += " [HIGH_BBOX]"
        row = dict(
            canonical_asset=r["canonical_asset"],
            count=int(r["count"]),
            nn_bbox_median=float(r["nn_bbox_median"]),
            nn_bbox_max=float(r["nn_bbox_max"]),
            flags=flag.strip(),
        )
        rows.append(row)
        short = r["canonical_asset"][-58:] if len(r["canonical_asset"]) > 58 else r["canonical_asset"]
        print(fmt.format(
            int(r["count"]),
            f'{r["nn_bbox_median"]:.5f}',
            f'{r["nn_bbox_max"]:.5f}',
            short,
        ) + flag)
    return rows


def low_close_pct(df: pd.DataFrame):
    sub = passes(df, REC["nn_bbox"], REC["nn_unique_ratio"],
                 REC["nn_mean_dist_norm"], REC["nn_close_pct"])
    low = sub[sub["nn_close_pct"] < 20].sort_values("nn_close_pct")
    rows = []
    print("\n" + "=" * 120)
    print(f"LOW CLOSE_PCT PAIRS (nn_close_pct < 20%, {len(low)} pairs)")
    print("=" * 120)
    if len(low) == 0:
        print("  (none)")
        return rows
    fmt = "{:>10} {:>10} {:>14} {:>12} {:>40} {:>40}"
    print(fmt.format("close_pct", "nn_bbox", "mean_dist_norm", "uniq_ratio",
                      "old_asset", "canonical"))
    print("-" * 120)
    for _, r in low.iterrows():
        row = dict(
            old_asset=r["old_asset"],
            canonical_asset=r["canonical_asset"],
            nn_close_pct=float(r["nn_close_pct"]),
            nn_bbox=float(r["nn_bbox"]),
            nn_mean_dist_norm=float(r["nn_mean_dist_norm"]),
            nn_unique_ratio=float(r["nn_unique_ratio"]),
        )
        rows.append(row)
        old_short = r["old_asset"].split("/")[-3] if "/" in r["old_asset"] else r["old_asset"]
        can_short = r["canonical_asset"].split("/")[-3] if "/" in r["canonical_asset"] else r["canonical_asset"]
        print(fmt.format(
            f'{r["nn_close_pct"]:.2f}',
            f'{r["nn_bbox"]:.5f}',
            f'{r["nn_mean_dist_norm"]:.5f}',
            f'{r["nn_unique_ratio"]:.4f}',
            old_short[-38:],
            can_short[-38:],
        ))
    return rows


def main():
    parser = argparse.ArgumentParser(description="Tier-2 gate threshold analysis")
    parser.add_argument("--csv", default=DEFAULT_CSV, help="Spike CSV path")
    parser.add_argument("--output", default=DEFAULT_OUT, help="JSON report output path")
    args = parser.parse_args()

    df = load(args.csv)
    print(f"Loaded {len(df)} pairs from {args.csv}")
    print(f"Columns: {list(df.columns)}")
    print(f"nn_bbox range: [{df['nn_bbox'].min():.6f}, {df['nn_bbox'].max():.6f}]")
    print(f"nn_unique_ratio range: [{df['nn_unique_ratio'].min():.4f}, {df['nn_unique_ratio'].max():.4f}]")
    print(f"nn_mean_dist_norm range: [{df['nn_mean_dist_norm'].min():.6f}, {df['nn_mean_dist_norm'].max():.6f}]")
    print(f"nn_close_pct range: [{df['nn_close_pct'].min():.2f}, {df['nn_close_pct'].max():.2f}]")

    # 1. Parameter sweep
    sweep_rows = sweep(df)
    print_sweep(sweep_rows)

    # 2. Baseline bbox buckets
    bucket_rows = bbox_buckets(df)

    # 3. Per-canonical grouping
    canonical_rows = per_canonical(df)

    # 4. Low close_pct
    low_rows = low_close_pct(df)

    # Write JSON report
    report = dict(
        total_pairs=len(df),
        recommended_config=REC,
        parameter_sweep=sweep_rows[:50],  # top 50
        baseline_bbox_buckets=bucket_rows,
        per_canonical=canonical_rows,
        low_close_pct=low_rows,
    )
    out_path = Path(args.output)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"\nJSON report written to {out_path}")


if __name__ == "__main__":
    main()
