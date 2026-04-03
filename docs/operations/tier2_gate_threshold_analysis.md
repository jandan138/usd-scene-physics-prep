---
title: Tier-2 Gate Threshold Analysis for Topo Precheck Recovery
code_reference: scripts/analyze_tier2_gate_thresholds.py
created_at: 2026-04-03
updated_at: 2026-04-03
maintainer: TraeAI
status: complete
---

# Tier-2 Gate Threshold Analysis

## Purpose

Offline analysis of 297 rejected topo_filesize dedup pairs to find optimal thresholds for a Tier-2 recovery gate that relaxes the current NN precheck while maintaining safety.

## Script

`scripts/analyze_tier2_gate_thresholds.py` — pandas-based parameter sweep over 4 threshold dimensions.

## Key Findings

### Recommended Config (nn_bbox<=0.05, nn_mean_dist_norm<=0.02, nn_unique_ratio>=0.5, nn_close_pct>=10%)

- **Recovers 77 / 297 pairs** (26% recovery rate)
- Rank #54 out of 240 sweep combinations (conservative)
- nn_bbox max among recovered: 0.04984

### Data Ranges in Spike CSV

| Metric | Min | Max |
|--------|-----|-----|
| nn_bbox | 0.000026 | 25.75 |
| nn_unique_ratio | 0.005 | 0.992 |
| nn_mean_dist_norm | 0.003 | 0.644 |
| nn_close_pct | 0.00 | 100.00 |

### Top Sweep Results

Most aggressive configs (nn_bbox<=0.5) recover up to 214 pairs but with nn_bbox_max=0.43 (risky). The nn_bbox<=0.15 tier recovers 117-136 pairs with nn_bbox_max=0.148. The recommended config at nn_bbox<=0.05 is the most conservative, recovering 77 pairs with nn_bbox_max<0.05.

### Baseline BBox Buckets (Recommended Config)

| Bucket | Count | nn_bbox p50 | nn_bbox p95 | nn_bbox max |
|--------|-------|-------------|-------------|-------------|
| <1 | 23 | 0.00679 | 0.03288 | 0.04974 |
| 1-5 | 25 | 0.00889 | 0.04660 | 0.04984 |
| 5-10 | 24 | 0.02575 | 0.03755 | 0.03834 |
| >10 | 5 | 0.00413 | 0.01485 | 0.01752 |

The >10 bucket is small (5 pairs) and well-behaved. The 5-10 bucket has tight spread. The <1 and 1-5 buckets have the widest spread with nn_bbox approaching the 0.05 ceiling.

### Per-Canonical Flags

- 12 unique canonicals among recovered pairs
- 5 canonicals flagged with HIGH_COUNT (>5 pairs) — these are bottle assets
- 3 canonicals flagged with HIGH_BBOX (max nn_bbox > 0.04)

### Low Close_Pct Pairs

9 pairs have nn_close_pct < 20%, all mapping to a single canonical (`08c39f48a87efb271053989885194f88`). Three at 12.42% with nn_bbox ~0.028, six at 18.42% with nn_bbox ~0.007. All have nn_unique_ratio > 0.79, suggesting good vertex correspondence despite low close percentage.

## Output

- JSON report: `check_reports/test0_rebuilt_dedup/tier2_threshold_analysis.json`
- Sections: parameter_sweep (top 50), baseline_bbox_buckets, per_canonical, low_close_pct
