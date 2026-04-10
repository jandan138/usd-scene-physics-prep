---
title: "GRScenes-test0 Full Dedup Handoff Runbook"
code_reference:
  - scripts/dlc/dedup_by_category.py
  - scripts/union_dedup_reports.py
  - scripts/c1_build_bulk_mapping_from_dedup_report.py
  - scripts/c1_autorun_categories.py
  - scripts/c1_bulk_step6_category_promote_scan_soft_delete.py
created_at: "2026-03-12"
updated_at: "2026-03-12"
maintainer: "Codex Worker-4"
status: "active"
---

# GRScenes-test0 Full Dedup Handoff Runbook

This document defines the handoff contract from normalize to dedup for the full
`GRScenes-test0` dataset, and the exact execution path for scan, merge,
mapping, and C1 autorun.

The key rule is simple:

**Full dedup starts only from a passed canonical `GRScenes-test0-normalized`
tree.**

Do not start dedup from any smoke subset such as
`GRScenes-test0-smoke-S1-normalized`, and do not start from any temporary fix
output such as `/tmp/test0_desc_override_fix_phase2_v3/out`. The smoke run only
proved the descendant-override repair on an isolated output root; it did not
authorize dedup on the full dataset.

## 1. Upstream Handoff Contract

Dedup may start only when the normalize owner hands over all items below for the
same full-run `RUN_ID`.

| Item | Required path or artifact | Required state |
|---|---|---|
| Canonical normalized dataset root | `GRScenes-test0-normalized/GRScenes_assets` and `GRScenes-test0-normalized/GRScenes100` | Exists and is the exact tree to be deduped |
| Phase 1 centers bundle | `check_reports/test0_full/<RUN_ID>/normalize/phase1/centers_merged/` | Contains the `centers_*.json` files used by Phase 2, plus `merge_summary.json` |
| Phase 2 normalize report | `check_reports/test0_full/<RUN_ID>/normalize/phase2/normalize_report.json` | Exists and corresponds to `GRScenes-test0-normalized` |
| Normalize-only pairwise report | `check_reports/test0_full/<RUN_ID>/normalize/test0_vs_normalized_pre_dedup.json` | Exists and compares source `GRScenes-test0` against `GRScenes-test0-normalized` |
| Phase 2 audit report | `check_reports/test0_full/<RUN_ID>/normalize/test0_normalize_phase2_audit.json` | Exists and was run against the same centers bundle and normalized root |
| Normalize gate verdict | `check_reports/test0_full/<RUN_ID>/normalize/normalize_gate_verdict.json` | Exists and passes |

The normalize gate verdict is the upstream contract. It must encode all of the
following pass conditions:

- `status == "pass_normalize_gate"`
- `passed == true`
- `metrics.pairwise_displaced_gt_0.01 == 0`
- `metrics.pairwise_ref_same_gt_0.01 == 0`
- `metrics.center_found == metrics.common_ref_prim_count`
- `metrics.matrix_mismatch == 0`
- `metrics.missing_pre_c1_count == 0`
- `metrics.pre_c1_inert_descendant_xform_override_count == 0`
- `metrics.became_inert_descendant_xform_override_count == 0`

If any one of those checks is red, dedup does not start.

## 2. Current Interpretation Of The Smoke Evidence

The smoke work under `docs/records/runs/test0-smoke/20260312_074731/` matters only as a
precondition for the full rerun:

- The old official smoke verdict failed because the refrigerator path drifted
  and a descendant xform override became inert.
- The isolated repair run passed in
  `check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/`.
- That isolated pass is not itself a valid dedup input.

The normalize owner must rerun the official full `GRScenes-test0-normalized`
output with the repaired code path and then hand over the canonical artifacts
listed in Section 1.

## 3. Full Dedup Execution For `GRScenes-test0-normalized`

The current full-dedup standard in this repo is the 3-mode path:

1. `geom_only`
2. `shape_invariant`
3. `topo_filesize`
4. 3-way union merge
5. C1 autorun from the merged union report

`scripts/merge_dedup_reports.py` is still useful for the legacy
`merged_geom_only.json` flow, but it is not the promote target for the current
full test0 run. The authoritative C1 input should be the union-merged
`all_categories_union_merged.json`.

### 3.1 Suggested run variables

```bash
RUN_ID=20260312_full_test0
DATASET=GRScenes-test0-normalized
BAK_ROOT=GRScenes-test0-normalized_bak
SCAN_ROOT=check_reports/test0_full/$RUN_ID/dedup
C1_BULK_DIR=$SCAN_ROOT/c1_bulk
GEOM_DIR=$SCAN_ROOT/geom_only
SHAPE_DIR=$SCAN_ROOT/shape_invariant
TOPO_DIR=$SCAN_ROOT/topo_filesize
UNION_DIR=$SCAN_ROOT/union_3way
CHUNK_TOTAL=10
```

`CHUNK_TOTAL` is the worker count. The same commands work locally in a shell
loop or remotely on DLC by launching one chunk per worker.

### 3.2 Run the three scan modes

Geom-only scan:

```bash
for i in $(seq 0 $((CHUNK_TOTAL - 1))); do
  ./scripts/isaac_python.sh scripts/dlc/dedup_by_category.py \
    --chunk-id "$i" \
    --chunk-total "$CHUNK_TOTAL" \
    --assets-root "$DATASET/GRScenes_assets" \
    --out-dir "$GEOM_DIR" \
    --merge-tolerance 0.005 \
    --float-quantize-eps 1e-2 \
    --mode all
done
```

Shape-invariant scan:

```bash
for i in $(seq 0 $((CHUNK_TOTAL - 1))); do
  ./scripts/isaac_python.sh scripts/dlc/dedup_by_category.py \
    --chunk-id "$i" \
    --chunk-total "$CHUNK_TOTAL" \
    --assets-root "$DATASET/GRScenes_assets" \
    --out-dir "$SHAPE_DIR" \
    --merge-tolerance 0.005 \
    --float-quantize-eps 1e-2 \
    --mode shape_invariant \
    --hausdorff-threshold 0.05
done
```

Topo-filesize scan:

```bash
for i in $(seq 0 $((CHUNK_TOTAL - 1))); do
  ./scripts/isaac_python.sh scripts/dlc/dedup_by_category.py \
    --chunk-id "$i" \
    --chunk-total "$CHUNK_TOTAL" \
    --assets-root "$DATASET/GRScenes_assets" \
    --out-dir "$TOPO_DIR" \
    --merge-tolerance 0.005 \
    --float-quantize-eps 1e-2 \
    --mode topo_filesize \
    --filesize-tolerance 0.02
done
```

Optional but recommended summaries:

```bash
python3 scripts/summarize_dedup_reports.py --reports-dir "$GEOM_DIR"
python3 scripts/summarize_dedup_reports.py --reports-dir "$SHAPE_DIR"
python3 scripts/summarize_dedup_reports.py --reports-dir "$TOPO_DIR"
```

### 3.3 Run the 3-way union merge

```bash
python3 scripts/union_dedup_reports.py --batch \
  --geom-dir "$GEOM_DIR" \
  --shape-dir "$SHAPE_DIR" \
  --topo-dir "$TOPO_DIR" \
  --output-dir "$UNION_DIR" \
  --summary "$UNION_DIR/summary.json"
```

Then concatenate the per-category union reports into the single file that
`c1_autorun_categories.py` expects:

```bash
UNION_DIR="$UNION_DIR" python3 - <<'PY'
import glob
import json
import os
from pathlib import Path

union_dir = Path(os.environ["UNION_DIR"])
all_dupes = []
sources = []
for path in sorted(glob.glob(str(union_dir / "*" / "*_union_merged.json"))):
    data = json.load(open(path, "r", encoding="utf-8"))
    all_dupes.extend(data.get("duplicates", []))
    sources.append(path)

out = union_dir / "all_categories_union_merged.json"
payload = {
    "meta": {
        "mode": "union_merged_all",
        "merged_from": sources,
        "categories": len(sources),
    },
    "duplicates": all_dupes,
}
out.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
print(out)
PY
```

### 3.4 Build mapping for one category if you need a manual checkpoint

`c1_autorun_categories.py` builds mappings automatically, but this command is
useful for spot checks or category-only retries:

```bash
CATEGORY=bottle
./scripts/isaac_python.sh scripts/c1_build_bulk_mapping_from_dedup_report.py \
  --report "$UNION_DIR/all_categories_union_merged.json" \
  --dataset-root "$DATASET" \
  --category "$CATEGORY" \
  --out-mapping-json "$C1_BULK_DIR/${CATEGORY}_geom_only_mapping.json" \
  --out-stats-json "$C1_BULK_DIR/${CATEGORY}_geom_only_mapping.stats.json"
```

The output filename still says `geom_only_mapping` because the C1 scripts key
off that filename convention even when the input report came from the union
merge.

### 3.5 Run C1 autorun

Before autorun, clear stale per-category mappings from earlier datasets. The
script only rebuilds a mapping when the output file is missing.

```bash
rm -f "$C1_BULK_DIR"/*_geom_only_mapping.json
rm -f "$C1_BULK_DIR"/*_geom_only_mapping.stats.json
```

Then run autorun:

```bash
python3 scripts/c1_autorun_categories.py \
  --dataset-root "$DATASET" \
  --bak-root "$BAK_ROOT" \
  --report "$UNION_DIR/all_categories_union_merged.json" \
  --c1-bulk-dir "$C1_BULK_DIR" \
  --group-label test0_union_3way \
  --out-version v1 \
  --no-skip-done
```

Using a namespaced `--c1-bulk-dir` keeps mapping caches, step6 directories, and
autorun ledger isolated under the current `RUN_ID`, so concurrent or prior runs
from other datasets do not cause false skips.

## 4. Gates Before Promote And Soft-Delete

These are the go/no-go gates for the state-changing part of the run.

### 4.1 Before any C1 autorun starts

All of the following must be true:

- The upstream normalize handoff contract in Section 1 is complete and green.
- Each scan mode has a complete per-category report set for the categories under
  `GRScenes-test0-normalized/GRScenes_assets`.
- `union_3way/summary.json` exists and its `totals.categories_processed`
  matches the category count in the dataset.
- `union_3way/all_categories_union_merged.json` exists.
- Stale `"$C1_BULK_DIR"/*_geom_only_mapping*.json` files were removed.

### 4.2 Before promote for a given category

For each category, the rewrite stage must have completed cleanly:

- `"$C1_BULK_DIR"/<category>_bulk_batch_v1/batch_summary.json` exists.
- `"$C1_BULK_DIR"/<category>_bulk_batch_v1/spotcheck_list.md` exists.
- The mapping stats file exists and either:
  - `mapping_pairs > 0`, which means the category is eligible for promote, or
  - `mapping_pairs == 0`, which means the category is a clean skip.

### 4.3 Before soft-delete for a given category

`scripts/c1_bulk_step6_category_promote_scan_soft_delete.py` promotes first and
then scans. A category may proceed to soft-delete only if all post-promote scans
are clean:

- `promote_to_layout_usd_report.json` exists.
- `post_promote_layout_scan_pxr.json` reports `hit_layouts == 0`.
- `post_promote_scene_files_scan_pxr.json` reports `hit_files == 0`.
- `post_promote_full_usd_scan_excluding_backups_pxr.json` reports
  `hit_files == 0`.

The script hard-aborts on the full-tree scan gate. Operationally, the handoff
rule should be stricter: all three post-promote scan reports must be zero-hit
before anyone treats the category as safe to delete.

### 4.4 Run completion gate after soft-delete

After soft-delete, all of the following must hold:

- `soft_delete_old_assets_report.json` exists.
- `post_soft_delete_layout_scan_pxr.json` reports `hit_layouts == 0`.
- Moved assets live only under
  `GRScenes-test0-normalized_bak/_dedup_assets/<group_label>_<stamp>/`.
- The autorun ledger under `"$C1_BULK_DIR"/_autorun/<group_label>_<stamp>/ledger.jsonl`
  reaches `run_done`.

If any scan gate is non-zero, the run stops and the category is not considered
promoted for handoff purposes.

## 5. Explicit Handoff Statement

The dedup owner may accept `GRScenes-test0-normalized` only when the normalize
owner hands over:

1. The canonical full normalized dataset root.
2. The phase1 centers bundle actually used by Phase 2.
3. The phase2 normalize report.
4. The normalize-only pairwise report.
5. The phase2 audit report.
6. A passing `normalize_gate_verdict.json` with all zero-failure metrics listed
   in Section 1.

Only after that contract is present should the dedup owner run the 3-mode scan,
3-way union merge, mapping build, and C1 autorun described above.
