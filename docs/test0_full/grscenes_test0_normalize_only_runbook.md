---
title: "GRScenes-test0 Normalize-Only Full Runbook"
code_reference:
  - "scripts/normalize_asset_transforms.py"
  - "scripts/placement_pairwise_compare.py"
  - "scripts/audit_normalize_phase2.py"
  - "scripts/check_normalize_gate_from_reports.py"
  - "scripts/build_normalize_gate_verdict.py"
created_at: "2026-03-12"
updated_at: "2026-03-12"
maintainer: "Codex Worker-3"
status: "active"
---

# Purpose

This runbook defines the exact normalize-only execution for:

- source: `GRScenes-test0`
- output: `GRScenes-test0-normalized`
- stop boundary: hard gate before any dedup, MDL rewrite, or other post-normalize mutation

This is the canonical full-run sequence for the current repository state on
`2026-03-12`.

# Locked Decisions

1. Use direct `isaac_python.sh` invocations and a local shell loop, not
   `scripts/dlc/submit_normalize_phase1.sh` or
   `scripts/dlc/submit_normalize_phase2.sh`.
   Those helpers are hard-coded to `GRScenes-test1`, and the Phase 2 helper
   also omits the required `--centers-dir`.
2. Run Phase 1 per category against the flat `GRScenes-test0` asset layout,
   then merge `centers_*.json` into one flat directory before Phase 2.
   `audit_normalize_phase2.py` and `normalize_asset_transforms.py --phase 2`
   only read `centers_*.json` from a single directory, not nested per-category
   subdirectories.
3. Treat the output root as fresh-only.
   Do not reuse an existing `GRScenes-test0-normalized`, because the audit step
   selects the earliest `layout.pre_c1_*.usd` backup in each scene directory.
4. Create a synthetic `layout.pre_c1_normalize_only.<RUN_ID>.usd` for every
   normalized scene immediately after Phase 2.
   This is required so the normalize audit targets the pre-dedup normalized
   state.
5. Use a zero-tolerance normalize gate before dedup:
   - pairwise `displaced_breakdown.gt_0.01 == 0`
   - pairwise `ref_same_breakdown.gt_0.01 == 0`
   - audit `center_found == common_ref_prim_count`
   - audit `matrix_mismatch == 0`
   - audit `missing_pre_c1_scenes == []`
   - audit inert descendant override counts both `0`
6. Stop before dedup even if normalization outputs exist, unless the hard gate
   passes and the run summary is archived.

# Fixed Source Inventory

The exact source inventory this runbook is written against is:

- `114` asset category directories under `GRScenes-test0/GRScenes_assets`
- `99` scenes total under `GRScenes-test0/GRScenes100`
- scene split: `69` home + `30` commercial

If these counts do not match at runtime, stop and refresh the runbook instead
of silently continuing.

# Known Source-Bad Assets

Phase 1 is expected to surface these inherited source defects:

- `cabinet/b98d6ccbeb75dfdeb60e27649a5b055a`
- `other/d41d8cd98f00b204e9800998ecf8427e`
- `person/351316cbb083f9f4df0cccd60cbfa848`

These are known bad inputs from the source dataset, not normalize regressions.
The full run still stops for any additional Phase 1 failures outside this
allowlist.

# Directory Layout

```text
/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/
├── GRScenes-test0/
│   ├── GRScenes_assets/
│   └── GRScenes100/
├── GRScenes-test0-normalized/                  # created by this run, must not pre-exist
│   ├── GRScenes_assets/                        # normalized-style layout: <cat>/<uid>/usd/<uid>.usd
│   └── GRScenes100/
└── check_reports/test0_full/<RUN_ID>/
    ├── normalize/
    │   ├── phase1/
    │   │   ├── categories.txt
    │   │   ├── phase1_exit_codes.tsv
    │   │   ├── <category>/
    │   │   │   ├── centers_<category>.json
    │   │   │   ├── normalize_report.json
    │   │   │   ├── stdout.log
    │   │   │   └── stderr.log
    │   │   └── centers_merged/
    │   │       ├── centers_*.json
    │   │       └── merge_summary.json
    │   ├── phase2/
    │   │   ├── normalize_report.json
    │   │   ├── stdout.log
    │   │   └── stderr.log
    │   └── verify/
    │       ├── test0_vs_normalized_pre_dedup.json
    │       ├── normalize_phase2_audit.json
    │       └── normalize_gate_verdict.json
    └── summary/
        ├── final_verdict.json
        └── summary.md
```

Notes:

- `GRScenes-test0` uses the flat legacy asset layout
  `<category>/<uid>/<uid>.usd`.
- `GRScenes-test0-normalized` is expected to use the normalized layout
  `<category>/<uid>/usd/<uid>.usd`.
- `Material/` is not part of this normalize-only stop point. Do not expect it
  under `GRScenes-test0-normalized` yet.

# Command Order

## 1. Set Variables And Enforce Fresh Roots

```bash
set -euo pipefail

REPO_ROOT=/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
SRC_ROOT=$REPO_ROOT/GRScenes-test0
SRC_ASSETS=$SRC_ROOT/GRScenes_assets
SRC_SCENES=$SRC_ROOT/GRScenes100
OUT_ROOT=$REPO_ROOT/GRScenes-test0-normalized

RUN_ID=${RUN_ID:-$(date -u +%Y%m%d_%H%M%S)}
REPORT_ROOT=$REPO_ROOT/check_reports/test0_full/$RUN_ID
PHASE1_ROOT=$REPORT_ROOT/normalize/phase1
CENTERS_MERGED=$PHASE1_ROOT/centers_merged
PHASE2_ROOT=$REPORT_ROOT/normalize/phase2
VERIFY_ROOT=$REPORT_ROOT/normalize/verify
SUMMARY_ROOT=$REPORT_ROOT/summary

export REPO_ROOT SRC_ROOT SRC_ASSETS SRC_SCENES OUT_ROOT
export RUN_ID REPORT_ROOT PHASE1_ROOT CENTERS_MERGED PHASE2_ROOT VERIFY_ROOT SUMMARY_ROOT

cd "$REPO_ROOT"

mkdir -p "$PHASE1_ROOT" "$CENTERS_MERGED" "$PHASE2_ROOT" "$VERIFY_ROOT" "$SUMMARY_ROOT"

test -d "$SRC_ASSETS"
test -d "$SRC_SCENES"
test ! -e "$OUT_ROOT"
```

## 2. Freeze The Expected Source Inventory

```bash
CATEGORY_COUNT=$(find "$SRC_ASSETS" -mindepth 1 -maxdepth 1 -type d | wc -l | tr -d ' ')
SCENE_COUNT=$(find "$SRC_SCENES" -mindepth 2 -maxdepth 2 -type d | wc -l | tr -d ' ')
HOME_SCENE_COUNT=$(find "$SRC_SCENES/home" -mindepth 1 -maxdepth 1 -type d | wc -l | tr -d ' ')
COMMERCIAL_SCENE_COUNT=$(find "$SRC_SCENES/commercial" -mindepth 1 -maxdepth 1 -type d | wc -l | tr -d ' ')

test "$CATEGORY_COUNT" = "114"
test "$SCENE_COUNT" = "99"
test "$HOME_SCENE_COUNT" = "69"
test "$COMMERCIAL_SCENE_COUNT" = "30"

find "$SRC_ASSETS" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort > "$PHASE1_ROOT/categories.txt"
```

## 3. Run Phase 1 Per Category

This loop continues across categories and records per-category exit status. The
hard stop comes after the allowlist check, not at the first known-bad category.

```bash
: > "$PHASE1_ROOT/phase1_exit_codes.tsv"

while IFS= read -r cat; do
  cat_report_dir="$PHASE1_ROOT/$cat"
  mkdir -p "$cat_report_dir"

  set +e
  ./scripts/isaac_python.sh scripts/normalize_asset_transforms.py \
    --assets-root "$SRC_ASSETS" \
    --scenes-root "$SRC_SCENES" \
    --output-root "$OUT_ROOT" \
    --category "$cat" \
    --phase 1 \
    --symlink-copy \
    --report-dir "$cat_report_dir" \
    > "$cat_report_dir/stdout.log" \
    2> "$cat_report_dir/stderr.log"
  rc=$?
  set -e

  printf '%s\t%s\n' "$cat" "$rc" >> "$PHASE1_ROOT/phase1_exit_codes.tsv"
done < "$PHASE1_ROOT/categories.txt"
```

## 4. Phase 1 Allowlist Gate

This accepts only the three known source-bad assets above and rejects all other
Phase 1 failures.

```bash
python3 - <<'PY'
import json
import os
import pathlib

phase1_root = pathlib.Path(os.environ["PHASE1_ROOT"])
categories = [line.strip() for line in (phase1_root / "categories.txt").read_text().splitlines() if line.strip()]
reports = sorted(p for p in phase1_root.glob("*/normalize_report.json") if p.parent.name != "centers_merged")

allowed_categories = {"cabinet", "other", "person"}
allowed_assets = {
    "cabinet/b98d6ccbeb75dfdeb60e27649a5b055a/b98d6ccbeb75dfdeb60e27649a5b055a.usd",
    "other/d41d8cd98f00b204e9800998ecf8427e/d41d8cd98f00b204e9800998ecf8427e.usd",
    "person/351316cbb083f9f4df0cccd60cbfa848/351316cbb083f9f4df0cccd60cbfa848.usd",
}

if len(reports) != len(categories):
    raise SystemExit(f"missing phase1 reports: expected {len(categories)} got {len(reports)}")

nonzero_categories = {}
for line in (phase1_root / "phase1_exit_codes.tsv").read_text().splitlines():
    cat, rc = line.split("\t", 1)
    nonzero_categories[cat] = int(rc)

unexpected_nonzero = sorted(cat for cat, rc in nonzero_categories.items() if rc != 0 and cat not in allowed_categories)
if unexpected_nonzero:
    raise SystemExit(f"unexpected nonzero phase1 categories: {unexpected_nonzero}")

observed_errors = set()
unexpected_errors = []
centers_missing = []
for report_path in reports:
    data = json.loads(report_path.read_text())
    centers = list(report_path.parent.glob("centers_*.json"))
    if not centers:
        centers_missing.append(report_path.parent.name)
    for err in data.get("errors", []):
        asset = (err.get("asset") or "").replace("/usd/", "/")
        observed_errors.add(asset)
        if asset not in allowed_assets:
            unexpected_errors.append({"report": str(report_path), "asset": asset, "error": err.get("error")})

if centers_missing:
    raise SystemExit(f"missing centers json for categories: {centers_missing}")
if unexpected_errors:
    raise SystemExit(f"unexpected phase1 errors: {unexpected_errors}")
if observed_errors != allowed_assets:
    raise SystemExit(f"phase1 error set changed: observed={sorted(observed_errors)}")

print("phase1 allowlist gate: ok")
PY
```

## 5. Merge Centers Into One Flat Directory

```bash
find "$PHASE1_ROOT" \
  -mindepth 2 -maxdepth 2 \
  -name 'centers_*.json' \
  ! -path "$CENTERS_MERGED/*" \
  -exec cp -f {} "$CENTERS_MERGED"/ \;

MERGED_COUNT=$(find "$CENTERS_MERGED" -mindepth 1 -maxdepth 1 -name 'centers_*.json' | wc -l | tr -d ' ')
test "$MERGED_COUNT" = "$CATEGORY_COUNT"

python3 - <<'PY'
import json
import os
dst = os.path.abspath(os.environ["CENTERS_MERGED"])
payload = {
    "src_root": os.path.abspath(os.environ["PHASE1_ROOT"]),
    "dst_root": dst,
    "merged_centers_files": sorted(
        f for f in os.listdir(dst) if f.startswith("centers_") and f.endswith(".json")
    ),
}
with open(os.path.join(dst, "merge_summary.json"), "w", encoding="utf-8") as f:
    json.dump(payload, f, indent=2, ensure_ascii=False)
PY
```

## 6. Run Phase 2 Across All 99 Scenes

```bash
./scripts/isaac_python.sh scripts/normalize_asset_transforms.py \
  --assets-root "$SRC_ASSETS" \
  --scenes-root "$SRC_SCENES" \
  --output-root "$OUT_ROOT" \
  --phase 2 \
  --centers-dir "$CENTERS_MERGED" \
  --report-dir "$PHASE2_ROOT" \
  > "$PHASE2_ROOT/stdout.log" \
  2> "$PHASE2_ROOT/stderr.log"
```

## 7. Phase 2 Report Gate

```bash
python3 - <<'PY'
import json
import os
from pathlib import Path

report = json.loads(Path(os.environ["PHASE2_ROOT"]).joinpath("normalize_report.json").read_text())
meta = report["meta"]

assert meta["scenes_processed"] == 99, meta
assert meta["errors_count"] == 0, meta
assert meta["descendant_overrides_found"] == meta["descendant_overrides_remapped"], meta
assert meta["prims_compensated"] > 0, meta

print("phase2 report gate: ok")
PY
```

## 8. Create Synthetic Pre-C1 Snapshots

This step is mandatory. `audit_normalize_phase2.py` audits the earliest
`layout.pre_c1_*.usd` in each normalized scene directory.

```bash
if find "$OUT_ROOT/GRScenes100" -name 'layout.pre_c1_*.usd' -print -quit | grep -q .; then
  echo "pre_c1 backups already exist under $OUT_ROOT; refusing to reuse output root" >&2
  exit 1
fi

find "$OUT_ROOT/GRScenes100" -mindepth 2 -maxdepth 2 -type d | while IFS= read -r scene_dir; do
  cp "$scene_dir/layout.usd" "$scene_dir/layout.pre_c1_normalize_only.$RUN_ID.usd"
done

PRE_C1_COUNT=$(find "$OUT_ROOT/GRScenes100" -name "layout.pre_c1_normalize_only.$RUN_ID.usd" | wc -l | tr -d ' ')
test "$PRE_C1_COUNT" = "99"
```

## 9. Run Full Normalize Verification

```bash
./scripts/isaac_python.sh scripts/placement_pairwise_compare.py \
  --left-root "$SRC_ROOT" \
  --right-root "$OUT_ROOT" \
  --left-mode current \
  --right-mode current \
  --label test0_vs_normalized_pre_dedup \
  --workers 8 \
  --out "$VERIFY_ROOT/test0_vs_normalized_pre_dedup.json"

./scripts/isaac_python.sh scripts/audit_normalize_phase2.py \
  --source-root "$SRC_ROOT" \
  --normalized-root "$OUT_ROOT" \
  --centers-dir "$CENTERS_MERGED" \
  --workers 8 \
  --out "$VERIFY_ROOT/normalize_phase2_audit.json"
```

## 10. Evaluate The Hard Gate And Archive The Verdict

`check_normalize_gate_from_reports.py` is the machine gate.
`build_normalize_gate_verdict.py` archives the operator-facing summary.

```bash
set +e
python3 scripts/check_normalize_gate_from_reports.py \
  --pairwise-report "$VERIFY_ROOT/test0_vs_normalized_pre_dedup.json" \
  --audit-report "$VERIFY_ROOT/normalize_phase2_audit.json" \
  --out "$VERIFY_ROOT/normalize_gate_verdict.json"
GATE_RC=$?
set -e

python3 scripts/build_normalize_gate_verdict.py \
  --run-id "$RUN_ID" \
  --pairwise-json "$VERIFY_ROOT/test0_vs_normalized_pre_dedup.json" \
  --audit-json "$VERIFY_ROOT/normalize_phase2_audit.json" \
  --out-json "$SUMMARY_ROOT/final_verdict.json" \
  --out-md "$SUMMARY_ROOT/summary.md"

test "$GATE_RC" -eq 0
```

# Expected Artifacts

## Output Dataset

- `GRScenes-test0-normalized/GRScenes_assets/<category>/<uid>/usd/<uid>.usd`
- `GRScenes-test0-normalized/GRScenes100/<home|commercial>/<scene_id>/layout.usd`
- `GRScenes-test0-normalized/GRScenes100/<home|commercial>/<scene_id>/layout.pre_c1_normalize_only.<RUN_ID>.usd`

## Phase 1 Reports

- `check_reports/test0_full/<RUN_ID>/normalize/phase1/categories.txt`
- `check_reports/test0_full/<RUN_ID>/normalize/phase1/phase1_exit_codes.tsv`
- `check_reports/test0_full/<RUN_ID>/normalize/phase1/<category>/centers_<category>.json`
- `check_reports/test0_full/<RUN_ID>/normalize/phase1/<category>/normalize_report.json`
- `check_reports/test0_full/<RUN_ID>/normalize/phase1/centers_merged/centers_*.json`
- `check_reports/test0_full/<RUN_ID>/normalize/phase1/centers_merged/merge_summary.json`

## Phase 2 And Gate Reports

- `check_reports/test0_full/<RUN_ID>/normalize/phase2/normalize_report.json`
- `check_reports/test0_full/<RUN_ID>/normalize/verify/test0_vs_normalized_pre_dedup.json`
- `check_reports/test0_full/<RUN_ID>/normalize/verify/normalize_phase2_audit.json`
- `check_reports/test0_full/<RUN_ID>/normalize/verify/normalize_gate_verdict.json`
- `check_reports/test0_full/<RUN_ID>/summary/final_verdict.json`
- `check_reports/test0_full/<RUN_ID>/summary/summary.md`

# Stop Conditions Before Dedup

Stop immediately and do not start dedup if any of these happen:

1. Source inventory no longer matches `114` categories and `99` scenes.
2. `GRScenes-test0-normalized` already exists, or any `layout.pre_c1_*.usd`
   already exists under the target output root.
3. Any Phase 1 category outside `cabinet`, `other`, `person` exits non-zero.
4. The Phase 1 error set is not exactly the three known source-bad assets.
5. Any discovered category is missing `centers_<category>.json`.
6. The merged centers directory does not contain one `centers_*.json` per
   discovered category.
7. `phase2/normalize_report.json` reports any errors, processes fewer than
   `99` scenes, or leaves descendant overrides unremapped.
8. Fewer than `99` synthetic `layout.pre_c1_normalize_only.<RUN_ID>.usd`
   backups are created.
9. `check_normalize_gate_from_reports.py` returns non-zero.
10. Any of the following gate metrics are non-zero or inconsistent:
    - `pairwise.aggregate.displaced_breakdown.gt_0.01`
    - `pairwise.aggregate.ref_same_breakdown.gt_0.01`
    - `audit.aggregate.totals.matrix_mismatch`
    - `audit.aggregate.totals.pre_c1_inert_descendant_xform_override_count`
    - `audit.aggregate.totals.became_inert_descendant_xform_override_count`
    - `audit.missing_pre_c1_scenes`
    - `audit.aggregate.totals.center_found != audit.aggregate.totals.common_ref_prim_count`

# Explicit Out Of Scope

Do not do any of the following in this run:

- asset dedup scanning
- C1 mapping / promote / soft-delete
- MDL path rewriting
- texture symlink creation
- layout patching beyond Phase 2 compensation and the synthetic `pre_c1` copy

The only handoff artifact that can authorize the next phase is a passing
normalize gate under:

- `check_reports/test0_full/<RUN_ID>/normalize/verify/normalize_gate_verdict.json`
- `check_reports/test0_full/<RUN_ID>/summary/final_verdict.json`
