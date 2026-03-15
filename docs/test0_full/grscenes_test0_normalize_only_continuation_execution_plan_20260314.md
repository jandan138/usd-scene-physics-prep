---
title: "GRScenes-test0 Normalize-Only Continuation Execution Plan"
created_at: "2026-03-14"
updated_at: "2026-03-14"
maintainer: "Codex"
status: "active"
code_reference:
  - "scripts/normalize_asset_transforms.py"
  - "scripts/create_pre_c1_normalize_only_snapshots.py"
  - "scripts/placement_pairwise_compare.py"
  - "scripts/audit_normalize_phase2.py"
  - "scripts/assemble_normalize_gate_bundle.py"
---

# Purpose

This document is the execution-ready continuation plan for the first stage
only:

- start from the already completed `GRScenes-test0` Phase 1 output
- run `phase2 -> pre_c1 snapshots -> pairwise -> audit -> normalize-only hard gate`
- stop before any dedup work

This document is written for handoff into a separate execution dialogue. It is
not a proposal and it is not a full rerun plan.

# Locked Decisions

1. Treat the existing Phase 1 output from run `20260313_023200` as canonical.
   Do not rerun Phase 1 now.
2. Do not use `scripts/test0_full_normalize.py` for this continuation.
   That wrapper requires fresh empty roots and always reruns Phase 1 first.
3. Run Phase 2 directly with `scripts/normalize_asset_transforms.py --phase 2`.
4. Keep all continuation reports in a fresh report root separate from
   `check_reports/test0_full/20260313_023200`.
5. Stop before dedup regardless of outcome.

# Canonical Inputs

Use these existing artifacts exactly as the Phase 2 input set:

- source dataset root:
  `GRScenes-test0`
- canonical normalized asset root:
  `GRScenes-test0-normalized/GRScenes_assets`
- canonical Phase 1 centers bundle:
  `check_reports/test0_full/20260313_023200/normalize/phase1/centers_all.json`
- canonical Phase 1 log:
  `check_reports/test0_full/20260313_023200/logs/phase1.log`

Current accepted Phase 1 metrics:

- normalized assets:
  `85612 / 85647`
- errors:
  `35`
- elapsed:
  `56416.7s`
- centers entries:
  `85612`

# Important Interpretation

The wrapper run `check_reports/test0_full/20260313_023200` is marked failed
because the Phase 1 process exited nonzero after recording known asset errors.
That does **not** invalidate the emitted Phase 1 outputs above.

For this continuation, the Phase 1 output is treated as usable input and the
real question is whether the normalize-only hard gate passes after Phase 2.

# Working Assumption On Bad Assets

Current operator judgment is that the `35` Phase 1 failures are likely
non-impacting empty-mesh or placeholder assets.

Known evidence currently on record:

- the observed failure shape is `No meshes found under /Root/Instance`
- the run includes at least the known bad source assets:
  - `cabinet/b98d6ccbeb75dfdeb60e27649a5b055a`
  - `other/d41d8cd98f00b204e9800998ecf8427e`
  - `person/351316cbb083f9f4df0cccd60cbfa848`
- `d41d8cd98f00b204e9800998ecf8427e` appears in `23` source scenes

This assumption does **not** change the gate rule:

- if normalize-only hard gate fails, first test whether the failure intersects
  any of these `35` Phase 1 bad assets before attributing the failure to Phase
  2 compensation logic

# Continuation Report Contract

Create a fresh continuation report root, for example:

- `check_reports/test0_full/20260314_phase2_probe_from_20260313_023200`

Required report tree:

```text
check_reports/test0_full/<CONTINUATION_RUN_ID>/
├── normalize/
│   ├── phase2/
│   │   └── normalize_report.json
│   ├── pre_c1_snapshot_report.json
│   ├── test0_vs_normalized_pre_dedup.json
│   └── test0_normalize_phase2_audit.json
└── summary/
    ├── normalize_gate_verdict.json
    ├── final_verdict.json
    └── summary.md
```

Do not write continuation outputs into the old failed wrapper run directory.

# Freshness Rules

The continuation is allowed to reuse:

- `GRScenes-test0-normalized/GRScenes_assets`
- `check_reports/test0_full/20260313_023200/normalize/phase1/centers_all.json`

The continuation is **not** allowed to reuse stale normalized scene outputs or
stale `pre_c1` snapshots.

Before running Phase 2, enforce all of the following:

- `GRScenes-test0-normalized/GRScenes100` must not contain prior scene outputs
  from an earlier Phase 2 run
- no normalized scene directory may already contain `layout.pre_c1_*.usd`
- if either condition is false, stop and isolate the scene tree before
  continuing

Reason:

- `audit_normalize_phase2.py` selects the earliest `layout.pre_c1_*` file in
  each normalized scene directory
- stale snapshots can silently poison the audit even when the new Phase 2 run
  itself is correct

# Exact Execution Order

## 1. Set Variables

```bash
set -euo pipefail

REPO_ROOT=/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep
SRC_ROOT=$REPO_ROOT/GRScenes-test0
OUT_ROOT=$REPO_ROOT/GRScenes-test0-normalized
PHASE1_RUN_ROOT=$REPO_ROOT/check_reports/test0_full/20260313_023200

CONT_RUN_ID=${CONT_RUN_ID:-20260314_phase2_probe_from_20260313_023200}
CONT_REPORT_ROOT=$REPO_ROOT/check_reports/test0_full/$CONT_RUN_ID
CONT_NORMALIZE_ROOT=$CONT_REPORT_ROOT/normalize
CONT_PHASE2_ROOT=$CONT_NORMALIZE_ROOT/phase2
CONT_SUMMARY_ROOT=$CONT_REPORT_ROOT/summary

PHASE1_CENTERS_DIR=$PHASE1_RUN_ROOT/normalize/phase1
PHASE1_CENTERS_JSON=$PHASE1_CENTERS_DIR/centers_all.json
PRE_C1_REPORT=$CONT_NORMALIZE_ROOT/pre_c1_snapshot_report.json
PAIRWISE_JSON=$CONT_NORMALIZE_ROOT/test0_vs_normalized_pre_dedup.json
AUDIT_JSON=$CONT_NORMALIZE_ROOT/test0_normalize_phase2_audit.json
```

## 2. Preflight Input Checks

```bash
cd "$REPO_ROOT"

test -d "$SRC_ROOT/GRScenes_assets"
test -d "$SRC_ROOT/GRScenes100"
test -d "$OUT_ROOT/GRScenes_assets"
test -f "$PHASE1_CENTERS_JSON"
test ! -e "$CONT_REPORT_ROOT"
```

Inventory sanity should still match the frozen `test0` counts:

- `114` categories
- `99` scenes total
- `69` home
- `30` commercial

## 3. Scene-Tree Freshness Check

This continuation assumes the normalized asset tree exists but the normalized
scene tree is fresh for this run.

Enforce:

```bash
test ! -e "$OUT_ROOT/GRScenes100"
```

If this fails, stop. Do not continue until the stale normalized scene tree is
isolated or removed in a controlled way.

## 4. Create Continuation Report Root

```bash
mkdir -p "$CONT_PHASE2_ROOT" "$CONT_SUMMARY_ROOT"
```

## 5. Run Phase 2

```bash
./scripts/isaac_python.sh scripts/normalize_asset_transforms.py \
  --assets-root "$SRC_ROOT/GRScenes_assets" \
  --scenes-root "$SRC_ROOT/GRScenes100" \
  --output-root "$OUT_ROOT" \
  --phase 2 \
  --centers-dir "$PHASE1_CENTERS_DIR" \
  --report-dir "$CONT_PHASE2_ROOT"
```

Phase 2 success requirements:

- exit code `0`
- report exists at:
  `"$CONT_PHASE2_ROOT/normalize_report.json"`

Immediate Phase 2 report checks:

- `meta.scenes_processed == 99`
- `meta.errors_count == 0`

If either check fails, stop and do not enter pairwise or audit.

## 6. Create Synthetic Pre-C1 Snapshots

```bash
python3 scripts/create_pre_c1_normalize_only_snapshots.py \
  --normalized-root "$OUT_ROOT" \
  --run-id "$CONT_RUN_ID" \
  --report-path "$PRE_C1_REPORT"
```

Snapshot success requirements:

- report exists at:
  `"$PRE_C1_REPORT"`
- `scene_count == 99`
- `missing_layout_count == 0`
- no snapshot conflicts

## 7. Run Pairwise Placement Compare

```bash
./scripts/isaac_python.sh scripts/placement_pairwise_compare.py \
  --left-root "$SRC_ROOT" \
  --right-root "$OUT_ROOT" \
  --left-mode current \
  --right-mode current \
  --label "test0-vs-normalized-pre-dedup-$CONT_RUN_ID" \
  --workers 8 \
  --out "$PAIRWISE_JSON"
```

Do not pass backup locator arguments here.

## 8. Run Phase 2 Audit

```bash
./scripts/isaac_python.sh scripts/audit_normalize_phase2.py \
  --source-root "$SRC_ROOT" \
  --normalized-root "$OUT_ROOT" \
  --centers-dir "$PHASE1_CENTERS_DIR" \
  --workers 8 \
  --out "$AUDIT_JSON"
```

## 9. Build Hard-Gate Summary

Always pass the explicit pairwise and audit report paths. Do not rely on
pattern inference.

```bash
python3 scripts/assemble_normalize_gate_bundle.py \
  --summary-dir "$CONT_SUMMARY_ROOT" \
  --run-id "$CONT_RUN_ID" \
  --pairwise-report "$PAIRWISE_JSON" \
  --audit-report "$AUDIT_JSON" \
  --python "$(command -v python3)"
```

Expected outputs:

- `"$CONT_SUMMARY_ROOT/normalize_gate_verdict.json"`
- `"$CONT_SUMMARY_ROOT/final_verdict.json"`
- `"$CONT_SUMMARY_ROOT/summary.md"`

# Hard Gate Definition

Normalize-only hard gate passes only if all of the following are true:

- pairwise `aggregate.displaced_breakdown.gt_0.01 == 0`
- pairwise `aggregate.ref_same_breakdown.gt_0.01 == 0`
- audit `aggregate.totals.center_found == aggregate.totals.common_ref_prim_count`
- audit `aggregate.totals.matrix_mismatch == 0`
- audit `missing_pre_c1_scenes == []`
- audit
  `aggregate.totals.pre_c1_inert_descendant_xform_override_count == 0`
- audit
  `aggregate.totals.became_inert_descendant_xform_override_count == 0`

Status naming to expect:

- `check_normalize_gate_from_reports.py` writes:
  `pass_normalize_gate` or `fail_normalize_gate`
- `build_normalize_gate_verdict.py` writes:
  `pass_normalize_s1` or `fail_normalize_s1`

Do not assume these status strings are identical.

# Stop Conditions

Stop immediately if any of the following happens:

- canonical Phase 1 centers file is missing
- normalized asset root is missing
- normalized scene tree already exists before this continuation begins
- Phase 2 exits nonzero
- Phase 2 report records `errors_count != 0`
- snapshot creation reports conflicts or missing layouts
- pairwise or audit command fails to produce output JSON
- hard gate is not fully green

# Decision Rules

## Pass

If the hard gate is fully green:

- accept the current canonical Phase 1 output as sufficient
- archive the continuation summary as the normalize-only completion record
- stop and hand off into dedup planning or execution in a separate chain

## Fail

If the hard gate fails:

1. inspect the failing scenes and prims
2. test for intersection with the known Phase 1 bad-asset set
3. only then classify the failure as either:
   - source-bad-asset impact
   - true normalize Phase 2 compensation regression

Do not start dedup after a failed normalize-only hard gate.

# Deliverables For The Next Dialogue

The execution dialogue should finish this document’s scope only when it has
produced all of the following:

- `normalize/phase2/normalize_report.json`
- `normalize/pre_c1_snapshot_report.json`
- `normalize/test0_vs_normalized_pre_dedup.json`
- `normalize/test0_normalize_phase2_audit.json`
- `summary/normalize_gate_verdict.json`
- `summary/final_verdict.json`
- `summary/summary.md`

# Related Documents

- status and rationale:
  `docs/test0_full/grscenes_test0_phase1_status_and_phase2_probe_plan_20260314.md`
- original full normalize-only runbook:
  `docs/test0_full/grscenes_test0_normalize_only_runbook.md`
