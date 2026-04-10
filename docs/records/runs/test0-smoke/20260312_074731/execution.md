---
title: test0 smoke s1 descendant override remap execution draft
created_at: 2026-03-12
updated_at: 2026-03-12
maintainer: codex
status: completed
---

# Purpose

This document records the actual execution path used to validate the
descendant override remap repair for `test0 smoke / S1`.

Scope for this execution cycle:

- keep the run in `normalize-only`
- do not enter dedup
- validate the new descendant override remap / compensation in an isolated
  `phase2-only` workspace
- write the resulting reports back into the repo

# Preconditions

- Repository root is available at:
  - `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep`
- Source subset exists:
  - `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1`
- Normalized output root for this cycle is defined:
  - `/tmp/test0_desc_override_fix_phase2_v3/out`
- Report root for this cycle is defined:
  - `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3`
- Current gate conclusion before repair is archived:
  - [normalize_gate_verdict_v2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/summary/normalize_gate_verdict_v2.json)

# Inputs

## Scene Scope

- Primary scene:
  - `home/MV7J6NIKTKJZ2AABAAAAADA8_usd`
- Primary failing prim:
  - `/Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0`
- Control prim:
  - `/Root/Meshes/Animation/nightstand/model_627f5d31c8996749a1f33bfef27562f3_0`

## Reference Reports

- Pairwise normalize-only report:
  - [s1_test0_vs_normalized_pre_dedup.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/s1_test0_vs_normalized_pre_dedup.json)
- Phase2 audit report:
  - [s1_normalize_phase2_audit_v2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/s1_normalize_phase2_audit_v2.json)
- Gate verdict report:
  - [normalize_gate_verdict_v2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/summary/normalize_gate_verdict_v2.json)
- Descendant override investigation report:
  - [orig_scene_ref_descendant_xform_overrides.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/investigation/orig_scene_ref_descendant_xform_overrides.json)

# Execution Steps

## 1. Freeze Starting Point

- Record current branch / worktree status:
  - command: `git status --short`
  - note: worktree was already dirty; this cycle only touched
    [normalize_asset_transforms.py](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/normalize_asset_transforms.py)
    and docs under
    [docs/records/runs/test0-smoke/20260312_074731](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/docs/records/runs/test0-smoke/20260312_074731)
- Record baseline gate inputs:
  - pairwise:
    [s1_test0_vs_normalized_pre_dedup.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/s1_test0_vs_normalized_pre_dedup.json)
  - audit:
    [s1_normalize_phase2_audit_v2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/s1_normalize_phase2_audit_v2.json)
  - verdict:
    [normalize_gate_verdict_v2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/summary/normalize_gate_verdict_v2.json)

## 2. Prepare Repair Branch / Workspace

- Confirm the exact files intended for code changes:
  - [normalize_asset_transforms.py](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/normalize_asset_transforms.py)
- Confirm the exact files intended for documentation and reports:
  - [execution.md](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/docs/records/runs/test0-smoke/20260312_074731/execution.md)
  - [test.md](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/docs/records/runs/test0-smoke/20260312_074731/test.md)
  - [summary.md](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/docs/records/runs/test0-smoke/20260312_074731/summary.md)
  - [remap_fix_phase2_v3](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3)

## 3. Prepare Isolated Phase2-Only Output

- Create a fresh temp output root without mutating the canonical normalized set.
- Reuse existing normalized assets by symlinking:
  `/tmp/test0_desc_override_fix_phase2_v3/out/GRScenes_assets ->
  /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1-normalized/GRScenes_assets`
- Command used:

```bash
set -e
TMP_ROOT=/tmp/test0_desc_override_fix_phase2_v3
OUT_ROOT="$TMP_ROOT/out"
REPORT_ROOT="$TMP_ROOT/reports"
mkdir -p "$OUT_ROOT" "$REPORT_ROOT"
if [ ! -e "$OUT_ROOT/GRScenes_assets" ]; then
  ln -s /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1-normalized/GRScenes_assets "$OUT_ROOT/GRScenes_assets"
fi
```

## 4. Execute Phase2-Only Validation

- Run the repaired `phase2` implementation against the original `test0 smoke`
  scene root while reusing the previously generated `phase1` centers.
- Command used:

```bash
./scripts/isaac_python.sh scripts/normalize_asset_transforms.py \
  --assets-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1/GRScenes_assets \
  --scenes-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1/GRScenes100 \
  --output-root /tmp/test0_desc_override_fix_phase2_v3/out \
  --phase 2 \
  --centers-dir /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/phase1/centers_merged \
  --report-dir /tmp/test0_desc_override_fix_phase2_v3/reports/normalize_phase2
```

- Result:
  - `1105` scene prims compensated
  - `1` descendant override remapped
  - `0` errors

- Normalization report:
  [normalize_report.json](/tmp/test0_desc_override_fix_phase2_v3/reports/normalize_phase2/normalize_report.json)

## 5. Prepare Audit-Compatible Pre-C1 Snapshot

- Run syntax or import validation:
  - first attempt used a copied source layout as `layout.pre_c1_*` and made
    `matrix_mismatch = 1105`
  - root cause: `audit_normalize_phase2.py` expects `pre_c1` to be the
    post-phase2, pre-downstream snapshot under the normalized root

- Corrective step:

```bash
cp /tmp/test0_desc_override_fix_phase2_v3/out/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd \
   /tmp/test0_desc_override_fix_phase2_v3/out/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.pre_c1_20260312_000000.usd
```

## 6. Static Validation

- Syntax / import validation command:

```bash
python3 -m py_compile \
  scripts/normalize_asset_transforms.py \
  scripts/audit_normalize_phase2.py \
  scripts/check_normalize_gate_from_reports.py \
  scripts/build_normalize_gate_verdict.py
```

- Result:
  - command exited `0`
  - no syntax errors

## 7. Focused Runtime Validation

- Refrigerator geometry diagnostic:

```bash
./scripts/isaac_python.sh scripts/debug_scene_prim_geometry_delta.py \
  --left-stage /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd \
  --right-stage /tmp/test0_desc_override_fix_phase2_v3/out/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd \
  --prim-path /Root/Meshes/Animation/refrigerator/model_8958330cdaa9b7dcda067c99f5916ca3_0 \
  --out /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/refrigerator_geometry_delta_fixed.json
```

- Nightstand control diagnostic:

```bash
./scripts/isaac_python.sh scripts/debug_scene_prim_geometry_delta.py \
  --left-stage /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd \
  --right-stage /tmp/test0_desc_override_fix_phase2_v3/out/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd \
  --prim-path /Root/Meshes/Animation/nightstand/model_627f5d31c8996749a1f33bfef27562f3_0 \
  --out /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/nightstand_geometry_delta_fixed.json
```

- Descendant override audits:

```bash
./scripts/isaac_python.sh scripts/audit_ref_descendant_xform_overrides.py \
  --scene /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd \
  --out /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/source_descendant_override_audit.json

./scripts/isaac_python.sh scripts/audit_ref_descendant_xform_overrides.py \
  --scene /tmp/test0_desc_override_fix_phase2_v3/out/GRScenes100/home/MV7J6NIKTKJZ2AABAAAAADA8_usd/layout.usd \
  --out /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/fixed_descendant_override_audit.json
```

- Pairwise comparison:

```bash
./scripts/isaac_python.sh scripts/placement_pairwise_compare.py \
  --left-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1 \
  --right-root /tmp/test0_desc_override_fix_phase2_v3/out \
  --left-mode current \
  --right-mode current \
  --label test0_desc_override_fix_phase2_v3 \
  --scene-filter MV7J6NIKTKJZ2AABAAAAADA8_usd \
  --workers 1 \
  --out /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/pairwise_compare.json
```

- Phase2 audit:

```bash
./scripts/isaac_python.sh scripts/audit_normalize_phase2.py \
  --source-root /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-smoke-S1 \
  --normalized-root /tmp/test0_desc_override_fix_phase2_v3/out \
  --centers-dir /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/phase1/centers_merged \
  --scene-filter MV7J6NIKTKJZ2AABAAAAADA8_usd \
  --workers 1 \
  --out /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/audit_phase2.json
```

- Gate verdict:

```bash
python3 scripts/check_normalize_gate_from_reports.py \
  --pairwise-report /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/pairwise_compare.json \
  --audit-report /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/audit_phase2.json \
  --out /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/normalize_gate_verdict.json
```

## 8. Compare Before / After

- Refrigerator:
  - before:
    [refrigerator_geometry_delta_v2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/refrigerator_geometry_delta_v2.json)
  - after:
    [refrigerator_geometry_delta_fixed.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/refrigerator_geometry_delta_fixed.json)
- Control:
  - before:
    [nightstand_geometry_delta_v2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/normalize/nightstand_geometry_delta_v2.json)
  - after:
    [nightstand_geometry_delta_fixed.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/nightstand_geometry_delta_fixed.json)
- Gate:
  - before:
    [normalize_gate_verdict_v2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/summary/normalize_gate_verdict_v2.json)
  - after:
    [normalize_gate_verdict.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/normalize_gate_verdict.json)

## 9. Decide Stop / Continue

- This isolated `phase2-only` validation passed.
- Do not start dedup from this temp output root.
- The next approved execution step should be to rerun the official `S1`
  normalize-only output with the same code path and regenerate canonical
  reports under the standard output root.

# Artifacts To Produce

- Updated normalize report:
  - [normalize_report.json](/tmp/test0_desc_override_fix_phase2_v3/reports/normalize_phase2/normalize_report.json)
- Updated pairwise report:
  - [pairwise_compare.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/pairwise_compare.json)
- Updated phase2 audit report:
  - [audit_phase2.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/audit_phase2.json)
- Updated gate verdict:
  - [normalize_gate_verdict.json](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_smoke/20260312_074731/remap_fix_phase2_v3/normalize_gate_verdict.json)
- Execution notes:
  - [execution.md](/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/docs/records/runs/test0-smoke/20260312_074731/execution.md)

# Outcome

- `phase2-only` validation closed the refrigerator regression.
- One scene-authored descendant override was detected and remapped.
- Pairwise, phase2 audit, and unified gate all passed after the audit baseline
  was corrected to use the post-phase2 `pre_c1` snapshot.
