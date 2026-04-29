---
title: C1 Parallel Acceleration Pipeline Design
code_reference:
  - scripts/c1_autorun_categories.py
  - scripts/c1_bulk_apply_layout_dedup.py
  - scripts/c1_bulk_step6_category_promote_scan_soft_delete.py
  - scripts/c1_build_bulk_mapping_from_dedup_report.py
  - scripts/rewrite_layout_asset_refs_with_compensation.py
  - scripts/placement_pairwise_compare.py
created_at: 2026-04-29
updated_at: 2026-04-29
maintainer: OpenCode
status: reviewed, v3
doc_class: record
---

# C1 Parallel Acceleration Pipeline Design

## Summary

Current C1 promoted-clone autorun processes 54+ categories **serially**, each category
taking ~4 hours on DLC, for a total wall time of ~9 days. The two primary serial
bottlenecks are:

1. **Step 2 (Apply)**: Each category reads the latest cumulative `layout.usd` and
   produces a sidecar — parallel sidecars from the same baseline would miss each
   other's changes.
2. **Step 6 (Scan)**: Each category independently scans all ~77,000 USD files for
   its own old asset references — 54× redundant file traversal.

This design introduces a **two-phase parallel pipeline** that reduces wall time
from ~9 days to ~6 hours (~30× speedup), by running per-category apply+audit in
parallel (Phase 1), then merging all sidecar changes and running a single batched
scan (Phase 2).

## Key Insight: Category Asset Disjointness

Analysis of the dedup report (`all_categories_union_merged.json`, 7,062 duplicate
groups) confirms:

- **No cross-category duplicate groups exist.** Every old→canonical pair maps
  assets within the same category (`GRScenes_assets/<category>/`).
- **Prim modifications are category-exclusive.** When category "book" rewrites
  asset references in `layout.usd`, it only touches prims that reference book
  assets. Category "chair" touches a disjoint set of prims.
- **Therefore:** sidecars produced independently from the same baseline can be
  safely merged by re-applying all category mappings in a single pass, with no
  conflicts.

## Architecture

```
                         dedup_report.json
                              │
              ┌───────────────┼───────────────┐
              ▼               ▼               ▼
         ┌─────────┐    ┌─────────┐     ┌─────────┐
         │ book    │    │ chair   │ ... │ window  │   Phase 1 (parallel)
         │ Apply   │    │ Apply   │     │ Apply   │   54 DLC jobs
         │ Audit   │    │ Audit   │     │ Audit   │   each ~1-2h
         └────┬────┘    └────┬────┘     └────┬────┘
              │               │               │
              ▼               ▼               ▼
    layout.book_v1.usd  layout.chair_v1.usd  ...
    audit_verdict.json  audit_verdict.json
    phase1_done.json    phase1_done.json
              │               │               │
              └───────────────┼───────────────┘
                              │
                    ┌─────────▼─────────┐
                    │    Phase 2 Gate    │
                    │  all phase1_done?  │
                    └─────────┬─────────┘
                              │
                    ┌─────────▼─────────┐
                    │     Phase 2        │ 1 DLC job, ~4.5h
                    │  Merge             │
                    │  Mega-Scan         │
                    │  Mega-Soft-Delete  │
                    └────────────────────┘
```

## Phase 1: Parallel Apply + Audit

### Contract

Each DLC job processes exactly **one category**:

1. **Build Mapping**: Extract this category's pairs from the unified dedup report.
2. **Apply (Step 2)**: Read **baseline** scene files (`layout.usd`,
   `start_result_interaction.usd`, `start_result_navigation.usd`), rewrite only
   this category's asset references in all three, write sidecars:
   - `layout.parallel_<group_label>.<cat>_<policy>_v1.usd`
   - `start_result_interaction.parallel_<group_label>.<cat>_<policy>_v1.usd`
   - `start_result_navigation.parallel_<group_label>.<cat>_<policy>_v1.usd`
3. **Audit (Step 3)**: Run placement pairwise comparison (sidecar vs baseline) to
   verify bbox/position/angle correctness.
4. **Write completion marker**: `<c1_bulk>/<cat>_<policy>_v1/phase1_done.json`.

### Key Properties

- All Phase 1 jobs read the **same baseline** `layout.usd` — no cumulative state.
- Each sidecar filename is category-specific → no write conflicts.
- Each `c1_bulk/<cat>_v1/` subdirectory is category-specific → no write conflicts.
- Audit is fully independent per category (compares sidecar vs baseline, not vs
  other categories).

### Implementation

New entry-point script: `scripts/c1_phase1_apply_and_audit.py`

```
Usage: c1_phase1_apply_and_audit.py --category <cat> --dataset-root <path>
       --bak-root <path> --report <path> --c1-bulk-dir <path>
       [--bbox-gated] [--bbox-policy <policy>] [--dedup-mode <mode>]
       [--group-label <label>] [--out-version <v>]
```

This script:
- Reuses `c1_build_bulk_mapping_from_dedup_report.py` logic for Step 1.
- Reuses `c1_bulk_apply_layout_dedup.py` logic for Step 2.
- Reuses `placement_pairwise_compare.py` logic for Step 3.
- Writes `phase1_done.json` on completion with status, elapsed time, and error
  details if any.

## Phase 2: Merge + Mega-Scan + Soft-Delete

### Contract

A single DLC job executes after all Phase 1 jobs have succeeded:

1. **Gate Check**: Verify all 54 `phase1_done.json` files exist with `status=ok`.
   Abort if any missing or failed.
2. **Merge**: Build combined mapping from all category `filtered_mapping.json`
   files, then re-apply rewrites to each scene's baseline files: `layout.usd`,
   `start_result_interaction.usd`, and `start_result_navigation.usd`.
3. **Mega-Scan**: One pass over all ~77,000 USD files using **pxr.Usd.Stage.Open()**
   (NOT byte/substring matching — required for `.usdc` binary files), checking for
   references to **any** old asset path from **any** category. Uses full
   `GRScenes_assets/<cat>/<uid>/usd/<uid>.usd` path matching to avoid UID collision
   false positives.
4. **Mega-Soft-Delete**: Move all old asset directories to
   `bak/_dedup_assets/<stamp>/` in one batch.
5. **Post-Delete Scan**: Scan all `layout.usd` and `start_result_*.usd` files to
   verify no stale references remain.

### Merge Algorithm

```
scene_files = ["layout.usd", "start_result_interaction.usd",
               "start_result_navigation.usd"]
combined_mapping = {}
conflict_log = []

for category in all_categories:
    cat_mapping = load_filtered_mapping(category)
    for old_key, new_val in cat_mapping.items():
        if old_key in combined_mapping:
            if combined_mapping[old_key] == new_val:
                continue  # benign same-target overlap
            conflict_log.append({
                "old_key": old_key,
                "existing": {"category": combined_src[old_key],
                             "target": combined_mapping[old_key]},
                "conflicting": {"category": category, "target": new_val}
            })
        else:
            combined_mapping[old_key] = new_val
            combined_src[old_key] = category

if conflict_log:
    # Tier 1: Log all conflicts with full context
    # Tier 2: Fall back to sequential re-apply per category
    # Tier 3: If sequential also fails, write conflict report and abort

for scene_dir in GRScenes100/**:
    for scene_file in scene_files:
        baseline_path = scene_dir / scene_file
        backup_path = baseline_path.with_suffix(".baseline.usd")
        cp baseline_path → backup_path  # preserve for rollback
        rewrite_layout(baseline_path, combined_mapping,
                       output=baseline_path)
```

**Correctness argument**: Since each category's mapping pairs operate on disjoint
old asset UIDs (category-exclusive), applying all mappings in a single pass
produces the same result as sequential cumulative application. The conflict guard
catches any violation of this invariant with structured diagnostics instead of
hard-crashing.

**Fallback**: If conflict guard fires, fall back to sequential re-apply (copy
baseline, apply cat1, save, apply cat2, save, ...). This is slower (~50 min
instead of ~30 min) but handles any edge case.

### Multi-Ref Prim Edge Case

If a single prim references assets from two different categories that are BOTH in
their respective dedup mappings (e.g., both `book` and `chair` old assets), the
sequential and merged approaches diverge:

- **Sequential**: Cat1 rewrites one ref → Cat2 rewrites the other → cumulative
  compensation applied.
- **Merged (combined mapping)**: The rewrite rejects multi-ref-changed prims (no
  compensation applied).

This scenario requires a single prim to reference assets from multiple deduped
categories — rare in GRScenes layouts (each prim typically has one reference to
its instance asset). Verification: scan a sample of layout files with the combined
mapping to confirm no prim has >1 matching old reference before merge.

### Mega-Scan Algorithm

Uses **pxr.Usd.Stage.Open()** (not byte/substring matching) — required because
`.usdc` files are binary and do not contain full asset paths as contiguous byte
strings. Also uses full `GRScenes_assets/<cat>/<uid>/usd/<uid>.usd` paths (not
UID-only) to avoid false positives from hash collisions.

```
old_asset_path_set = union of all old asset paths from all category mappings
                     (full relative paths like
                      "GRScenes_assets/chair/abc123/usd/abc123.usd")

for each USD file in dataset (recursive, ~77,000 files):
    # EXCLUDE backup and intermediate artifacts:
    #   - files containing ".pre_" in name (intentional old-ref backups)
    #   - files containing ".parallel_" in name (Phase 1 sidecars)
    #   - files in bak/_dedup_assets/ directory
    if file matches exclusion pattern:
        continue

    stage = Usd.Stage.Open(file)
    for prim in stage.TraverseAll():
        for ref in get_asset_references(prim):
            rel_path = resolve_to_relative(ref)
            if rel_path in old_asset_path_set:
                record hit(file, rel_path)

if hits > 0:
    report hit files with matched old asset paths
    exit with error
```

Performance: The set lookup is O(1) regardless of set size (500 or 50,000).
File traversal is the bottleneck — ~77,000 files × ~4 prims/file ≈ 300,000
prim traversals. Estimated wall time: ~20-30 minutes (vs. 54 × 20 min for
per-category scans).

### Implementation

New entry-point script: `scripts/c1_phase2_merge_scan_delete.py`

```
Usage: c1_phase2_merge_scan_delete.py --dataset-root <path> --bak-root <path>
       --c1-bulk-dir <path> [--bbox-policy <policy>] [--out-version <v>]
       [--group-label <label>] [--categories-file <path>]
```

## Verification Strategy

| Stage | Check | Pass Criterion | Failure Action |
|-------|-------|---------------|----------------|
| Phase 1 per-cat | Bbox/position audit | `audit_verdict.passed=true` | Retry single category job |
| Phase 1 gate | All 54 jobs done | All `phase1_done.json` status=ok | Wait/retry failed jobs |
| Phase 2 merge | No overlapping UIDs | Conflict guard passes | Hard error (requires manual investigation) |
| Phase 2 mega-scan | No old UID references | `hit_files == 0` | Fall back to per-category isolation scans |
| Phase 2 soft-delete | All old assets moved | Per-asset success | Log failures; continue with remaining |
| Phase 2 post-delete | No stale layout refs | `hit_layouts == 0` | Report residual file paths |

### Lightweight Pre-Check (before mega-scan)

Before the full 77,000-file scan, run a fast check on only the scene files:
- ~100 `layout.usd` + ~100 `start_result_interaction.usd` + ~100 `start_result_navigation.usd`
  (~300 files, ~30 seconds). Skip `.pre_` backups and `.parallel_` sidecars.
This catches the vast majority of issues early and provides per-scene/category
granularity if failures occur.

## Error Handling

| Scenario | Strategy |
|----------|----------|
| Phase 1 job fails | Retry up to 3×; category jobs are idempotent |
| Phase 1 job timeout | Set generous timeout (e.g., 4h); large categories may need more |
| Gate check fails | Do not start Phase 2; report which categories still pending |
| Merge conflict guard fires | Hard stop; inspect reports; manual resolution (expected: never) |
| Mega-scan finds hits | Auto-fallback: run per-category isolation scans to identify culprit(s) |
| Soft-delete fails for some assets | Log failed paths; continue; report at end |
| Phase 2 job killed mid-run | Restart from merge step (merge and scan are idempotent with fresh output) |

## Isolation Guarantees

| Concern | Mitigation |
|---------|------------|
| Existing DLC job `dlcoh2xvu5efzkyo` | New jobs use distinct name prefix `test0_parallel_phase1_*`; never touch the existing workspace's autorun state |
| Existing serial autorun | No code changes to `c1_autorun_categories.py` that affect serial path; new scripts are additive |
| Development isolation | All work done in a git worktree on a feature branch; `main` branch untouched until merge |
| Workspace integrity | Phase 2 saves baseline as `layout.baseline.usd` before merge; restore possible |
| Concurrent Phase 1 writes | Each category writes to disjoint subdirectories (`c1_bulk/<cat>_v1/`) and disjoint sidecar filenames (`layout.parallel_<label>.<cat>_v1.usd`) |

## DLC Orchestration

Following the existing pattern from `scripts/orchestrate_test0_rebuilt_normalize.py`:

### Orchestrator Script: `scripts/orchestrate_c1_parallel.py`

| Command | Action |
|---------|--------|
| `submit-phase1` | Discover remaining categories (from `GRScenes_assets/` minus completed ones validated via `phase1_done.json`), pre-validate all paths, then submit one DLC job per category. Writes `run_manifest.json` with job names. |
| `resume` / `gate-check` | Poll DLC API for all Phase 1 job statuses. Cross-check each job's `phase1_done.json` for logical success (not just DLC Running→Succeeded). If all pass → auto-submit Phase 2. If any fail → report failures and stop. |
| `status` | Print table of all Phase 1 job statuses + Phase 2 status. |

### Gate Logic (Phase 1 → Phase 2)

Two-tier verification per job:
1. **DLC status** = `Succeeded` (via `./dlc get job <id> -w <workspace>`)
2. **Logical success** = `phase1_done.json.status == "ok" AND audit_verdict.passed == true`

Both must pass for the gate to open. This catches the case where DLC reports
success but the audit failed (e.g., bbox errors above threshold).

### Pre-submission Validation

Before submitting ANY DLC jobs:
1. Verify all 4 data source mounts are correct (especially `d-f1dsz5nbamclxgydo8` for `/cpfs/user/zhuzihou/`)
2. Verify all absolute paths exist and are accessible
3. Verify dedup report is parseable and has the expected category list
4. Run mock submit (`DLC_BIN=echo`) to verify command resolution
5. Optionally: submit a single smoke-test job (fast category like `bathtub`, mapping_pairs_0)
   to verify the end-to-end path

| Phase | Jobs | GPU/Job | CPU/Job | Memory/Job | Timeout | Timing |
|-------|------|---------|---------|------------|---------|--------|
| Phase 1 | 54 (one per category) | 1 | 14 | 100Gi | 4h | ~1-2h (wall, parallel) |
| Phase 2 | 1 | 1 | 14 | 100Gi | 8h | ~4.5h |
| **Total** | **55** | — | — | — | — | **~6h wall** |

Timeout rationale:
- Phase 1: 4h = estimated 1-2h × 2 safety margin. Largest categories (`other`:
  19,119 groups, `book`: 12,827 groups) may be closer to upper bound.
- Phase 2: 8h = estimated 4.5h × 1.8 safety margin. Merge is fast (~30 min),
  mega-scan dominates (~3.5h), soft-delete is fast (~30 min).
- Implementation: `launch_job.sh` must be updated to support `DLC_JOB_TIMEOUT`
  (currently hardcoded to `--job_max_running_time_minutes=0`). Change default to
  `${DLC_JOB_TIMEOUT:-0}` for backward compatibility.

### Phase 1 Job Command Template

```bash
bash scripts/dlc/launch_job.sh \
  test0_parallel_phase1_<category> 0 1 \
  d-mzps5b7joy2axmqpa8,d-d49o5g0h2818sw8j1g,d-8wz4emfs21s5ajs9oz,d-f1dsz5nbamclxgydo8 \
  "custom /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/c1_phase1_apply_and_audit.py \
    --category <category> \
    --dataset-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/dataset \
    --bak-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/bak \
    --report /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/check_reports/test0_rebuilt_dedup/v8_prededup/union_3way/all_categories_union_merged.json \
    --c1-bulk-dir /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/c1_bulk \
    --group-label test0_transitive_apply_seeded \
    --bbox-gated --bbox-policy bbox_primary_rmse_observe \
    --dedup-mode geom_only --v-matrix-mode auto \
    --out-version v1"
```

### Phase 2 Job Command Template

```bash
bash scripts/dlc/launch_job.sh \
  test0_parallel_phase2_merge 0 1 \
  d-mzps5b7joy2axmqpa8,d-d49o5g0h2818sw8j1g,d-8wz4emfs21s5ajs9oz,d-f1dsz5nbamclxgydo8 \
  "custom /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/scripts/c1_phase2_merge_scan_delete.py \
    --dataset-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/dataset \
    --bak-root /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/bak \
    --c1-bulk-dir /cpfs/user/zhuzihou/assets/dedup_workspaces/test0_transitive_apply_20260421_103046/c1_bulk \
    --bbox-policy bbox_primary_rmse_observe \
    --out-version v1 \
    --group-label test0_transitive_apply_seeded"
```

## Files to Create

| File | Purpose |
|------|---------|
| `scripts/c1_phase1_apply_and_audit.py` | Phase 1 per-category apply + audit entry point |
| `scripts/c1_phase2_merge_scan_delete.py` | Phase 2 merge + mega-scan + soft-delete |
| `scripts/scan_utils.py` | Extracted shared scan functions (`_scan_stage_for_old_assets`, `_scan_tree_pxr`) from `c1_bulk_step6_category_promote_scan_soft_delete.py` for reuse by Phase 2 |
| `scripts/orchestrate_c1_parallel.py` | DLC orchestration (submit-phase1, gate-check, submit-phase2, status) following `orchestrate_test0_rebuilt_normalize.py` pattern |
| `tests/test_c1_parallel_merge.py` | Tests for merge logic, gate check, scan exclusion |
| `tests/test_c1_phase1_apply_and_audit.py` | Tests for Phase 1 script |
| `tests/test_c1_phase2_merge_scan_delete.py` | Tests for Phase 2 merge + scan logic |

## Files to Modify

| File | Change |
|------|--------|
| `scripts/dlc/launch_job.sh` | Support `DLC_JOB_TIMEOUT` env var (currently hardcoded `--job_max_running_time_minutes=0`) |
| `scripts/c1_bulk_step6_category_promote_scan_soft_delete.py` | Extract `_scan_stage_for_old_assets` and `_scan_tree_pxr` into `scripts/scan_utils.py`; keep existing script as thin CLI wrapper |
| `scripts/c1_autorun_categories.py` | No changes to serial path; may optionally extract `_is_step6_complete()` and `_is_bbox_done()` into a shared library if Phase 1 script wants to reuse them |

## Risk Assessment

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|------------|
| Cross-category UID conflict (contrary to analysis) | Very Low | High | Conflict guard with structured error + auto-fallback to sequential re-apply |
| Multi-ref prims (prim references assets from 2+ deduped categories) | Very Low | Medium | Pre-merge scan validates no multi-ref prims exist; if found, manual review before merge |
| Phase 1 job fails due to large category | Low | Medium | Idempotent retry up to 3×; 4h timeout; largest categories known from history |
| Mega-scan false positives (`.pre_` backup files, `.parallel_` sidecars) | Medium | Medium | File exclusion rules prevent scanning backup/sidecar files; only scan live scene + asset files |
| Mega-scan false negatives (.usdc binary files missed by substring matching) | High | High | **Mitigated**: design now requires pxr.Usd.Stage.Open() based scanning, not byte matching |
| Merge re-apply diverges from sidecars | Low | High | Per-scene spot-check: compare merged prims vs sidecar prims for 5 randomly sampled scenes |
| 54 concurrent DLC jobs exceed quota | Low | Medium | Historical precedent: 79 concurrent per-category jobs ran successfully |
| Currently running serial job `dlcoh2xvu5efzkyo` conflict | Very Low | High | Phase 1 only submitted AFTER serial job completes or is stopped; explicit gate in design |
| `start_result_*.usd` rewrite gaps (only `layout.usd` was in original design) | N/A | N/A | **Fixed in review**: Phase 1 now rewrites all 3 scene files; Phase 2 merge handles all 3 |
| DLC timeout not set (jobs hang forever) | Medium | High | **Fixed**: 4h Phase 1 timeout, 8h Phase 2 timeout; `launch_job.sh` updated to support `DLC_JOB_TIMEOUT` |

## Precondition: Synchronization with Serial DLC

**CRITICAL**: Phase 1 jobs must NOT run concurrently with the existing serial DLC
job `dlcoh2xvu5efzkyo`. Both read and write to the same workspace
(`dataset/GRScenes100/**/layout.usd`). Concurrent execution would cause race
conditions on baseline `layout.usd` reads.

Phase 1 must be submitted **only after** the serial job has either:
- Completed successfully (all 54 remaining categories done), or
- Failed/been stopped and the workspace is stable (no ongoing writes to `layout.usd`)

If the serial job is still running: wait for it to finish.

## What "Baseline" Means

The "baseline" `layout.usd` is the **current state** of each scene's layout at the
time Phase 1 starts. In the current workspace, categories `bed` through `desk`
have already been applied by previous runs. The baseline INCLUDES these already-applied
categories. Phase 1 only processes the **remaining** 54 categories (those not yet
completed).

## Migration Path from Current Serial Run

The currently running serial DLC job `dlcoh2xvu5efzkyo` completes independently.
After it finishes:

1. **If serial job succeeds**: All categories are done. Parallel pipeline can be
   used for future runs or for re-verification.
2. **If serial job fails mid-way**: The workspace has cumulative results through
   the last completed category. Stop the serial job, then use the existing
   `layout.usd` (with applied categories) as Phase 1 baseline for the remaining
   54 categories.
3. **Both approaches share the same workspace structure**: No migration needed.

The parallel pipeline is designed as an **alternative execution strategy** for the
same C1 workflow, not a replacement that requires workspace conversion.

## Implementation Notes

### CLI Argument Fabrication Bug

During initial implementation of `scripts/c1_phase1_apply_and_audit.py`, the
subprocess command arguments for all three steps (build mapping, bulk apply,
placement audit) were **invented** based on what seemed plausible, rather than
verified against the actual `argparse.ArgumentParser` definitions in the target
scripts. This caused every subprocess call to fail at runtime with
`unrecognized arguments` errors, since the fabricated arg names did not match
anything the target scripts accepted.

**Root cause**: The integration code was written without inspecting the target
scripts' argparse definitions. Arg names like `--layout-root`, `--out-dir`,
`--subset-dirs`, `--mode`, `--output-basename`, `--bak-root`, and
`--no-bbox-gated` were fabricated — none of these exist in the respective
target scripts.

### Fix Approach

The fix referenced `scripts/c1_autorun_categories.py` as the **gold standard**
for correct subprocess command construction. This script already shells out to
the same three target scripts and has been tested through extensive serial runs.
The fix involved:

1. Reading the actual argparse definitions in each target script.
2. Cross-referencing every argument against `c1_autorun_categories.py`'s
   `_run_bbox_gated` and `_build_bbox_audit_cmd` functions (lines 423-420).
3. Replacing all fabricated args with the correct ones.
4. Also correcting the subprocess launcher from `sys.executable` to `ISAAC_PY`
   (Isaac Sim Python wrapper required for pxr imports).

Fixed in commit `0220513`.

### Lesson

**Always verify subprocess command arguments against the target script's
argparse definitions before writing integration code.** When an existing,
battle-tested script (like `c1_autorun_categories.py`) already shells out to
the same targets, use it as the authoritative reference for correct argument
names and values.

### Phase 2 Function Signature Mismatches

The initial Phase 2 implementation plan called the `rewrite_layout()` function
and constructed `MappingPair` objects with fabricated signatures. The actual
`MappingPair` namedtuple (`scripts/rewrite_layout_asset_refs_with_compensation.py`)
uses fields `(old, canonical)` but the plan's `_load_mapping_from_dict()` constructed
them as `MappingPair(old_key=..., canonical_key=...)` — nonexistent field names.
Additionally, `rewrite_layout()` uses keyword-only parameters (`mapping_pairs`,
`apply_compensation`, `set_instanceable`) that were either missing or incorrectly
passed as positional args.

Fixed in commit `b3a0636`:
- Corrected `MappingPair` construction to `MappingPair(old=..., canonical=...)`
- Added missing required args: `--group-label`, `--set-instanceable`, `--v-matrix-mode`
- Corrected `rewrite_layout()` call to use keyword arguments matching the actual signature

### Phase 2 Sequential Merge Dead-End

The initial Phase 2 implementation's `_sequential_merge()` function wrote sidecar
files (with `--out-name`) instead of modifying the scene files in-place. This meant
the sequential merge produced per-category sidecars just like Phase 1, leaving
the baseline scene files untouched — a dead-end.

**Fix** (commit `c2bf569`): Changed `_sequential_merge()` to pass the same file
as both input and output (`--scene-files` with in-place rewrites), copying the
baseline to a `.baseline` backup first (matching the `_combined_merge()` approach).

### Phase 2 Empty Mapping Guard

Some categories have zero dedup pairs (filtered_mapping.json is empty or doesn't
exist). The initial Phase 2 code called `rewrite_layout()` with an empty mapping,
which caused unnecessary no-op rewrites to every scene file. Additionally,
empty mappings passed to `merge_category_mappings()` added no pairs but were
logged as successful.

**Fix** (commit `c2bf569`): Added explicit guard in `discover_category_mappings()`
to skip categories with empty mappings. Added guard in `_combined_merge()` and
`_sequential_merge()` to skip categories with no mapping file.

### Phase 2 V Matrix Compensation Fix

Phase 2 merges all category sidecars by re-applying the combined mapping to the
baseline `layout.usd` (same as Phase 1 apply). The initial implementation used
`v_matrix_mode="none"` as a lazy workaround, which caused visual displacement
because the merge skipped V matrix compensation for deduped asset placements.

**Fix**: Changed `v_matrix_mode` from `"none"` to `"auto"` in both
`_combined_merge()` and `_sequential_merge()`, and added `--mode-reports-dir`
CLI argument to both `c1_phase2_merge_scan_delete.py` and
`orchestrate_c1_parallel.py` so the mode reports directory
(`check_reports/test0_rebuilt_dedup/v8_prededup`) is passed through to the
`rewrite_layout()` call. This allows the V matrix compensation logic to look up
dedup mode information (geom_only vs shape_invariant vs transitive) and compute
the correct transform compensation for each asset pair.

`--certificate-jsonl` is NOT passed for Phase 2, because combining certificates
from all categories is complex. For transitive pairs without certificates,
`rewrite_layout` records an `xform_compensation_error` (the reference is still
rewritten, but without V compensation). Since transitive pairs are rare in
practice, this is an acceptable trade-off.

### Specific Arguments: Wrong vs Correct

#### Step 1 – Build Mapping (`c1_build_bulk_mapping_from_dedup_report.py`)

| Wrong (fabricated) | Correct |
|---|---|
| `--c1-bulk-dir <path>` | `--out-mapping-json <path>` and `--out-stats-json <path>` |
| `--out-dir <path>` | `--out-certificate-jsonl <path>`, `--out-certificate-summary-json <path>`, `--out-certified-graph-json <path>` |
| `--dedup-mode` always passed | Only passed when `--bbox-gated` is set |
| (missing) `--dataset-root` | Added (required by target script) |

#### Step 2 – Bulk Apply (`c1_bulk_apply_layout_dedup.py`)

| Wrong (fabricated) | Correct |
|---|---|
| `--bak-root <path>` | Script does not accept this arg (removed) |
| `--no-bbox-gated` | Script only has `--bbox-gated` (store_true); no inverse flag exists |
| `--bbox-gated` / `--no-bbox-gated` ternary | Just pass `--bbox-gated` when enabled (no else branch) |
| (missing) `--mapping-stats-json` | Added for bbox-gated traceability |
| (missing) `--certificate-jsonl` | Added for bbox-gated traceability |
| (missing) `--reject-ledger-jsonl` | Added for bbox-gated reject tracking |

#### Step 3 – Audit (`placement_pairwise_compare.py`)

| Wrong (fabricated) | Correct |
|---|---|
| `--layout-root <path>` | `--left-root <path>` and `--right-root <path>` |
| `--subset-dirs GRScenes100:<path>` | `--scene-list-json <path>` (JSON array of scene IDs) |
| `--report-dir <path>` | `--out <path>` and `--verdict-out <path>` |
| `--out-name <name>` | `--right-layout-name <name>` |
| `--mode audit` | `--left-mode current` and `--right-mode current` |
| `--output-basename <name>` | Script does not accept this arg (removed) |
| `--bbox-gated` | Script uses `--certificate-jsonl` for certificate-aware semantics |

#### General Issues

| Wrong | Correct |
|---|---|
| `sys.executable` (system Python) | `ISAAC_PY` (Isaac Sim Python wrapper, required for pxr) |
| Sidecar name: `layout.parallel_{label}.{category}_{policy}_{version}.usd` | `layout.{label}_{policy}_{version}.usd` (matches autorun convention) |
