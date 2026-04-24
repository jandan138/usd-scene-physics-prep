---
title: Transitive Full Baseline Export Design
code_reference:
  - scripts/c1_autorun_categories.py
  - scripts/c1_bulk_apply_layout_dedup.py
  - scripts/rewrite_layout_asset_refs_with_compensation.py
  - check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout_transitive_full_dlc_20260415
created_at: 2026-04-20
updated_at: 2026-04-21
maintainer: OpenCode
status: approved
doc_class: record
---

# Transitive Full Baseline Export Design

## Goal

Generate a new slim delivery dataset under `/cpfs/user/zhuzihou/assets` from the
successful dry-run transitive-capable GRScenes-test0 full rerun outputs, while
preserving the current repo-side baseline and rerun roots as provenance. This
design does not describe a true promoted duplicate-removed clone.

## Current Context

The final dry-run rerun result already exists and is successful as a metrics and
delivery-snapshot source.

Current authoritative inputs:

- rerun dataset root:
  `GRScenes-test0-rebuilt-normalize-prededup-transitive-rerun-20260413`
- rerun result root:
  `check_reports/test0_rebuilt_dedup/c1_bulk_v8_tier2_rollout_transitive_full_dlc_20260415`
- final conclusion doc:
  `docs/records/research/operations/2026-04-18_transitive_full_rerun_conclusion.md`

Important constraints:

- Step 6 stayed in `dry_run`, so final scene outputs are the generated
  `layout.c1_v8_tier2_rollout_transitive_full_dlc_bbox_primary_rmse_observe_v1.usd`
  files, not overwritten `layout.usd`
- later investigation confirmed that category dry-run apply outputs wrote the
  same suffixed layout filenames per scene, so these files are dry-run-derived
  scene snapshots rather than a cumulative promote-applied baseline
- the repo-side rerun dataset and full rerun outputs must remain intact for
  traceability
- `Material/` may remain untrimmed for this phase

## Product Decision

Create a **new** slim delivery directory rather than mutating the current rerun
dataset root.

Recommended output naming pattern:

- `/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418/`

The new directory becomes the human-facing delivery set. The current rerun root
remains the authoritative provenance source.

## Delivery Directory Shape

The exported dataset should have this high-level structure:

```text
/cpfs/user/zhuzihou/assets/GRScenes-test0-transitive-full-baseline-20260418/
  GRScenes100/
    home/<scene_id>/layout.usd
    commercial/<scene_id>/layout.usd
  GRScenes_assets/
    <category>/<asset_uid>/usd/<asset_uid>.usd
  Material/
  MANIFEST.json
  asset_pruning_summary.json
  dangling_references.json
  README.md
```

## Scene Export Rule

For each scene directory:

1. Read the final rerun-produced layout file:
   `layout.c1_v8_tier2_rollout_transitive_full_dlc_bbox_primary_rmse_observe_v1.usd`
2. Copy that file into the delivery dataset as `layout.usd`
3. Do not expose the long rerun suffix in the delivery dataset

This makes the delivery dataset immediately usable and avoids forcing downstream
users to understand the rerun naming convention.

## Asset Export Rule

Only keep assets that are actually referenced by the exported final scene
layouts.

Reference closure rule:

- collect direct asset references reachable from every exported `layout.usd`
- retain only those `GRScenes_assets/...` assets in the delivery dataset
- remove all other asset directories from the delivery dataset by omission

This is the actual slimming mechanism. It is not based on historical candidate
counts or union-removable counts. It is based only on the exported dry-run scene
outputs and therefore should not be confused with producing a true
duplicate-removed promoted clone.

## Material Rule

For this phase, `Material/` is copied as a whole.

Rationale:

- keeps this export pass low risk
- avoids premature material/texture closure complexity
- still allows meaningful size reduction from asset trimming alone

Material slimming can be a later follow-up if needed.

## Dangling Reference Policy

The current normalized/prededup chain still contains known missing-asset
warnings. The export should treat these conservatively.

Policy:

- do not rewrite final layouts to remove dangling references in this phase
- export the final layouts exactly as produced by the authoritative rerun
- record any unresolved references into `dangling_references.json`
- surface counts and representative paths in `MANIFEST.json` and `README.md`

This keeps the export aligned with the dry-run-derived delivery snapshot rather
than introducing an extra behavior-changing cleanup step.

## Manifest And Summary Files

### `MANIFEST.json`

Must record:

- source rerun dataset root
- source full rerun result root
- full rerun job id if known
- generation timestamp
- exported scene count
- exported asset count
- copied material mode (`whole_copy`)
- dangling reference count
- key metrics copied from the final rerun conclusion

### `asset_pruning_summary.json`

Must record:

- original asset count in source dataset root
- retained asset count
- omitted asset count
- per-category retained/omitted counts

### `README.md`

Must explain:

- this is a slim delivery dataset derived from dry-run transitive-capable full
  rerun outputs
- `layout.usd` in this directory corresponds to the final rerun layout outputs
- the repo-side baseline and rerun roots remain the provenance source of truth
- known dangling-reference warnings are recorded, not rewritten away
- this directory is not proof that the repo-side baseline was promote-applied
  into a duplicate-removed clone

## Verification Requirements

The export is only acceptable if all of the following pass:

### 1. Structure verification

- required top-level paths exist
- scene directories contain `layout.usd`
- manifest and summary files exist

### 2. Layout identity verification

- sampled delivery `layout.usd` files must match the corresponding final rerun
  `layout.c1_v8_tier2_rollout_transitive_full_dlc_bbox_primary_rmse_observe_v1.usd`
  byte-for-byte or via deterministic content comparison

### 3. Reference closure verification

- scan all delivery `layout.usd` files
- every resolvable asset reference must exist in the delivery dataset
- unresolved references must be captured in `dangling_references.json`

### 4. Slimming verification

- retained/omitted asset counts must be computed and written
- spot-check that known mapped-away assets from categories like `bottle` are not
  retained if they are no longer referenced by any final scene layout

## Non-Goals

This export does not:

- overwrite or mutate the current rerun dataset root
- mutate repo-side authoritative rerun outputs
- attempt final promote semantics beyond the available dry-run-derived outputs
- trim `Material/` by reference closure in this phase
- rewrite dangling references out of final layouts

## Recommended Implementation Shape

Implement as a repeatable export script rather than a one-off manual copy.

Recommended script path:

- `scripts/export_transitive_full_baseline_dataset.py`

Recommended inputs:

- `--dataset-root`
- `--full-rerun-root`
- `--output-root`

The script should be safe to rerun into a fresh destination and should not
require modification of the source dataset roots.

## Bottom Line

The correct low-risk next step described by this historical design is to derive
a dry-run-derived delivery dataset from the successful full rerun outputs, with
final scene layouts exposed as `layout.usd`, unused assets omitted,
`Material/` copied whole, and provenance preserved in manifest files rather
than by mutating the current baseline roots.
