---
title: "Task 2 spec compliance review"
code_reference:
  - docs/index.md
  - docs/getting-started/index.md
  - docs/how-to/index.md
  - docs/reference/index.md
  - docs/records/index.md
  - docs/superpowers/plans/2026-04-10-docs-reorganization.md
created_at: 2026-04-10
updated_at: 2026-04-10
maintainer: OpenCode
status: active
doc_class: record
---

# Task 2 spec compliance review

## Scope

- Reviewed worktree: `/root/.config/superpowers/worktrees/usd-scene-physics-prep/docs-reorganization-20260410`
- Reviewed requirement set: Task 2 only from `docs/superpowers/plans/2026-04-10-docs-reorganization.md`
- Review mode: spec compliance only, not style

## Findings

- Clarification-aware re-review performed after the approved temporary-link adjustment.
- The five landing pages still exist and remain the only Task 2 content files in scope.
- The temporary-link adjustment is implemented cleanly in the landing pages:
  - `docs/index.md` now uses current paths for overview/usage/faq and `INDEX.md#operations`.
  - `docs/getting-started/index.md`, `docs/how-to/index.md`, and `docs/reference/index.md` now point to current pre-move locations.
  - `docs/records/index.md` now points to `changes/`, `research/`, `test0_full/`, `test0_smoke/`, and `tmp/` via parent-relative links.
- The four required directories exist: `docs/getting-started`, `docs/how-to`, `docs/reference`, `docs/records`.
- `docs/operations/index.md` was not created.
- Fallback heading verification with `grep -nE` still proves the same five required headings.
- No doc moves into Task 3-6 target paths have started.
- Non-compliance remains: the reviewed worktree contains an extra Task 2 out-of-scope repo file, `docs/changes/2026-04-10_task2_code_review.md`.
- No commit was made for Task 2; the latest commit remains `9b92f24` and the work is still uncommitted.

## Evidence

- Planned Task 2 content read from `docs/superpowers/plans/2026-04-10-docs-reorganization.md` lines 292-390.
- Verified headings with:

```bash
grep -nE "^# Documentation Home|^# Getting Started|^# How-To Guides|^# Reference|^# Records" docs/index.md docs/getting-started/index.md docs/how-to/index.md docs/reference/index.md docs/records/index.md
```

- Verified reviewed worktree status with:

```bash
git status --short --untracked-files=all
```

- Verified `docs/operations/index.md` absence with:

```bash
 test -e docs/operations/index.md; printf "%s" $?
 ```

- Verified no later-task target paths were touched with:

```bash
git status --short --untracked-files=all -- docs/getting-started/project_overview.md docs/getting-started/quickstart.md docs/getting-started/faq.md docs/how-to/workflow_examples.md docs/how-to/export_scenes.md docs/how-to/interaction_preprocessing.md docs/how-to/layout_json_for_blender.md docs/how-to/layout_usd_to_layout_json_deep_dive.md docs/how-to/navigation_preprocessing.md docs/how-to/normalize_asset_transforms.md docs/how-to/prep_interaction_root_scene.md docs/how-to/rebuild_test0_from_legacy.md docs/how-to/rewrite_layout_asset_refs_with_compensation.md docs/how-to/scene_subset_package.md docs/how-to/simready.md docs/how-to/uid_subset_package.md docs/how-to/usd_to_glb_in_subset.md docs/reference/assets_and_materials.md docs/reference/dependencies.md docs/reference/modules docs/reference/scripts docs/reference/specs docs/reference/specs-normalizer docs/records/changes docs/records/research docs/records/runs docs/records/archive
```

- Verified latest commit stayed unchanged with:

```bash
 git log --oneline --decorate -1
 ```

- Reviewed extra Task 2 out-of-scope file content:

```text
docs/changes/2026-04-10_task2_code_review.md
```

## Decision

- Rejected under the clarified review criteria due to the extra Task 2 out-of-scope file under `docs/changes/`.
