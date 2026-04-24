---
title: Promoted Clone Workspace Helper Review
code_reference:
  - scripts/prepare_promoted_clone_workspace.py
  - tests/test_prepare_promoted_clone_workspace.py
  - docs/records/changes/2026-04-21_promoted_clone_workspace.md
created_at: 2026-04-21
updated_at: 2026-04-21
maintainer: OpenCode
status: active
doc_class: record
---

# Promoted Clone Workspace Helper Review

## Scope

Reviewed the recently added promoted-clone workspace helper for correctness risks, edge cases, maintainability, test adequacy, and operational safety for the intended `/cpfs/user/zhuzihou/dedup_workspaces/...` rollout.

## Verification

Command run:

```bash
python -m pytest tests/test_prepare_promoted_clone_workspace.py -q
```

Result:

- passed with `4 passed in 0.05s`

## Findings

1. `scripts/prepare_promoted_clone_workspace.py:74-75,89-99,147-152`

The public helper accepts `policy_tag` and `out_version`, and uses them when seeding `c1_bulk`, but the generated run command hard-codes `POLICY_TAG` and omits `--out-version`. Any non-default call will prepare one category tree and run `c1_autorun_categories.py` against another naming convention, which can silently bypass the seeded certs and rebuild or operate on the wrong directories.

2. `scripts/prepare_promoted_clone_workspace.py:32-40,135-145`

Seeding eligibility is gated only on `01_cert/filtered_mapping.json`. In bbox-gated apply mode, downstream execution also consumes the companion stats and certificate artifacts under `01_cert/`. If a category contains only `filtered_mapping.json` or a partial cert bundle, this helper will seed an incomplete category that later fails during apply. The helper should either validate the full required bundle or deliberately rebuild from scratch instead of treating a partial `01_cert/` as reusable.

3. `scripts/prepare_promoted_clone_workspace.py:103-115,147-177`

The default `source_report` and `mode_reports_dir` paths are inferred but never validated before the workspace is created and copied. For the eventual real CPFS run, a typo or stale layout under the upstream root will still produce a fully populated workspace plus runnable shell script, and the operator only discovers the problem when the long Isaac/autorun job starts. This is an avoidable operational risk and is not exercised by the tests.

## Test Adequacy Notes

- Existing tests cover the happy path, `01_cert`-only seeding, refusal to overwrite an existing workspace root, and basic command emission.
- Missing coverage for custom `policy_tag` / `out_version`, incomplete `01_cert` bundles, and missing inferred `source_report` / `mode_reports_dir` inputs.

## Decision

- Review result: changes required
