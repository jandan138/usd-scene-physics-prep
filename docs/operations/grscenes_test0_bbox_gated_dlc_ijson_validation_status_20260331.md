---
title: "GRScenes-test0 BBox-Gated DLC ijson Validation Status"
created_at: "2026-03-31"
updated_at: "2026-03-31"
maintainer: "Codex"
status: "active"
code_reference:
  - "scripts/isaac_python.sh"
  - "scripts/dlc/run_task.sh"
  - "third_party/runtime_deps/isaac_py310/ijson/__init__.py"
  - "check_reports/test0_bbox_gated/20260331_dlc_ijson_smoke_v2/run_manifest.json"
---

# Summary

This note archives the validation result for the DLC `ijson` runtime fix.

The important change is simple:

- smoke `v1` failed because the remote DLC image did not provide `ijson`
- the runtime fix was updated to expose a repo-vendored `ijson`
- smoke `v2` succeeded on real DLC and imported `ijson` from the repo-vendored
  path

This means the `ijson` runtime blocker has been cleared for the real DLC job
entrypoint.

# Validation Target

Validation run:

- run root:
  - `check_reports/test0_bbox_gated/20260331_dlc_ijson_smoke_v2/`
- display name:
  - `t0ijsonsmokev2_0_1`
- job id:
  - `dlc8btgcq9lvbgdv`

# Live Result

Live DLC query result:

- `Status = Succeeded`
- `ReasonCode = JobSucceeded`
- finish time:
  - `2026-03-31T04:04:01Z`

Representative source:

- `./dlc get job dlc8btgcq9lvbgdv --workspace_id 270969 --show_detail`

# Pod Log Evidence

Live pod logs show:

```text
ISAAC_MODULE_OK {"module": "ijson", ...,
"origin": "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/third_party/runtime_deps/isaac_py310/ijson/__init__.py"}
```

The smoke script itself also printed:

```text
{"status": "ok", ..., "ijson": "/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/third_party/runtime_deps/isaac_py310/ijson/__init__.py", ...}
```

That is the key proof:

- the real DLC job no longer depends on the base image having `ijson`
- it can now import `ijson` from the repo-controlled runtime dependency path

# What Stage The Project Is In Now

The main bbox-gated A/B project is no longer in the "fix DLC runtime blocker"
stage.

It has now moved to:

- runtime blocker cleared
- ready to launch the next fresh real-data Policy A / Policy B rerun

In plain language:

- before this validation, the project was blocked because jobs died before cert
- after this validation, the environment problem is no longer the thing stopping
  us
- the next real step is to start a new fresh A/B run and see whether the
  pipeline now progresses through cert/apply/audit as intended

# Next Recommended Step

Launch a new fresh bbox-gated real-data A/B rerun, rather than reusing `v5`.

Related documents:

- `docs/operations/grscenes_test0_bbox_gated_dlc_ijson_remediation_plan_20260330.md`
- `docs/operations/grscenes_test0_bbox_gated_status_change_20260330.md`
