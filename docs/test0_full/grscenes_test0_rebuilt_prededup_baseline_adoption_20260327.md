---
title: "GRScenes-test0-rebuilt-normalize-prededup Baseline Adoption"
created_at: "2026-03-27"
updated_at: "2026-03-27"
maintainer: "Codex"
status: "completed"
code_reference:
  - "check_reports/test0_rebuilt_full/20260315_chain_fix_v1/run_manifest.json"
  - "check_reports/test0_rebuilt_full/20260315_chain_fix_v1/preflight/baseline_verification.json"
  - "check_reports/test0_rebuilt_full/20260315_chain_fix_v1/summary/normalize_gate_verdict.json"
  - "check_reports/test0_rebuilt_full/20260315_chain_fix_v1/summary/final_verdict.json"
  - "check_reports/test0_rebuilt_full/20260315_chain_fix_v1/summary/prededup_baseline_binding_verdict.json"
  - ".codex/worklogs/subagents/2026-03-26/resume-download-grscenes-test0-normalize-prededup.md"
  - ".codex/worklogs/subagents/2026-03-27/worker-baseline-verify.md"
---

# Purpose

This note is the provenance-closure artifact for promoting
`/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup`
from a conditionally accepted baseline candidate to the formally confirmed
canonical upstream baseline for bbox-gated dedup.

# Target Root

- Local adopted baseline root:
  `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup`

# Passed Normalize-Only Lineage

The direct formal normalize-only PASS bundle for the rebuilt chain-fix run is:

- run id:
  `20260315_chain_fix_v1`
- source root:
  `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt`
- normalized output root recorded by the run:
  `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalized`

Direct PASS artifacts:

- `check_reports/test0_rebuilt_full/20260315_chain_fix_v1/run_manifest.json`
- `check_reports/test0_rebuilt_full/20260315_chain_fix_v1/preflight/baseline_verification.json`
- `check_reports/test0_rebuilt_full/20260315_chain_fix_v1/summary/normalize_gate_verdict.json`
- `check_reports/test0_rebuilt_full/20260315_chain_fix_v1/summary/final_verdict.json`

# Current Provenance Hypothesis

Current accepted hypothesis:

- the local root
  `GRScenes-test0-rebuilt-normalize-prededup`
- is the preserved local landing / adopted rename of the remote standardized
  20260315 normalized output:
  `aliyun-a-oss-demo:pjlab-bjpai-zhuzihou-assets/GRScenes-test0-rebuilt-normalized-standard-20260315`

Supporting evidence already on record:

- local root contains `layout.pre_c1_normalize_only.20260315_chain_fix_v1.usd`
- download worklog records the remote source and local destination:
  `.codex/worklogs/subagents/2026-03-26/resume-download-grscenes-test0-normalize-prededup.md`

# Verification Block

## Remote source

- `aliyun-a-oss-demo:pjlab-bjpai-zhuzihou-assets/GRScenes-test0-rebuilt-normalized-standard-20260315`

## Local target

- `/cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup`

## Verification command

```bash
env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy -u ALL_PROXY -u all_proxy \
  rclone lsf \
  aliyun-a-oss-demo:pjlab-bjpai-zhuzihou-assets/GRScenes-test0-rebuilt-normalized-standard-20260315

find /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup -type f | wc -l

env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy -u ALL_PROXY -u all_proxy \
  rclone size \
  aliyun-a-oss-demo:pjlab-bjpai-zhuzihou-assets/GRScenes-test0-rebuilt-normalized-standard-20260315

env -u HTTP_PROXY -u HTTPS_PROXY -u http_proxy -u https_proxy -u ALL_PROXY -u all_proxy \
  rclone check \
  /cpfs/shared/simulation/zhuzihou/dev/usd-scene-physics-prep/GRScenes-test0-rebuilt-normalize-prededup \
  aliyun-a-oss-demo:pjlab-bjpai-zhuzihou-assets/GRScenes-test0-rebuilt-normalized-standard-20260315 \
  --one-way
```

## Verification result

- Remote probe succeeded and returned:
  - `GRScenes100/`
  - `GRScenes_assets/`
  - `Material/`
- Local file count:
  - `193663`
- Remote object count:
  - `193663`
- Remote total size:
  - `147.192 GiB (158046412525 Byte)`
- `rclone check --one-way` result:
  - `0 differences found`
  - `193663 matching files`

## Verification conclusion

- The exact local root matches the remote standardized 20260315 source.
- This closes the exact-root provenance gap.
- Together with the direct PASS bundle for `20260315_chain_fix_v1`, this is sufficient to formally adopt the local root as the canonical upstream baseline for bbox-gated dedup.

# Adoption Statement

> `GRScenes-test0-rebuilt-normalize-prededup` is adopted as the canonical local
> upstream baseline for bbox-gated dedup. It is the preserved local copy of the
> passed normalize-only 20260315 rebuilt standardized output lineage, and it is
> the baseline root to use for subsequent bbox-gated dedup implementation and
> validation work.

# Status Transition Rule

This note is now final because:

1. the verification block contains a successful exact-root comparison
2. the conclusion explicitly adopts the local root as the canonical upstream baseline
