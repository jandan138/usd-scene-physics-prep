---
title: "Task 1 Transitive BBox Gating Code Review"
created_at: "2026-04-13"
updated_at: "2026-04-13"
maintainer: "OpenCode"
status: "active"
code_reference:
  - "scripts/compute_vertex_transform.py"
  - "tests/test_compute_vertex_transform.py"
---

# Summary

Reviewed the Task 1 worktree changes for transitive bbox gating in:

- `scripts/compute_vertex_transform.py`
- `tests/test_compute_vertex_transform.py`

Review focus:

- correctness bugs
- behavioral regressions
- fragile witness-path selection logic
- certificate evaluation risks
- schema/naming risks that would hurt integration
- missing test coverage for Task 1 additions

Conclusion:

- no actionable quality issues found in the reviewed Task 1 scope

# Commands Run

- `git status --short -- scripts/compute_vertex_transform.py tests/test_compute_vertex_transform.py`
- `git diff -- scripts/compute_vertex_transform.py tests/test_compute_vertex_transform.py`
- `python -m pytest tests/test_compute_vertex_transform.py -k 'not test_topo_filesize_with_v_computation'`

# Verification Result

- targeted test result: `19 passed, 1 deselected`
- excluded known pre-existing baseline failure:
  - `tests/test_compute_vertex_transform.py::TestPairCertificate::test_topo_filesize_with_v_computation`

# Notes

- witness-path selection now deterministically prefers the lowest-risk shortest path via `_path_witness_key`
- transitive certificate evaluation reuses `_evaluate_certificate_with_V`, which keeps direct and transitive bbox precheck behavior aligned
- added tests cover both successful transitive witness recording and fail-closed endpoint precheck behavior
