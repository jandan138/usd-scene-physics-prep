---
title: "Task 1 code review: doc_manager grouping and doc_class handling"
code_reference:
  - scripts/doc_manager.py
  - tests/test_doc_manager.py
created_at: 2026-04-10
updated_at: 2026-04-10
maintainer: OpenCode
status: Active
doc_class: record
---

# Task 1 code review: doc_manager grouping and doc_class handling

## Scope

- Reviewed current working-tree changes in `scripts/doc_manager.py` and `tests/test_doc_manager.py`.
- Checked reported test command: `python -m pytest tests/test_doc_manager.py -q`.

## Findings

1. `scripts/doc_manager.py:86-90`
   Root-level docs are grouped using the full filename as the section key when the document is directly under `docs/`.
   On this repo, files like `docs/agent-team-playbook.md` and `docs/MAINTENANCE_WORKFLOW.md` would produce headings like `Agent-Team-Playbook.Md` and `Maintenance Workflow.Md`, which is incorrect index behavior on realistic paths.

2. `scripts/doc_manager.py:21,115-117,181`
   Validation accepts `doc_class: archive`, but index generation has only `Primary Docs` and `Records` sections and routes every non-`primary` class into `Records`.
   This is an inconsistency between validation and generation that the tests do not cover.

## Verification

- `python -m pytest tests/test_doc_manager.py -q` -> `3 passed`

## Decision

- Review outcome: not approved.
