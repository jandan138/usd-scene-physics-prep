---
title: "Codex Agent Team Playbook"
code_reference: "CODEX.md"
created_at: "2026-03-12"
updated_at: "2026-03-12"
maintainer: "codex"
status: "active"
---

# Codex Agent Team Playbook

## Summary

This document adapts the repository's existing Claude-oriented agent team design
to Codex. The goal is not runtime parity. The goal is operational parity:
Codex should make similar decomposition decisions, respect the same ownership
boundaries, and avoid the same classes of merge conflict.

## What Stays The Same

- The main interactive session is the team lead
- File ownership remains governed by `.claude/file-ownership.md`
- High-conflict files must be handled serially
- Project architecture and execution rules still come from `CLAUDE.md`
- Isaac Sim scripts still run through `./scripts/isaac_python.sh`
- Documentation updates stay part of the definition of done for significant work

## What Changes In Codex

Claude's `Agent Teams` runtime uses project-defined agent files, explicit team
creation, task objects, and worktree-oriented isolation. Codex uses a different
execution model:

- `spawn_agent` creates sub-agents on demand
- `explorer` is best for bounded read-only codebase research
- `worker` is best for implementation, refactoring, verification, and docs work
- The main session must explicitly assign file ownership in each spawned prompt
- The main session must explicitly police overlap, because `.claude/agents/*.md`
  are not native Codex runtime configuration

## Trigger Phrase Contract

To make Codex usage feel closer to the repository's Claude workflow, treat these
user prefixes as a formal protocol:

- `启动agent teams完成任务：`
- `进入agent teams模式：`

When either prefix appears, the main Codex session should:

1. switch into team-lead behavior explicitly
2. read `CODEX.md`, `CLAUDE.md`, and `.claude/file-ownership.md` before splitting work
3. prefer sub-agent delegation over single-threaded execution when the task is meaningfully decomposable
4. state the proposed role split before spawning agents
5. keep final review, integration, and verification in the main session

This does not create a Claude-native runtime. It creates a stable prompt-level
contract so the same human instruction reliably causes Codex to behave like a
team lead using multi-agent support.

## Claude To Codex Mapping

| Claude role | Codex role | Typical use |
|---|---|---|
| `codebase-explorer` | `explorer` | Trace code paths, answer narrow architecture questions |
| `feature-designer` | `explorer` | Produce design notes before implementation |
| `architecture-planner` | `explorer` | Analyze refactor direction or system impacts |
| `feature-implementer` | `worker` | Implement a scoped feature in owned files |
| `code-refactorer` | `worker` | Perform a bounded refactor in owned files |
| `bug-fixer` | `worker` | Diagnose and fix a concrete bug |
| `docs-writer` | `worker` | Update `docs/` and keep front matter valid |
| `usd-asset-validator` | `worker` | Run validation commands and summarize results |
| `isaacsim-usd-inspector` | `worker` | Execute domain-specific inspection flows |
| `version-commit-agent` | main Codex session | Integrate results and decide merge order |

## Team Lead Procedure In Codex

### 1. Read Before Delegating

Before splitting work, read:

- `CLAUDE.md`
- `.claude/file-ownership.md`
- any relevant design or operations docs in `docs/`

If the task is simple or touches one file, do it in the main session instead of
spawning a team.

### 2. Partition By File Ownership

Use `.claude/file-ownership.md` as the hard boundary for parallelism.

- Non-overlapping file sets can be delegated in parallel
- Overlapping file sets must be serialized
- If ownership is unclear, keep the work in the main session until clarified
- If the trigger phrase was used, explain explicitly whether the task will be
  parallel, partially parallel, or effectively serial

### 3. Delegate With Explicit Ownership

Every implementation-oriented sub-agent prompt should state:

- the Claude role it is emulating
- the exact file set it owns
- files it must not modify
- verification commands it must run
- the requirement not to revert unrelated changes

Recommended prompt skeleton:

```text
Emulate the repository's <role> behavior.
You own only these files: <paths>.
Do not modify: <paths>.
Read CLAUDE.md and .claude/file-ownership.md before editing.
Run these checks: <commands>.
You are not alone in the codebase; do not revert others' changes.
Return changed files, commands run, results, and unresolved risks.
```

### 4. Integrate In The Main Session

The main Codex session should:

- review sub-agent outputs
- inspect the changed files before finalizing
- resolve any remaining conflicts
- run final verification
- update docs when behavior changed materially

## Conflict Management

Treat these files as high-conflict unless the task proves otherwise:

- `set_physics/pxr_utils/data_clean.py`
- `set_physics/pxr_utils/usd_physics.py`
- `set_physics/preprocess_for_interaction.py`
- `specs_normalizer/normalize.py`

Rules:

1. Never let two Codex workers edit the same file concurrently.
2. If multiple tasks need the same high-conflict file, finish one fully before starting the next.
3. Keep docs changes in one place; avoid separate workers editing the same markdown file.
4. If a worker returns edits outside its assignment, the lead must review carefully before accepting them.

## Verification Model

Codex should preserve the project's existing command discipline:

- Pure `pxr` and general Python scripts: `python <script>.py`
- Isaac Sim dependent flows: `./scripts/isaac_python.sh <script>.py`
- Docs validation: `python scripts/doc_manager.py --validate`
- Docs index refresh when docs changed: `python scripts/doc_manager.py --gen-index`

For bug fixes or risky refactors, require exact reproduction and verification
commands in the worker's final report.

## Documentation Expectations

Claude's playbook requires strong documentation hygiene. Codex should follow the
same expectation even though the runtime is different.

When a task produces durable project knowledge, record it in `docs/` with valid
YAML front matter. Typical cases:

- new workflows
- architecture changes
- non-obvious bug root causes
- operational procedures
- significant validation results

If a spawned sub-agent is used for research only, the lead should still capture
the result in a doc when the information is likely to matter again.

## When Codex Will Not Behave Like Claude

Do not expect 1:1 runtime behavior. Differences include:

- no native consumption of `.claude/agents/*.md` as executable agent registry
- no automatic worktree isolation equivalent
- no Claude-style `TeamCreate` / `TaskCreate` object model
- different summarization and convergence behavior across sub-agents

The adaptation target is consistency of engineering process, not UI-level or
runtime-level sameness.

## Minimal Operating Checklist

1. Read `CLAUDE.md` and `.claude/file-ownership.md`.
2. Decide whether the task is single-threaded or worth delegation.
3. Split only by non-overlapping owned file sets.
4. Spawn `explorer` for bounded research and `worker` for production work.
5. State ownership and verification requirements explicitly in every sub-agent prompt.
6. Review returned edits in the main session.
7. Run final verification commands.
8. Update `docs/` and regenerate the docs index when behavior or workflow changed.
