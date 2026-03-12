# CODEX.md

This file defines how Codex should emulate the repository's existing agent-team workflow.

## Purpose

This project already has a mature Claude-oriented team setup under `.claude/`.
Codex must not invent a parallel governance model. Instead, reuse the same role
boundaries, file ownership rules, and conflict-avoidance constraints while
mapping execution onto Codex sub-agents.

## Source Of Truth

Before spawning sub-agents, read these files:

- `CLAUDE.md` for project architecture, commands, and domain constraints
- `.claude/file-ownership.md` for file ownership and high-conflict paths
- `docs/agent-team-playbook.md` for the original team model
- `docs/architecture/codex_agent_team_playbook.md` for the Codex mapping

If these documents conflict, follow this order:

1. `CLAUDE.md`
2. `.claude/file-ownership.md`
3. `docs/architecture/codex_agent_team_playbook.md`
4. `docs/agent-team-playbook.md`

## Codex Team Lead Model

In this repository, the main Codex session acts as the team lead.

- The main session owns planning, task decomposition, and final integration
- Sub-agents are helpers, not independent leads
- Do not delegate the immediate blocking decision when the main session can make it directly
- Do not spawn multiple writers against the same file set

## Trigger Phrase

If the user begins a request with either of these prefixes:

- `启动agent teams完成任务：`
- `进入agent teams模式：`

then Codex must treat that as an explicit instruction to use the repository's
team-lead workflow instead of default single-agent execution.

When triggered:

- first decide whether the task merits sub-agent decomposition
- if decomposition is justified, actively use Codex multi-agent capability
- prefer parallel delegation only for non-overlapping file ownership
- explain the split briefly before spawning sub-agents
- keep final integration and verification in the main session

If the task is too small, too coupled, or touches a single high-conflict file,
stay in team-lead mode but explain why the work will remain effectively
single-threaded for that task.

## Role Mapping

Map Claude roles onto Codex roles as follows:

- `codebase-explorer`, `architecture-planner`, `feature-designer` -> Codex `explorer`
- `feature-implementer`, `code-refactorer`, `bug-fixer`, `docs-writer` -> Codex `worker`
- Domain validators and test specialists -> Codex `worker` unless read-only exploration is enough

When useful, include the Claude agent name in the spawned prompt so the
sub-agent knows which behavioral contract it is emulating.

## Parallelism Rules

- Check `.claude/file-ownership.md` before spawning parallel workers
- If two tasks touch the same file, run them serially
- Treat these as high-conflict files unless proven otherwise:
  - `set_physics/pxr_utils/data_clean.py`
  - `set_physics/pxr_utils/usd_physics.py`
  - `set_physics/preprocess_for_interaction.py`
  - `specs_normalizer/normalize.py`
- `docs/` is effectively owned by the lead or a single docs-focused worker for a given task

## Delegation Protocol

Every spawned Codex sub-agent should receive:

- the concrete goal
- the exact files or module area it owns
- any paths it must not modify
- required verification commands
- a reminder that it is not alone in the codebase and must not revert others' changes

Use prompts with this structure:

1. Role being emulated
2. Owned file set
3. Required outputs
4. Verification steps
5. Constraints from `CLAUDE.md` and `.claude/file-ownership.md`

When the trigger phrase above is used, the lead should explicitly say which
Claude-style role each spawned sub-agent is emulating.

## Documentation Rule

Codex sub-agents do not automatically inherit the documentation discipline
described in `CLAUDE.md`. The main session must enforce it explicitly.

For significant tasks, require sub-agents to report:

- findings
- changed files
- commands run
- results and unresolved risks

If the task materially changes behavior, update `docs/` in the same session when
appropriate and keep YAML front matter valid.

## Environment Constraints

- Pure `pxr` scripts run with `python`
- Isaac Sim scripts must run via `./scripts/isaac_python.sh`
- Never flatten USD composition
- Never write absolute asset paths into USD files
- Never move Isaac Sim-only imports into pure utility modules

## Practical Difference From Claude

Codex can reproduce the workflow discipline, but not Claude's native
`Agent Teams` runtime. Expect similar task decomposition, not identical runtime
behavior.
