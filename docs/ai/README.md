---
type: ai-entrypoint
owner: self
---

# AI Agent Entrypoint

This directory is the shared AI-native starting point for DART. It keeps
durable agent workflow policy in tracked docs while tool-specific command and
skill files stay thin, generated, or compatibility-focused.

## Read Order

For general DART agent work:

1. `AGENTS.md`
2. `docs/ai/principles.md`
3. `docs/ai/README.md` (this file)
4. `docs/ai/north-star.md`
5. `docs/ai/workflows.md`
6. `docs/ai/verification.md`
7. The task-specific developer doc listed in `AGENTS.md`

For multi-session work, also read `docs/ai/sessions.md` and
`docs/dev_tasks/README.md`.

For project planning, also read `docs/plans/README.md`,
`docs/plans/dashboard.md`, and `docs/plans/north-star-roadmap.md`.

For authoring or executing orchestrator-defined work packets, also read
`docs/ai/orchestration.md`.

For AI component maintenance or durable AI-infra self-improvement, read
`docs/ai/terminology.md`, `docs/ai/components.md`,
`docs/ai/verification.md`, and
`docs/onboarding/ai-tools.md`.

## Choosing The Next Task

Use `$dart-next` in Codex or `/dart-next` in Claude/OpenCode when the goal is
to let an agent select the next bounded DART task from tracked evidence. The
workflow reads the north star, plan dashboard, active dev-task rules, and
verification policy, then routes the selected task through the most specific
workflow such as `dart-new-task`, `dart-fix-ci`, `dart-docs-update`, or
`dart-plan-update`.

Useful constraints include `mode=select`, `mode=execute`, `mode=pr`,
`size=tiny|small|medium|large`, `days=N`, `focus=<topic>`, and
`area=<dimension>`. `focus` is a preference rather than a hard filter: the
workflow should favor matching tasks, then explain when a higher-evidence or
better-bounded task is selected instead. PR creation, PR comments, review
re-triggers, thread resolution, CI reruns, pushes, merges, and branch deletion
still require explicit maintainer/user approval.

## Source Ownership

| Surface                       | Role                                                                                  |
| ----------------------------- | ------------------------------------------------------------------------------------- |
| `AGENTS.md`                   | Root pointer board and mandatory high-level rules                                     |
| `docs/ai/principles.md`       | AI-infra axioms and manual audit checklist                                            |
| `docs/ai/terminology.md`      | Canonical AI-facing terms and migration candidates                                    |
| `docs/ai/`                    | Durable AI-native mission, workflow map, session rules, and verification expectations |
| `docs/ai/capabilities.json`   | Machine-readable capability status, category, and gate profile                        |
| `docs/ai/orchestration.md`    | Orchestrator/executor roles and the work-packet contract                              |
| `docs/onboarding/ai-tools.md` | Tool compatibility and adapter maintenance details                                    |
| `.claude/commands/`           | Editable workflow source for DART user-invoked workflow capabilities                  |
| `.claude/skills/`             | Editable domain-skill source for DART on-demand Agent Skills                          |
| `.codex/skills/`              | Generated Codex adapter entrypoints for DART workflow and domain-skill capabilities   |
| `.opencode/command/`          | Generated OpenCode command adapter entrypoints                                        |
| `scripts/sync_ai_commands.py` | Adapter sync and AI docs consistency checker                                          |

Do not hand-edit generated `.codex/` or `.opencode/` files. Update the source
surface, then run `pixi run sync-ai-commands`.

## Model Routing

DART uses the two-role operating model in `docs/ai/orchestration.md`: an
orchestrator session owns understanding, decomposition, sequencing, and
review, while executor sessions implement one well-defined work packet at a
time. The current default routing is Claude Code as the orchestrator and
Codex — via the generated `$dart-*` skills, especially
`$dart-execute-packet` — as the executor for orchestrator-sized chunks of
work. Either role may be filled by any capable agent, and independent review
should use a different session than the one that authored the change. This
routing affects agent workflow design, not human contributor requirements:
every AI workflow must map back to public docs and `pixi run ...` commands
that a contributor can run without any AI tool.

## Safety Boundary

AI agents may inspect files, make local edits requested by the user, and run
local verification. GitHub mutations require explicit maintainer/user approval,
including pushes, PR comments, review-thread resolution, reviewer requests,
merges, and review re-triggers such as `@codex review`.

For automated review comments from bot accounts: never reply inline, verify each
claim locally, and treat any push, comment, thread resolution, or re-trigger as
an external mutation needing explicit maintainer/user approval. See
`docs/onboarding/ai-tools.md` § "Handling Automated Reviews" for the full loop.

## Required Gates

Use `docs/ai/verification.md` to select the strongest gate that matches the
work. AI-surface changes use its AI docs/adapters gate set.

Before committing, DART still requires `pixi run lint`. Code changes require
the build and test gates listed in `AGENTS.md` and the relevant developer docs.

Substantial AI-infra changes also require the principle audit in
`docs/ai/principles.md`; record the result in the final response or PR Testing
section.
