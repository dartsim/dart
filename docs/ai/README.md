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

For documentation structure or placement work, also read `docs/README.md`,
`docs/information-architecture.md`, and `docs/AGENTS.md`.

For authoring or executing orchestrator-defined work packets, also read
`docs/ai/orchestration.md`.

For AI component maintenance or durable AI-infra self-improvement, read
`docs/ai/terminology.md`, `docs/ai/components.md`,
`docs/ai/verification.md`, and
`docs/onboarding/ai-tools.md`.

## Visibility And Context Budget

`AGENTS.md`, this read order, and workflow `Required Reading` blocks are the
visibility contract for agents. Do not copy the same rule into every workflow;
move it to the owner doc, then add only the pointer needed for the workflow to
load it.

Always-loaded surfaces such as `AGENTS.md`, `docs/ai/principles.md`, and
`docs/ai/north-star.md` must stay compact. Put procedures, compatibility
details, and examples in the owner docs named by those entrypoints.

When a documented rule is missed, use `dart-audit-agent-compliance` to diagnose
whether the issue is owner placement, weak wording, missing required reading, a
workflow description, or generated-adapter sync.

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

## Starting Autonomous Projects

Use `$dart-ultrawork` in Codex or `/dart-ultrawork` in
Claude/OpenCode when a task should run as an autonomous project from either a
brief or one up-front decision interview. The workflow uses
`docs/dev_tasks/<task>/` as the project home, records acceptance criteria,
risks, decisions, verification, progress, and handoff state, and then routes
well-defined packets through the orchestrator/executor model. For ordinary
bounded single-session work, use `dart-new-task` unless the user explicitly
asks for the autonomous project-home loop.

## Source Ownership

| Surface                        | Role                                                                                  |
| ------------------------------ | ------------------------------------------------------------------------------------- |
| `AGENTS.md`                    | Root pointer board and mandatory high-level rules                                     |
| `docs/ai/principles.md`        | AI-infra axioms and manual audit checklist                                            |
| `docs/ai/terminology.md`       | Canonical AI-facing terms and migration candidates                                    |
| `docs/ai/`                     | Durable AI-native mission, workflow map, session rules, and verification expectations |
| `docs/ai/capabilities.json`    | Machine-readable capability status, category, and gate profile                        |
| `docs/ai/branch-profile.json`  | Machine-readable branch facts, required paths, exclusions, and AI-infra gates         |
| `docs/ai/agent-scenarios.json` | Seven deterministic fresh-session routing and verification contracts                  |
| `docs/ai/orchestration.md`     | Orchestrator/executor roles and the work-packet contract                              |
| `docs/onboarding/ai-tools.md`  | Tool compatibility and adapter maintenance details                                    |
| `.claude/commands/`            | Editable workflow source for DART user-invoked workflow capabilities                  |
| `.claude/skills/`              | Editable domain-skill source for DART on-demand Agent Skills                          |
| `.agents/skills/`              | Generated Codex adapter entrypoints for DART workflow and domain-skill capabilities   |
| `.codex/config.toml`           | Trusted-project bounded agent concurrency and delegation-depth policy                 |
| `.codex/agents/`               | Discoverable read-only scout, reviewer, and release-auditor profiles                  |
| `.codex/hooks.json`            | Maintained fast Codex command-hook configuration                                      |
| `.opencode/command/`           | Generated OpenCode command adapter entrypoints                                        |
| `scripts/sync_ai_commands.py`  | Adapter sync and AI docs consistency checker                                          |

Do not hand-edit generated `.agents/skills/` or `.opencode/command/` files.
Update the source surface, then run `pixi run sync-ai-commands`. Files under
`.codex/` are maintained project runtime configuration and must be reviewed
like scripts rather than regenerated.

## Agent-Friendly Setup And Diagnosis

Run `pixi run ai-setup` once in a checkout to synchronize adapters and install
the cross-tool Git pre-commit guard. Run `pixi run ai-doctor` at session start
or after a discovery/setup failure; it reports versions, project trust-sensitive
surfaces, instruction chains, skills, agents, hooks, tasks, and recovery
commands without modifying the checkout.

Use `pixi run check-agent-hook` for the fast staged-file structural gate,
`pixi run test-ai-infra` for focused infrastructure tests, and
`pixi run check-ai-infra` for the aggregate non-mutating gate. The aggregate
also exercises deterministic orientation, small-change, failure-diagnosis,
documentation, component, simulation-verification, and release-maintenance
scenarios. Full build/test
selection remains owned by `docs/ai/verification.md` and the task-specific
developer docs.

Codex loads project `.codex/` configuration and hooks only after the checkout
is trusted. Review project hooks with `/hooks`; changed hook definitions require
review again. A skipped Codex hook is not a correctness boundary: the installed
Git hook and explicit pre-commit gates remain authoritative.

## Model Routing

DART uses the two-role operating model in `docs/ai/orchestration.md`: an
orchestrator session owns understanding, decomposition, sequencing, and review,
while executor sessions implement one well-defined work packet at a time. No
project model is pinned. For complex, high-value, multi-workstream work, prefer
Codex 5.6 Sol with Ultra reasoning when available; use a lean task contract and
delegate bounded read-only scouting or review to the project profiles in
`.codex/agents/`. Those profiles inherit the selected parent model. Smaller
tasks should use the lightest capable model and a focused workflow.

Claude Code, OpenCode, Gemini CLI, future Codex models, and human contributors
remain supported: roles are not products, authoring and review stay separate,
and every workflow maps to public docs and `pixi run ...` commands. DART 6.20
has a separately maintained compatibility-first catalog; never infer that a
`main` path, task, or language/toolchain fact exists on the release branch.

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
