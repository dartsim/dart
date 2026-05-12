# AI Components

This document defines how DART maintains AI-facing components.

## Ownership Model

`AGENTS.md` is the root pointer board. `docs/ai/` owns durable AI-native policy.
`docs/onboarding/ai-tools.md` owns compatibility details. The current editable
workflow source is `.claude/commands/`, and the current editable domain-skill
source is `.claude/skills/`.

Generated surfaces are first-class entrypoints for their tools:

- `.codex/skills/` for Codex;
- `.opencode/command/` for OpenCode.

Generated files include source metadata and must not be hand-edited.

## Adding A Workflow

1. Add the concise workflow source under `.claude/commands/dart-<name>.md`.
2. Keep detailed policy in `docs/ai/` or `docs/onboarding/`.
3. Add the workflow to `AGENTS.md` and `docs/ai/workflows.md`.
4. Run `pixi run sync-ai-commands`.
5. Run `pixi run check-ai-commands`.

## Adding A Domain Skill

1. Add `.claude/skills/dart-<name>/SKILL.md`.
2. Keep the skill lightweight and point to full docs.
3. Add the skill to `AGENTS.md` and `docs/ai/workflows.md`.
4. Run `pixi run sync-ai-commands`.
5. Run `pixi run check-ai-commands`.

## Public Path Requirement

Every AI workflow must map back to public docs and `pixi run ...` commands so a
contributor can complete the same work manually. AI tooling can make the path
faster; it must not be the only path.

## Checks

`scripts/sync_ai_commands.py` verifies adapter parity, metadata budgets, and the
`docs/ai/workflows.md` capability index. `pixi run check-ai-commands` is the
non-mutating CI check.
