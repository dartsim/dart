# AI Agent Entrypoint

This directory is the shared AI-native starting point for DART. It keeps
durable agent workflow policy in tracked docs while tool-specific command and
skill files stay thin, generated, or compatibility-focused.

## Read Order

For general DART agent work:

1. `AGENTS.md`
2. `docs/ai/README.md` (this file)
3. `docs/ai/workflows.md`
4. `docs/ai/verification.md`
5. The task-specific onboarding doc listed in `AGENTS.md`

For multi-session work, also read `docs/ai/sessions.md` and
`docs/dev_tasks/README.md`.

For AI component maintenance, read `docs/ai/components.md` and
`docs/onboarding/ai-tools.md`.

## Source Ownership

| Surface                       | Role                                                                                 |
| ----------------------------- | ------------------------------------------------------------------------------------ |
| `AGENTS.md`                   | Root pointer board and mandatory high-level rules                                    |
| `docs/ai/`                    | Durable AI-native policy, workflow map, session rules, and verification expectations |
| `docs/onboarding/ai-tools.md` | Tool compatibility and adapter maintenance details                                   |
| `.claude/commands/`           | Temporary editable source for DART workflow command bodies                           |
| `.claude/skills/`             | Editable source for DART domain skills                                               |
| `.codex/skills/`              | Generated first-class Codex workflow and skill entrypoints                           |
| `.opencode/command/`          | Generated OpenCode command entrypoints                                               |
| `scripts/sync_ai_commands.py` | Adapter sync and AI docs consistency checker                                         |

Do not hand-edit generated `.codex/` or `.opencode/` files. Update the source
surface, then run `pixi run sync-ai-commands`.

## Model Routing

Codex is the primary implementation model for DART's first AI-native rollout.
Claude Code and OpenCode remain supported compatibility surfaces, and Claude is
still useful for fallback and independent review. This routing affects agent
workflow design, not human contributor requirements: every AI workflow must map
back to public docs and `pixi run ...` commands that a contributor can run
without any AI tool.

## Safety Boundary

AI agents may inspect files, make local edits requested by the user, and run
local verification. GitHub mutations require explicit maintainer or user
approval, including pushes, PR comments, review-thread resolution, reviewer
requests, merges, and review re-triggers such as `@codex review`.

For automated review comments from bot accounts, make code fixes locally and do
not reply inline. After explicit approval to update the PR, push the fix
silently and use the approved top-level re-trigger path.

## Required Gates

Use the strongest gate that matches the work. At minimum, AI-surface changes
must pass:

```bash
pixi run lint-md
pixi run check-lint-md
pixi run sync-ai-commands
pixi run check-ai-commands
pixi run check-docs-policy
pixi run check-lint-spell
```

Before committing, DART still requires `pixi run lint`. Code changes require
the build and test gates listed in `AGENTS.md` and the relevant onboarding docs.
