---
name: dart-improve-docs
description: "DART Improve Docs: capture completed-task learnings into durable documentation"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-improve-docs.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-improve-docs

Use this skill in Codex when you want the same workflow that Claude Code and
OpenCode expose as `/dart-improve-docs`.

## Invocation

- Claude Code/OpenCode: `/dart-improve-docs <arguments>`
- Codex: `$dart-improve-docs <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Improve docs from completed work: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/AGENTS.md
@docs/onboarding/ai-tools.md

## Skip If

Stop without editing when all are true:

- no new patterns, gotchas, or failure modes were discovered
- no workflow improvement is needed
- existing docs already cover what happened
- the task was routine and left no durable insight

## Routing

- High-level module guidance: module `AGENTS.md`
- Detailed implementation, debugging, or workflow guidance: `docs/onboarding/*.md`
- Repeatable workflow: `.claude/commands/`
- On-demand domain knowledge: `.claude/skills/`
- AI tool compatibility: `docs/onboarding/ai-tools.md`

`AGENTS.md` files are pointer boards. Keep detailed examples and gotchas in onboarding docs.

## Workflow

1. Search existing docs before adding content.
2. Prefer update, remove, consolidate, or restructure over adding new files.
3. Avoid ephemeral branch, PR, commit, or username details.
4. If adding a workflow command or skill, edit `.claude/` source files and run `pixi run sync-ai-commands`.
5. Run `pixi run lint`; for AI workflow changes, also run `pixi run check-ai-commands`.

## Output

- What learning was captured
- Where it was documented
- Checks run
