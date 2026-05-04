---
name: dart-docs-update
description: "DART Docs Update: update documentation without code changes"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-docs-update.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-docs-update

Use this skill in Codex when you want the same workflow that Claude Code and
OpenCode expose as `/dart-docs-update`.

## Invocation

- Claude Code/OpenCode: `/dart-docs-update <arguments>`
- Codex: `$dart-docs-update <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Update documentation: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/README.md

## Workflow

1. `git checkout -b docs/<topic> origin/main`
2. Edit only: `docs/**`, `README.md`, `.claude/`
3. `git push -u origin HEAD && gh pr create`
