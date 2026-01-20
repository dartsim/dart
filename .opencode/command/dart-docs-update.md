---
description: Update documentation (no code changes)
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-docs-update.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Update documentation: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/README.md

## Workflow

1. `git checkout -b docs/<topic> origin/main`
2. Edit only: `docs/**`, `README.md`, `.claude/`
3. `git push -u origin HEAD && gh pr create`
