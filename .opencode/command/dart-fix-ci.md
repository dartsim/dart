---
description: Debug and fix failing CI checks
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-fix-ci.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Fix CI failure: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/ci-cd.md

## Workflow

1. `gh pr checks $1` or `gh run view <RUN_ID>`
2. `gh run view <RUN_ID> --log-failed`
3. Reproduce: `pixi run test` or `ctest -R <TEST>`
4. Fix (formatting: `pixi run lint`, infra: `gh run rerun --failed`)
5. Push and `gh run watch <RUN_ID> --interval 30`
