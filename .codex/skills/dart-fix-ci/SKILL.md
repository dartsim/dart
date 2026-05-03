---
name: dart-fix-ci
description: "DART Fix CI: debug and fix failing CI checks"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-fix-ci.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-fix-ci

Use this skill in Codex when you want the same workflow that Claude Code and
OpenCode expose as `/dart-fix-ci`.

## Invocation

- Claude Code/OpenCode: `/dart-fix-ci <arguments>`
- Codex: `$dart-fix-ci <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

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
