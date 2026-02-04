---
description: Resolve a GitHub issue with proper fix and test
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-fix-issue.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Fix GitHub issue: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md

## Workflow

1. `gh issue view $1` - Validate issue
2. `git checkout -b fix/issue-$1 origin/main`
3. Fix with minimal changes + add regression test
4. `pixi run test-all && pixi run lint`
5. `git push -u origin HEAD && gh pr create`
6. **Before PR**: If task used `docs/dev_tasks/<task>/`, remove the folder (include in this PR, not after merge)

## CRITICAL: Dual-PR for Bug Fixes

PRs to BOTH `release-6.16` AND `main`. See `docs/onboarding/contributing.md`.
