---
description: Start new feature/bugfix/refactor in DART
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-new-task.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Start a new task in DART: $ARGUMENTS

## Required Reading

Read these files first:
@AGENTS.md
@docs/onboarding/building.md
@docs/onboarding/contributing.md
@docs/onboarding/code-style.md

## Workflow

1. **Understand the task** - Parse: goal, constraints, type (feature|bugfix|refactor|docs)
2. **Assess scope** - Multi-phase or multi-session? Create `docs/dev_tasks/<task>/` (see `docs/dev_tasks/README.md` for criteria)
3. **Setup** - Create branch from `origin/main`: `feature/<topic>`, `fix/<topic>`, etc.
4. **Implement** - Keep commits focused, follow code style
5. **Verify** - Run `pixi run lint` before committing, then `pixi run test-all`
6. **PR** - `git push -u origin HEAD` then `gh pr create --draft --milestone "DART 7.0"` (use `DART 6.16.x` for release-6.16); follow `.github/PULL_REQUEST_TEMPLATE.md`
7. **Cleanup** - Before PR: if task used `docs/dev_tasks/<task>/`, remove the folder (include in this PR, not after merge)

## Type-Specific

- **Bugfix**: Requires PRs to BOTH `release-6.16` AND `main`
- **Refactor**: No behavior changes
- **Feature**: Add tests + docs
