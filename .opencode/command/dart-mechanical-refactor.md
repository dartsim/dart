---
description: perform a behavior-preserving mechanical refactor
argument-hint: "<transformation-description>"
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-mechanical-refactor.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Perform mechanical refactor: $ARGUMENTS

## Required Reading

@AGENTS.md
@CONTRIBUTING.md
@docs/onboarding/code-style.md

## Workflow

1. Define the exact transformation and scope before editing.
2. Create a branch from the target branch, usually `origin/main`.
3. Prefer scriptable or automated edits when the transformation is repetitive.
4. Keep behavior unchanged; do not mix in feature work or cleanup outside scope.
5. If reorganizing files, update CMake, pixi tasks, generated indexes, and docs.
6. Run focused checks first, then broader checks according to risk:
   - `pixi run lint`
   - `pixi run build`
   - `pixi run test-unit`
   - `pixi run test-all` when feasible
   - `pixi run -e cuda test-all` when feasible on Linux CUDA hosts
7. Ask for explicit maintainer/user approval before pushing or opening a PR.
   After approval, open a PR with a clear scope statement and no
   behavior-change claim unless tested.

## Output

- Transformation summary
- Files or areas changed
- Verification run
- Any residual risk
