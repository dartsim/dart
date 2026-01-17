Start a new task in DART: $ARGUMENTS

## Required Reading

Read these files first:
@AGENTS.md
@docs/onboarding/building.md
@docs/onboarding/contributing.md
@docs/onboarding/code-style.md

## Workflow

1. **Understand the task** - Parse: goal, constraints, type (feature|bugfix|refactor|docs)
2. **Setup** - Create branch from `origin/main`: `feature/<topic>`, `fix/<topic>`, etc.
3. **Implement** - Keep commits focused, follow code style
4. **Verify** - `pixi run lint` then `pixi run test-all`
5. **PR** - `git push -u origin HEAD` then `gh pr create --draft`

## Type-Specific

- **Bugfix**: Requires PRs to BOTH `release-6.16` AND `main`
- **Refactor**: No behavior changes
- **Feature**: Add tests + docs
