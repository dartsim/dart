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

## CRITICAL: Dual-PR for Bug Fixes

PRs to BOTH `release-6.16` AND `main`. See `docs/onboarding/contributing.md`.
