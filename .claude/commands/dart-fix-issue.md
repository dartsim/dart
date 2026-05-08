---
description: resolve a GitHub issue with a fix and regression test
agent: build
---

Fix GitHub issue: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md

## Workflow

1. `gh issue view $1` - Validate issue
2. `git checkout -b fix/issue-$1 origin/main`
3. Fix with minimal changes + add regression test
4. `pixi run lint`, then run the smallest relevant tests; use `pixi run test-all` before finalizing when feasible
5. `git push -u origin HEAD && gh pr create`
6. **Before PR**: If task used `docs/dev_tasks/<task>/`, remove the folder (include in this PR, not after merge)

## CRITICAL: Dual-PR for Bug Fixes

PRs to BOTH `release-6.16` AND `main`. See `docs/onboarding/contributing.md`.
