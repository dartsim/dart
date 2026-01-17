---
name: dart-contribute
description: DART contribution workflow - branching, PRs, code review, dual-PR for bugfixes
---

# DART Contribution Workflow

Load this skill when contributing code to DART.

## Full Documentation

For complete guide: `docs/onboarding/contributing.md`

For code style: `docs/onboarding/code-style.md`

## Branch Naming

- `feature/<topic>` - New features
- `fix/<topic>` - Bug fixes
- `refactor/<topic>` - Refactoring
- `docs/<topic>` - Documentation

## PR Workflow

```bash
# Create branch
git checkout -b <type>/<topic> origin/main

# Make changes, then
pixi run lint
pixi run test-all

# Push and create PR
git push -u origin HEAD
gh pr create --draft
```

## CRITICAL: Bug Fix Dual-PR

Bug fixes require PRs to **BOTH** branches:

1. **`release-6.16`** - Current release
2. **`main`** - Next release

Steps:

1. Fix on `main` first
2. Cherry-pick to `release-6.16`
3. Create separate PRs for each

## Code Review

- Address all feedback
- Keep changes minimal
- Update tests if behavior changed
- Run full validation before pushing fixes

## CI Loop

```bash
gh run watch <RUN_ID> --interval 30
```

Fix failures until green.
