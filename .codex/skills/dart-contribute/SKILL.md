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
gh pr create --draft --milestone "DART 7.0"
```

Rule of thumb: run `pixi run lint` before committing so auto-fixes are included.

Use `.github/PULL_REQUEST_TEMPLATE.md` and ensure the PR description covers Summary, Motivation, Changes, Testing, Breaking Changes, and Related Issues.

## Milestones (Required)

Always set a milestone when creating PRs:

| Target Branch  | Milestone                     |
| -------------- | ----------------------------- |
| `main`         | `DART 7.0` (or next major)    |
| `release-6.16` | `DART 6.16.x` (current patch) |

```bash
# Set milestone on existing PR
gh pr edit <PR#> --milestone "DART 7.0"

# List available milestones
gh api repos/dartsim/dart/milestones --jq '.[] | .title'
```

## CRITICAL: Bug Fix Dual-PR

Bug fixes require PRs to **BOTH** branches:

1. **`release-6.16`** - Current release
2. **`main`** - Next release

Steps:

1. Fix on `release-6.16` first
2. Cherry-pick to `main`
3. Create separate PRs for each

## CHANGELOG (After PR Created)

After creating a PR, check if CHANGELOG.md needs updating:

| Change Type                      | Update CHANGELOG?                    |
| -------------------------------- | ------------------------------------ |
| Bug fixes                        | ✅ Yes                               |
| New features                     | ✅ Yes                               |
| Breaking changes                 | ✅ Yes (in Breaking Changes section) |
| Documentation improvements       | ✅ Yes (in Tooling and Docs)         |
| CI/tooling changes               | ✅ Yes (in Tooling and Docs)         |
| Refactoring (no behavior change) | ⚠️ Maybe (if significant)            |
| Dependency bumps                 | ⚠️ Maybe (if user-facing)            |
| Typo fixes                       | ❌ No                                |

Format: `- Description. ([#PR](https://github.com/dartsim/dart/pull/PR))`

```bash
# Example entry in CHANGELOG.md under appropriate section:
- Added AI-native documentation with AGENTS.md and module-specific guides. ([#2446](https://github.com/dartsim/dart/pull/2446))
```

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
