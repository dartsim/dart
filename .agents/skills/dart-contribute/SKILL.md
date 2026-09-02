---
name: dart-contribute
description: "DART Contribute: branching, PRs, review workflow, and dual-PR bugfixes"
---

<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/skills/dart-contribute/SKILL.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# DART Contribution Workflow

Load this skill when contributing code to DART.

## Full Documentation

For complete guide: `docs/onboarding/contributing.md`

For code style: `docs/onboarding/code-style.md`

For automated review handling: `docs/onboarding/ai-reviews.md`

## Branch Naming

- `feature/<topic>` - New features
- `fix/<topic>` - Bug fixes
- `refactor/<topic>` - Refactoring
- `docs/<topic>` - Documentation

Features, docs, and non-bugfix refactors branch from `origin/main`. Bug fixes
that apply to the current release line branch from the active DART 6 LTS
`origin/release-6.*` branch first and then reach `main` as a second PR (the
dual-PR rule in `docs/onboarding/contributing.md`).

## PR Path

Run `pixi run lint` before committing and the task-type gate set from
`docs/ai/verification.md`; use `dart-pr` to commit, push, and open the PR after
explicit maintainer/user approval, and `dart-changelog` to decide the
`CHANGELOG.md` entry per `docs/onboarding/changelog.md`. Use plain descriptive
commit messages and PR titles without agent tags. Update published PR branches
with additive commits after merging the target branch; never rebase or
force-push a published PR branch unless the maintainer explicitly asks.

## Milestones (Required)

| Target Branch                          | Milestone                      |
| -------------------------------------- | ------------------------------ |
| `main`                                 | `DART 7.0` (or next major)     |
| Active DART 6 LTS `release-6.*` branch | Branch-matching DART 6.x patch |

```bash
# After explicit maintainer/user approval, set milestone on an existing PR
gh pr edit <PR#> --milestone "DART 7.0"

# List available milestones
gh api repos/dartsim/dart/milestones --jq '.[] | .title'
```
