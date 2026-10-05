---
name: dart-contribute
description: "DART Contribute: branching, PRs, and review workflow"
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

## Branch Naming

- `feature/<topic>` - New features
- `fix/<topic>` - Bug fixes
- `refactor/<topic>` - Refactoring
- `docs/<topic>` - Documentation

## PR Workflow

```bash
# DART 6.20 development and bug fixes start from main without tracking it
git fetch origin main
git switch --no-track -c <type>/<topic> origin/main

# Make changes, then
pixi run lint
pixi run test-all
# For package, collision, constraint, dependency, or downstream compatibility changes
pixi run -e gazebo test-gz

# After explicit maintainer/user approval, push and create PR
branch=$(git branch --show-current)
git push -u origin "HEAD:${branch}"
gh pr create --draft --base <target-branch> --milestone "<milestone>"
```

Use the next DART 6.x release milestone for `main` PRs (currently
`DART 6.20.0`) and the branch-matching milestone for maintenance PRs.

Rule of thumb: run `pixi run lint` before committing so auto-fixes are included.

Use `.github/PULL_REQUEST_TEMPLATE.md` and keep DART's default order: Summary, Motivation / Problem, Changes / Key Changes, optional Before / After, Testing, Breaking Changes, and Related Issues / PRs. Keep Summary first as the reviewer skim target. If the motivation is necessary to understand the outcome, make the first Summary sentence problem-oriented, then put the fuller why in Motivation / Problem rather than moving Motivation above Summary.

Write PR descriptions for a user or downstream maintainer who is not already familiar with the implementation. Lead Summary and Motivation with what changes for them, what stays compatible, how they opt in or migrate, and why the evidence matters; keep implementation mechanics in Changes unless they explain user-visible risk.

Write the body for a human skimming it: bullets and highlights only; state the mechanism in one sentence and leave further detail to the code, a design doc, or the linked issue. In Testing, list only checks that CI does not run on the PR (a reproduction with a reporter's toolchain, a negative check that proves a new test bites, hardware-specific runs, independent review passes), not the lint, unit, gate, or platform jobs that the PR's CI runs anyway.

When a PR has meaningful user-facing API, workflow, behavior, or performance impact, add a concise Before / After section. Cover only relevant dimensions, phrase rows as user-visible before/after outcomes, and for performance claims name the baseline explicitly: CPU path, parent commit, `main`, or prior implementation, plus workload, metric, and important limitations.

Use plain descriptive commit messages and PR titles. Do not prefix them with agent tags such as `[codex]`, `[claude]`, or `[opencode]`.

For already-published PRs, keep history inspectable with additive commits. If
the PR branch needs the latest target branch, use explicit maintainer/user
approval to update that published branch by merging the target branch and
pushing normally. Do not rebase published PR branches by default because that
invalidates existing CI runs and makes PR review/comment history harder to
follow. Rebase or force-push only when the maintainer explicitly requests it.

## Milestones (Required)

Always set a milestone when creating PRs after explicit maintainer/user
approval:

| Target Branch                    | Milestone                                       |
| -------------------------------- | ----------------------------------------------- |
| `main`                           | Next DART 6.x release (currently `DART 6.20.0`) |
| Maintenance `release-6.*` branch | Branch-matching DART 6.x release                |

```bash
# After explicit maintainer/user approval, set milestone on existing PR
gh pr edit <PR#> --milestone "DART 6.20.0"

# List available milestones
gh api repos/dartsim/dart/milestones --jq '.[] | .title'
```

## Bug Fixes

After explicit maintainer/user approval, open bug-fix PRs against `main`, the
development branch for the next release (currently DART 6.20). Backports to
`release-6.19`, the maintenance branch, use `dart-backport-pr`.

## CHANGELOG (After Approved PR Exists)

Use `docs/onboarding/changelog.md` as the source of truth. `CHANGELOG.md` is
written for users of the released library and packages, so keep each entry to
what the reader must know or do. After the approved PR exists, check if
`CHANGELOG.md` needs updating:

| Change Type                      | Update CHANGELOG?                    |
| -------------------------------- | ------------------------------------ |
| Bug fixes                        | ✅ Yes                               |
| New features                     | ✅ Yes                               |
| Breaking changes                 | ✅ Yes (in Breaking Changes section) |
| Documentation improvements       | ❌ No, unless a user-run workflow changes |
| CI/tooling/AI-harness changes    | ❌ No, unless a user-run workflow changes |
| Refactoring (no behavior change) | ⚠️ Maybe (if significant)            |
| Dependency bumps                 | ⚠️ Maybe (if user-facing)            |
| Typo fixes                       | ❌ No                                |

Format: match the file's established style — `*` bullets nested under the
category bullet, with the bare PR link on its own line at the end:

```markdown
* Category

  * Reader-visible outcome, stated for release readers:
    [#2446](https://github.com/dartsim/dart/pull/2446)
```

Keep entries concise. If details need more than a few wrapped lines, move the
details to the owner doc, plan, or migration note and link that document.
Do not add one bullet per PR when several PRs ship one reader-visible outcome;
merge them into one human-readable release-note entry.

## Code Review

- Investigate each review finding; fix it, track it as a follow-up, or record
  a no-fix rationale (`docs/ai/orchestration.md` § Review Loop)
- Keep changes minimal
- Update tests if behavior changed
- Run full validation, then ask for explicit maintainer/user approval before
  pushing fixes

## CI Loop

```bash
gh run watch <RUN_ID> --interval 30
```

Fix failures until green.
