---
name: dart-contribute
description: "DART Contribute: branching, PRs, review workflow, and dual-PR bugfixes"
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
# DART 6.20 maintenance work starts from the release branch without tracking it
git fetch origin release-6.20
git switch --no-track -c <type>/<topic> origin/release-6.20

# Bug fixes that apply to the current release line start from the active DART 6 LTS branch
DART6_LTS_BRANCH=$(git branch -r --list 'origin/release-6.*' | sed 's|.*/||' | sort -V | tail -1)
git switch --no-track -c "fix/<topic>-${DART6_LTS_BRANCH#release-}" "origin/$DART6_LTS_BRANCH"

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

Use the branch-matching DART 6.x patch milestone for release-branch PRs. Use
`--base main --milestone "DART 7.0"` only for the separate DART 7 companion PR
when a bug fix also applies to `main`.

Rule of thumb: run `pixi run lint` before committing so auto-fixes are included.

Use `.github/PULL_REQUEST_TEMPLATE.md` and keep DART's default order: Summary, Motivation / Problem, Changes / Key Changes, optional Before / After, Testing, Breaking Changes, and Related Issues / PRs. Keep Summary first as the reviewer skim target. If the motivation is necessary to understand the outcome, make the first Summary sentence problem-oriented, then put the fuller why in Motivation / Problem rather than moving Motivation above Summary.

Write PR descriptions for a user or downstream maintainer who is not already familiar with the implementation. Lead Summary and Motivation with what changes for them, what stays compatible, how they opt in or migrate, and why the evidence matters; keep implementation mechanics in Changes unless they explain user-visible risk.

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

| Target Branch                          | Milestone                      |
| -------------------------------------- | ------------------------------ |
| `main`                                 | `DART 7.0` (or next major)     |
| Active DART 6 LTS `release-6.*` branch | Branch-matching DART 6.x patch |

```bash
# After explicit maintainer/user approval, set milestone on existing PR
gh pr edit <PR#> --milestone "DART 7.0"

# List available milestones
gh api repos/dartsim/dart/milestones --jq '.[] | .title'
```

## CRITICAL: Bug Fix Dual-PR

Bug fixes require PRs to **BOTH** release lines:

1. **Active DART 6 LTS `release-6.*` branch** - Current DART 6 maintenance line
2. **`main`** - Next release

Steps:

1. Fix on the active DART 6 LTS branch first
2. Cherry-pick to `main`
3. After explicit maintainer/user approval, create separate PRs for each

## CHANGELOG (After Approved PR Exists)

Use `docs/onboarding/changelog.md` as the source of truth. After the approved PR
exists, check if `CHANGELOG.md` needs updating:

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

Format: `- Reader-visible outcome. ([#PR](https://github.com/dartsim/dart/pull/PR))`

Keep entries concise. If details need more than a few wrapped lines, move the
details to the owner doc, plan, or migration note and link that document.
Do not add one bullet per PR when several PRs ship one reader-visible outcome;
merge them into one human-readable release-note entry.

```bash
# Example entry in CHANGELOG.md under appropriate section:
- Added AI-native documentation with AGENTS.md and module-specific guides. ([#2446](https://github.com/dartsim/dart/pull/2446))
```

## Code Review

- Address all feedback
- Keep changes minimal
- Update tests if behavior changed
- Run full validation, then ask for explicit maintainer/user approval before
  pushing fixes

## CI Loop

```bash
gh run watch <RUN_ID> --interval 30
```

Fix failures until green.
