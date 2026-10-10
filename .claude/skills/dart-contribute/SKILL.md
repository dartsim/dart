---
name: dart-contribute
description: "DART Contribute: branching, PRs, and review workflow"
---

# DART Contribution Workflow

Load this skill when contributing code to DART.

## Full Documentation

For complete guide: `docs/onboarding/contributing.md`

For code style: `docs/onboarding/code-style.md`

For target branches and milestones: `docs/onboarding/release-management.md`
§ "Release target" on the freshly fetched PR base.

## Branch Naming

- `feature/<topic>` - New features
- `fix/<topic>` - Bug fixes
- `refactor/<topic>` - Refactoring
- `docs/<topic>` - Documentation

## PR Workflow

Resolve the target and required milestone below before publishing a PR.

```bash
# Development and new fixes start from main without tracking it
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
# Prepare the plain title and filled template, then require both checks to pass
printf '%s\n' "$pr_title" | pixi run python scripts/check_local_paths.py --stdin
pixi run python scripts/check_local_paths.py --text-file "$pr_body_file"
gh pr create --draft --base <target-branch> --milestone "<milestone>" \
  --title "$pr_title" --body-file "$pr_body_file"
```

Then follow `docs/onboarding/ai-tools.md` § "PR Lifecycle" from draft to merge.

Rule of thumb: run `pixi run lint` before committing so auto-fixes are included.

Follow `docs/onboarding/contributing.md` § "PR Descriptions" for the
Effect-first template order, comparisons, plots, collapsed raw tables,
simulation evidence, and Testing content.

Commit messages are checked through the managed `commit-msg` hook. The PR
Text workflow is the backstop for commits that use `--no-verify` or otherwise
skip local hooks.

Use plain descriptive commit messages and PR titles. Do not prefix them with agent tags such as `[codex]`, `[claude]`, or `[opencode]`.

Describe private plans in prose or link public PRs/issues; keep private, local,
and machine-specific paths out of published text. Before every `gh pr create`
or `gh pr edit`, run the title and body checks above on the exact proposed
text (including retained text for edits). Both must exit 0 before publication.

For already-published PRs, keep history inspectable with additive commits. If
the PR branch needs the latest target branch, use explicit maintainer/user
approval to update that published branch by merging the target branch and
pushing normally. Do not rebase published PR branches by default because that
invalidates existing CI runs and makes PR review/comment history harder to
follow. Rebase or force-push only when the maintainer explicitly requests it.

## Milestones (Required)

Resolve the target from the live PR base, or use `main` for new development.
Fetch that base and read its `docs/onboarding/release-management.md`
§ "Release target". After explicit maintainer/user approval, set
`DART <Next release>` only after confirming that exact milestone is open on
GitHub. Use the full target version from the table;
package versions, branch minors, and the newest milestone do not determine it.

```bash
# Inspect the fetched base's release-target table
git show origin/<target-branch>:docs/onboarding/release-management.md

# List all open milestones, then verify the exact resolved title
gh api --paginate 'repos/dartsim/dart/milestones?state=open' --jq '.[] | .title'

# After explicit maintainer/user approval, set the resolved milestone
gh pr edit <PR#> --milestone "<milestone>"
```

## Bug Fixes

After explicit maintainer/user approval, open bug-fix PRs against `main`.
Backports to a maintained `release-6.*` branch use `dart-backport-pr`.
Release-specific packaging, CI, and branch guidance may target that branch
directly. Release branches may be cut from approved stabilization commits
before the first release tag or from published release tags.

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
