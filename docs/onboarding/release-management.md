# DART Release Management

This guide collects release workflows that are too detailed for a command file.
Use it with the release workflow sources in `.claude/commands/`:

| Command                    | Use Case                                                   |
| -------------------------- | ---------------------------------------------------------- |
| `/dart-backport-pr`        | Port a merged `main` fix to a release branch               |
| `/dart-fix-ci`             | Fix CI failures on any base branch, incl. release branches |
| `/dart-release-merge-main` | Merge the active release branch back to main               |
| `/dart-release-packaging`  | Prepare a release version bump and changelog               |

Codex exposes the same workflows as generated `$dart-*` skill adapters from
those sources. OpenCode receives generated command adapters through
`.opencode/command/`.

## Branches and Milestones

| Target Branch / Scope                              | Milestone                                  |
| -------------------------------------------------- | ------------------------------------------ |
| `main`                                             | `DART 7.0`                                 |
| Active DART 6 LTS branch, currently `release-6.20` | Branch-matching DART 6.x release milestone |

Bug fixes that apply to released users need two PRs: one for the active release
branch and one for `main`. For DART 6 LTS fixes, target the active
`release-6.*` branch (the highest maintained DART 6 release branch on the
remote; currently `release-6.20`) rather than an older minor line. If the fix
already exists on `main`, backport with `git cherry-pick -x` so the release
commit records its source.

## Backport Checks

Before opening a backport PR:

```bash
gh pr view <SOURCE_PR> --json state,mergedAt,baseRefName,mergeCommit
git fetch origin <RELEASE_BRANCH> main
git cherry -v --abbrev=40 origin/<RELEASE_BRANCH> origin/main | grep <COMMIT_HASH>
```

`git cherry` output uses `+` for a commit that still needs backporting and `-`
for an equivalent patch already present on the release branch.

Backport branches should start from the release branch:

```bash
git checkout -B backport/<SOURCE_PR>-to-<RELEASE_BRANCH> origin/<RELEASE_BRANCH>
git cherry-pick -x <COMMIT_HASH>
```

For merge commits, use `git cherry-pick -x -m 1 <MERGE_COMMIT_HASH>`.

AI-infra and workflow-doc backports need one extra check before cherry-picking.
Compare the release branch's `docs/ai/capabilities.json`, `docs/ai/workflows.md`,
`.claude/commands/`, `.claude/skills/`, `.codex/skills/`, and
`.opencode/command/` against `main`. If the release branch intentionally has a
smaller workflow surface, adapt the guidance to the release owners and
regenerate only the affected adapters; do not add main-only workflows just to
make the patch apply. Record a changelog entry when the backport changes
contributor or agent behavior.

## Release Branch CI Fixes

When release-branch CI fails:

1. Inspect the failing run and job logs.
2. Search `main` for an existing fix before writing a new one.
3. Prefer cherry-picking proven fixes from `main`.
4. If the release branch needs a unique fix, keep it minimal and document why.
5. Run `pixi run lint` before committing and run the smallest relevant tests.
6. Open or update the release-branch PR and monitor CI until green.

Useful commands:

```bash
gh run view <RUN_ID> --log-failed
gh run view <RUN_ID> --job <JOB_ID> --log
git log origin/main --oneline --grep="<ERROR_KEYWORD>" -10
```

## Release Packaging

Release packaging PRs bump version metadata and prepare the changelog. Use
[changelog.md](changelog.md) for entry structure, release-summary cleanup, and
the release audit.

Derive values from the new version:

```bash
NEW_VERSION=<VERSION>
BASE_BRANCH="release-$(echo "$NEW_VERSION" | cut -d. -f1-2)"
RELEASE_DATE=$(date +%Y-%m-%d)
```

Typical files to check:

- `package.xml`
- `pixi.toml`
- `CHANGELOG.md`
- examples or tutorials with `find_package(DART ...)` version requirements

Commit title convention:

```text
Packaging <NEW_VERSION>
```

Create the PR against the release branch and set the matching release milestone.

## Merging Release Branch Into Main

Use this after publishing a release to align `main` with the active release
branch. This is a forward merge from release to `main`, not a backport.

Auto-detect the active release branch:

```bash
git fetch origin
RELEASE_BRANCH=$(git branch -r | grep -oE 'origin/release-[0-9]+\.[0-9]+' | sort -V | tail -1 | sed 's|origin/||')
RELEASE_VERSION=$(git show origin/$RELEASE_BRANCH:package.xml | grep -oP '(?<=<version>)[^<]+')
```

Verify the repository has enough history:

```bash
test "$(git rev-parse --is-shallow-repository)" = "false"
git merge-base origin/main origin/$RELEASE_BRANCH
```

Create the merge branch:

```bash
git checkout -B merge/$RELEASE_BRANCH-into-main origin/main
git merge origin/$RELEASE_BRANCH -m "Merge $RELEASE_BRANCH into main (v$RELEASE_VERSION)"
```

### Conflict Strategy

| Conflict Type                       | Default Resolution                    | Reason                          |
| ----------------------------------- | ------------------------------------- | ------------------------------- |
| Deleted in main, updated in release | Keep deleted                          | DART 7 intentionally removed it |
| Added in both branches              | Prefer main unless release is unique  | Main has DART 7-compatible form |
| Content conflicts                   | Prefer main modernization             | Main carries C++23/DART 7 work  |
| `CHANGELOG.md`                      | Review manually                       | Avoid losing release notes      |
| Files only in release               | Keep only if still relevant to `main` | Some release-only files differ  |

Useful conflict helpers:

```bash
# Keep files deleted on main.
git status --porcelain | grep "^DU" | sed 's/^DU //' | xargs -r git rm -f

# Prefer main for add/add conflicts.
git status --porcelain | grep "^AA" | sed 's/^AA //' | while read -r file; do
  git checkout --ours "$file" && git add "$file"
done

# Prefer main for content conflicts except CHANGELOG.md.
git status --porcelain | grep "^UU" | sed 's/^UU //' | grep -v CHANGELOG.md | while read -r file; do
  git checkout --ours "$file" && git add "$file"
done
```

For `CHANGELOG.md`, compare both sides. If a fix appears in both the DART 7 and
release sections, keep the DART 7 entry on `main`. If a fix exists only in the
release section and also applies to `main`, add it to the DART 7 section.
Release packaging entries usually stay only on the release branch.

Verify all conflicts are resolved:

```bash
git diff --name-only --diff-filter=U
```

Then commit, push, and open a PR targeting `main` with milestone `DART 7.0`.

## Reference PRs

- [#2487 - Merge release-6.16 into main (v6.16.5)](https://github.com/dartsim/dart/pull/2487)
- [#2411 - Merge release-6.16 into main (v6.16.4)](https://github.com/dartsim/dart/pull/2411)
