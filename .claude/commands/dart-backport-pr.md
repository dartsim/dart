---
description: backport a merged main PR to a release branch
argument-hint: "<pr-number> [release-branch]"
agent: build
---

Backport PR or commits: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md
@docs/onboarding/release-management.md
@docs/onboarding/changelog.md

## Workflow

For a source change involving model/scene structure, physics behavior, or OSG
output, use the release branch's `dart-verify-sim` workflow to preserve the text
oracle and assessed visual evidence. Document a visual exception when the
release branch cannot render the claim.

1. Verify the source PR or commit is merged to `main`:
   ```bash
   gh pr view <SOURCE_PR> --json state,mergedAt,baseRefName,mergeCommit
   ```
2. Check whether an equivalent change already exists on the release branch:
   ```bash
   git fetch origin <RELEASE_BRANCH> main
   git cherry -v --abbrev=40 origin/<RELEASE_BRANCH> origin/main | grep <COMMIT_HASH>
   ```
3. Create a release branch from the release target without resetting an
   existing local branch:
   ```bash
   BRANCH=backport/<SOURCE_PR>-to-<RELEASE_BRANCH>
   if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
     git switch "$BRANCH"
   else
     git switch --no-track -c "$BRANCH" origin/<RELEASE_BRANCH>
   fi
   ```
4. Cherry-pick with provenance: `git cherry-pick -x <COMMIT_HASH>`.
5. Resolve conflicts minimally; stop and ask if conflicts are broad or change behavior.
6. Run `/dart-changelog decide` or `$dart-changelog decide` against the
   backport diff and release target before opening the release PR. If an entry
   is required but needs the backport PR number, draft the decision and keep the
   finalize/update follow-up local until explicit approval permits another
   push. Do not skip the changelog decision just because this is a backport.
7. Run `pixi run lint` and the smallest relevant release-branch checks.
8. Ask for explicit maintainer/user approval before pushing or opening the PR.
   After approval, open the PR against the release branch with milestone
   matching that release branch and use the PR template. If the changelog
   decision was deferred for the PR number, run `/dart-changelog finalize` or
   `$dart-changelog finalize` and push the follow-up only after explicit
   approval.

## Output

- Backport PR URL
- Source PR/commit
- Conflicts resolved, if any
- Changelog decision and any deferred finalize follow-up
- Checks run and CI status
