---
description: backport a merged main PR to a release branch
argument-hint: "<source-pr> base=<release-branch>"
agent: build
---

Backport PR or commits: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md
@docs/onboarding/release-management.md
@docs/onboarding/changelog.md

## Workflow

For a source change involving model/scene structure, physics behavior, or GUI
output, use the target branch's `dart-verify-sim` workflow to preserve the text
oracle and assessed visual evidence. Document a visual exception when the
target branch cannot render the claim.

1. Verify the source PR or commit is merged to `main`:
   ```bash
   gh pr view <SOURCE_PR> --json state,mergedAt,baseRefName,mergeCommit
   ```
2. Check whether an equivalent change already exists on the release branch:
   ```bash
   git fetch origin <RELEASE_BRANCH> main
   git cherry -v --abbrev=40 origin/<RELEASE_BRANCH> origin/main | grep <COMMIT_HASH>
   ```
3. For AI-infra or workflow-doc backports, compare the release branch capability
   inventory and adapter directories against `main`. If the release branch has a
   smaller workflow surface, adapt to the release branch instead of importing
   main-only workflows.
4. Create a release branch from the release target:
   ```bash
   git checkout -B backport/<SOURCE_PR>-to-<RELEASE_BRANCH> origin/<RELEASE_BRANCH>
   ```
5. Cherry-pick with provenance: `git cherry-pick -x <COMMIT_HASH>`.
6. Resolve conflicts minimally; stop and ask if conflicts are broad or change behavior.
7. Run the `dart-changelog` routine for the release-target decision before
   opening the backport PR.
8. Run `pixi run lint` and the smallest relevant release-branch checks.
9. Ask for explicit maintainer/user approval before pushing or opening the PR.
   After approval, open the PR against the release branch with milestone
   matching that release branch and use the PR template.

## Output

- Backport PR URL
- Source PR/commit
- Conflicts resolved, if any
- Changelog decision
- Checks run and CI status
