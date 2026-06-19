---
description: backport a merged main PR to a release branch
agent: build
---

Backport PR or commits: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md
@docs/onboarding/release-management.md

## Workflow

1. Verify the source PR or commit is merged to `main`:
   ```bash
   gh pr view <SOURCE_PR> --json state,mergedAt,baseRefName,mergeCommit
   ```
2. Check whether an equivalent change already exists on the release branch:
   ```bash
   git fetch origin <RELEASE_BRANCH> main
   git cherry -v --abbrev=40 origin/<RELEASE_BRANCH> origin/main | grep <COMMIT_HASH>
   ```
3. Create a release branch from the release target:
   ```bash
   git switch --no-track -C backport/<SOURCE_PR>-to-<RELEASE_BRANCH> origin/<RELEASE_BRANCH>
   ```
4. Cherry-pick with provenance: `git cherry-pick -x <COMMIT_HASH>`.
5. Resolve conflicts minimally; stop and ask if conflicts are broad or change behavior.
6. Run `pixi run lint` and the smallest relevant release-branch checks.
7. Ask for explicit maintainer/user approval before pushing or opening the PR.
   After approval, open the PR against the release branch with milestone
   matching that release branch and use the PR template.

## Output

- Backport PR URL
- Source PR/commit
- Conflicts resolved, if any
- Checks run and CI status
