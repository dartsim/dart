---
description: debug and fix CI failures on a release branch
argument-hint: "<pr-number|run-id> [release-branch]"
agent: build
---

Fix release-branch CI: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/ci-cd.md
@docs/onboarding/release-management.md

## Workflow

1. Inspect the failing run:
   ```bash
   gh run view <RUN_ID> --log-failed
   gh run view <RUN_ID> --job <JOB_ID> --log
   ```
2. Check whether an equivalent fix already exists on `main`.
3. If continuing an existing PR, fetch and checkout that branch. Otherwise
   branch from the release branch without resetting an existing local branch:
   ```bash
   git fetch origin <RELEASE_BRANCH>
   BRANCH=fix/<issue>-<release-branch>
   if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
     git switch "$BRANCH"
   else
     git switch --no-track -c "$BRANCH" origin/<RELEASE_BRANCH>
   fi
   ```
4. Prefer cherry-picking a proven `main` fix. If a new fix is required, keep it release-scoped and minimal.
5. Explain why the failure was not caught earlier and whether workflow coverage should change.
6. Run `pixi run lint` and release-relevant build/tests.
7. Ask for explicit maintainer/user approval before pushing, creating, or
   updating the release-branch PR; after approval, use the current release
   milestone and PR template.
8. Monitor CI until green.

## Output

- Root cause
- Fix summary
- PR URL
- CI status
- Prevention recommendation, if any
