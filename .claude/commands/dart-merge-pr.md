---
description: monitor CI and merge a ready PR
agent: build
---

Monitor and merge PR after explicit maintainer/user approval: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/ci-cd.md
@docs/onboarding/contributing.md
@docs/onboarding/testing.md

## Workflow

1. Resolve PR number, repository, base branch, head branch, and head SHA.
2. Verify PR state:
   ```bash
   gh pr view <PR> --json state,isDraft,mergeable,mergeStateStatus,reviewDecision,headRefOid,title,url
   gh pr checks <PR> --json name,state,bucket,link,workflow
   ```
3. Run local pre-merge validation on the current head after the latest pushed
   change: `pixi run test-all` and `pixi run -e cuda test-all`. Do not
   substitute the default-environment run for the CUDA-environment run; if CUDA
   cannot execute, record that command's skip or blocker explicitly.
4. If any required check is pending, watch quietly until it passes or fails.
5. If any local validation or hosted check fails, use `/dart-fix-ci` or
   `$dart-fix-ci`; do not merge.
6. Confirm merge method from repository settings. DART uses squash/rebase, not merge commits.
7. Ask for explicit maintainer/user approval before any merge action.
8. Merge only after that approval, when local validation and hosted checks are green, the PR is not draft,
   and GitHub reports it mergeable.
9. Use the current head SHA when merging so a moved branch cannot be merged accidentally.
10. For squash merges, use the recent DART title convention:

- commit title: `<PR title> (#<PR number>)`
- no agent prefix

## Notes

- If GitHub reports `BEHIND` but `mergeable=MERGEABLE` and required checks are
  green, do not update the branch without explicit maintainer/user approval
  just to make it current.
- Recent DART human-authored PRs use single-parent squash commits with the exact PR title plus `(#number)`.
- Branch deletion is handled by repo settings unless the user explicitly asks for manual cleanup.

## Output

- Local validation state for `pixi run test-all` and `pixi run -e cuda test-all`
- Final CI state
- Merge method and merge commit SHA
- Whether the PR is merged or what blocks it
