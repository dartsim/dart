---
description: monitor CI and merge a ready PR
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-merge-pr.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Monitor and merge PR: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/ci-cd.md
@docs/onboarding/contributing.md

## Workflow

1. Resolve PR number, repository, base branch, head branch, and head SHA.
2. Verify PR state:
   ```bash
   gh pr view <PR> --json state,isDraft,mergeable,mergeStateStatus,reviewDecision,headRefOid,title,url
   gh pr checks <PR> --json name,state,bucket,link,workflow
   ```
3. If any required check is pending, watch quietly until it passes or fails.
4. If any check fails, use `/dart-fix-ci` or `$dart-fix-ci`; do not merge.
5. Confirm merge method from repository settings. DART uses squash/rebase, not merge commits.
6. Merge only when checks are green, the PR is not draft, and GitHub reports it mergeable.
7. Use the current head SHA when merging so a moved branch cannot be merged accidentally.
8. For squash merges, use the recent DART title convention:
   - commit title: `<PR title> (#<PR number>)`
   - no agent prefix

## Notes

- If GitHub reports `BEHIND` but `mergeable=MERGEABLE` and required checks are green, do not update the branch just to make it current.
- Recent DART human-authored PRs use single-parent squash commits with the exact PR title plus `(#number)`.
- Branch deletion is handled by repo settings unless the user explicitly asks for manual cleanup.

## Output

- Final CI state
- Merge method and merge commit SHA
- Whether the PR is merged or what blocks it
