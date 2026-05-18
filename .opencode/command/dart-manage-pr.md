---
description: manage an open DART pull request through CI, review, and cleanup
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-manage-pr.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Manage an open DART pull request after explicit maintainer/user approval for
mutations: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md
@docs/onboarding/ci-cd.md
@docs/onboarding/ai-tools.md

## Identify the PR

Use the PR number or URL from `$ARGUMENTS`. If none is provided, infer the PR
from the current branch:

```bash
gh pr view --json number,url,headRefName,baseRefName
```

Then inspect the full state:

```bash
gh pr view <PR_NUMBER> --json number,title,state,isDraft,baseRefName,headRefName,mergeStateStatus,milestone,url,reviewDecision,statusCheckRollup
gh pr checks <PR_NUMBER>
```

## Workflow

1. Confirm scope and policy:
   - Check that the base branch, milestone, title, and PR template are correct.
   - For bug fixes, verify the required `release-6.16` and `main` dual-PR flow.
   - Inspect local state before editing:
     ```bash
     git status --short --branch
     git diff --stat
     git diff --check
     ```
2. Monitor CI:
   ```bash
   gh pr checks <PR_NUMBER> --watch --interval 30 --fail-fast
   ```
   If checks are still queued or running, report the current jobs and keep
   watching unless the user asked only for status.
3. Fix failures:
   - Inspect the newest failed run or job, not an older cancelled run.
   - Use the `dart-fix-ci` workflow for non-trivial CI debugging.
   - Reproduce locally with the relevant `pixi run ...` task or focused test.
   - Before committing fixes, run `pixi run lint`; also run build or tests when
     code or behavior changed.
   - Commit only intended files. Push only after explicit maintainer/user
     approval, then continue monitoring the PR.
   - For already-published PRs, prefer additive follow-up commits so reviewers
     can inspect each update. Amend or force-push only after explicit
     maintainer/user approval and only when the user explicitly requests it or
     when there is a clear reason such as removing sensitive content or
     repairing broken branch history.
4. Address reviews:
   - Use the `dart-review-pr` workflow for substantive review feedback.
   - Never reply to AI-generated review comments from bot users such as
     `chatgpt-codex-connector[bot]`, `github-actions[bot]`, or `copilot[bot]`.
   - When a draft PR is first marked ready for review, Codex review is expected
     to start automatically. Before posting `@codex review`, wait a reasonable
     time for the PR-body Codex activity indicator or a submitted Codex review.
   - Apply AI-review fixes silently. After explicit maintainer/user approval
     and after the branch is ready, push, resolve reviewed and addressed
     threads, and request a fresh AI review only when the automatic first review
     did not appear after a reasonable wait, or when the approved follow-up push
     addressed Codex review comments:
     ```bash
     gh pr comment <PR_NUMBER> --body "@codex review"
     ```
   - For human reviewers, reply only when a response is useful after a fix or
     when a question needs clarification.
5. Mark ready or merge only when appropriate:
   - Confirm required checks are passing and review requirements are satisfied.
   - If the PR is draft and ready, mark it ready only when the user or task asks.
   - Do not merge unless explicitly asked or the workflow clearly includes merge.
   - PR comments, review re-triggers, thread resolution, reviewer requests,
     ready-for-review transitions, merges, and branch deletion are external
     mutations and require explicit maintainer/user approval.
   - Confirm the merge method from repository settings or the user. Recent DART
     `main` PRs use single-parent PR-title commits, so prefer squash/rebase
     over merge commits unless the repository settings or user request differ.
   - Use the current head SHA when merging so a moved branch cannot be merged
     accidentally.
6. Clean up after merge:
   - Confirm the PR merged and identify the head branch before deleting.
   - After explicit maintainer/user approval, prefer merge-time deletion with
     the approved merge method and head SHA:
     ```bash
     gh pr merge <PR_NUMBER> --squash --match-head-commit <HEAD_SHA> --delete-branch
     ```
     Use `--rebase` or `--merge` instead of `--squash` when requested.
   - After explicit maintainer/user approval, otherwise delete only the PR
     branch after confirming it has landed:
     ```bash
     git push origin --delete <HEAD_BRANCH>
     git switch main
     git pull --ff-only
     git branch -D <HEAD_BRANCH>
     ```
     Use force-delete locally only after explicit maintainer/user approval and
     after confirming the PR branch has landed; squash and rebase merges do not
     preserve the branch tip in `main` ancestry.

## Output

Report:

- PR number, URL, base, head, draft state, milestone, and merge status.
- CI summary: passing, failing, pending, or skipped checks.
- Review summary and whether `@codex review` was triggered.
- Commits pushed, merge action, and branch cleanup action.
- Remaining blockers or next action.
