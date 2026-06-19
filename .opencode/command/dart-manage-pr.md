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

## Invocation Contract

When the user says `manage <PR>` or `continue managing <PR>` without limiting
the request to status-only, treat that as approval to run the full PR-management
loop to the next terminal state:

- required policy metadata checked and corrected when stale;
- CI monitored until green, failed, or blocked;
- merge conflicts reproduced and resolved locally;
- review comments addressed, pushed, resolved, and re-reviewed when appropriate;
- PR body/testing evidence refreshed when it no longer matches the branch.

This explicit approval covers routine PR-maintenance mutations needed for that
loop: additive fix commits and pushes, PR description/metadata corrections,
resolving already-addressed review threads, rerunning failed CI jobs, and
requesting a fresh AI review after follow-up fixes. It does **not** cover
merging the PR into the target branch, force-pushes, branch deletion, PR
closure, base-branch changes, or human reviewer requests; ask separately for
those.

Do not call the PR managed just because checks are green. Continue until the PR
is mergeable with required checks complete and addressed review threads are
resolved, or until a concrete blocker remains.

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
   - For bug fixes, verify the required DART 6 LTS + `main` dual-PR flow.
   - Confirm the PR body's testing/status section matches the current head and
     does not point reviewers to deleted dev-task evidence as still pending.
   - Confirm the PR body is readable and follows template order: Summary,
     Motivation / Problem, Changes / Key Changes, optional Before / After,
     Testing, Breaking Changes, and Related Issues / PRs. Keep Summary first as
     the skimmable user/downstream outcome; if problem context must lead, fold
     it into Summary and keep the fuller why in Motivation. Mechanics belong in
     Changes unless they explain user-visible risk.
   - When the PR has meaningful user-facing API, workflow, behavior, or
     performance impact, confirm the body has a concise Before / After section
     that compares the relevant old and new surfaces. For performance claims,
     ensure the baseline is explicit: CPU path, parent commit, `main`, or prior
     implementation, plus workload, metric, and important limitations. Each
     row should be phrased as a user-visible before/after, with backend details
     used only as supporting context.
   - For visual PR evidence, ensure transient screenshots, headless renders,
     GIFs, and videos are hosted as GitHub PR/issue Markdown attachments
     (`https://github.com/user-attachments/assets/...`) rather than committed to
     the branch. If evidence files were committed only for the PR description,
     remove them and replace the PR body entry with a GitHub attachment URL. If
     the current tooling cannot upload an attachment, ask a maintainer to upload
     the local artifact through the PR editor instead of keeping it under
     `docs/assets/`.
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
   Also poll mergeability:
   ```bash
   gh pr view <PR_NUMBER> --json mergeStateStatus,headRefOid,isDraft,reviewDecision
   ```
   If GitHub reports conflicts, fetch the target branch and resolve them before
   treating green checks as sufficient.
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
   - Before every push, first merge the latest base branch into the PR branch
     (on every push, not just the first) so each pushed/CI-tested state reflects
     the current target base branch and conflicts surface early:
     ```bash
     git fetch origin <base-branch>
     git merge --no-ff origin/<base-branch>  # never rebase a published PR branch
     # rebuild + retest if the merge touched code
     git push                                 # after explicit approval
     ```
     The local base merge is a routine pre-push step; the push itself still
     requires explicit maintainer/user approval. Do not rebase a published PR
     branch by default because it invalidates existing CI runs and makes PR
     review/comment history harder to follow. Rebase or force-push only when the
     maintainer explicitly requests it.
4. Address reviews:
   - Use the `dart-review-pr` workflow for substantive review feedback.
   - Never reply to AI-generated review comments from bot users such as
     `chatgpt-codex-connector[bot]`, `github-code-quality[bot]`,
     `github-actions[bot]`, or `copilot[bot]`.
   - When a draft PR is first published, request Codex review with a top-level
     `@codex review` once explicit maintainer/user approval covers PR comments;
     it can run while the PR remains draft. If Codex already shows an activity
     signal or submitted review, do not post a duplicate trigger.
   - Apply AI-review fixes silently. After explicit maintainer/user approval
     and after the branch is ready, push, resolve reviewed and addressed
     threads, and request a fresh AI review only when the approved follow-up
     push addressed Codex review comments, or when the first trigger has a
     concrete timeout/blocker:
     ```bash
     gh pr comment <PR_NUMBER> --body "@codex review"
     ```
   - For human reviewers, reply only when a response is useful after a fix or
     when a question needs clarification.
   - After posting `@codex review`, keep monitoring until a submitted review,
     a visible activity signal, or a concrete timeout/blocker is observed.
5. Mark ready or merge only when appropriate:
   - Confirm review requirements are satisfied and local validation matches the
     intended transition.
   - If the PR is draft, mark it ready after explicit approval once Codex is
     clean and local validation passed on the current head: `pixi run test-all`
     for build coverage, `pixi run test`/`pixi run test-py` for affected
     C++/Python behavior, plus the Gazebo gate when package or downstream
     compatibility could be affected. Hosted CI may still be pending.
   - Confirm required hosted checks are passing before merge.
   - Do not merge unless explicitly asked or the workflow clearly includes merge.
   - PR comments, review re-triggers, thread resolution, reviewer requests,
     ready-for-review transitions, merges, and branch deletion are external
     mutations and require explicit maintainer/user approval.
   - Confirm the merge method from repository settings or the user. Recent DART
     PRs usually use single-parent PR-title commits, so prefer squash/rebase
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
     git switch <BASE_BRANCH>
     git pull --ff-only
     git branch -D <HEAD_BRANCH>
     ```
     Use force-delete locally only after explicit maintainer/user approval and
     after confirming the PR branch has landed; squash and rebase merges do not
     preserve the branch tip in target-branch ancestry.

## Output

Report:

- PR number, URL, base, head, draft state, milestone, and merge status.
- CI summary: passing, failing, pending, or skipped checks.
- Review summary and whether `@codex review` was triggered.
- Commits pushed, merge action, and branch cleanup action.
- Remaining blockers or next action.
