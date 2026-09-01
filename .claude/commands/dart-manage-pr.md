---
description: manage an open DART pull request through CI, review, merge, and cleanup
argument-hint: "[pr-number] [status|mode=manage|merge]"
agent: build
---
Manage an open DART pull request after explicit maintainer/user approval for
mutations: $ARGUMENTS
## Required Reading
@AGENTS.md
@docs/onboarding/contributing.md
@docs/onboarding/ci-cd.md
@docs/onboarding/ai-tools.md

## Modes

- `status`: report-only; no mutations.
- `mode=manage` (default): run the full loop below to the next terminal state.
- `mode=merge`: maintainer-only; merges happen only under this mode, through
  the step 5 pre-merge gate and explicit maintainer/user approval to merge.

## Invocation Contract

When the user says `manage <PR>` or `continue managing <PR>` without limiting
the request to status-only, treat that as approval to run the full PR-management
loop to the next terminal state:

- required policy metadata checked and corrected when stale;
- CI monitored until green, failed, or blocked;
- merge conflicts reproduced and resolved locally;
- review comments addressed, pushed, resolved, re-reviewed when appropriate;
- PR body/testing evidence refreshed when it no longer matches the branch.

This explicit approval covers routine PR-maintenance mutations for that loop:
additive fix commits and pushes, description/metadata corrections, resolving
already-addressed review threads, rerunning failed CI jobs, and requesting a
fresh AI review after follow-up fixes. It does **not** cover merging (see
`mode=merge`), force-pushes, branch deletion, PR closure, base-branch changes,
or human reviewer requests; ask separately for those.

Do not call the PR managed just because checks are green: continue until it is
mergeable with required checks complete and addressed threads resolved, or a
concrete blocker remains.

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
   - Check that the base branch, title, and PR template are correct, and that
     the base-matching milestone is set (`DART 7.0` for a `main` base, the
     branch-matching DART 6.x release milestone for a `release-*` base); if
     it is missing, set it only after explicit maintainer/user approval.
   - For bug fixes, verify the required DART 6 LTS + `main` dual-PR flow.
   - Confirm the PR body's testing/status section matches the current head and
     does not point reviewers to deleted dev-task evidence as still pending.
   - Confirm the PR body is readable and follows template order (Summary first
     as the skimmable user/downstream outcome, then Motivation, Changes,
     optional Before / After, Testing, Breaking Changes, Related Issues /
     PRs); mechanics belong in Changes unless they explain user-visible risk.
   - When the PR has meaningful user-facing API, workflow, behavior, or
     performance impact, confirm a concise Before / After section compares the
     old and new surfaces, with an explicit baseline for performance claims
     (path, parent commit or `main`, workload, metric, limitations) and rows
     phrased as user-visible before/after.
   - Ensure transient visual evidence (screenshots, headless renders, GIFs,
     videos) is hosted as GitHub PR/issue Markdown attachments
     (`https://github.com/user-attachments/assets/...`), never committed to
     the branch; if evidence files were committed only for the PR body, remove
     them and link an attachment URL instead (ask a maintainer to upload when
     tooling cannot).
   - For claim-dependent 3D behavior, verify an optional `Visual verification`
     subsection agrees with the text oracle and records claims, limitations,
     view/debug layers, and reproduce commands.
   - Inspect local state before editing: `git status --short --branch`,
     `git diff --stat`, `git diff --check`.
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
   - For already-published PRs, prefer additive follow-up commits. Amend or
     force-push only after explicit maintainer/user approval, when the user
     explicitly requests it or a clear reason exists (removing sensitive
     content, repairing broken history).
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
     requires explicit maintainer/user approval. Never rebase or force-push a
     published PR branch unless the maintainer explicitly requests it.
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
   - For substantive code PRs, an independent review session (a human, or a
     separate session running `dart-review-pr` via `/dart-review-pr` or
     `$dart-review-pr`) must record its outcome — findings or an explicitly
     clean result — before merge approval; docs-only/mechanical are exempt.
   - For human reviewers, reply only when a response is useful after a fix or
     when a question needs clarification.
   - After posting `@codex review`, keep monitoring until a submitted review,
     a visible activity signal, or a concrete timeout/blocker is observed.
5. Mark ready or merge only when appropriate:
   - Confirm review requirements are satisfied and local validation matches the
     intended transition.
   - If the PR is draft, mark it ready after explicit approval once Codex is
     clean and local validation passed on the current head (`pixi run
     test-all`; focused `pixi run test`/`test-py` and the Gazebo gate as the
     touched surface requires). Hosted CI may still be pending.
   - `mode=merge` gate (maintainer-only): before any merge, re-run local
     validation on the current head after the latest pushed change
     (`pixi run test-all`, plus the Gazebo gate when downstream compatibility
     could be affected); merge only when required hosted checks and review are
     green, the milestone is set, an independent review recorded a clean
     result on the current post-fix head (after findings, a clean re-review;
     the step 4 docs-only/mechanical exemption also satisfies this), the
     PR is not draft and GitHub reports it mergeable, and explicit merge
     approval is given.
   - PR comments, review re-triggers, thread resolution, reviewer requests,
     ready-for-review transitions, merges, and branch deletion are external
     mutations and require explicit maintainer/user approval.
   - Confirm the merge method from repository settings or the user (recent
     DART PRs prefer squash/rebase single-parent PR-title commits) and use the
     current head SHA when merging so a moved branch cannot be merged
     accidentally.
6. Clean up after merge:
   - Confirm the PR merged and identify the head branch before deleting. After
     explicit maintainer/user approval, prefer merge-time deletion with the
     approved merge method and head SHA:
     ```bash
     gh pr merge <PR_NUMBER> --squash --match-head-commit <HEAD_SHA> --delete-branch
     ```
     Use `--rebase` or `--merge` instead of `--squash` when requested.
   - After explicit maintainer/user approval, otherwise delete only the PR
     branch after confirming it has landed (`git push origin --delete
     <HEAD_BRANCH>`, then update the local checkout); force-delete locally
     only after explicit maintainer/user approval and after confirming the
     branch landed — squash/rebase merges do not preserve the branch tip in
     target-branch ancestry.

## Output

Report:

- PR number, URL, base, head, draft state, milestone, and merge status.
- CI summary, review summary, and whether `@codex review` was triggered.
- Commits pushed, merge action, and branch cleanup action.
- Remaining blockers or next action.
