---
name: dart-manage-pr
description: "DART Manage PR: manage an open DART pull request through CI, review, merge, and cleanup"
---

<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-manage-pr.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-manage-pr

Use this skill in Codex to run the DART `dart-manage-pr` workflow. The editable
workflow source lives in `.claude/commands/`; this file is its generated adapter
in the shared `.agents/skills/` catalog.

## Invocation

- Claude Code/OpenCode: `/dart-manage-pr <arguments>`
- Codex: `$dart-manage-pr <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Manage an open DART pull request after explicit maintainer/user approval for
mutations: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md
@docs/onboarding/ci-cd.md
@docs/onboarding/testing.md
@docs/onboarding/ai-tools.md

## Modes

- `mode=manage` (default): run the full PR-management loop below to the next
  terminal state.
- `mode=merge`: maintainer-only. Complete the local pre-merge validation in
  step 6 and merge a ready PR only after explicit maintainer/user approval for
  the merge.

## Invocation Contract

When the user says `manage <PR>` or `continue managing <PR>` without limiting
the request to status-only, treat that as approval to run the full
PR-management loop to the next terminal state:

- required policy metadata checked and corrected when stale;
- CI monitored until green, failed, or blocked;
- merge conflicts reproduced and resolved locally;
- review comments addressed, pushed, resolved, and re-reviewed when appropriate;
- PR body/testing evidence refreshed when it no longer matches the branch.

This explicit approval covers routine PR-maintenance mutations for that loop:
additive fix commits and pushes, PR description/metadata corrections, resolving
already-addressed review threads, rerunning failed CI jobs, and requesting a
fresh AI review after follow-up fixes. It does **not** cover merging the PR into
the target branch, force-pushes, branch deletion, PR closure, base-branch
changes, or human reviewer requests; ask separately for those.

Do not call the PR managed just because checks are green. Continue until the PR
is mergeable with required checks complete and addressed review threads
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
   - Check that the base branch, title, and PR template are correct.
   - Verify the milestone is set before merge: `DART 7.0` for a `main` base, the
     branch-matching DART 6.x patch milestone for a `release-*` base. If it is
     missing, set it only after explicit maintainer/user approval.
   - For bug fixes, verify the required DART 6 LTS + `main` dual-PR flow.
   - Confirm the PR body's testing/status section matches the current head and
     does not point reviewers to deleted dev-task evidence as still pending.
   - Confirm the PR body follows template order: Summary, Motivation / Problem,
     Changes / Key Changes, optional Before / After, Testing, Breaking Changes,
     and Related Issues / PRs. Keep Summary first as the skimmable outcome; fold
     leading problem context into Summary and keep the fuller why in Motivation.
   - When the PR has user-facing API, workflow, behavior, or performance impact,
     confirm a concise Before / After section compares old and new surfaces; for
     performance claims make the baseline explicit (CPU path, parent commit,
     `main`, or prior implementation) plus workload, metric, and limitations.
   - Ensure transient visual evidence (screenshots, headless renders, GIFs,
     videos) is hosted as GitHub PR/issue attachments rather than committed to
     the branch; if committed only for the PR body, replace it with an
     attachment URL or ask a maintainer to upload.
   - When the claim depends on 3D structure or behavior, accept an optional
     `Visual verification` subsection after Testing and verify it agrees with
     the text oracle, covers explicit claims, names what is not proved and any
     limitations, records view/debug layers, and includes reproduce commands.
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
   watching unless the user asked only for status. Also poll mergeability:
   ```bash
   gh pr view <PR_NUMBER> --json mergeStateStatus,headRefOid,isDraft,reviewDecision
   ```
   If GitHub reports conflicts, fetch the target branch and resolve them before
   treating green checks as sufficient.
3. Fix failures:
   - Inspect the newest failed run or job, not an older cancelled run. Use the
     `dart-fix-ci` workflow for non-trivial CI debugging.
   - Reproduce locally with the relevant `pixi run ...` task or focused test.
   - Before committing fixes, run `pixi run lint`; also build or test when code
     or behavior changed. Commit only intended files.
   - Prefer additive follow-up commits for published PRs. Amend or force-push
     only after explicit maintainer/user approval and only when the user
     requests it or a clear reason exists (removing sensitive content, repairing
     branch history).
   - Merge the latest base branch into the PR branch before any push, and follow
     the base-merge, automated-review, and bot no-reply rules in
     `docs/onboarding/ai-tools.md`; each push, PR comment, review re-trigger, or
     thread resolution needs explicit maintainer/user approval.
4. Address reviews:
   - Use the `dart-review-pr` workflow for substantive review feedback and the
     automated-review handling in `docs/onboarding/ai-tools.md` (no inline bot
     replies; verify claims locally; apply AI-review fixes silently).
   - For human reviewers, reply only when a response is useful after a fix or
     when a question needs clarification.
   - After an approved push that addressed Codex comments on a PR that already
     had a Codex review, post a fresh top-level `@codex review`; that PR comment
     needs explicit maintainer/user approval and must not duplicate an active
     trigger.
   - For substantive code PRs, an independent review session (a human, or a
     separate agent session running `/dart-review-pr`) must record findings
     before merge approval; docs-only and mechanical changes are exempt.
5. Mark ready or merge only when appropriate:
   - Confirm review requirements are satisfied and local validation matches the
     intended transition.
   - If the PR is draft, mark it ready after explicit approval once Codex is
     clean and local validation passed on the current head: default
     `pixi run test-all`, plus `pixi run -e cuda test-all` on Linux hosts with a
     visible NVIDIA CUDA runtime. Hosted CI may still be pending.
   - Use the current head SHA when merging so a moved branch cannot be merged
     accidentally. Prefer squash/rebase over merge commits per repository
     settings; recent DART `main` PRs use single-parent PR-title commits.
6. `mode=merge` gate (maintainer-only): before any merge, run local pre-merge
   validation on the current head after the latest pushed change:
   `pixi run test-all` and, on Linux hosts with a visible NVIDIA CUDA runtime,
   `pixi run -e cuda test-all`; do not substitute the default run for the CUDA
   run, and record a skip or blocker explicitly. Merge only after CI and review
   are green, the milestone is set, an independent review recorded findings, the
   PR is not draft, GitHub reports it mergeable, and explicit merge approval is
   given. PR comments, review re-triggers, thread resolution, reviewer requests,
   ready-for-review transitions, merges, and branch deletion are external
   mutations that require explicit maintainer/user approval.
7. Clean up after merge:
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
     Squash and rebase merges do not preserve the branch tip in `main` ancestry,
     so force-delete locally only after confirming the PR branch landed and only
     after explicit maintainer/user approval.

## Output

Report:

- PR number, URL, base, head, draft state, milestone, and merge status.
- CI summary: passing, failing, pending, or skipped checks.
- Review summary, independent-review status, and whether `@codex review` ran.
- Local pre-merge validation state when `mode=merge` ran.
- Commits pushed, merge action, and branch cleanup action.
- Remaining blockers or next action.
