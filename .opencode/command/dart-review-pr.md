---
description: review a PR or address review feedback
argument-hint: "<pr-number> [mode=review|feedback]"
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-review-pr.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Review or respond to PR: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/code-style.md
@docs/onboarding/ai-tools.md (for AI-generated review handling)

## Workflow

Pick the sub-workflow from `mode=` in `$ARGUMENTS`, defaulting to `review`.

### Review

```bash
gh pr view $1 && gh pr diff $1
```

Check code style, tests, docs, and focused commits. Record findings as
read-only output; do not push, comment, resolve threads, or re-trigger review
without explicit maintainer/user approval for that external mutation.

### Address Feedback

```bash
gh pr view $1 --comments
```

Apply minimal fixes locally and verify. For published PRs, prefer a new
follow-up commit so reviewers can inspect each round; amend or force-push only
after explicit maintainer/user approval and only when the user requests it or a
clear reason exists (removing sensitive content, repairing branch history).

1. Make the local fix silently (no reply), then run the relevant local gates,
   including `pixi run lint` before any commit.
2. Merge the latest base branch into the PR branch before any push, and follow
   the base-merge, automated-review, and bot no-reply rules in
   `docs/onboarding/ai-tools.md`. If the push is rejected because the remote
   head moved, fetch and compare it before retrying and validate an equivalent
   remote fix instead of pushing a duplicate.
3. Ask for explicit maintainer/user approval before any push, PR comment, thread
   resolution, or review re-trigger. After approval, push silently, resolve only
   reviewed and addressed thread IDs via GraphQL, and — when the approved push
   addressed Codex comments — re-trigger once with
   `gh pr comment $1 --body "@codex review"`.
4. Apply the same no-inline-reply loop to `github-code-quality[bot]` findings;
   do not re-trigger Codex solely for non-Codex bot findings unless Codex
   comments were also addressed.
5. Monitor CI (`gh pr checks $1`) and repeat until no actionable comments remain.
   For draft PRs, mark ready after explicit approval once Codex is clean and
   local validation passes on the current head (default `pixi run test-all`,
   plus `pixi run -e cuda test-all` on Linux hosts with a visible NVIDIA CUDA
   runtime); merge still waits for required hosted checks.

## Output

- PR number and whether the pass was a review or a feedback round
- Findings or fixes applied, with file/line references
- Which actions were local-only and which external mutations were explicitly
  approved
- Codex/CI state and any remaining blocker
