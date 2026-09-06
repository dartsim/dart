---
name: dart-review-pr
description: "DART Review PR: review a PR or address review feedback"
---

<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-review-pr.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-review-pr

Use this skill in Codex to run the DART `dart-review-pr` workflow. The editable
workflow source lives in `.claude/commands/`; this file is its generated adapter
in the shared `.agents/skills/` catalog.

## Invocation

- Claude Code: `/dart-review-pr <arguments>`
- Codex: `$dart-review-pr <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Review or respond to PR: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/code-style.md
@docs/onboarding/ai-reviews.md
@docs/ai/verification.md

## Workflow

Pick the sub-workflow from `mode=` in `$ARGUMENTS`, defaulting to `review`.

### Review

```bash
gh pr view $1 && gh pr diff $1
```

Check code style, tests, docs, and focused commits. When a claim depends on 3D
structure or behavior, require the `dart-verify-sim` text oracle and assessed
visual/debug evidence rather than accepting a screenshot alone.
Record findings as read-only output; do not push, comment, resolve threads, or re-trigger review
without explicit maintainer/user approval for that external mutation.

### Address Feedback

Use the paginated review/CI inspection commands and the single Review-Fix Loop
Workflow in `docs/onboarding/ai-reviews.md`. Collect the completed batch, verify
claims, and repair the underlying defect family. That owner defines trigger
ownership, current-head completion, the two-round strategy checkpoint, false
positive dispositions, blockers, and readiness; do not restart a per-comment
fix/push/review loop here.

For published PRs, prefer a new
follow-up commit so reviewers can inspect each round; amend or force-push only
after explicit maintainer/user approval and only when the user requests it or a
clear reason exists (removing sensitive content, repairing branch history).

Run the relevant local gates, including `pixi run lint` before every commit.
Merge the latest base before each approved push and apply the owner's remote
divergence recovery if the head moved. Reuse existing explicit authority for
this PR, action, and scope; ask only where it is missing. No inline bot replies.
Monitor CI (`gh pr checks $1`); readiness and merge remain separately gated and
require approval for the corresponding external mutation.

## Output

- PR number and whether the pass was a review or a feedback round
- Findings or fixes applied, with file/line references
- Reviewed head, completion/trigger evidence, completed round count, and any
  strategy-checkpoint outcome
- Which actions were local-only and which external mutations were explicitly
  approved
- Codex/CI state and any remaining blocker
