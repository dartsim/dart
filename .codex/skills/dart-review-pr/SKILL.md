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
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-review-pr <arguments>`
- Codex: `$dart-review-pr <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Review or respond to PR: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/code-style.md
@docs/onboarding/ai-tools.md (for AI-generated review handling)

## To Review

```bash
gh pr view $1 && gh pr diff $1
```

Check: code style, tests, docs, focused commits

## To Address Feedback

```bash
gh pr view $1 --comments
```

Apply minimal fixes locally and verify. Do not push, comment, resolve threads,
or re-trigger review without explicit maintainer/user approval for that
external mutation.

For published PRs, prefer a new follow-up commit for review fixes so reviewers
can inspect what changed since the previous round. Amend or force-push only
after explicit maintainer/user approval and only when the user explicitly
requests it or when there is a clear reason such as removing sensitive content
or repairing broken branch history.

## AI-Generated Reviews (Codex, Copilot, etc.)

When a draft PR is first published, wait a reasonable time for the automatic
Codex review to start. The PR body may show a small Codex activity indicator
(for example, an eyes/count badge) before the submitted review appears. Do not
post `@codex review` for that first review unless no indicator and no review
appears after a reasonable wait.

1. Make the local fix silently (no reply)
2. Run the relevant local gates, including `pixi run lint` before any commit
3. Ask for explicit maintainer/user approval before push, thread resolution,
   PR comment, or review re-trigger
4. If approved, push the fix silently
5. If approved, resolve only reviewed and addressed thread IDs via GraphQL (see
   ai-tools.md)
6. After the approved push, if the fixes addressed Codex review comments, ask
   for explicit maintainer/user approval for the PR comment, then re-trigger:
   `gh pr comment $1 --body "@codex review"`
7. Monitor CI: `gh pr checks $1`
8. Check for new review, repeat until no comments + CI green

Full iterative loop: `docs/onboarding/ai-tools.md` § "Autonomous Review-Fix-Monitor Loop"
