---
name: dart-close-issue
description: "DART Close Issue: draft and optionally post a GitHub issue closing message"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-close-issue.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-close-issue

Use this skill in Codex to run the DART `dart-close-issue` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a generated Codex adapter entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-close-issue <arguments>`
- Codex: `$dart-close-issue <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Close or prepare closing message for issue: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md

## Workflow

1. Inspect the issue: `gh issue view <ISSUE_NUMBER> --comments`.
2. Verify the stated resolution:
   - fixed by merged PR
   - already fixed on `main`
   - no longer reproducible
   - out of scope or needs a different tracker
3. Draft a concise closing message:
   - thank the reporter
   - state the concrete resolution
   - link the fixing PR or relevant docs when available
4. Only post and close if the user explicitly requested action and explicit
   maintainer/user approval has been given:
   ```bash
   gh issue comment <ISSUE_NUMBER> --body "<message>"
   gh issue close <ISSUE_NUMBER>
   ```

## Output

- Closing message
- Whether it was posted
- Whether the issue was closed
