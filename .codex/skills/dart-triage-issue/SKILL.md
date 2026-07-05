---
name: dart-triage-issue
description: "DART Triage Issue: triage a GitHub issue and recommend next action"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-triage-issue.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-triage-issue

Use this skill in Codex to run the DART `dart-triage-issue` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a generated Codex adapter entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-triage-issue <arguments>`
- Codex: `$dart-triage-issue <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Triage issue: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md

## Workflow

1. Inspect the issue:
   ```bash
   gh issue view <ISSUE_NUMBER> --comments
   ```
2. Determine whether the report is still valid on current `origin/main`.
3. Classify status:
   - still valid
   - already fixed
   - needs more information
   - duplicate
   - out of scope
4. If validity depends on behavior, reproduce or inspect the relevant code path.
5. Do not close or comment unless the user explicitly asks for that action.

## Output

- Status
- Rationale in 1-3 bullets
- Recommended next step
- Suggested reply text, if useful
