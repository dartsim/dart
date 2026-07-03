---
description: triage a GitHub issue and recommend next action
argument-hint: "<issue-number>"
agent: build
allowed-tools: Read, Grep, Glob, Bash(git status:*), Bash(git log:*), Bash(git diff:*), Bash(gh pr view:*), Bash(gh pr list:*), Bash(gh issue view:*), Bash(gh search:*)
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-triage-issue.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

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
