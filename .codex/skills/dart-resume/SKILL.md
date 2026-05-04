---
name: dart-resume
description: "DART Resume: continue work from a previous session"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-resume.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-resume

Use this skill in Codex when you want the same workflow that Claude Code and
OpenCode expose as `/dart-resume`.

## Invocation

- Claude Code/OpenCode: `/dart-resume <arguments>`
- Codex: `$dart-resume <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Resume unfinished work: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/ci-cd.md

## Step 1: Recon (no changes)

```bash
git status -sb && git branch -vv && git log -5 --oneline
gh pr list --head "$(git branch --show-current)"
```

## Step 2: Reconstruct

From: branch name, commits, diffs, issue/PR description

## Step 3: Continue

- Propose 3-6 step plan
- `pixi run test-all` before done
- `git push -u origin HEAD`
