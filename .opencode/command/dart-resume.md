---
description: Continue work from previous session
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-resume.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

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
