---
name: dart-resume
description: "DART Resume: continue work from a previous session"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-resume.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-resume

Use this skill in Codex to run the DART `dart-resume` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

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
@docs/onboarding/contributing.md

## Step 1: Recon (no changes)

```bash
git rev-parse --show-toplevel
git status -sb && git branch -vv && git log -10 --oneline --decorate
git diff --stat && git stash list
gh pr list --head "$(git branch --show-current)"
gh pr status
```

## Step 2: Reconstruct

Infer the task from branch name, commits, diffs, issue/PR description, and any
`docs/dev_tasks/<task>/` state. If the goal is still unclear after recon, stop
and ask.

## Step 3: Continue

- Propose a 3-6 step plan before editing.
- Continue with minimal scope and preserve existing user changes.
- Run `pixi run lint` before committing.
- Run relevant tests; use `pixi run test-all` before done when feasible.
- Push with `git push -u origin HEAD` and create/update the PR only after
  explicit maintainer/user approval.

## Safety

No destructive git commands (`reset --hard`, dropping stashes, deleting branches)
without explicit confirmation.
