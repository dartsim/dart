---
description: analyze or clean stale repository branches
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-branch-cleanup.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Analyze or clean branches: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/ci-cd.md
@docs/onboarding/contributing.md

## Modes

- `analyze`: inspect only, no deletions
- `action`: delete or prepare follow-up only when ownership and safety are clear

Default to `analyze` if the requested mode is ambiguous.

## Workflow

1. `git fetch --all --prune`
2. Determine target branch, usually `origin/main`.
3. For each branch:
   ```bash
   git rev-list --left-right --count <TARGET>...<BRANCH>
   git log --oneline <TARGET>..<BRANCH>
   git diff --stat <TARGET>..<BRANCH>
   git cherry -v <TARGET> <BRANCH>
   ```
4. Classify:
   - `ahead=0`: safe deletion candidate
   - equivalent commits already landed: deletion candidate
   - small, current, useful diff: keep or rebase into PR
   - large or unclear diff: document follow-up before action
5. Ask before deleting when ownership, branch purpose, or remote impact is unclear.

## Output

- Branch summary with ahead/behind count and last commit date
- Useful commits or risks
- Recommendation: delete, keep, rebase, or needs follow-up
- Actions taken, if action mode was explicitly requested
