---
name: dart-branch-cleanup
description: "DART Branch Cleanup: analyze or clean stale repository branches"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-branch-cleanup.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-branch-cleanup

Use this skill in Codex to run the DART `dart-branch-cleanup` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-branch-cleanup <arguments>`
- Codex: `$dart-branch-cleanup <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Analyze or clean branches: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/ci-cd.md
@docs/onboarding/contributing.md

## Modes

- `analyze`: inspect only, no deletions
- `action`: prepare follow-up, or delete only after ownership/safety are clear
  and the maintainer/user gives explicit approval for each local or remote
  branch deletion

Default to `analyze` if the requested mode is ambiguous.

## Workflow

1. `git fetch --all --no-prune`
2. List stale remote-tracking refs without deleting them:
   ```bash
   git remote prune origin --dry-run
   ```
3. Determine target branch, usually `origin/main`.
4. For each branch:
   ```bash
   git rev-list --left-right --count <TARGET>...<BRANCH>
   git log --oneline <TARGET>..<BRANCH>
   git diff --stat <TARGET>..<BRANCH>
   git cherry -v <TARGET> <BRANCH>
   ```
5. Classify recommendations only; deleting a candidate requires explicit
   maintainer/user approval:
   - `ahead=0`: safe deletion candidate
   - equivalent commits already landed: deletion candidate
   - small, current, useful diff: keep or rebase into PR
   - large or unclear diff: document follow-up before action
6. Ask for explicit maintainer/user approval before pruning refs or deleting any
   local or remote branch, even when ownership, branch purpose, and remote
   impact look clear.

## Output

- Branch summary with ahead/behind count and last commit date
- Useful commits or risks
- Recommendation: delete, keep, rebase, or needs follow-up
- Actions taken after explicit maintainer/user approval, if action mode was
  explicitly requested
