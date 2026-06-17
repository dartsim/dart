---
name: dart-release-merge-main
description: "DART Release Merge Main: merge the active release branch back into main"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-release-merge-main.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-release-merge-main

Use this skill in Codex to run the DART `dart-release-merge-main` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-release-merge-main <arguments>`
- Codex: `$dart-release-merge-main <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Merge release branch into main: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/release-management.md
@docs/onboarding/ci-cd.md

## Workflow

1. Auto-detect or confirm the active release branch and release version:
   ```bash
   git fetch origin
   git branch -r | grep -oE 'origin/release-[0-9]+\.[0-9]+' | sort -V | tail -1
   ```
2. Verify the clone is not shallow and a merge base exists.
3. Create `merge/<release-branch>-into-main` from `origin/main`.
4. Merge the release branch with a title like
   `Merge release-6.19 into main (v6.19.x)`.
5. Resolve conflicts using `docs/onboarding/release-management.md`:
   - deleted in main, updated in release: keep deleted
   - added in both: prefer main unless release has unique needed content
   - content conflicts: prefer main modernization except for manually reviewed changelog entries
   - files only on release: keep if still relevant to main
6. Verify no unresolved conflicts remain.
7. Run `pixi run lint` and relevant checks.
8. Ask for explicit maintainer/user approval before pushing or creating the PR.
   After approval, create a PR targeting `main` with milestone `DART 7.0` and
   use the PR template.
9. Monitor CI until green.

## Output

- PR URL and CI status
- Release branch and release version
- Conflict counts and resolution summary
- Any manual changelog decisions
