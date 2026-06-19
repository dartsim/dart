---
name: dart-backport-pr
description: "DART Backport PR: backport a merged main PR to a release branch"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-backport-pr.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-backport-pr

Use this skill in Codex to run the DART `dart-backport-pr` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-backport-pr <arguments>`
- Codex: `$dart-backport-pr <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Backport PR or commits: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md
@docs/onboarding/release-management.md

## Workflow

1. Verify the source PR or commit is merged to `main`:
   ```bash
   gh pr view <SOURCE_PR> --json state,mergedAt,baseRefName,mergeCommit
   ```
2. Check whether an equivalent change already exists on the release branch:
   ```bash
   git fetch origin <RELEASE_BRANCH> main
   git cherry -v --abbrev=40 origin/<RELEASE_BRANCH> origin/main | grep <COMMIT_HASH>
   ```
3. Create a release branch from the release target without resetting an
   existing local branch:
   ```bash
   BRANCH=backport/<SOURCE_PR>-to-<RELEASE_BRANCH>
   if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
     git switch "$BRANCH"
   else
     git switch --no-track -c "$BRANCH" origin/<RELEASE_BRANCH>
   fi
   ```
4. Cherry-pick with provenance: `git cherry-pick -x <COMMIT_HASH>`.
5. Resolve conflicts minimally; stop and ask if conflicts are broad or change behavior.
6. Run `pixi run lint` and the smallest relevant release-branch checks.
7. Ask for explicit maintainer/user approval before pushing or opening the PR.
   After approval, open the PR against the release branch with milestone
   matching that release branch and use the PR template.

## Output

- Backport PR URL
- Source PR/commit
- Conflicts resolved, if any
- Checks run and CI status
