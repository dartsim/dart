---
description: backport a merged main PR to a release branch
argument-hint: "<source-pr> base=<release-branch>"
agent: build
---

<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-backport-pr.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Backport PR or commits: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/contributing.md
@docs/onboarding/release-management.md
@docs/onboarding/changelog.md

## Workflow

For a source change involving model/scene structure, physics behavior, or GUI
output, use the target branch's `dart-verify-sim` workflow to preserve the text
oracle and assessed visual evidence. Document a visual exception when the
target branch cannot render the claim.

1. Verify the source PR or commit is merged to `main`:
   ```bash
   gh pr view <SOURCE_PR> --json state,mergedAt,baseRefName,mergeCommit
   ```
2. Check whether an equivalent change already exists on the release branch:
   ```bash
   git fetch origin <RELEASE_BRANCH> main
   git cherry -v --abbrev=40 origin/<RELEASE_BRANCH> origin/main | grep <COMMIT_HASH>
   ```
3. For AI-infra or workflow-doc backports, compare the release branch capability
   inventory and adapter directories against `main`. If the release branch has a
   smaller workflow surface, adapt to the release branch instead of importing
   main-only workflows.
4. Create a release branch from the release target without resetting an
   existing local branch:
   ```bash
   BRANCH=backport/<SOURCE_PR>-to-<RELEASE_BRANCH>
   if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
     git switch "$BRANCH"
     # Reuse only a clean branch already based on the release target:
     # `git status --short` must be empty and
     # `git merge-base --is-ancestor origin/<RELEASE_BRANCH> "$BRANCH"` must
     # hold; otherwise stop and ask before resetting or cherry-picking.
   else
     git switch --no-track -c "$BRANCH" origin/<RELEASE_BRANCH>
   fi
   ```
5. Cherry-pick with provenance: `git cherry-pick -x <COMMIT_HASH>`.
6. Resolve conflicts minimally; stop and ask if conflicts are broad or change behavior.
7. Run `/dart-changelog decide` or `$dart-changelog decide` against the
   backport diff and release target before opening the backport PR. If an entry
   is required but needs the backport PR number, draft the decision and keep the
   finalize/update follow-up local until explicit approval permits another
   push. Do not skip the changelog decision just because this is a backport.
8. Run `pixi run lint` and the smallest relevant release-branch checks.
9. Ask for explicit maintainer/user approval before pushing or opening the PR.
   After approval, open the PR against the release branch with milestone
   matching that release branch and use the PR template.

## Output

- Backport PR URL
- Source PR/commit
- Conflicts resolved, if any
- Changelog decision
- Checks run and CI status
