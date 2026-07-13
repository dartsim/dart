---
name: dart-release-ci-fix
description: "DART Release CI Fix: debug and fix CI failures on the DART 6.20 release branch"
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-release-ci-fix.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

# dart-release-ci-fix

Use this skill in Codex to run the DART `dart-release-ci-fix` workflow. The editable
workflow source currently lives in `.claude/commands/`, and this generated
Codex skill is a first-class Codex entrypoint.

## Invocation

- Claude Code/OpenCode: `/dart-release-ci-fix <arguments>`
- Codex: `$dart-release-ci-fix <arguments>`

Treat the text after the skill name as `$ARGUMENTS`. When the workflow
references `$1`, `$2`, etc., map those to the positional values supplied by the
user.

## Command Body

Fix release-branch CI: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/ci-cd.md
@docs/onboarding/release-management.md

## Workflow

For a failure involving model/scene structure, physics behavior, or OSG output,
use `dart-verify-sim` to reproduce the claim with text and assessed visual
evidence. Document a visual exception when OSG/Xvfb is unavailable or not
applicable.

1. Inspect the failing run:
   ```bash
   gh run view <RUN_ID> --log-failed
   gh run view <RUN_ID> --job <JOB_ID> --log
   ```
2. Resolve the target from live state. Default to `release-6.20`; for a PR,
   verify its base rather than trusting a stale handoff. Check whether an
   equivalent fix already exists on `main`, but treat it as reference evidence.
3. If continuing an existing PR, fetch and checkout that branch. Otherwise
   branch from the resolved release branch without resetting an existing local
   branch:
   ```bash
   RELEASE_BRANCH=${RELEASE_BRANCH:-release-6.20}
   git fetch origin "$RELEASE_BRANCH"
   BRANCH=fix/<issue>-${RELEASE_BRANCH}
   if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
     git switch "$BRANCH"
   else
     git switch --no-track -c "$BRANCH" "origin/$RELEASE_BRANCH"
   fi
   ```
4. Adapt a proven `main` fix only when its assumptions exist here. Preserve
   C++17, pybind11, `dart::utils`, OSG, and downstream Gazebo/gz-physics
   behavior; otherwise make the smallest release-scoped fix.
5. Explain why the failure was not caught earlier and whether workflow coverage should change.
6. Run `pixi run lint` and release-relevant build/tests.
7. Ask for explicit maintainer/user approval before pushing, creating, or
   updating the release-branch PR; after approval, use the current release
   milestone and PR template.
8. Monitor CI until green.

## Output

- Root cause
- Fix summary
- PR URL
- CI status
- Prevention recommendation, if any
