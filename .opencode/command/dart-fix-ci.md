---
description: debug and fix failing CI checks on any base branch
argument-hint: "<pr-number|run-id> [base=<branch>]"
agent: build
---
<!-- AUTO-GENERATED FILE - DO NOT EDIT MANUALLY -->
<!-- Source: .claude/commands/dart-fix-ci.md -->
<!-- Sync script: scripts/sync_ai_commands.py -->
<!-- Run `pixi run sync-ai-commands` to update -->

Fix CI failure: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/ci-cd.md
@docs/onboarding/release-management.md

## Workflow

1. Identify the base branch. Default to `main`; use `base=<branch>` from
   `$ARGUMENTS` or the PR's own base when it is a `release-*` branch. For a
   `release-*` base, also read `docs/onboarding/release-management.md` and apply
   the release caveats called out below.
2. Identify failing checks: `gh pr checks <PR_NUMBER>` or `gh run view <RUN_ID>`.
3. Inspect the first real failure:
   ```bash
   gh run view <RUN_ID> --log-failed
   gh run view <RUN_ID> --job <JOB_ID> --log
   ```
4. If a job is still in progress, wait for logs instead of guessing.
5. Choose where to fix. Check whether an equivalent fix already exists on
   `main`. For a `release-*` base, branch from the release branch and prefer
   cherry-picking the proven `main` fix; keep any new fix release-scoped and
   minimal:
   ```bash
   git fetch origin <RELEASE_BRANCH>
   git checkout -B fix/<issue>-<release-branch> origin/<RELEASE_BRANCH>
   ```
6. Reproduce locally with the smallest relevant command:
   - formatting: `pixi run lint`
   - tests: `pixi run test`, `pixi run test-unit`, or another existing
     focused `pixi run ...` test task
   - coverage: add targeted tests for uncovered changed lines
7. Fix the root cause with minimal scope. Explain why the failure was not caught
   earlier and whether workflow coverage should change.
8. If the failure is infrastructure-only, ask for explicit maintainer/user
   approval before rerunning the failed job or running:
   ```bash
   gh run rerun <RUN_ID> --failed
   ```
9. Ask for explicit maintainer/user approval before pushing, CI re-triggers, or
   other GitHub mutations; after approval, push and watch CI until green. For a
   `release-*` base, use the current release milestone and the PR template when
   creating or updating the release-branch PR.

## Output

- Root cause
- Fix or rerun action
- Commands run
- Current CI status
- Prevention recommendation, if any
