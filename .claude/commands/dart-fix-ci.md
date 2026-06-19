---
description: debug and fix failing CI checks
agent: build
---

Fix CI failure: $ARGUMENTS

## Required Reading

@AGENTS.md
@docs/onboarding/ci-cd.md

## Workflow

1. Identify failing checks: `gh pr checks <PR_NUMBER>` or `gh run view <RUN_ID>`.
2. Inspect the first real failure:
   ```bash
   gh run view <RUN_ID> --log-failed
   gh run view <RUN_ID> --job <JOB_ID> --log
   ```
3. If a job is still in progress, wait for logs instead of guessing.
4. Reproduce locally with the smallest relevant command:
   - formatting: `pixi run lint`
   - tests: `pixi run test`, `pixi run test-py`, or another existing
     focused `pixi run ...` test task
   - coverage: add targeted tests for uncovered changed lines
5. Fix the root cause with minimal scope.
6. If the failure is infrastructure-only, ask for explicit maintainer/user
   approval before rerunning the failed job or running:
   ```bash
   gh run rerun <RUN_ID> --failed
   ```
7. Ask for explicit maintainer/user approval before pushing, CI re-triggers, or
   other GitHub mutations; after approval, push and watch CI until the PR is
   green.

## Output

- Root cause
- Fix or rerun action
- Commands run
- Current CI status
