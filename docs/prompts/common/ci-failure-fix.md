# DART: Fix Failing CI Run

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
# DART: Fix Failing CI Run

Context
- CI failure link: <CI_URL>
- PR or branch (if any): <PR_URL_OR_BRANCH>
- Goal: make CI green with minimal, test-backed changes.

Workflow
- Start at repo root; read `AGENTS.md`, `CONTRIBUTING.md`, and `docs/onboarding/ci-cd.md` / `docs/onboarding/build-system.md` if present (note if missing and continue).
- Identify failing job(s) from the link or `gh pr checks <PR_NUMBER>`.
- Pull logs: `gh run view <RUN_ID> --job <JOB_ID> --log` (or `gh run view <RUN_ID> --log`).
- If logs are missing, download run-level logs (`gh api /repos/<OWNER>/<REPO>/actions/runs/<RUN_ID>/logs`) or re-run the single job using its `databaseId` (`gh run view <RUN_ID> --json jobs --jq '.jobs[] | {name, databaseId}'` then `gh run rerun <RUN_ID> --job <DATABASE_ID>`).
- Reproduce locally with the smallest relevant `pixi run ...` task or `ctest -R <TEST>`; keep scope tight.

Fix + coverage
- Add or adjust tests if behavior changes or the failure indicates a bug.
- If formatting fails, run `pixi run lint-cpp` (or `pixi run lint` if broader).
- If Codecov patch fails, add targeted coverage for new lines or branches.

CI loop
- Commit small, reviewable changes; push to the same branch.
- Watch CI until green: `gh pr checks <PR_NUMBER> --watch --fail-fast` (or `gh run watch <RUN_ID> --interval 30`).
- If a job fails, stop and fix that first, then repeat.

Output
- Short status, root cause, and fix summary.
- Commands run.
- Remaining CI checks (if any) and how to resume.
```
