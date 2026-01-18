# DART: Fix Failing CI

> **Prefer command**: Use `/dart-fix-ci` in Claude Code or OpenCode.
> This template is for tools without command support.

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: Fix Failing CI

Context
- CI failure: <CI_URL>
- PR/branch: <PR_URL_OR_BRANCH>

Workflow
1. Read `AGENTS.md`, `docs/onboarding/ci-cd.md`.
2. Identify failing job: `gh pr checks <PR_NUMBER>` or from the URL.
3. Get logs: `gh run view <RUN_ID> --log-failed`.
4. Reproduce locally: `pixi run test` or `ctest -R <TEST>`.

Fix
- Apply minimal fix for the root cause.
- Add/adjust tests if the failure indicates a bug.
- If formatting fails: `pixi run lint`.
- If coverage fails: add tests for uncovered lines.
- If infrastructure failure (runner lost): `gh run rerun <RUN_ID> --failed`.

CI loop
- Commit and push.
- Watch: `gh run watch <RUN_ID> --interval 30`.
- Fix failures one at a time until green.

Output
- Root cause and fix summary.
- Commands run.
- CI status.
```
