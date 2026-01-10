# New Issue Prompt

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
Resolve GitHub issue <ISSUE> in `dartsim/dart`.

Non-negotiables
- Treat this as still possibly unfixed: first, confirm whether it reproduces on the latest `origin/main`.
- Work in the correct repo root (verify with `git rev-parse --show-toplevel`).
- Add/update regression tests appropriate to the fix.
- Use the repo's standard workflow (`pixi run ...`); don't invent new entry points.
- If using `docs/dev_tasks/<TASK_NAME>/` for task tracking, keep plan/progress there and add a resume prompt file in that folder with full context for a fresh agent.
- Must not break `pixi run -e gazebo test-gz` without any further Gazebo patching (do not modify `patch_gz_physics`, except the existing narrow exception for newer DART CMake config version).

Workflow
1) Intake/validity
- Use `gh issue view <ISSUE>` to read the issue details and confirm it's still relevant on the current `origin/main`.
- If it no longer reproduces, explain why (e.g., already fixed by <commit/PR>) and stop.

2) Reproduction and plan
- Produce minimal, explicit repro steps (commands/files) based on the issue + repo reality.
- Identify the root cause and the smallest fix.
- Identify where a regression test should live and what it should assert.

3) Implement + tests
- Create a new branch for the issue.
- Implement the fix and add/revise unit tests.

4) Local checks (use repo guidance for parallelism)
- Run `pixi run test` (quick) and `pixi run test-all` (final), using the repo's parallelism guidance (or ~2/3 of logical cores if unspecified).
  - If parallelism flags aren't obvious, inspect `pixi.toml` task definitions and apply the correct `-j`/env vars for the underlying runner.
- Run `pixi run -e gazebo test-gz` with the same parallelism guidance and confirm no Gazebo patch changes are needed.

5) Lint + git hygiene
- Run `pixi run lint` before committing.
- Keep commits focused and clean.

6) Sync + push + PR
- Sync with the latest `origin/main` (merge/rebase per repo convention), then push the branch to `origin` using `git push -u origin HEAD`.
- Create/update the PR with `gh pr create` (no need to comment conversationally on the issue/PR unless asked).

7) CI loop (fail-fast)
- Monitor CI in real time; find the latest run id with `gh run list -b <branch>`, then `gh run watch <id> --interval 30`.
- If any job fails, stop and fix that failure first (don't wait for the rest), push updates, and repeat until all CI is green.

Done criteria / final report
- Issue no longer reproduces; regression tests cover it.
- `pixi run test`, `pixi run test-all`, and `pixi run -e gazebo test-gz` pass.
- CI green on the PR branch.
- Final message includes: summary of changes, files touched, commands run, and PR link.

If pushing/creating a PR isn't possible due to auth/permissions, stop and tell me exactly what you need.
```
