# DART: New Task (Core Template)

<!--
CRITICAL: FOR AGENTS READING THIS FILE
=======================================
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
=======================================
-->

## Prompt

```text
# DART: New Task (Core Template)

Task
- Do: <describe goal + constraints in 1-5 sentences>
- Done means: <clear acceptance criteria>
- Out of scope: <optional>

Workflow (must follow)
- Confirm repo root + correct working dir (`git rev-parse --show-toplevel`).
- Follow existing guidance (`AGENTS.md`, `docs/**`, `CONTRIBUTING.md`); don't invent new entry points (use `pixi run ...`).
- Create and maintain a short `update_plan` (3-6 steps, exactly one `in_progress`).
- If using `docs/dev_tasks/<TASK_NAME>/` for task tracking, keep plan/progress there and add a resume prompt file in that folder with full context for a fresh agent.
- When the task is complete, remove `docs/dev_tasks/<TASK_NAME>/` and add a brief note to the most relevant `docs/onboarding/*.md`.
- Prefer small, reviewable commits; don't do drive-by refactors.

Local iteration
- Use parallelism per repo guidance; if unspecified, use ~2/3 of logical cores (only where applicable).
- Add/revise unit tests when behavior changes or bugs are fixed.
- Run `pixi run lint` before committing.
- Run `pixi run test` for quick checks; run `pixi run test-all` before calling it done (unless this task is docs-only and you say so explicitly).

Git / PR / CI loop
- If currently on `main`, first update from `origin/main`, then create a branch (e.g., `feature/<topic>`, `fix/<topic>`, `refactor/<topic>`, `build/<topic>`).
- Keep branch synced: rebase onto the latest `origin/main` before pushing important updates.
- Push with upstream tracking: `git push -u origin HEAD`.
- Create/update a *draft* PR via `gh pr create` early (so CI runs). Follow the PR template; set Milestone to `DART 7.0` when targeting `main`.
- CI fail-fast loop: find the latest run with `gh run list -b <branch>`, then `gh run watch <id> --interval 30` (blocking), fix the first failure immediately, push, and repeat until green.

Add-on
# Add-on: Bug Fix
- First, produce a minimal repro (ideally as a failing unit test).
- Fix the root cause with the smallest change that makes the test pass.
- Add/adjust regression coverage for edge cases seen in the repro or logs.

# Add-on: Refactor
- Preserve behavior (no functional changes unless explicitly requested).
- Keep diffs mechanical and incremental; prefer "move then change" sequencing.
- Add tests only if behavior was unclear/unsafe before (otherwise rely on existing coverage).

# Add-on: Build System / CMake / Packaging
- Treat downstream compatibility as a requirement (install/export, find_package, config versions).
- Prefer `pixi run ...` tasks over calling build tools directly.
- If changes could affect downstream consumers, include a short "compat notes" doc update (where `docs/**` expects it).

# Add-on: New Feature
- Start with a small design note in the PR description (API, UX, defaults, migration/compat).
- Implement behind the smallest surface area; add unit tests + minimal docs/examples as appropriate.
- Call out any behavior change, new knobs, or compatibility risks explicitly.
```
