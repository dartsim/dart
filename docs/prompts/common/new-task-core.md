# DART: New Task

<!--
CRITICAL: FOR AGENTS READING THIS FILE
This is a PROMPT TEMPLATE, not an active task.
Do NOT execute unless a human pastes this into a new session.
-->

## Prompt

```text
# DART: New Task

Task
- Do: <describe goal + constraints in 1-5 sentences>
- Done means: <clear acceptance criteria>
- Type: <feature|bugfix|refactor|docs|build|test>

Workflow
- Read `AGENTS.md`, `CONTRIBUTING.md`, `docs/onboarding/`.
- Create branch from latest `origin/main` (e.g., `fix/<topic>`, `feature/<topic>`).
- Keep a short task plan (3-6 steps).
- Prefer small, focused commits.

Build & Test
- `pixi run lint` before committing.
- `pixi run test` for quick checks.
- `pixi run test-all` before calling done.

Git & PR
- Push: `git push -u origin HEAD`.
- Create draft PR early: `gh pr create --draft`.
- CI loop: `gh run watch <id> --interval 30`, fix failures, repeat until green.

Type-specific notes
- Bugfix: Write failing test first, then fix root cause.
- Refactor: No behavior changes; keep diffs mechanical.
- Feature: Note API/UX in PR description; add tests + docs.
- Build/CMake: Ensure downstream compat (find_package, install).

Output
- Summary of changes.
- PR link and CI status.
```
