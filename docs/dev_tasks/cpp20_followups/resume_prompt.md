# Resume Prompt - C++20 Modernization Follow-ups

You are a Codex agent working in `/home/js/dev/dartsim/dart/build_system`.

Goal: continue C++20 modernization with no behavior changes after the initial
cpp20 task. Focus on internal-only refactors (non-owning views, ranges,
container membership cleanups).

Current status:

- Phase 2 (container membership cleanup) is active; see
  `docs/dev_tasks/cpp20_followups/00_plan.md`.
- Phases 0-1 are complete (discovery + internal non-owning views).
- Keep this task independent of the main cpp20 PRs unless instructed.

Required docs:

- Read `docs/onboarding/ci-cd.md` and `docs/onboarding/build-system.md`.
- Read `CONTRIBUTING.md`.
- Read `docs/dev_tasks/cpp20_followups/README.md`,
  `docs/dev_tasks/cpp20_followups/00_plan.md`, and
  `docs/dev_tasks/cpp20_followups/01_progress.md`.

Workflow:

1. Validate or refine the plan in `docs/dev_tasks/cpp20_followups/00_plan.md`.
2. Execute phases sequentially; update progress before moving on.
3. Keep changes no-op and avoid public API changes unless necessary.
4. Use existing tooling (`pixi run ...`).

If any instructions conflict, ask the user before proceeding.
