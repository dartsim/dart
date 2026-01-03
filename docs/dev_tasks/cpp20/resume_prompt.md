# Resume Prompt - C++20 Modernization

You are a Codex agent working in `/home/js/dev/dartsim/dart/build_system`.

Goal: modernize the codebase to idiomatic C++20 with no behavior changes.
Public API/ABI breaks are acceptable for DART 7 when needed for span migrations,
as long as `pixi run -e gazebo test-gz` passes without Gazebo code changes.

Current status:

- Phase 5 is complete (PR #2371).
- Phase 6 (internal non-owning views) is complete on `cpp20/phase6`.
- Phase 7 (container membership cleanup) is active on `cpp20/phase6`.
- Plan extends through Phase 9; see `docs/dev_tasks/cpp20/00_plan.md`.
- Prior PRs: dartsim/dart#2367 (phase 4 spans), dartsim/dart#2371 (phase 5).
- Latest validation: not run yet for Phase 6/7; run `pixi run test-all` and
  `pixi run -e gazebo test-gz` before opening a PR.

Required docs:

- Read `docs/onboarding/ci-cd.md` and `docs/onboarding/build-system.md`.
- Read `CONTRIBUTING.md`.
- Read `docs/dev_tasks/cpp20/README.md`,
  `docs/dev_tasks/cpp20/00_plan.md`, and
  `docs/dev_tasks/cpp20/01_progress.md`.

Workflow:

1. Validate or refine the multi-phase plan in
   `docs/dev_tasks/cpp20/00_plan.md`.
2. Execute phases sequentially. Do not start the next phase until you update
   `docs/dev_tasks/cpp20/01_progress.md` and finish the minimal `pixi run`
   validation for the current phase.
3. Keep changes no-op: mechanical modernizations only. Avoid algorithmic
   changes, behavior changes, or new dependencies.
4. Skip generated or third-party code unless there is a clear need to update
   it.
5. Prefer implementation-only refactors. If touching public headers, replace
   const vector-reference getters with `std::span` where appropriate and remove
   redundant span helper accessors; expect API/ABI breaks.
6. Use C++20 features where they improve clarity: `std::span`,
   `std::string_view`, `std::optional`, `std::variant`, `std::ranges`,
   `std::chrono`, `std::erase_if`, `constexpr`, `[[nodiscard]]`,
   `[[maybe_unused]]`.
7. Use existing tooling (`pixi run ...`). Do not invent new entry points.

Deliverables per phase:

- Code changes that preserve behavior.
- Progress update in `docs/dev_tasks/cpp20/01_progress.md`.
- Brief summary of what changed and what remains.
- Run `pixi run lint`, `pixi run test`, and `pixi run -e gazebo test-gz` after
  Phase 3 changes.

If any instructions conflict, ask the user before proceeding.
