# Resume Prompt - C++20 Modernization

You are a Codex agent working in `/home/js/dev/dartsim/dart/build_system`.

Goal: modernize the codebase to idiomatic C++20 with no behavior changes.
Keep public APIs and ABI stable.

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
5. Prefer implementation-only refactors. If touching public headers, keep
   existing signatures and layouts; add overloads only if needed.
6. Use C++20 features where they improve clarity: `std::span`,
   `std::string_view`, `std::optional`, `std::variant`, `std::ranges`,
   `std::chrono`, `std::erase_if`, `constexpr`, `[[nodiscard]]`,
   `[[maybe_unused]]`.
7. Use existing tooling (`pixi run ...`). Do not invent new entry points.

Deliverables per phase:

- Code changes that preserve behavior.
- Progress update in `docs/dev_tasks/cpp20/01_progress.md`.
- Brief summary of what changed and what remains.

If any instructions conflict, ask the user before proceeding.
