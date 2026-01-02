# C++20 Modernization Follow-ups Plan (00)

## Status

- Active: Phase 2 (container membership cleanup) in progress.

## Objective

Extend C++20 modernization with no behavior changes after the initial cpp20
pass, focusing on internal-only refactors.

## Guardrails

- No functional changes, no new dependencies.
- Prefer implementation-only refactors; avoid public API changes unless needed.
- Avoid `std::format` until the toolchain baseline supports it.
- Keep changes mechanical and reviewable.
- Skip generated or third-party code unless required.

## Phase 0 - Discovery and scoping

- Identify remaining internal-only opportunities (`std::span`,
  `std::string_view`, `std::ranges`, `contains`, `std::erase_if`).
- Confirm changes do not overlap with the merged cpp20 PRs.

## Phase 1 - Internal non-owning views

- Convert `.cpp`-local helpers and lambdas to accept `std::span` or
  `std::string_view` where lifetimes are already stable.
- Keep public API unchanged.

## Phase 2 - Container membership cleanup

- Prefer `contains` for associative membership checks.
- Replace manual `find`/`find_if` loops with `std::ranges` algorithms where
  clarity improves.

## Phase 3 - Algorithm cleanups

- Use `std::erase`/`std::erase_if` for container cleanup where applicable.
- Prefer range-based loops over index loops when readable.

## Phase 4 - Validation and wrap-up

- Run `pixi run test-all` and `pixi run -e gazebo test-gz`.
- Summarize decisions in onboarding if needed and remove this folder per
  `docs/dev_tasks/README.md`.
