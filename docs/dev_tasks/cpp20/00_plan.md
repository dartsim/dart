# C++20 Modernization Plan (00)

## Status

- Active: executing phases; Phase 3 in progress.

## Objective

Adopt C++20 idioms across the codebase with no behavior changes.

## Guardrails

- No functional changes, no ABI breaks, no new dependencies.
- Prefer internal-only changes; if public headers are touched, keep signatures
  stable.
- Execute phases sequentially; update `docs/dev_tasks/cpp20/01_progress.md` and
  run pixi checks before starting the next phase.
- Follow the code-style C++20 guidance: avoid `std::format` until the toolchain
  baseline supports it, avoid overly complex ranges rewrites, and keep SFINAE
  where Eigen compile-time traits require it.
- Skip generated or third-party code unless there is a clear need to update it.

## Phase 0 - Discovery and guardrails

- Confirm C++20 is the default standard across all targets.
- Establish modernization rules and identify safe refactor areas.
- Record scope and readiness in `docs/dev_tasks/cpp20/01_progress.md`.

## Phase 1 - Mechanical no-op cleanup

- Replace legacy constructs with clearer equivalents (`nullptr`, `using`,
  `override`, `= default`, `= delete`) where behavior is unchanged.
- Add `[[nodiscard]]` or `[[maybe_unused]]` annotations for correctness
  signals.
- Convert internal macro constants to `constexpr` when no ABI impact exists.
- Align any remaining non-C++20 target compile feature settings with the
  project default.

## Phase 2 - Standard library modernization (internal only)

- Use C++20 types (`std::span`, `std::string_view`, `std::optional`,
  `std::variant`, `std::array`, `std::chrono`) in implementation code where
  lifetimes are already stable.
- Prefer range-based for loops and `std::ranges` algorithms when it improves
  clarity without changing behavior.
- Replace manual container cleanup with `std::erase_if` where applicable.

## Phase 3 - Additive public header updates

- Add overloads or helper types that expose C++20 convenience without removing
  existing APIs.
- Preserve ABI by keeping existing exported signatures and layouts intact.

## Phase 4 - Consolidation and validation

- Run the standard `pixi run` workflows and resolve any warnings introduced by
  modernization.
- Update docs only if new guidance is needed for future maintainers.
