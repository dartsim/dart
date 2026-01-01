# C++20 Modernization Plan (00)

## Status

- Active: executing phases; Phase 3 in progress.

## Objective

Adopt C++20 idioms across the codebase with no behavior changes.

## Guardrails

- No functional changes, no new dependencies.
- Public API/ABI breaks are acceptable for DART 7 when needed for span
  migrations, but Gazebo must remain compatible (`pixi run -e gazebo test-gz`
  passes) without changing Gazebo code.
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

## Phase 3 - Public API span migrations

- Replace const vector-reference getters with `std::span` return types.
- Remove redundant span helper accessors now covered by the primary getters.
- Keep behavior unchanged while allowing API/ABI breaks where required.

## Phase 4 - Span input consolidation

<<<<<<< HEAD
- Remove redundant `std::vector` overloads where `std::span` covers the same
  read-only use cases.
- Update call sites (including python bindings) to pass spans explicitly.
- Keep move-optimized overloads when ownership transfer matters.

## Phase 5 - String/view cleanup

- Convert internal-only read-only string parameters to `std::string_view`.
- Avoid touching public APIs unless needed for consistency or performance.

## Phase 6 - Algorithm and ranges cleanup

- Replace manual loops with `std::ranges`/algorithm helpers where readability
  improves without behavior changes.
- Expand `std::erase_if` usage for container cleanup where applicable.

## Phase 7 - Consolidation and validation

- Run the standard `pixi run` workflows and resolve any regressions introduced
  by modernization, including `pixi run -e gazebo test-gz`.
- Summarize key design decisions in onboarding and remove
  `docs/dev_tasks/cpp20/` per `docs/dev_tasks/README.md`.
=======
- Run the standard `pixi run` workflows and resolve any warnings introduced by
  modernization, including `pixi run -e gazebo test-gz`.
- Update docs only if new guidance is needed for future maintainers.
>>>>>>> 6e0d6392b10 (Modernize getters to std::span)
