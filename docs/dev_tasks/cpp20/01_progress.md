# C++20 Modernization Progress (01)

## Status

- Current phase: Phase 1 (mechanical cleanup)
- Code changes: Phase 1 complete; Phase 2 not started.

## Phase checklist

- Phase 0 - Discovery and guardrails: Complete
- Phase 1 - Mechanical no-op cleanup: Complete
- Phase 2 - Standard library modernization: Not started
- Phase 3 - Additive public header updates: Not started
- Phase 4 - Consolidation and validation: Not started

## Notes

- C++20 is enabled via compile features on core targets; remaining straggler
  targets were aligned to the project default in Phase 1.
- Modernization guardrails match the code-style guidance: avoid `std::format`
  until the baseline supports it, avoid overly complex ranges rewrites, and
  preserve Eigen SFINAE traits.
- Phase 1 started: aligned remaining non-C++20 compile feature settings on
  straggler targets; replaced a legacy `NULL` check with `nullptr`, defaulted
  a trivial destructor in the GUI trackball manipulator, and converted several
  `typedef` aliases to `using` in GUI, dynamics, utils, and test sources.
- Converted C-style aliases in `dart/collision/dart/DARTCollide.cpp` to `using`
  and replaced the local epsilon macro with a `constexpr`.
- Remaining `NULL` usage is confined to the IKFast header (third-party); keep
  it unchanged unless we decide to vendor-update it.
- Remaining `typedef` usage is limited to third-party baselines and IKFast.
- Phase 1 checks: `pixi run lint`, `pixi run test` (both passed).
- Phase 2 pending: no standard library modernization changes yet.
