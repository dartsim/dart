# C++20 Modernization Progress (01)

## Status

- Current phase: Phase 3 (public API span migrations)
- Code changes: Phase 3 in progress.

## Phase checklist

- Phase 0 - Discovery and guardrails: Complete
- Phase 1 - Mechanical no-op cleanup: Complete
- Phase 2 - Standard library modernization: Complete
- Phase 3 - Additive public header updates: In progress
- Phase 4 - Consolidation and validation: Not started

## Notes

- C++20 is enabled via compile features on core targets; remaining straggler
  targets were aligned to the project default in Phase 1.
- Guardrails updated for DART 7: public API/ABI breaks are allowed when needed
  for span migrations, but Gazebo must stay compatible (`pixi run -e gazebo
test-gz`) without changing Gazebo code.
- Phase 1: aligned remaining non-C++20 compile feature settings on straggler
  targets; replaced a legacy `NULL` check with `nullptr`, defaulted a trivial
  destructor in the GUI trackball manipulator, and converted several `typedef`
  aliases to `using` in GUI, dynamics, utils, and test sources.
- Converted C-style aliases in `dart/collision/dart/DARTCollide.cpp` to `using`
  and replaced the local epsilon macro with a `constexpr`.
- Remaining `NULL` usage is confined to the IKFast header (third-party); keep
  it unchanged unless we decide to vendor-update it.
- Remaining `typedef` usage is limited to third-party baselines and IKFast.
- Phase 1 checks: `pixi run lint`, `pixi run test` (both passed).
- Phase 2: use `std::erase`/`std::erase_if` for container cleanup and
  introduce `std::span` in internal helper functions for pointer vector
  conversions.
- Phase 2 checks: `pixi run lint`, `pixi run test` (both passed).
- Phase 3 started: replace const vector-reference getters with `std::span`
  returns (e.g., collision contacts, point/mesh/line segment accessors,
  Jacobian/MetaSkeleton/Skeleton/IK accessors, constraint solver skeleton
  views, package paths, cloneable vectors) and remove redundant span helpers.
  Updated call sites to handle spans (including mimic constraint configuration
  and cloneable vector copies).
- Phase 3 checks: `pixi run lint`, `pixi run test`, and
  `pixi run -e gazebo test-gz` (all passed; some deprecation warnings emitted
  from gz-physics during test-gz build).
