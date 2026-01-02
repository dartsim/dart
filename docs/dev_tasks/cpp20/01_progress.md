# C++20 Modernization Progress (01)

## Status

- Current phase: Phase 4 (consolidation and validation)
- Code changes: Phase 4 in progress.

## Phase checklist

- Phase 0 - Discovery and guardrails: Complete
- Phase 1 - Mechanical no-op cleanup: Complete
- Phase 2 - Standard library modernization: Complete
- Phase 3 - Additive public header updates: Complete
- Phase 4 - Consolidation and validation: In progress

## Notes

- C++20 is enabled via compile features on core targets; remaining straggler
  targets were aligned to the project default in Phase 1.
- Guardrails updated for DART 7: public API/ABI breaks are allowed when needed
  for span migrations, but Gazebo must stay compatible
  (`pixi run -e gazebo test-gz`) without changing Gazebo code.
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
- Phase 3: replace const vector-reference getters with `std::span`
  returns (e.g., collision contacts, point/mesh/line segment accessors,
  Jacobian/MetaSkeleton/Skeleton/IK accessors, constraint solver skeleton
  views, package paths, cloneable vectors) and remove redundant span helpers.
  Updated call sites to handle spans (including mimic constraint configuration
  and cloneable vector copies).
- Phase 3 follow-up: update MetaSkeleton and Skeleton index-based setters/getters
  to accept `std::span`, removing span-to-vector conversions in IK and
  dynamics test utilities.
- Phase 3 follow-up: remove span-to-vector conversions in mimic joint handling
  (ConstraintSolver checks, SDF mimic parsing, mimic pendulum example, and
  mimic constraint test).
- Phase 3 follow-up: add span overloads for `Joint::setActuatorTypes`,
  `Joint::setMimicJointDofs`, and pointer-based `InverseKinematics::setDofs`.
- Phase 3 checks: `pixi run lint`, `pixi run test`, and
  `pixi run -e gazebo test-gz` (all passed; some deprecation warnings emitted
  from gz-physics during test-gz build).
- Phase 4: rerun the standard pixi workflows and resolve any regressions before
  finishing the phase.
- Phase 4 checks: `pixi run test-all` (local run with 2/3 core count passed).
