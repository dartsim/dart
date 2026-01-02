# C++20 Modernization Progress (01)

## Status

- Current phase: Phase 7 (consolidation and validation)
- Code changes: Phase 7 in progress.

## Phase checklist

- Phase 0 - Discovery and guardrails: Complete
- Phase 1 - Mechanical no-op cleanup: Complete
- Phase 2 - Standard library modernization: Complete
- Phase 3 - Additive public header updates: Complete
- Phase 4 - Span input consolidation: Complete
- Phase 5 - String/view cleanup: Complete
- Phase 6 - Algorithm and ranges cleanup: Complete
- Phase 7 - Consolidation and validation: In progress
- Phase 8 - Additional C++20 refinements: Not started
- Phase 9 - Wrap-up and documentation: Not started

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
- Phase 3: replace const vector-reference getters with `std::span`
  returns (e.g., collision contacts, point/mesh/line segment accessors,
  Jacobian/MetaSkeleton/Skeleton/IK accessors, constraint solver skeleton
  views, package paths, cloneable vectors) and remove redundant span helpers.
  Updated call sites to handle spans (including mimic constraint configuration
  and cloneable vector copies).
- Phase 3 checks: `pixi run lint`, `pixi run test`, and
  `pixi run -e gazebo test-gz` (all passed; some deprecation warnings emitted
  from gz-physics during test-gz build).
- Phase 4: remove remaining span-to-vector conversion in
  `tests/integration/io/test_IkFast.cpp` now that `setPositions` accepts spans.
- Phase 4: remove redundant `std::vector` overloads where `std::span` covers
  read-only inputs and update call sites (including python bindings) to pass
  spans explicitly.
- Phase 4: removed vector overloads for `Joint::setActuatorTypes`,
  `Joint::setMimicJointDofs`, `InverseKinematics::setDofs`,
  `PolyhedronVisual::setVertices`, and `PointCloudShape::setColors`, updating
  GUI, SDF, tests, examples, and python bindings to pass spans explicitly.
- Phase 4 checks: `pixi run test-all` and `pixi run -e gazebo test-gz` passed;
  gz-physics emits sign-compare warnings from vendored gtest headers.
- Phase 5: convert internal read-only string parameters to `std::string_view`
  in SDF parsing helpers and profiler formatting utilities; update common
  string trimming/splitting helpers and HTTP cache utility helpers to accept
  `std::string_view`; adjust Python bindings to keep accepting `str` inputs.
- Phase 5 checks: `DART_PARALLEL_JOBS=42 CTEST_PARALLEL_LEVEL=42 pixi run
test-all` and `DART_PARALLEL_JOBS=42 CTEST_PARALLEL_LEVEL=42 pixi run -e
gazebo test-gz` (passed; gz-physics emits deprecation warnings).
- Phase 6: replace manual find/find_if uses with `std::ranges::find`,
  `std::ranges::find_if`, and `std::ranges::any_of` in collision groups,
  skeleton queries, sensor manager membership checks, and world skeleton/frame
  lookups.
- Phase 6 checks: `DART_PARALLEL_JOBS=42 CTEST_PARALLEL_LEVEL=42 pixi run
  test-all` and `DART_PARALLEL_JOBS=42 CTEST_PARALLEL_LEVEL=42 pixi run -e
  gazebo test-gz` (passed; gz-physics emits deprecation warnings).
- Phase 7: consolidation and validation is starting; plan extended with Phase 8
  refinements and a Phase 9 wrap-up to keep modernization work going.
