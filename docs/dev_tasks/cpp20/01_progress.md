# C++20 Modernization Progress (01)

## Status

- Current phase: Phase 9 (validation and wrap-up)
- Code changes: Phase 9 in progress.

## Phase checklist

- Phase 0 - Discovery and guardrails: Complete
- Phase 1 - Mechanical no-op cleanup: Complete
- Phase 2 - Standard library modernization: Complete
- Phase 3 - Additive public header updates: Complete
- Phase 4 - Span input consolidation: Complete
- Phase 5 - String/view cleanup: Complete
- Phase 6 - Internal non-owning views: Complete
- Phase 7 - Container membership cleanup: Complete
- Phase 8 - Algorithm cleanups: Complete
- Phase 9 - Validation and wrap-up: In progress

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
- Phase 5: internal string utilities, HTTP cache helpers, and SDF parsing
  helpers now prefer `std::string_view`; broadened `std::ranges` and
  `contains` usage in core subsystems; validation completed (PR #2371).
- Phase 6: adopt `std::span` for `.cpp`-local helpers in
  `dart/math/optimization/Problem.cpp` and lambda indices in
  `dart/math/lcp/other/StaggeringSolver.cpp`.
- Phase 7: start replacing associative membership checks with `contains` in
  `dart/gui/Viewer.cpp`, `dart/dynamics/Linkage.cpp`,
  `dart/utils/SkelParser.cpp`, and `dart/utils/urdf/urdf_world_parser.cpp`.
- Phase 7: use `contains` for membership checks in observer/subject tracking,
  name manager lookups, composite aspect requirements, collision result
  caches, referential skeleton membership, and node destructor sets.
- Phase 7: adopt `std::ranges::find` for membership checks in
  `dart/sensor/SensorManager.cpp` and referential skeleton body/joint queries.
- Phase 7: apply `std::ranges::find` in world, constraint solver, and
  constrained group membership checks.
- Phase 7: replace remaining `std::find`/`std::find_if` in dynamics, collision,
  GUI, and LCP solvers with `std::ranges` equivalents; use `contains` for
  non-body entity set membership in `BodyNode`.
- Phase 8: replace remove/erase idioms with `std::erase` in constraint and
  world skeleton cleanup paths.
- Phase 8: convert index/iterator loops to range-based loops in
  `ConstrainedGroup` and `World` skeleton cleanup.
- Phase 8: use `std::erase_if` for line segment connection removal.
- Phase 8: apply `std::erase_if` for mesh node cleanup in
  `dart/gui/render/MeshShapeNode.cpp`.
- Phase 8: replace `std::sort` with `std::ranges::sort` in core profiling,
  dynamics, collision, and geometry utilities.
- Phase 8: replace `std::sort` with `std::ranges::sort` in dart8 profiling,
  serializer ordering, and LCP solver examples.
- Phase 9: validation pending (`pixi run test-all` and
  `pixi run -e gazebo test-gz`).
