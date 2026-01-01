# Collision Detector Task - Next

## Current Status

- Phase 1 complete with an initial architecture proposal in 02_architecture.md.
- Research covered the collision API surface, contact conventions, solver and world integration, test and benchmark coverage, and build and packaging touchpoints.
- Phase 2 scaffolding started: DARTCollisionDetector now delegates pair iteration and contact merging to DartCollisionEngine.
- Added axis-aligned bounding box prechecks in DartCollisionEngine to skip narrowphase on separated pairs.
- Added a sweep-based candidate pair pass for single-group queries to reduce pair checks.
- Cached per-object world AABBs in DARTCollisionObject, with DARTCollisionDetector calling updateEngineData before queries.
- Added a sweep-based candidate pair pass for group-group queries using combined membership flags.
- Respect negative penetration filtering in DartCollisionEngine when requested by CollisionOption.
- Added initial core data structs and wired DARTCollisionObject to populate them on update.
- Added cylinder-sphere and cylinder-plane pair support in the DART narrowphase, with new DARTCollisionDetector tests.
- Drafted a lightweight performance and benchmark plan for the new engine path.
- Routed broadphase pair generation through core entries and sweep helpers.
- Added plane-sphere and plane-box support in the DART narrowphase and enabled DART plane tests.
- Skip narrowphase for core entries marked as missing or unsupported.
- Added core-shape dispatch in DartCollisionEngine to avoid per-pair shape lookups.
- Added a DART collision benchmark variant mirroring the box stack scenario.
- Added initial core query result and option types for future standalone use.
- Added DART raycast support for core shapes with DART-focused unit coverage and AABB pruning.
- Added a resume prompt and progress tracking doc for the task.
- Added box-cylinder and cylinder-cylinder narrowphase support with DART-only unit coverage.
- Added distance query MVP for primitives with DART-only unit coverage.
- Expanded DART raycast tests to cover box, cylinder, and plane hits.
- Added DART distance edge-case coverage and a DART distance benchmark.
- Added sweep-based pruning for distance queries with AABB lower-bound checks.
- Documented the current core/adapter boundary and raycast placement in the architecture proposal.
- Expanded distance tests for sphere-plane, sphere-box, sphere-cylinder, and box-cylinder with nearest-point checks.
- Added raycast inside-hit coverage for sphere, box, and cylinder.
- Added raycast surface-start coverage for sphere, box, cylinder, and plane.
- Added raycast parallel plane surface-start miss coverage.
- Added rotated box distance coverage and a rotated plane raycast check.
- Added rotated plane offset raycast coverage.
- Added tangent sphere and parallel plane raycast coverage.
- Added cylinder-cylinder nearest-point checks and tangent cylinder raycast coverage.
- Added cylinder cap rim raycast coverage.
- Added cylinder parallel raycast miss coverage.
- Added tangent box raycast coverage.
- Added box parallel raycast miss coverage.
- Added rotated box parallel raycast miss coverage.
- Added rotated plane distance coverage for sphere-plane.
- Added rotated box raycast coverage and rotated box/cylinder plane distance checks.
- Added rotated cylinder raycast coverage.
- Added rotated cylinder parallel raycast miss coverage.
- Added box-plane offset distance coverage.
- Added group-group distance coverage to validate pair selection and nearest points.
- Added diagonal box-box distance coverage to validate oblique nearest points.
- Added group-group distance filter coverage to validate filter application.
- Added overlapping sphere distance coverage for negative distance semantics.
- Added overlapping box distance coverage for negative distance semantics.
- Added overlapping cylinder-plane distance coverage for negative distance semantics.
- Added single-group distance filter coverage.
- Added group-group distance coverage for shared-object skipping.
- Added diagonal cylinder-cylinder distance coverage for oblique separation.
- Added perpendicular cylinder-cylinder distance coverage for non-parallel axes.
- Added rotated box-box distance coverage for aligned separation.
- Added skewed box-box distance coverage for mixed rotations.
- Added skewed box-cylinder distance coverage for non-parallel axes.
- Added distance coverage for filters rejecting all pairs.
- Added rotated box-cylinder distance coverage for axial separation.
- Added diagonal box-cylinder distance coverage for oblique separation with axial offsets.
- Added ellipsoid-as-sphere distance coverage.
- Added ellipsoid-as-sphere raycast coverage.
- Added ellipsoid-as-sphere collision coverage.
- Added empty-group distance coverage for single and group-group queries.
- Tightened distance AABB pruning once a penetrating pair is found.
- Added diagonal sphere-cylinder distance coverage for oblique separation with axial offsets.
- Added overlapping sphere-plane distance coverage for negative distance semantics.
- Added overlapping box-plane distance coverage for negative distance semantics.
- Added plane offset coverage for sphere distance and raycast queries.
- Added tilted cylinder-plane distance coverage for oblique extent checks.
- Added tilted cylinder-plane overlap distance coverage.
- Added DistanceResult clamping status coverage.
- Added zero-length raycast coverage.
- Added coverage for DistanceOption without nearest points.
- Added empty-group raycast coverage.
- Added empty-group collision coverage.
- Added missing-shape distance coverage.
- Added missing-shape raycast coverage.
- Added missing-shape collision coverage.
- Moved raycast implementation into DartCollisionEngine with detector forwarding.
- Added minor raycast early-exit and hit reservation optimizations.
- Added a DART raycast benchmark covering closest-hit and all-hit queries.
- Moved core engine implementation files into `dart/collision/dart/engine` and updated includes to use the engine path.
- Renamed adapter file names to `DartCollision*` and updated includes to match the new paths.
- Gated DART raycast behind `DARTCollisionDetector::setRaycastEnabled` to keep the gazebo ray intersection expectations unchanged; DART raycast tests and benchmarks enable it explicitly.
- Verified `pixi run -e gazebo test-gz` passes after the layout and raycast gating changes.
- Fixed CI build issues by adding an out-of-line `DARTCollisionDetector` destructor, updating distance filter tests to use shared_ptrs, and formatting the engine file.
- Fixed plane AABB handling to avoid NaNs and restored single-group distance result ordering to match group insertion order.
- Added axis-aligned distance paths for box-box, cylinder-box, and parallel cylinder-cylinder cases, plus plane-aligned nearest point selection for box-plane, sphere-plane, and cylinder-plane.
- Updated ellipsoid-as-sphere core radius to use diameters.
- Fixed box inside-hit raycast normal/fraction selection.
- Latest local runs: `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R UNIT_collision_DartRaycast` and `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R UNIT_collision_DartDistance` pass.
- Captured raycast benchmark baseline via `pixi run bm bm_raycast_dart -- --benchmark_filter=BM_RaycastDart` (CPU scaling enabled; results may be noisy).
- Added tilted plane nearest-point coverage for sphere-plane and cylinder-plane distances.
- Added tilted plane offset coverage for cylinder-plane nearest points.
- Added tilted plane offset coverage for sphere-plane nearest points.
- Added tilted plane offset coverage for box-plane nearest points.
- Switched the default collision detector to the built-in detector and
  deprecated backend-selection APIs.
- Migrated examples and tutorials away from backend-selection APIs and updated
  relevant docs to describe legacy backends.

## Next Actions

- Extend raycast coverage to edge cases and future shape types.
- Expand distance coverage for additional rotated or oblique configurations and refine nearest-point accuracy.
- Explore additional distance broadphase pruning and candidate ordering improvements.
- Validate build and install paths after the engine layout change.
- Revisit the raycast enablement default once gazebo expectations change.
- Audit remaining downstream-facing docs/examples for deprecated backend
  selection and update as needed.

## Questions and Decisions Needed

- Confirm any additional raycast API surface changes required before standalone packaging.

## Decisions Captured

- Core namespace: `dart::collision`.
- Implementation lives under `dart/collision/dart`.
- Core builds into the `dart` target for now (no separate CMake target).
- Default collision detector is the built-in detector, with backend selection
  deprecated during the migration window.
- No additional downstream constraints beyond keeping the gazebo integration workflow green.
- Engine class name: `DartCollisionEngine`.
