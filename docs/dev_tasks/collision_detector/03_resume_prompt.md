# Collision Detector Task - Resume Prompt

Use this prompt to resume the task with a fresh Codex agent.

## Prompt

You are Codex in the DART repo at `/home/js/dev/dartsim/dart/lcp`.

Read first:

- `AGENTS.md`
- Onboarding docs under `docs/onboarding/`
- `CONTRIBUTING.md`
- `docs/dev_tasks/README.md`
- Collision detector task docs under `docs/dev_tasks/collision_detector/`

Constraints:

- Use `pixi run ...` for tooling (no new entry points).
- Use `rg` for search.
- Avoid non-ASCII in edits.
- Follow C++20 style and naming (PascalCase under `dart/`).
- Do not mention external backend names in any code, comments, or docs.
- Keep DART public API stable; use deprecation paths if needed.
- Commit in reasonable chunks and push at logical times.

Decisions:

- Core namespace is `dart::collision`.
- Implementation lives under `dart/collision/dart` for now.
- Core builds into the `dart` target (no standalone CMake target yet).
- Default collision detector is the built-in detector; parity and performance
  tracking now inform legacy backend removal.
- Engine class name is `DartCollisionEngine`.
- Future direction: DART8 consolidates under `dart/collision/`, dropping other backends.

Current status:

- Phase 2 (core implementation) is in progress.
- Core data structs exist (`CoreShape`, `CoreObject`) with cached world AABBs.
- Broadphase uses a sweep-based pass for MVP.
- Narrowphase supports sphere, box, cylinder, and plane primitives, including box-cylinder and cylinder-cylinder.
- Raycast is implemented for those primitives with AABB pruning and unit coverage.
- Distance query MVP is implemented for primitives with unit coverage and benchmarks.
- Distance queries use sweep pruning plus AABB lower-bound checks.
- The architecture proposal documents the current core/adapter boundary and raycast placement.
- Distance tests cover sphere-plane, sphere-box, sphere-cylinder, and box-cylinder nearest points.
- Raycast tests include inside-hit cases for sphere, box, and cylinder.
- Raycast tests include surface-start hits for sphere, box, cylinder, and plane.
- Raycast tests include parallel plane surface-start misses.
- Distance tests include a rotated box case; raycast tests include a rotated plane hit.
- Raycast tests include tangent sphere hits and parallel plane misses.
- Distance tests include cylinder-cylinder nearest points; raycast tests include tangent cylinder hits.
- Raycast tests include cylinder cap rim hits.
- Raycast tests include cylinder parallel misses.
- Raycast tests include tangent box hits.
- Raycast tests include box parallel misses.
- Raycast tests include rotated box parallel misses.
- Distance tests include a rotated plane case for sphere-plane queries.
- Distance tests include rotated box/cylinder plane cases; raycast tests include rotated box hits.
- Raycast tests include rotated cylinder hits.
- Distance tests include a group-group case to validate pair selection and nearest points.
- Distance tests include diagonal box-box coverage for oblique nearest points.
- Distance tests include group-group filter coverage for distance queries.
- Distance tests include overlapping sphere coverage for negative distances.
- Distance tests include overlapping box coverage for negative distances.
- Distance tests include overlapping cylinder-plane coverage for negative distances.
- Distance tests include single-group filter coverage for distance queries.
- Distance tests include group-group shared-object skip coverage.
- Distance tests include diagonal cylinder-cylinder coverage for oblique separation.
- Distance tests include perpendicular cylinder-cylinder coverage for non-parallel axes.
- Distance tests include rotated box-box coverage for aligned separation.
- Distance tests include skewed box-box coverage for mixed rotations.
- Distance tests include skewed box-cylinder coverage for non-parallel axes.
- Distance tests include coverage for filters rejecting all pairs.
- Distance tests include rotated box-cylinder coverage for axial separation.
- Distance tests include diagonal box-cylinder coverage for oblique separation with axial offsets.
- Distance tests include ellipsoid-as-sphere coverage.
- Raycast tests include ellipsoid-as-sphere coverage.
- Collision tests include ellipsoid-as-sphere coverage.
- Distance tests include empty-group coverage for single and group-group queries.
- Distance pruning skips separated pairs once a negative minimum distance is found.
- Distance tests include diagonal sphere-cylinder coverage for oblique separation with axial offsets.
- Distance tests include overlapping sphere-plane coverage for negative distances.
- Distance tests include overlapping box-plane coverage for negative distances.
- Distance tests include plane offset coverage for sphere queries; raycast tests include plane offset hits.
- Distance tests include a tilted cylinder-plane case for oblique extent coverage.
- Distance tests include DistanceResult clamping status coverage.
- Raycast tests include zero-length ray coverage.
- Distance tests include coverage for DistanceOption without nearest points.
- Raycast tests include empty-group coverage.
- Collision tests include empty-group coverage.
- Distance tests include missing-shape coverage.
- Raycast tests include missing-shape coverage.
- Collision tests include missing-shape coverage.
- Raycast runs in DartCollisionEngine with detector forwarding.
- Raycast reserves hits for all-hit queries and exits early on zero-fraction hits.
- Added a DART raycast benchmark covering closest-hit and all-hit queries.
- Core engine implementation files live under `dart/collision/dart/engine`, with adapter headers in `dart/collision/dart`.
- Adapter file names use the `DartCollision*` prefix; class names remain unchanged.
- DART raycast is gated behind `DARTCollisionDetector::setRaycastEnabled`.
- Added axis-aligned distance paths for box-box, cylinder-box, and parallel cylinder-cylinder cases.
- Added plane-aligned nearest point selection for box-plane, sphere-plane, and cylinder-plane distances.
- Updated ellipsoid-as-sphere core radius to use diameters.
- Fixed box inside-hit raycast normal/fraction selection.
- Latest local runs: `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R UNIT_collision_DartRaycast` and `pixi run ctest --test-dir build/default/cpp/Release --output-on-failure -R UNIT_collision_DartDistance` pass.
- Captured raycast benchmark baseline via `pixi run bm bm_raycast_dart -- --benchmark_filter=BM_RaycastDart` (CPU scaling enabled; results may be noisy).
- Added tilted plane nearest-point coverage for sphere-plane and cylinder-plane distances.
- Default collision detector switched to the built-in detector in world and
  constraint solver initialization.
- Backend-selection APIs are deprecated; legacy detector classes carry
  deprecation notes.
- Examples/tutorials migrated off backend-selection APIs; tests/benchmarks
  suppress deprecated calls.
- `.skel` collision detector selection is deprecated and ignored (with warning).

Where to look:

- Core engine and adapter: `dart/collision/dart/`.
- Core engine implementation files: `dart/collision/dart/engine/`.
- Task tracking: `docs/dev_tasks/collision_detector/00_plan.md`, `docs/dev_tasks/collision_detector/01_next.md`, `docs/dev_tasks/collision_detector/02_architecture.md`, and `docs/dev_tasks/collision_detector/progress.md`.
- CI monitoring: follow `docs/onboarding/ci-cd.md` (use `gh run list` and `gh run watch`).

Next focus:

- Expand distance coverage for additional rotated or oblique configurations and refine nearest-point accuracy.
- Extend raycast coverage to additional edge cases and future shape types.
- Keep parity/performance tracking current for legacy backend removal.
- Keep task docs updated after each checkpoint.
