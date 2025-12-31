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
- Commit after each checkpoint.

Decisions:
- Core namespace is `dart::collision`.
- Implementation lives under `dart/collision/dart` for now.
- Core builds into the `dart` target (no standalone CMake target yet).
- Default detector switch happens only after parity and performance are proven.
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
- Distance tests include a rotated box case; raycast tests include a rotated plane hit.
- Raycast tests include tangent sphere hits and parallel plane misses.
- Distance tests include cylinder-cylinder nearest points; raycast tests include tangent cylinder hits.
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
- Distance tests include rotated box-box coverage for aligned separation.
- Distance tests include coverage for filters rejecting all pairs.
- Distance tests include rotated box-cylinder coverage for axial separation.
- Distance tests include ellipsoid-as-sphere coverage.
- Raycast tests include ellipsoid-as-sphere coverage.
- Collision tests include ellipsoid-as-sphere coverage.
- Distance tests include empty-group coverage for single and group-group queries.
- Distance pruning skips separated pairs once a negative minimum distance is found.

Where to look:
- Core engine and adapter: `dart/collision/dart/`.
- Task tracking: `docs/dev_tasks/collision_detector/00_plan.md`, `docs/dev_tasks/collision_detector/01_next.md`, `docs/dev_tasks/collision_detector/02_architecture.md`, and `docs/dev_tasks/collision_detector/progress.md`.

Next focus:
- Expand distance coverage for additional rotated or oblique configurations and refine nearest-point accuracy.
- Extend raycast coverage to additional edge cases and future shape types.
- Decide when to move raycast into the core engine once query types stabilize.
- Keep task docs updated after each checkpoint.
- Keep task docs updated after each checkpoint.
