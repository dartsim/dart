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

Where to look:
- Core engine and adapter: `dart/collision/dart/`.
- Task tracking: `docs/dev_tasks/collision_detector/00_plan.md`, `docs/dev_tasks/collision_detector/01_next.md`, `docs/dev_tasks/collision_detector/02_architecture.md`, and `docs/dev_tasks/collision_detector/progress.md`.

Next focus:
- Expand distance coverage to additional primitives and refine nearest-point accuracy.
- Extend raycast coverage to edge cases and future shape types.
- Keep task docs updated after each checkpoint.
- Keep task docs updated after each checkpoint.
