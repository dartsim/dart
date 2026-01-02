# Multi Solver World Refactor Plan (00)

## Status

- Completed: PR 00 landed on `main` (multi-solver scaffolding).
- Completed: follow-up milestone 01 implemented on `feature/multi_solver-next`.
- Next plan: see `docs/dev_tasks/multi_solver/02_next.md`.
- Resume prompt: see `docs/dev_tasks/multi_solver/02_resume_prompt.md`.

## What Exists Today (feature/multi_solver-next)

- World orchestration and config: `dart/simulation/World.hpp`, `dart/simulation/World.cpp`
  - `WorldConfig`, `WorldConfig::SolverRouting`, `CollisionDetectorType`,
    `RigidSolverType`
  - Solver enable/disable and ordering controls
- Solver abstraction and implementations:
  - Solver types: `dart/simulation/solver/SolverTypes.hpp`
  - Base: `dart/simulation/solver/Solver.hpp`, `dart/simulation/solver/Solver.cpp`
  - Classic solver wrapper: `dart/simulation/solver/classic_rigid/ClassicRigidSolver.hpp`, `dart/simulation/solver/classic_rigid/ClassicRigidSolver.cpp`
  - ECS-backed rigid scaffold: `dart/simulation/solver/rigid/RigidSolver.hpp`, `dart/simulation/solver/rigid/RigidSolver.cpp`
- ECS scaffolding:
  - Opaque entity handle: `dart/simulation/EcsEntity.hpp`
  - ECS access bridge (internal-only): `dart/simulation/detail/WorldEcsAccess.hpp`
    (registry access + EnTT conversion helpers)
  - ECS-only rigid components (internal-only): `dart/simulation/detail/RigidSolverComponents.hpp`
  - Object scaffolding: `dart/simulation/object/Object.hpp`,
    `dart/simulation/object/TypeList.hpp` (public), plus internal-only
    `dart/simulation/object/ObjectWith.hpp`
- Tests:
  - Solver stepping, routing, enable/disable coverage:
    `tests/integration/simulation/test_World.cpp`
- Build deps:
  - EnTT added as required dep: `cmake/DARTFindDependencies.cmake`, `dart/CMakeLists.txt`, `package.xml`

## First PR Milestone (Small, Shippable)

### Scope

- Preserve classic Skeleton solver behavior and public APIs.
- Keep multi-solver scheduling in `World`.
- Keep ECS registry ownership in `World` and the sync-only `RigidSolver` path.
- Hide EnTT from installed/public headers as much as possible.
- Keep test coverage for solver stepping order and world behavior.

### Non-Goals

- No new ECS simulation objects or physics behavior.
- No deprecation or removal of the classic solver.
- No changes to `dart8/`.
- No public exposure of `entt::registry` or raw solver pointers.

## Public API Decisions

- `World` exposes config via:
  - `WorldConfig` (`World::create(WorldConfig)`), including
    `WorldConfig::SolverRouting` for mapping Skeletons (and reserving an
    object slot for future ECS object APIs) to solver types, plus
    `CollisionDetectorType` and `RigidSolverType`.
- `World` does not expose solver pointers; control is via enable/disable and
  ordering APIs.
- `Solver` is the base solver name; `WorldSolver` remains as a compatibility
  alias for existing code.
- `RigidSolverType` lives in `dart/simulation/solver/SolverTypes.hpp` so
  `World.hpp` can forward declare `Solver` without pulling solver internals.
- `EcsEntity` is the only public ECS handle (opaque value type).
- EnTT types must not appear in installed headers (see “Internal Design”).
- Skeletons are handled exclusively by `ClassicRigidSolver` for now; World
  asserts if a different solver type is requested.

## Internal Design (Milestone Target)

- World owns solvers as `SolverEntry` and steps all enabled solvers in
  registration order each frame. Skeletons are routed to a single solver using
  `WorldConfig::SolverRouting` (fixed to Classic for now); object routing is
  configured but deferred until ECS object lifecycle APIs exist.
- Classic solver is the default constraint/collision backend and remains
  Skeleton-based.
- ECS registry lives in `World::EcsData` (private in `World.cpp`).
- Classic Skeletons remain on the legacy Aspect/Properties/Composite path and
  are routed to the solver selected by `WorldConfig::SolverRouting` (default:
  `ClassicRigidSolver`).
- `RigidSolver` is ECS-only; it syncs ECS components but does not consume
  Skeleton data (default object solver via `WorldConfig::SolverRouting`).
- EnTT access is internal-only:
  - `WorldEcsAccess.hpp` and `ObjectWith-impl.hpp` include EnTT but are
    excluded from installed headers and component aggregates.
  - Public ECS scaffolding (`EcsEntity`, `Object`, `TypeList`) remains
    EnTT-free.

## Migration Direction (dart8)

- Keep `dart8/` intact for now.
- Long-term plan: bring ECS-first workflows into `dart/` and migrate features,
  then remove `dart8/` in a later, low-risk PR once parity is clear.

## Test Plan

- Quick: `pixi run test`
- Gazebo gate: `DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz`
- Final: `pixi run test-all`
- Targeted: keep/extend `tests/integration/simulation/test_World.cpp` to
  validate solver scheduling and enable/disable behavior.

## Workflow Constraints (Docs-First)

- Use `pixi run ...` tasks only (no new entry points).
- Follow code style in `docs/onboarding/code-style.md`.
- Keep docs concise; avoid footer metadata in Markdown.
- Gazebo integration policy is documented in
  `docs/onboarding/build-system.md` (no local gz-physics patches beyond
  version requirement).
