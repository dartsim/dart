# Multi Solver World Refactor Plan (01)

## Status

- Completed on `feature/multi_solver-next`.
- Last local test: `DART_PARALLEL_JOBS=12 CTEST_PARALLEL_LEVEL=12 pixi run test`.

## Goal

- Establish an internal ECS object onboarding path while preserving classic
  solver behavior.

## Scope

- Add internal ECS entity lifecycle helpers in `detail::WorldEcsAccess`
  (create/destroy/valid) so new ECS objects can be registered without exposing
  EnTT in public headers.
- Add minimal entity lifecycle hooks on `Solver` using `EcsEntity`, called
  by World on create/destroy to keep solvers in sync.
- Keep classic Skeleton data on the legacy path and notify only the solver
  configured by `WorldConfig::SolverRouting` (default: `ClassicRigidSolver`).
- Route ECS-backed simulation objects (`simulation::object::Object` derivatives)
  to the solver configured by `WorldConfig::SolverRouting` (default:
  `RigidSolver`).
- Extend the ECS-backed rigid solver to update ECS-only components (kept in
  `dart/simulation/detail`, not installed), with no Skeleton mirroring.
- Add integration tests for solver enable/disable and ECS entity lifecycle.

## Progress (Implemented)

- World routes Skeletons and ECS entities to the configured solver types:
  `dart/simulation/World.hpp`, `dart/simulation/World.cpp`.
- ECS entity lifecycle helpers added to `detail::WorldEcsAccess`:
  `dart/simulation/detail/WorldEcsAccess.hpp`, `dart/simulation/World.cpp`.
- Solver entity hooks added in `dart/simulation/solver/Solver.hpp` and
  plumbed through World.
- ECS-backed rigid solver updates ECS-only state:
  `dart/simulation/solver/rigid/RigidSolver.hpp`,
  `dart/simulation/solver/rigid/RigidSolver.cpp`,
  `dart/simulation/detail/RigidSolverComponents.hpp`.
- Tests cover routing, enable/disable, and ECS lifecycle:
  `tests/integration/simulation/test_World.cpp`.
- Solver type enum split into `dart/simulation/solver/SolverTypes.hpp` so
  `World.hpp` can forward declare `Solver`.

## Non-Goals

- No public EnTT exposure or raw solver pointers.
- No classic solver deprecation.
- No `dart8/` removal or migration in this milestone.
- No full ECS physics pipeline (constraints/collision) yet.

## Open Questions (Next Milestone)

- Ownership model: default to World-owned ECS entity lifetimes; objects hold
  `EcsEntity` + `World*` handles.
- Component placement: keep ECS-only components in `dart/simulation/detail`
  and exclude them from installed headers until the public object API is ready.

## Test Plan

- `pixi run test`
- `DART_PARALLEL_JOBS=8 pixi run -e gazebo test-gz`
- `pixi run test-all` before PR

## Next Plan

- See `docs/dev_tasks/multi_solver/02_next.md`.
