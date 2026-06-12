# PLAN-080: Rigid-Body Dynamics Solver (First DART 7 Solver)

- Operating state: `PLAN-080` in [`dashboard.md`](dashboard.md)
- Outcome: the DART 7 `World` runs full rigid-body dynamics — gravity,
  articulated-body forward dynamics, collision-driven contacts, and constraint
  solving — matching legacy DART 6 on shared scenes and then improving on it,
  delivered as the first solver under a multi-solver, multi-physics architecture
  that keeps the public facade free of solver, coupler, and backend types.
- Current evidence:
  - The rigid-body MVP shipped (PR #2705, merged 2026-05-25): `World::step()`
    runs gravity, articulated forward dynamics for all joint types (fixed,
    revolute, prismatic, screw, universal, planar, ball, free), a floating base,
    derived quantities, actuator/limit primitives, a collision-query bridge,
    and the required dart-gui example. The active follow-up line adds the
    legacy `dynamics::Skeleton` / `simulation::World` to DART 7
    `Multibody` bridge, including the retained `addSkeleton()` URI-loading
    facade, joint-family/property transfer, branching and root offsets,
    collision shape import with local transforms, compound shapes, and dartpy
    coverage. It also upgrades `World::collide()` with body-type filtering,
    native broad-phase pruning, and a persistent native collision-query world.
    The semi-implicit default pipeline now resolves rigid-rigid and articulated
    link contacts in a single unified boxed-LCP. Remaining gaps are solver
    scaling polish (warm starting and friction-cone iteration), richer
    model-loading diagnostics/load-result ergonomics, visual/material import,
    multi-DOF joint-property forces, and separate deferred actuator,
    mimic/coupler, loop-closure, integrator, and COM-Jacobian slices.
  - Legacy DART 6 dynamics baseline: `dart/dynamics/`, `dart/constraint/`,
    `dart/simulation/world.cpp`.
  - Reusable foundations: boxed-LCP library `dart/math/lcp/` (PLAN-020), native
    collision engine `dart/collision/native/` (PLAN-035/036/037), compute-graph
    executor in `dart/simulation/compute/`.

## Owner Docs

- Architecture rationale:
  [`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md)
- Public facade:
  [`../design/simulation_cpp_api.md`](../design/simulation_cpp_api.md),
  [`../design/simulation_python_api.md`](../design/simulation_python_api.md)
- Active implementation tracking:
  `docs/dev_tasks/rigid_body_dynamics_solver/` (gap matrix, roadmap, RESUME).

## Workstreams

Sequenced; see the dev-task roadmap for slice-level detail.

1. **Foundations** — World gravity (first slice), force-reset policy,
   single-body quantities (momentum, energy, COM).
2. **Articulated-body forward dynamics** — joint effort input, ABA joint
   accelerations, mass matrix / Coriolis / gravity generalized forces,
   damping/springs, manifold-aware integration.
3. **Collision bridge** — DART 7 collision geometry/material value
   objects and a World-owned bridge to the native collision engine producing
   typed contact buffers with broad-phase-pruned, filterable, cached queries.
4. **Constraint & contact solver** — wire boxed-LCP into contact + joint-limit
   solving with restitution/friction/ERP-CFM, islands, impulse update, and
   solver scaling polish.
5. **Joint features & actuators** — actuator types, limits, Coulomb friction,
   mimic/coupler, armature.
6. **Loop closures & improvements** — closure projection/solve, pluggable
   integrator/substepping, jacobians, model-format loading bridge.

## Acceptance Criteria

- Each workstream lands as small verifiable slices with focused tests; gap-matrix
  rows flip to PRESENT with evidence.
- Parity gate: DART 7 dynamics match legacy DART 6 on identical scenes
  (energy-drift bounds and trajectory tolerance) using `release-6.*` branch
  refs before any DART 7 promotion claim for the affected capability.
- The public API never exposes `Solver`, `Coupler`, `PhysicsDomain`, schedule
  phase types, ECS storage, component types, execution-backend names, or the
  registry; `pixi run check-api-boundaries` stays green.
- Multi-solver readiness held throughout: dynamics stay in a solver-shaped unit,
  the substep schedule stays expressible, and Model/State/Control/Contacts stay
  conceptually separate so a second solver or domain can be added without
  reshaping the `World`.

## Revision Triggers

- A phase completes, splits, or is reprioritized.
- The native-collision bridge or boxed-LCP contract changes shape.
- Parity evidence reveals a needed design change in the architecture doc.
- A maintainer reprioritizes against other DART 7 clean-break work.
