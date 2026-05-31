# PLAN-080: Rigid-Body Dynamics Solver (First Experimental Solver)

- Operating state: `PLAN-080` in [`dashboard.md`](dashboard.md)
- Outcome: the experimental `World` runs full rigid-body dynamics — gravity,
  articulated-body forward dynamics, collision-driven contacts, and constraint
  solving — matching legacy DART 6 on shared scenes and then improving on it,
  delivered as the first solver under a multi-solver, multi-physics architecture
  that keeps the public facade free of solver, coupler, and backend types.
- Current evidence:
  - Experimental `step()` is only basic free-body integration with no gravity,
    no articulated dynamics, no contacts, no constraint solver
    (`dart/simulation/experimental/compute/world_step_stage.cpp`).
  - Legacy DART 6 dynamics baseline: `dart/dynamics/`, `dart/constraint/`,
    `dart/simulation/world.cpp`.
  - Reusable foundations: boxed-LCP library `dart/math/lcp/` (PLAN-020), native
    collision engine `dart/collision/native/` (PLAN-035/036/037), compute-graph
    executor in `dart/simulation/experimental/compute/`.

## Owner Docs

- Architecture rationale:
  [`../design/simulation_solver_architecture.md`](../design/simulation_solver_architecture.md)
- Public facade:
  [`../design/simulation_experimental_cpp_api.md`](../design/simulation_experimental_cpp_api.md),
  [`../design/simulation_experimental_python_api.md`](../design/simulation_experimental_python_api.md)
- Active implementation tracking:
  `docs/dev_tasks/rigid_body_dynamics_solver/` (gap matrix, roadmap, RESUME).

## Workstreams

Sequenced; see the dev-task roadmap for slice-level detail.

1. **Foundations** — World gravity (first slice), force-reset policy,
   single-body quantities (momentum, energy, COM).
2. **Articulated-body forward dynamics** — joint effort input, ABA joint
   accelerations, mass matrix / Coriolis / gravity generalized forces,
   damping/springs, manifold-aware integration.
3. **Collision bridge** — experimental collision geometry/material value
   objects and a World-owned bridge to the native collision engine producing
   typed contact buffers.
4. **Constraint & contact solver** — wire boxed-LCP into contact + joint-limit
   solving with restitution/friction/ERP-CFM, islands, impulse update.
5. **Joint features & actuators** — actuator types, limits, Coulomb friction,
   mimic/coupler, armature.
6. **Loop closures & improvements** — closure projection/solve, pluggable
   integrator/substepping, jacobians, model-format loading bridge.

## Acceptance Criteria

- Each workstream lands as small verifiable slices with focused tests; gap-matrix
  rows flip to PRESENT with evidence.
- Parity gate: experimental dynamics match legacy DART 6 on identical scenes
  (energy-drift bounds and trajectory tolerance) before any DART 7 promotion
  claim for the affected capability.
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
