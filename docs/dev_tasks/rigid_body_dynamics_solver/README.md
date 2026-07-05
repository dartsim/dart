# Rigid-Body Dynamics Solver — Dev Task

Bring the DART 7 `World` (`dart/simulation/`) to DART
6-equivalent — then better — rigid-body simulation, implemented as the **first
solver** under a multi-solver, multi-physics architecture.

## Current Status

- [x] Gap analysis: legacy DART 6 rigid-body sim vs DART 7 World
      ([`01-gap-analysis.md`](01-gap-analysis.md)).
- [x] Architecture design for solvers / domain assignment / coupling /
      model-state separation
      ([`../../design/simulation_solver_architecture.md`](../../design/simulation_solver_architecture.md)).
- [x] Phased roadmap ([`02-roadmap.md`](02-roadmap.md)).
- [x] Phase 0.1: World gravity in rigid-body integration (first implementation
      slice — `World::setGravity`/`getGravity`, dartpy `world.gravity`, tests).
- [x] Phase 0.2: persistent applied force/torque inputs; default stepping
      assembles a transient SoA force buffer and leaves applied loads for
      callers to clear or update explicitly.
- [x] Phase 0.3: rigid-body quantities (linear/angular momentum, kinetic and
      potential energy).
- [x] Phase 1: articulated-body forward dynamics (RNEA-based, including the
      later floating-base and supported joint-type slices; joint
      effort/acceleration, link mass/inertia, link offsets, and public dynamics
      accessors; verified vs analytical dynamics and DART-6 parity checks).
- [x] Phase 2: collision bridge (`CollisionShape` sphere/box/capsule/
      cylinder/plane/triangular mesh on rigid bodies and links, plus
      `MeshShape`/`ConvexMeshShape`/`HeightmapShape`/`SoftMeshShape`
      model-loading translation, multiple collision shapes per body/link, and
      `World::collide()` query bridging to the native engine, returning
      `Contact` points; C++ + dartpy + tests. Filtering covers
      `CollisionQueryOptions::includeSameMultibodyLinkPairs` /
      dartpy `include_same_multibody_link_pairs`; body-type filtering now
      covers rigid-body, rigid-body/link, and link/link pairs; broad-phase
      pruning now uses native collision candidate pairs; `World::collide()`
      now persists the native collision world across queries and only rebuilds
      when collision geometry changes).
- [ ] Phase 3: constraint & contact solver (started: velocity-level sequential
      contact for free rigid bodies; the current semi-implicit default routes
      mixed free-rigid / articulated-link contact through the unified boxed-LCP
      stage when multibody structures are present. Remaining work is Subsystem A
      polish from `RESUME.md`: warm starting, friction-cone iteration, scaling
      around the unified solve, and separate joint-limit/motor/island slices).
- [ ] Phase 4: joint features & actuators (started: spring stiffness, rest
      position, damping, effort limits, armature, joint-space Coulomb friction,
      and Force/Passive/Velocity/Servo/Acceleration/Locked actuator behavior on
      supported joints; remaining: mimic/coupler work).
- [ ] Phase 5: loop closures & improvements.

### DART 7 B2 gate — rigid open-chain dynamics parity

- [x] World-parity harness:
      `tests/unit/simulation/world/test_world_parity.cpp` compares
      classic `dart::simulation::World` vs
      `dart::simulation::World` on shared open-chain scenes
      (gravity free-fall, pendulum/double-pendulum integration, 1e4-step drift,
      and a held-torque controlled scene), within documented tolerances. Runs
      under `pixi run test-simulation`. All open-chain scenarios
      reach parity on `main` to machine epsilon; contact/constraint parity (B3)
      is deferred to the unified solver (PR #2838). See `RESUME.md` for the
      per-scenario table and deferred items.

## Goal

A user can build a rigid-body / articulated scene on the DART 7 `World`,
step it, and get dynamics that match legacy DART 6 (within tolerance) and then
exceed it (armature, pluggable integrator, fresh-by-default reads, backend-
neutral compute, model/state separation) — without the public API exposing
solvers, couplers, ECS storage, or execution backends.

## Boundaries

- Keep public configuration at the method-family / policy level (`WorldOptions`,
  handles, and value objects), not public solver, coupler, ECS, backend, or
  registry types.
- Rigid-body work must continue to compose with the existing multibody,
  deformable, rigid IPC, variational-integrator, and differentiable paths
  through the centralized built-in schedule instead of adding another parallel
  default-step switch.
- Direct file loading remains outside this task; compose through maintained
  `dart::io` readers and the DART 7 skeleton/world conversion bridges.
- Do not modify `/home/js/multiphysics-api-design.md` (external user doc).
- Do not name solvers/presets/examples after other engines; use method/approach
  names. Do not reference specific external engines by name in core code or
  docs.

## Key Decisions

- Dynamics live in solver-shaped units; the `World` owns composition, time, and
  the step schedule. See the architecture design doc.
- Entities are assigned to a solver by **physics domain**, not geometry.
- Coupling is pairwise and swappable; solvers never branch on coupler identity.
- Model/State/Control/Contacts are conceptually separate (precondition for
  batching/differentiability); the easy path hides this.
- Default gravity is `(0, 0, -9.81)`, matching legacy DART 6 and the user's
  multiphysics API vision. Gravity is added to the transient force buffer as
  `mass * gravity`, not stored in the per-body force accumulator.
- Applied rigid-body forces and torques are persistent inputs, matching main's
  #2698 integration convention.
- Parity with DART 6 on shared scenes is the gate before any DART 8 promotion
  claim.

## Immediate Next Steps

1. Treat the model-loading and unified contact/constraint line as landed on
   `main` via PR #2838; do not look for the retired
   `feature/experimental-model-loader` branch as the active publication path.
2. Continue Subsystem A polish from `RESUME.md`: warm starting, friction-cone
   iteration, and other scaling work around the unified contact solve.
3. Keep richer model-loading diagnostics, visual/material import, actuator,
   mimic/coupler, loop-closure, integrator, and COM-Jacobian work as separate
   deferred slices unless the active solver-polish work directly requires them.

## Relationship To The API-Design Dev Task

`docs/dev_tasks/simulation_api_design/` tracks the public _facade
shape_ (handles, naming, Pythonic dartpy). This task tracks the _dynamics
implementation_ behind that facade. Keep facade changes in that task; keep
solver/dynamics changes here. When this task completes, promote durable
decisions into `docs/onboarding/` and the design docs, then delete this folder
in the completing PR.
