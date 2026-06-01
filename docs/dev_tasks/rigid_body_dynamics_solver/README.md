# Rigid-Body Dynamics Solver — Dev Task

Bring the experimental `World` (`dart/simulation/experimental/`) to DART
6-equivalent — then better — rigid-body simulation, implemented as the **first
solver** under a multi-solver, multi-physics architecture.

## Current Status

- [x] Gap analysis: legacy DART 6 rigid-body sim vs experimental World
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
- [ ] Phase 2: collision bridge (started: `CollisionShape` sphere/box/capsule
      on rigid bodies and links + `World::collide()` query bridging to the
      native engine, returning `Contact` points; C++ + dartpy + tests.
      Remaining: cylinder/plane/mesh, self-collision/filtering, broad-phase
      pruning, persistent collision world).
- [ ] Phase 3: constraint & contact solver (started: velocity-level sequential
      contact solver between free rigid bodies with accumulated normal impulses,
      restitution, two-tangent Coulomb friction, positional correction, and a
      public static-body convention — a body drops onto a static ground and
      rests, an elastic head-on collision swaps velocities, and a sliding box
      brakes via friction. Remaining: joint-limit/motor constraints, contacts on
      multibody links, islands, and the boxed-LCP formulation).
- [ ] Phase 4: joint features & actuators (started: spring stiffness + rest
      position and damping are applied as passive generalized forces for
      revolute/prismatic joints; remaining: actuator types, limits, Coulomb
      friction, mimic/coupler, armature).
- [ ] Phase 5: loop closures & improvements.

## Goal

A user can build a rigid-body / articulated scene on the experimental `World`,
step it, and get dynamics that match legacy DART 6 (within tolerance) and then
exceed it (armature, pluggable integrator, fresh-by-default reads, backend-
neutral compute, model/state separation) — without the public API exposing
solvers, couplers, ECS storage, or execution backends.

## Non-Goals (for early phases)

- No second solver or non-rigid domain implementation yet (architecture must
  _allow_ it; see roadmap "multi-solver readiness").
- No public solver/coupler/domain types or registries.
- No direct file-based loader in the experimental library yet; compose through
  legacy `dart::io::readSkeleton`/`readWorld` and the experimental
  skeleton-to-multibody bridge.
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

1. Keep the local `feature/experimental-model-loader` branch clean and validated
   while it carries the model-loading/contact follow-up commits ahead of origin.
2. Continue remaining Phase 2 shape backlog from `RESUME.md`: cylinder/plane/
   mesh support and additional collision shapes per body.
3. Continue Subsystem A polish from `RESUME.md`: warm starting, friction-cone
   iteration, and other scaling work around the unified contact solve.

## Relationship To The API-Design Dev Task

`docs/dev_tasks/simulation_experimental_api_design/` tracks the public _facade
shape_ (handles, naming, Pythonic dartpy). This task tracks the _dynamics
implementation_ behind that facade. Keep facade changes in that task; keep
solver/dynamics changes here. When this task completes, promote durable
decisions into `docs/onboarding/` and the design docs, then delete this folder
in the completing PR.
