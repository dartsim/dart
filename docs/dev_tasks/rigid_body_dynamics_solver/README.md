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

### DART 7 B2 gate — rigid open-chain dynamics parity

- [x] DART 7 regression harness:
      `tests/unit/simulation/world/test_world_dart7_regression.cpp` covers
      gravity free-fall, pendulum/double-pendulum integration, 1e4-step drift,
      static-ground contact settling, and a held-torque controlled scene using
      only the DART 7 `World` API. The retired same-branch classic-vs-DART-7
      parity harness established machine-epsilon agreement before promotion;
      future cross-version parity evidence belongs on `release-6.*` branches,
      not in main-branch classic World tests.

## Goal

A user can build a rigid-body / articulated scene on the DART 7 `World`,
step it, and get dynamics that match legacy DART 6 (within tolerance) and then
exceed it (armature, pluggable integrator, fresh-by-default reads, backend-
neutral compute, model/state separation) — without the public API exposing
solvers, couplers, ECS storage, or execution backends.

## Non-Goals (for early phases)

- No second solver or non-rigid domain implementation yet (architecture must
  _allow_ it; see roadmap "multi-solver readiness").
- No public solver/coupler/domain types or registries.
- No direct file-based loader in the DART 7 simulation library yet; compose
  through `dart::io::readSkeleton` and the DART 7 skeleton-to-multibody bridge.
  Use `release-6.*` branches for DART 6 whole-World loader parity evidence.
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
2. Continue Subsystem A polish from `RESUME.md`: warm starting, friction-cone
   iteration, and other scaling work around the unified contact solve.

## Relationship To The API Design Docs

`docs/design/simulation_cpp_api.md` and
`docs/design/simulation_python_api.md` track the public _facade shape_ (handles,
naming, Pythonic dartpy). This task tracks the _dynamics implementation_ behind
that facade. Keep facade changes in the design docs; keep solver/dynamics
changes here. When this task completes, promote durable decisions into
`docs/onboarding/` and the design docs, then delete this folder
in the completing PR.
