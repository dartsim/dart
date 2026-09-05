# Rigid-Body Dynamics Solver — Dev Task

Develop dependable DART 7 rigid-body and articulated simulation under
[PLAN-080](../../plans/080-rigid-body-dynamics-solver.md), using the readiness
and prerequisite contracts in [PLAN-040](../../plans/040-dart7-release-hardening.md).

## Current Status

Routing refreshed 2026-09-04. Start with WP-040.1; WP-080.1 requires accepted
WP-040.1 and WP-030.3. Follow their plan dependencies before claiming a physics
packet. The slice checkboxes below preserve implementation history and do not
close the current milestones. Contact-polish details remain backlog until
admitted by the current PLAN-080 packet sequence.

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
      stage when multibody structures are present. Remaining backlog includes
      polish from `RESUME.md`: warm starting, friction-cone iteration, scaling
      around the unified solve, and separate joint-limit/motor/island slices).
- [ ] Phase 4: joint features & actuators (started: spring stiffness, rest
      position, damping, effort limits, armature, joint-space Coulomb friction,
      and Force/Passive/Velocity/Servo/Acceleration/Locked actuator behavior on
      supported joints; remaining: mimic/coupler work).
- [ ] Phase 5: loop closures & improvements.

### DART 7 B2 gate — rigid open-chain dynamics parity

> Historical evidence (2026-09-01): the parity harness cited below
> (`tests/unit/simulation/experimental/world/test_world_parity.cpp`) was
> removed when the experimental World was promoted in PR #2932, and nothing
> replaced it. The checkbox records the gate as it passed in June 2026 (the
> harness landed via PR #2842). Current acceptance follows independent oracles
> in PLAN-040; DART 6 comparisons are supplemental. Existing promotion-checker
> requirements remain enforced until WP-040.2 migrates them.

- [x] World-parity harness (historical, see the note above):
      `test_world_parity.cpp` compared the classic DART 6 world against the
      then-experimental DART 7 world on shared open-chain scenes
      (gravity free-fall, pendulum/double-pendulum integration, 1e4-step drift,
      and a held-torque controlled scene), within documented tolerances. It ran
      under the then-existing simulation label. All open-chain scenarios
      reach parity on `main` to machine epsilon; contact/constraint parity (B3)
      is deferred to the unified solver (PR #2838). See `RESUME.md` for the
      per-scenario table and deferred items.

## Goal

A user can build and step a rigid-body or articulated scene through an intuitive
DART 7 API, with correctness established by independent physical oracles.
Common workflows use validated defaults; advanced method/device preferences
follow the [public API boundary](../../onboarding/api-boundaries.md).

## Boundaries

- Keep common configuration simple and validate explicit advanced selections;
  follow the public API boundary for method/policy values and private internals.
- Rigid-body work must continue to compose with the existing multibody,
  deformable, rigid IPC, variational-integrator, and differentiable paths
  through the centralized built-in schedule instead of adding another parallel
  default-step switch.
- Direct file loading remains outside this task; compose through maintained
  `dart::io` readers and the DART 7 skeleton/world conversion bridges.
- Do not modify the maintainer's external multiphysics API design notes (an untracked personal document).
- Use method/approach names for solvers, presets and examples. Named engines
  are permitted in cited comparative research under the
  [current research scope decision](../../design/simulation_solver_architecture.md#verified-engine-lessons-and-dart-inferences).

## Key Decisions

- [Solver architecture](../../design/simulation_solver_architecture.md) owns
  domain/representation compatibility, solver portfolios, shared or partitioned
  coupling, and model/state/control ownership. [Compute decisions](../../design/scalable_compute_decisions.md)
  owns data/execution contracts; PLAN-040 and PLAN-080 own readiness and admitted work.
- Default gravity is `(0, 0, -9.81)`, matching legacy DART 6 and the user's
  multiphysics API vision. Gravity is added to the transient force buffer as
  `mass * gravity`, not stored in the per-body force accumulator.
- Applied rigid-body forces and torques are persistent inputs, matching main's
  #2698 integration convention.
- PLAN-040 owns DART 7 readiness. DART 8 is a future major release, not the
  promotion target for this task.

## Immediate Next Steps

1. Treat the model-loading and unified contact/constraint line as landed on
   `main` via PR #2838; do not look for the retired
   `feature/experimental-model-loader` branch as the active publication path.
2. Read PLAN-040 and PLAN-080, then use the dashboard to claim the next ready
   packet. Start with WP-040.1; WP-080.1 is blocked until WP-040.1 and WP-030.3
   are accepted. Subsystem A polish is not an independent next-step override.
3. Keep richer model-loading diagnostics, visual/material import, actuator,
   mimic/coupler, loop-closure, integrator, and COM-Jacobian work as separate
   deferred slices unless the active solver-polish work directly requires them.

## Relationship To The API-Design Dev Task

PLAN-041 and PLAN-042 (`docs/plans/041-official-simulation-api-promotion.md`,
`docs/plans/042-dart7-public-api-and-source-layout.md`) own the public _facade
shape_ (handles, naming, Pythonic dartpy); the former
`docs/dev_tasks/simulation_api_design/` folder is retired. This task tracks the
_dynamics implementation_ behind that facade. Keep facade changes there; keep
solver/dynamics changes here. When this task completes, promote durable
decisions into `docs/onboarding/` and the design docs, then delete this folder
in the completing PR.
