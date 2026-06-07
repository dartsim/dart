# Roadmap: Rigid-Body Dynamics Solver (First Solver)

Historical phased roadmap for the first rigid-body solver. It is useful for
understanding why the slices were ordered this way, but it is not current status.
For live status use [`README.md`](README.md),
[`RESUME.md`](RESUME.md), and the durable solver architecture design.

Phased plan to bring the experimental `World` to DART 6-equivalent (then better)
rigid-body simulation as the first solver under the architecture in
[`docs/design/simulation_solver_architecture.md`](../../design/simulation_solver_architecture.md).

Each phase is multi-session and should land as several small, independently
verifiable slices (one PR per slice). Every slice runs `pixi run lint`, focused
C++ tests under `tests/unit/simulation/experimental/`, `pixi run build`, and
`pixi run test-py` when dartpy changes. Update the gap matrix
([`01-gap-analysis.md`](01-gap-analysis.md)) rows as they flip to PRESENT.

## Phase 0 — Foundations (single-body dynamics)

Goal: a free rigid body falls correctly under gravity; the integration path is
trustworthy and tested.

- **Slice 0.1 (first slice): World gravity.** Add `World::setGravity`/
  `getGravity` (default `(0, 0, -9.81)`); apply gravity as an acceleration in
  `RigidBodyIntegrationStage` (`a = F/m + g`), not by mutating the force
  accumulator. dartpy property + stub. Update zero-gravity assumptions in
  existing tests; add a free-fall test. Changelog.
- Slice 0.2: define per-step control/force reset policy for rigid bodies
  (decide and document whether the force accumulator persists or clears each
  step; align with the Model/State/Control separation).
- Slice 0.3: rigid-body quantities — linear/angular momentum, kinetic/potential
  energy, COM (single body) as public read APIs with tests.

## Phase 1 — Articulated-body forward dynamics

Goal: tree multibodies simulate under gravity and joint forces, matching DART 6
forward dynamics for open chains.

- Joint generalized force/effort input on `Joint` (Control).
- Articulated-body forward dynamics (Featherstone ABA) producing joint
  accelerations; integrate generalized position/velocity with manifold-aware
  joint integration.
- Mass matrix, Coriolis, gravity generalized forces (and accessors).
- Joint damping and spring stiffness/rest position in the dynamics.
- Verify against legacy DART 6 on shared scenes (double pendulum, falling chain)
  within tolerance.

## Phase 2 — Collision bridge

Goal: experimental bodies/links carry collision geometry and produce contact
data via the maintained native collision engine.

- Public collision-shape/material value objects on rigid bodies and links
  (box, sphere, capsule, cylinder, plane, mesh) — facade over geometry.
- World-owned collision query bridge to `dart/collision/native/`; broad-phase +
  narrow-phase producing typed contact buffers (Contacts).
- Self-collision toggle and collision filtering.
- No solving yet — contacts are queries (kinematics-only pipeline can consume
  them).

## Phase 3 — Constraint & contact solver

Goal: DART 6-equivalent constrained stepping (stacking boxes, robot on ground).

- Wire the existing boxed-LCP library (`dart/math/lcp/`, PLAN-020 contract) into
  a contact constraint solve.
- Contact model parameters: restitution, friction (pyramid; cone as
  improvement), ERP/CFM or split-impulse position correction, slip compliance.
- Island/group detection for solver scaling.
- Impulse-based velocity update integrated into the substep schedule.
- Joint limit and motor (servo) constraints.

## Phase 4 — Joint features & actuators

- Actuator types (FORCE/PASSIVE/SERVO/VELOCITY/ACCELERATION/LOCKED) and the
  mixed forward/inverse per-DOF behavior.
- Position/velocity/accel/force limits enforced via constraints.
- Coulomb joint friction.
- Mimic and coupler relations (promote from LoopClosure metadata where it fits).
- Armature/rotor inertia (improvement over DART 6).

## Phase 5 — Loop closures & improvements

- Loop-closure kinematic projection and dynamic constraint solving (currently
  rejected at runtime).
- Pluggable integrator and explicit substepping policy.
- COM jacobians, body jacobians, full diagnostics.
- Model loading bridge: parsed `dynamics::Skeleton` import for the
  experimental tree-joint families (Weld/Revolute/Prismatic/Screw/Universal/
  Ball/Planar/Free), legacy `simulation::World` import, and default URI-string
  loading are in place in C++ and dartpy, including one centered collidable
  Box/Sphere/Capsule/Cylinder/Mesh shape per link. Explicit read options are
  available for format selection, SDF default root-joint selection, and URDF
  package directories; remaining legacy-only joints, source-offset/multiple/
  visual shape import, resource retriever bindings, diagnostics, and richer
  load-result ergonomics remain.

## Multi-Solver / Multi-Physics Readiness (carried through every phase)

Even while only the rigid-body solver exists, keep these seams honest so a
second solver or domain can be added without reshaping the World:

- Keep dynamics inside a solver-shaped unit, not inlined into `World`.
- Keep the substep schedule (prepare / pre-couple / couple / post-couple)
  expressible even when `couple` is empty.
- Keep Model/State/Control/Contacts conceptually separate in storage and views.
- Select the rigid-body path by method-family/default, not by a hard-wired
  branch that a future domain would have to edit.
- Do not expose solver/coupler/domain types or execution backends publicly.

## Verification & Parity Strategy

- Cross-check experimental dynamics against legacy DART 6 on identical scenes
  (energy drift bounds, trajectory tolerance) — this is the parity gate before
  any DART 8 promotion claim.
- Add focused unit tests per slice; add a benchmark when claiming a perf gain.
- Track DART 8 promotion against the contract in the public-facade design doc.
