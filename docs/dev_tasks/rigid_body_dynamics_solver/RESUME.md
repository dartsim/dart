# Resume: Rigid-Body Dynamics Solver

## Last Session Summary

Implemented and verified Phases 0.1, 0.2, 0.3, and 1 of the rigid-body solver on
the experimental `World`:

- Phase 0.1 — World gravity (`setGravity`/`getGravity`, default `(0,0,-9.81)`,
  applied as acceleration; serialization; dartpy `world.gravity`).
- Phase 0.2 — per-step force/torque reset (consumed then cleared each step).
- Phase 0.3 — rigid-body quantities (linear/angular momentum, kinetic and
  potential energy) in C++ and dartpy.
- Phase 1 — articulated-body forward dynamics: RNEA-based forward dynamics for
  fixed-base trees with revolute/prismatic joints in
  `compute/multi_body_dynamics.{hpp,cpp}` (`MultiBodyForwardDynamicsStage`,
  added to the default `World::step` pipeline). Added `Joint` force/acceleration
  accessors, `Link` mass/inertia accessors, and a `JointSpec`/`LinkOptions`
  `transformFromParent` link offset, with dartpy bindings + stubs. Verified
  against analytical pendulum acceleration, prismatic free-fall, and energy
  conservation over a swing.
- Phase 4 (partial) — joint passive dynamics + limits: per-coordinate spring
  stiffness + rest position and damping coefficient applied as passive
  generalized forces, and per-coordinate position limits enforced as hard stops
  in the integrator (`Joint::setPositionLimits`/`getPositionLowerLimits`/
  `getPositionUpperLimits`, dartpy `set_position_limits`/`position_lower_limits`/
  `position_upper_limits`). C++ + dartpy + analytical tests.
- Phase 4 (partial) — generalized-coordinate dynamics accessors: public
  `MultiBody::getMassMatrix`/`getInverseMassMatrix`/`getCoriolisForces`/
  `getGravityForces`/`getCoriolisAndGravityForces` (dartpy `mass_matrix`,
  `inverse_mass_matrix`, `coriolis_forces`, `gravity_forces`,
  `coriolis_and_gravity_forces`), factored out of the forward-dynamics RNEA into
  `compute::computeMultiBodyDynamicsTerms`. Verified analytically (single
  pendulum `M = I + m L^2`, gravity force `-m g L`) and via the
  `M qddot + C + g = tau` equation-of-motion identity on a 2-DOF chain. C++ +
  dartpy tests.
- Phase 2 (partial) — collision query bridge: public `CollisionShape`
  (sphere/box) attachable to rigid bodies, public `Contact`, and
  `World::collide()` bridging to `dart/collision/native` via pairwise
  narrow-phase queries (returns contact point/normal/depth; the native normal is
  negated so `Contact.normal` points bodyA→bodyB). C++ + dartpy + tests. New
  files: `body/collision_shape.hpp`, `body/contact.hpp`,
  `comps/collision_geometry.{hpp,cpp}`. The experimental lib now PRIVATE-links
  `dart-collision-native`.
- Phase 3 (partial) — contact solver: `World::step()` resolves contacts between
  free rigid bodies with a sequential-impulse solver — accumulated (clamped)
  normal impulses, restitution, two-tangent Coulomb friction-pyramid impulses,
  and a projection-based positional correction. Added a public static-body
  convention (`RigidBodyOptions::isStatic`, `RigidBody::setStatic`/`isStatic`,
  `comps::StaticBodyTag`, dartpy `is_static`) and per-body contact material
  (`RigidBody::setRestitution`/`setFriction`, `comps::ContactMaterial`, dartpy
  `restitution`/`friction`). Rigid-body integration was split into
  `RigidBodyVelocityStage` / `RigidBodyContactStage` / `RigidBodyPositionStage`
  (the combined `RigidBodyIntegrationStage` remains for direct use) so the solve
  runs at the velocity level between velocity and position integration. Native
  contact normal is negated to point bodyA→bodyB. C++ + dartpy tests: head-on
  spheres stop (e=0) or swap (e=1), a sphere drops onto a static ground and
  rests, and a sliding box brakes via friction.

All gates passed at each slice: `pixi run lint`, focused C++ tests
(`test_world`, `test_serialization`, `test_compute_graph`), and
`test_experimental_world.py`. Changes are uncommitted on `main`.

Key implementation note: each link's center of mass is assumed at the link frame
origin; the joint motion subspace is transformed by the post-joint link offset
(`Ad(transformFromParent^{-1}) * S_jointframe`) — this was essential to get
correct gravity torques. See `compute/multi_body_dynamics.cpp`.

## Current Branch

Working in `/home/js/dev/dartsim/dart/task_2` on `main` (snapshot). Create a
feature branch before pushing (e.g. `feature/experimental-rigid-body-gravity`).
Check `git status` / `git log -3 --oneline` for the live state.

## Immediate Next Step

Working slices of Phases 0.1–4 are landed and verified; remaining work deepens
Phases 2–5. Suggested next slices, in rough dependency order:

- **Phase 3 next:** a public static-body convention (so a ground can be created
  — currently non-positive mass is treated as immovable internally but the
  public ctor/`setMass` reject it); a Baumgarte/penetration positional
  correction; restitution and a friction model; joint-limit constraints; then
  wiring the boxed-LCP library (`dart/math/lcp/`, PLAN-020) and islands.
- **Phase 2 next:** capsule/cylinder/plane/mesh shapes, collision shapes on
  multibody links, self-collision/filtering, broad-phase pruning, and a
  persistent collision world instead of rebuilding per `collide()`.
- **Phase 4 next:** actuator types (FORCE/PASSIVE/SERVO/VELOCITY/ACCELERATION/
  LOCKED), velocity/effort limits, Coulomb joint friction, mimic/coupler, and
  armature. (Position limits and the public generalized mass-matrix / Coriolis /
  gravity accessors are done — `MultiBody::getMassMatrix`/`getInverseMassMatrix`/
  `getCoriolisForces`/`getGravityForces`/`getCoriolisAndGravityForces`.)
- **Phase 5:** loop-closure kinematic projection + dynamic solving, pluggable
  integrator/substepping, body/COM Jacobians, and a model-format loading bridge.

Phase 3 (friction + LCP), the remainder of Phase 4, and Phase 5 are each a major
subsystem; budget them as multiple verified slices.

When committing this session's work: create a feature branch, run the pre-commit
checklist (`pixi run lint`, build, tests), and open a PR with milestone
`DART 7.0`.

## Context That Would Be Lost

- The architecture (solver owns dynamics, World owns composition; domain-driven
  assignment; pairwise swappable couplers; substep windowing prepare/pre-couple/
  couple/post-couple; Model/State/Control/Contacts separation) lives in
  `docs/design/simulation_solver_architecture.md`. Keep that the source of
  truth; do not duplicate it.
- User constraints: do NOT name Genesis (or other engines) in core code/docs;
  use method/approach names. Do not modify `/home/js/multiphysics-api-design.md`.
- The legacy DART 6 dynamics live in `dart/dynamics/` (Featherstone ABA in
  `detail/articulated_dynamics_algorithms.hpp`, `Skeleton::computeForwardDynamics`)
  and `dart/constraint/` (`BoxedLcpConstraintSolver`, `ContactConstraint`). The
  boxed-LCP library is `dart/math/lcp/` (Dantzig/PGS), already contracted under
  PLAN-020.
- The maintained native collision engine is `dart/collision/native/`
  (PLAN-035/036/037); Phase 2 bridges it into the experimental world.
- Existing experimental tests assume zero gravity — changing the default is a
  deliberate, documented decision (matches DART 6 and the user vision).
- Two related dev tasks: `simulation_experimental_api_design` (facade shape) and
  this one (dynamics). Keep concerns separate.

## How To Resume

```bash
cd /home/js/dev/dartsim/dart/task_2
git status && git log -3 --oneline
```

Then read, in order:

```bash
docs/dev_tasks/rigid_body_dynamics_solver/README.md
docs/dev_tasks/rigid_body_dynamics_solver/01-gap-analysis.md
docs/dev_tasks/rigid_body_dynamics_solver/02-roadmap.md
docs/design/simulation_solver_architecture.md
docs/design/simulation_experimental_cpp_api.md
```

Then continue the current phase from "Immediate Next Step".
