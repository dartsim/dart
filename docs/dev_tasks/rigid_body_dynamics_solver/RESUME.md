# Resume: Rigid-Body Dynamics Solver

## Last Session Summary

Implemented and verified, in fully-tested slices, the rigid-body solver's
joint-space dynamics layer on the experimental `World` (Phases 0-1 complete;
Phases 2-5 partial — the contact-LCP subsystem is the main remaining work, see
the implementation plan below):

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
- Phase 4 (partial) — inverse dynamics: public
  `MultiBody::computeInverseDynamics(qddot)` (dartpy `compute_inverse_dynamics`)
  returning `tau = M qddot + C qdot + g` (with armature) via RNEA
  (`compute::computeMultiBodyInverseDynamics`). Verified analytically and by a
  forward/inverse round-trip. C++ + dartpy tests.
- Phase 4 (partial) — generalized impulse response: public
  `MultiBody::computeImpulseResponse(f)` (dartpy `compute_impulse_response`)
  returning `dqdot = M^-1 f` (M includes armature) — the joint-space primitive
  for impulse-based constraint dynamics. Verified analytically and by the
  `M dqdot = f` identity. C++ + dartpy tests. (The full constrained impulse
  dynamics still needs the constraint solve.)
- Phase 5 (partial) — link Jacobians: public `MultiBody::getJacobian(link)`
  (body frame) and `getWorldJacobian(link)` (world axes, link-origin referenced)
  (dartpy `get_jacobian`/`get_world_jacobian`) return the 6 x DOF spatial
  Jacobian. The body Jacobian uses the recursion `J_i = X_i J_parent` with own
  columns = the joint subspace; the world Jacobian rotates both blocks by the
  link world rotation, which is accumulated within the dynamics pass (so robust
  to stale frame caches). Verified by screw-theory twist values, the 2-DOF
  column structure, and a finite-difference cross-check against FK. COM
  Jacobians are deferred. C++ + dartpy tests.
- Phase 4 (partial) — joint velocity and effort limits: per-coordinate
  velocity and effort (force/torque) lower/upper bounds
  (`Joint::setVelocityLimits`/`getVelocityLowerLimits`/`getVelocityUpperLimits`,
  `setEffortLimits`/`getEffortLowerLimits`/`getEffortUpperLimits`; dartpy
  `set_velocity_limits`/`set_effort_limits` + `*_lower_limits`/`*_upper_limits`).
  Effort is clamped before the solve; velocity is clamped each step. The
  `comps::JointLimits` struct now carries position/velocity/effort lower/upper
  pairs (serialized round-trip tested). C++ + dartpy tests.
- Phase 4 (partial) — joint armature (rotor/reflected inertia): per-coordinate
  `Joint::setArmature`/`getArmature` (dartpy `joint.armature`) added to the
  joint-space mass-matrix diagonal in both the forward dynamics and the public
  mass-matrix accessor. An improvement over legacy DART, which lacks armature.
  Verified analytically (M = I + m L^2 + armature and the reduced gravity
  acceleration). C++ + dartpy tests.
- Phase 4 (partial) — Coulomb (dry) joint friction:
  `Joint::setCoulombFriction`/`getCoulombFriction` (dartpy
  `joint.coulomb_friction`). The integrator applies a bounded velocity-level
  friction impulse per coordinate, using the mass-matrix diagonal as the
  effective inertia: stiction (holds at rest) when the required holding impulse
  is within the bound, kinetic friction otherwise. Verified analytically
  (stiction below the bound; net step `(F - mu)/m * dt` above it). C++ + dartpy
  tests.
- Phase 4 (partial) — joint actuator types + first constraint solve:
  `Joint::setActuatorType`/`getActuatorType` with public `ActuatorType` (dartpy
  `joint.actuator_type`, `ActuatorType`). `Force` (default) applies the clamped
  commanded effort; `Passive` ignores it (passive spring/damping/friction still
  apply); `Velocity` drives the joint to `Joint::setCommandVelocity` (dartpy
  `joint.command_velocity`) via a **coupled velocity-level equality constraint**
  `lambda = (J M^-1 J^T)^-1 (target - J qdot)`, `qdot += M^-1 J^T lambda`,
  reaching the target exactly even under inertial coupling (verified on a 2-link
  chain). This is the first real constraint solve and establishes the
  `J M^-1 J^T` machinery the contact LCP will reuse; the integration step was
  restructured into a global velocity phase (friction → velocity constraint →
  velocity limits) plus a per-joint position write-back. The remaining modes
  (`Servo`/`Acceleration`/`Locked`/`Mimic`) are defined but rejected by the
  forward dynamics. C++ + dartpy tests.
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
`test_experimental_world.py`. All work is committed on the
`feature/experimental-rigid-body-dynamics` branch.

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
- **Phase 4 next:** the remaining constraint-coupled actuator modes
  (SERVO/ACCELERATION/LOCKED) and mimic/coupler. `Velocity` is done (an equality
  constraint, see above); SERVO/ACCELERATION reuse the same `J M^-1 J^T`
  machinery with different targets/gains, LOCKED is a zero-velocity+frozen-
  position constraint, and mimic/coupler couple two joints' coordinates.
  (Done in Phase 4: position/velocity/effort limits, armature, Coulomb joint
  friction, Force/Passive/Velocity actuator types, inverse dynamics, the
  generalized mass-matrix / Coriolis / gravity accessors, and the impulse
  response.)
- **Shared foundation needed next:** the full (inequality) constraint solver
  wired into `World::step`. The joint-space primitives are now all in place —
  mass matrix, `M^-1`, the impulse response `M^-1 f`, inverse dynamics, the body-
  and world-frame link Jacobians — and the **equality-constraint solve**
  (`J M^-1 J^T lambda = c`) already exists and is exercised by the Velocity
  actuator. What remains: assembling contact/joint-limit constraints (contact
  Jacobians from the body/world Jacobians + collision normals) and the
  **boxed-LCP/PGS solve** (inequalities + friction cone) on top of the existing
  equality machinery, plus islands. It unblocks the remaining Phase 3 (contacts
  on multibody links, joint limit/servo-force constraints, boxed-LCP, islands),
  the remaining Phase 4 actuator modes and mimic/coupler, and Phase 5 loop-
  closure dynamics. Budget it as a dedicated multi-session subsystem.
- **Phase 5:** loop-closure kinematic projection + dynamic solving, pluggable
  integrator/substepping, body/COM Jacobians, and a model-format loading bridge.

Phase 3 (friction + LCP), the remainder of Phase 4, and Phase 5 are each a major
subsystem; budget them as multiple verified slices.

## Contact solver — concrete implementation plan (derived from the code)

The architecture has been mapped; the next session can implement directly:

1. **Generalize the contact body reference.** Today `Contact`
   (`body/contact.hpp`) holds `RigidBody bodyA/bodyB`. Introduce a lightweight
   `CollisionBody` handle (entity + `World*`) with
   `getEntity`/`getWorld`/`getName`/`isRigidBody`/`isLink` (+ `asRigidBody`/
   `asLink`), and change `Contact` to hold `CollisionBody`. The existing contact
   stage already extracts `contact.bodyA.getEntity()`
   (`compute/world_step_stage.cpp`), so it keeps compiling; dartpy
   `Contact.body_a` returns `CollisionBody` — keep a `name` property so the
   existing python test `{contact.body_a.name, contact.body_b.name}` passes.
2. **Link collision shapes.** Add `Link::setCollisionShape`/`getCollisionShape`
   storing `comps::CollisionGeometry` on the link entity (mirror `RigidBody`;
   the component doc already says "a body or link").
3. **`collide()` includes links.** `World::collide()` (`world.cpp`) iterates
   `CollisionGeometry + Transform + RigidBodyTag` and uses `comps::Transform`
   for the pose. Add a second pass over `CollisionGeometry + comps::Link` using
   `FrameCache.worldTransform` for the pose, pushing `ObjectEntry`s; build
   `CollisionBody` contacts.
4. **Rigid-body contact stage skips link pairs.** In `RigidBodyContactStage`
   guard each contact with `registry.all_of<comps::RigidBodyTag>(entity)` for
   both bodies; link contacts are handled by the new articulated contact solve.
5. **Articulated contact response.** For a contact on link `i` with world normal
   `n` at point `p`: the contact-point world Jacobian is the link world Jacobian
   with the linear block shifted to `p` (linear rows += `skew(o - p)` _ angular
   rows, where `o` is the link origin). The normal row is
   `Jn = n^T _ J_point_linear`(1 x dof); effective mass`m_eff = 1 / (Jn M^-1 Jn^T)`; normal impulse
`lambda_n = max(0, -m_eff \* (Jn qdot))`(remove approaching normal velocity,
clamp >= 0); apply`qdot += M^-1 Jn^T lambda_n`. This reuses the `J M^-1 J^T`
machinery already built for the Velocity actuator. Then add
restitution/friction and a positional (Baumgarte/split-impulse) correction,
then a boxed-LCP for simultaneous contacts (`dart/math/lcp`, PLAN-020) and
   islands.
6. **Verify by emergent behavior** (the only non-closed-form tests in the task,
   so budget time to tune tolerances): a fixed-base link with a sphere shape
   drops under gravity and rests on a static rigid ground (penetration stops,
   normal velocity -> 0); likewise a 1-DOF prismatic "leg".

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
