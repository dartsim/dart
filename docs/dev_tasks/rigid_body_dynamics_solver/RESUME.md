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
- Phase 5 (partial) — screw-joint dynamics: a `Screw` joint couples rotation and
  translation by a pitch (`Joint::setPitch`/`getPitch`, dartpy `joint.pitch`),
  motion subspace `[axis; pitch*axis]`, added to `jointMotionTransform` and
  `jointSubspaceInJointFrame`. Verified by `M = I_axis + m pitch^2` and the
  gravity acceleration. C++ + dartpy tests.
- Phase 5 (partial) — universal-joint (2-DOF) dynamics: a `Universal` joint
  rotates about `axis` then `axis2` (`JointSpec`/`LinkOptions::axis2`, dartpy
  `JointSpec.axis2` + read-only `joint.axis2`). Added the Universal cases to
  `jointMotionTransform` (`R(theta1,axis) R(theta2,axis2)`) and
  `jointSubspaceInJointFrame` (col0 `R(theta2,axis2)^T axis`, col1 `axis2`), and
  the **velocity-product bias `cJ = Sdot qdot`** to `recursiveNewtonEuler`.
  - **Resolved finding (config-dependent subspace):** multi-DOF joints whose
    joint-frame subspace depends on configuration (universal col0
    `R(theta2,axis2)^T axis`) need the RNEA acceleration recursion to include
    `cJ = Sdot qdot` (Featherstone). Without it, M and gravity are right but
    Coriolis is wrong (the earlier symptom was half the correct theta1 accel).
    For the universal joint, `cJ_jointframe = (s1 x axis2) * theta1dot *
theta2dot` with `s1 = R(theta2,axis2)^T axis` (angular; linear zero), mapped
    to the child frame by the post-joint offset adjoint. This is now implemented
    generically: `LinkDynamics::biasTerms` holds quadratic terms
    `coeff * qdot[a] * qdot[b]` computed once per step in `buildDynamicsTree`
    (`jointBiasTerms`) and applied in `recursiveNewtonEuler`; constant-subspace
    joints contribute none. Verified directly by matching the engine's Coriolis
    force to the Christoffel symbols finite-differenced from the
    (cJ-independent) mass matrix, plus the closed-form M/gravity at q=0. C++ +
    dartpy tests.
- Phase 5 (partial) — planar-joint (3-DOF) dynamics: two in-plane translations
  plus a rotation about the plane normal (`axis` = normal, `axis2` = first
  in-plane direction), matching the kinematics convention. The translation
  columns are `R(theta_rot, normal)^T * inplane_axis` (configuration dependent),
  so they reuse the `biasTerms` `cJ` mechanism (rotation-translation coupling).
  Verified by the closed-form M/gravity at q=0 and the Christoffel-symbol
  Coriolis cross-check with a nonzero link offset. C++ + dartpy tests. The
  remaining unimplemented joints are Ball and Free (now done, below).
- Phase 5 (partial) — ball-joint (3-DOF) and free-joint (6-DOF) dynamics with
  manifold integration, which also enables a **floating base** (model it as a
  fixed base link + a `Free` joint to the moving link). Both use body-twist
  generalized velocities so their subspaces are constant (ball: body angular
  velocity, subspace `[I;0]`; free: `[linear; angular]` to match the
  `[translation; rotation vector]` position layout, subspace permutes to the
  `[angular; linear]` twist). The new piece is **manifold position integration**:
  ball orientation `R_new = R_old Exp(omega dt)`, free pose advances translation
  in the parent frame (`R*v dt`) and orientation by `Exp(omega dt)`, replacing
  `q += qdot*dt` for these joint types in `simulateMultiBody` (added `rotationExp`
  / `rotationLog` helpers). Position limits do not apply to rotation vectors and
  are skipped for ball/free. Verified by the closed-form M/gravity (ball
  pendulum), torque-free isotropic spin, free-fall, and combined translate-spin
  closed-form integration. C++ + dartpy tests.
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

**All joint types are now implemented and verified** (fixed, revolute,
prismatic, screw, universal, planar, ball, free) with a floating base via a
`Free` joint, including the config-dependent-subspace `cJ` term and SO(3)/SE(3)
manifold integration. The two remaining roadmap subsystems are each a dedicated
multi-session effort with a specific architectural prerequisite, detailed below.

### Subsystem A — full constraint solver / boxed-LCP (two-sided contacts)

What works today: rigid-body-vs-rigid-body and rigid-body-vs-static contacts
(`RigidBodyContactStage`, sequential impulse + friction + restitution +
positional correction); link-vs-static-rigid-body one-sided articulated contact
(inside `simulateMultiBody`). **Missing:** link-vs-dynamic-rigid-body and
link-vs-link (two-sided) contacts, and a coupled simultaneous boxed-LCP/PGS
solve over all contacts at once.

- **Architectural prerequisite (the real blocker):** the default `World::step`
  pipeline (see `world.cpp` ~line 835) runs the rigid-body stages _fully_ before
  the multibody stage: `RigidBodyVelocityStage` → `RigidBodyContactStage` →
  `RigidBodyPositionStage` → `MultiBodyForwardDynamicsStage` → `KinematicsStage`.
  So when the multibody contact solve runs, rigid bodies have already had their
  velocity _and position_ integrated this step. A correct coupled two-sided solve
  needs the pipeline reordered into phases: **(1) all velocity integration
  (rigid + multibody unconstrained `qddot`→`qdot`), (2) one unified contact /
  constraint solve over all bodies, (3) all position integration**. This touches
  the core step loop and all 274 passing tests, so do it as its own slice with
  the suite as the guardrail (revert if it destabilizes, as was done for the
  first universal-joint attempt).
- **After reordering:** assemble every contact as a constraint row with a
  Jacobian that maps the contact impulse to each involved body's velocity —
  rigid bodies via `invMass`/`invWorldInertia` + arm (already in
  `RigidBodyContactStage`), links via `J_point` + `M^-1` (already in
  `simulateMultiBody`). Stack them and run the **boxed-LCP/PGS** in
  `dart/math/lcp/` (normal `lambda_n >= 0`, friction cone `|lambda_t| <= mu
lambda_n`) instead of the two separate per-stage Gauss-Seidel loops. Add
  islands for performance later.
- **(DONE) Smaller correct increment — two-sided link-vs-dynamic-rigid-body.**
  `simulateMultiBody`'s contact solve now couples a dynamic rigid-body obstacle:
  `LinkContact::otherBody` carries it, and the contact rows include its inverse
  mass/inertia + arm so the normal and friction impulses are applied to both the
  link (`M^-1 J^T`) and the rigid body (equal-and-opposite, via its
  `comps::Velocity`). An immovable obstacle leaves `otherBody` null and reduces
  to the original one-sided solve. Accepts a one-step position lag for the rigid
  body (it is position-integrated earlier in the pipeline), corrected by
  Baumgarte. Verified by total X-momentum conservation when a prismatic striker
  link hits a free body (C++ + dartpy). **Still remaining:** link-vs-link
  contacts and the coupled simultaneous boxed-LCP (both need the pipeline
  reordering above).

### Subsystem B — model loading (URDF/SDF/MJCF) into the experimental World

`dart::io` (`dart/io/read.hpp`) parses to the **legacy** types only:
`readSkeleton` → `dynamics::SkeletonPtr`, `readWorld` → `simulation::WorldPtr`.
There is no experimental-World loader yet, so this needs a translation layer:

- Walk the legacy `Skeleton`'s `BodyNode` tree; for each `BodyNode` create an
  experimental `Link` (mass, inertia) and for each parent `Joint` map the legacy
  joint type → experimental `JointType` + axis/axis2/pitch and the
  parent-joint-to-child-link transform (`transformFromParent`).
- **Impedance mismatch to resolve:** the experimental dynamics assumes each
  link's COM is at the link-frame origin (`spatialInertia`), whereas
  URDF/Skeleton place the COM at an arbitrary offset from the body frame and
  express child-joint origins relative to the body frame. Either (a) extend the
  experimental spatial inertia to carry a COM offset
  (`I = [[I_C - m c× c×, m c×],[-m c×, m 1]]`), or (b) reframe each link so the
  link frame sits at the COM and shift child-joint `transformFromParent` and the
  parent joint transform accordingly. Option (a) is the cleaner standalone slice
  and is independently verifiable (offset-COM pendulum).
- Verify a known URDF loads with the right DOF count, tree structure, and a sane
  forward-dynamics step, in C++ and dartpy.

### Smaller deferred items

- **Phase 2:** capsule/cylinder/plane/mesh shapes, self-collision/filtering,
  broad-phase pruning, a persistent collision world instead of rebuilding per
  `collide()`.
- **Phase 4:** remaining actuator modes (SERVO/ACCELERATION/LOCKED) and
  mimic/coupler — reuse the existing `J M^-1 J^T` equality machinery.
- **Phase 5:** loop-closure dynamic solving, pluggable integrator/substepping,
  body/COM Jacobians.

## Contact solver — concrete implementation plan (derived from the code)

The architecture has been mapped; the next session can implement directly.

**Status: steps 1-4 below are DONE** (committed on the branch) — `CollisionBody`
handle, link collision shapes, links in `World::collide()`, and the rigid-body
stage skipping link pairs. The remaining work is **step 5 (articulated contact
response)** and beyond.

1. **(DONE) Generalize the contact body reference.** `Contact`
   (`body/contact.hpp`) now holds a `CollisionBody` handle
   (`body/collision_body.{hpp,cpp}`) with
   `getEntity`/`getWorld`/`getName`/`isRigidBody`/`isLink`/`asRigidBody`/
   `asLink`. dartpy `Contact.body_a`/`body_b` return `CollisionBody`.
2. **(DONE) Link collision shapes.** `Link::setCollisionShape`/
   `getCollisionShape`/`hasCollisionShape` store `comps::CollisionGeometry` on
   the link entity (dartpy `link.set_collision_shape`/`collision_shape`/
   `has_collision_shape`).
3. **(DONE) `collide()` includes links.** `World::collide()` now has a second
   pass over `CollisionGeometry + comps::Link + comps::FrameCache`, posing each
   link by `FrameCache.worldTransform`.
4. **(DONE) Rigid-body contact stage skips link pairs.** `RigidBodyContactStage`
   guards each contact with `registry.all_of<comps::RigidBodyTag>` for both
   bodies; link contacts are left for the articulated contact solve.
5. **(DONE for link-vs-static) Articulated contact response.** Implemented for
   link-vs-static-rigid-body contacts (one-sided) inside `simulateMultiBody`'s
   velocity phase (`LinkContact`, contact-point normal Jacobian, unilateral
   normal impulse + Baumgarte), routed by `MultiBodyForwardDynamicsStage::execute`
   via `world.collide()`. **Two-tangent Coulomb friction** (accumulated-impulse,
   friction-cone bounded) and **per-contact restitution** are also implemented.
   Verified by a prismatic-leg drop-and-rest, a sliding-link friction brake, and
   a dropped-link bounce (C++ + dartpy). **Still pending:** link-vs-dynamic-body
   and link-vs-link (two-sided) contacts, and a boxed-LCP for coupled
   simultaneous contacts. The original plan, for reference:
   - In `MultiBodyForwardDynamicsStage::execute`, call `world.collide()` once and
     route to each multibody the contacts whose link belongs to it and whose
     other body `isRigidBody()` and is static (`comps::StaticBodyTag`). Pass them
     into `simulateMultiBody`.
   - Map the contact's link entity to its index in the `DynamicsTree`; get the
     body Jacobian from `linkBodyJacobians(tree)` and the world Jacobian via the
     link world rotation; shift the linear block to the contact point `p`:
     `J_point_linear = R*J_body_linear - skew(p - o)*R*J_body_angular`, `o` =
     link world origin (`tree.links[i].worldTransform.translation()`).
   - Normal row `Jn = n^T J_point_linear` (1 x dof). **Sign care:** orient `n`
     to point from the obstacle into the link (the contact normal points
     bodyA->bodyB; flip if the link is bodyA). Normal velocity `vn = Jn*nextVel`.
   - If `vn < 0` (approaching) apply `lambda_n = max(0, -m_eff*vn)` with
     `m_eff = 1/(Jn M^-1 Jn^T)` (reuse `mb.massMatrix.inverse()` /
     `J M^-1 J^T`); `nextVelocity += M^-1 Jn^T lambda_n`. Add a small Baumgarte
     bias `+ beta*depth/dt` to stop sinking. Iterate a few times for multiple
     contacts.
   - Later: restitution/friction (tangent rows + cone), then the boxed-LCP for
     simultaneous coupled contacts (`dart/math/lcp`, PLAN-020) and islands, and
     link-vs-dynamic-body / link-vs-link (two-sided) contacts.
6. **Verify by emergent behavior** (the only non-closed-form tests in the task,
   so budget time to tune tolerances): a fixed-base 1-DOF prismatic "leg" with a
   sphere shape descends under gravity and rests on a static rigid ground
   (penetration stops, normal velocity -> 0).

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
