# Resume: Rigid-Body Dynamics Solver

## B2 Gate — Rigid Open-Chain Dynamics Parity Harness (2026-06-01)

Status of the DART 7 "Rigid dynamics parity" checkable gate (PLAN-080 B2):
"Shared open-chain scenes match the classic DART 6 path within documented
tolerances for gravity, integration, drift, and controls."

The dedicated world-parity suite lives in
`tests/unit/simulation/experimental/world/test_world_parity.cpp`. It builds the
same scene in both `dart::simulation::World` (classic DART 6 path) and
`dart::simulation::experimental::World`, steps both with matched dt, and asserts
state agreement within named tolerance constants. It runs under the
`simulation-experimental` ctest label, i.e. `pixi run
test-simulation-experimental` (or focused `ctest -R test_world_parity`).

This is the **start of the B2 evidence**, scoped to OPEN-CHAIN (no-contact)
rigid dynamics, which does not depend on the unified contact/constraint solver
(PR #2838). Per-scenario results on current `main`:

| Gate facet  | Scenario (test case)                                      | Result | Tolerance |
| ----------- | --------------------------------------------------------- | ------ | --------- |
| Gravity     | Free rigid body free-fall (single + repeated step)        | PASS   | 1e-12     |
| Gravity     | Two-link revolute chain under gravity (single + repeated) | PASS   | 1e-10     |
| Integration | Revolute pendulum (single + repeated short horizon)       | PASS   | 1e-10     |
| Integration | Prismatic free-fall joint (single + repeated)             | PASS   | 1e-10     |
| Drift       | Revolute pendulum, 1e4 steps                              | PASS   | 1e-9      |
| Drift       | Double pendulum (chaotic), 1e4 steps                      | PASS   | 1e-9      |
| Controls    | Single revolute, held torque (single + 200 steps)         | PASS   | 1e-9      |

All open-chain scenarios reach parity on current `main` — the two code paths
agree to **machine epsilon** (measured ~1e-14..1e-17 deltas; a development probe
printed the per-run deltas and the tolerance comments record them). The
tolerances are deliberately set a few orders of magnitude above the observed
agreement so they survive floating-point reassociation across compilers without
encoding a fake-loose pass; a real integrator/dynamics divergence (especially in
the chaotic double pendulum) would blow far past them.

**Not yet covered by this harness (deferred, not faked):**

- **Contact/constraint parity (B3)** — the existing
  `DynamicSphereRestsOnStaticGroundLikeClassicWorld` case exercises the current
  per-stage contact solver, but the full contact/friction/joint-limit parity
  suite belongs to the B3 gate and lands with the unified solver in PR #2838.
- **Floating-base / multi-link free-fall as free bodies** — multi-body
  free-fall parity here is covered via the articulated two-link chain under
  gravity; a free-joint floating-base parity row is a natural follow-up.
- **PD feedback control** — the controls facet uses a constant held torque
  (deterministic, re-applied each step on both paths). A closed-loop PD
  controller computing torque from state would exercise the same `setForce`
  path and is a straightforward extension.

The harness is designed to EXTEND, not replace: new scenarios add a
classic/experimental runner pair plus a `TEST` using the existing comparison
helpers and named tolerance constants.

## Last Session Summary

Implemented and verified, in fully-tested slices, the rigid-body solver's
joint-space dynamics layer on the experimental `World` (Phases 0-1 complete;
Phases 2-5 partial — the contact-LCP subsystem is the main remaining work, see
the implementation plan below):

- Phase 0.1 — World gravity (`setGravity`/`getGravity`, default `(0,0,-9.81)`,
  applied as acceleration; serialization; dartpy `world.gravity`).
- Phase 0.2 — force/torque components are persistent applied loads after the
  #2698 convention re-alignment; `World::step()` assembles a transient SoA force
  buffer with `mass * gravity` and leaves applied force/torque for callers to
  clear or update explicitly.
- Phase 0.3 — rigid-body quantities (linear/angular momentum, kinetic and
  potential energy) in C++ and dartpy.
- Phase 1 — articulated-body forward dynamics: RNEA-based forward dynamics for
  fixed-base trees with revolute/prismatic joints in
  `compute/multibody_dynamics.{hpp,cpp}` (`MultibodyForwardDynamicsStage`,
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
  `q += qdot*dt` for these joint types in `simulateMultibody` (added `rotationExp`
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
  `Multibody::getMassMatrix`/`getInverseMassMatrix`/`getCoriolisForces`/
  `getGravityForces`/`getCoriolisAndGravityForces` (dartpy `mass_matrix`,
  `inverse_mass_matrix`, `coriolis_forces`, `gravity_forces`,
  `coriolis_and_gravity_forces`), factored out of the forward-dynamics RNEA into
  `compute::computeMultibodyDynamicsTerms`. Verified analytically (single
  pendulum `M = I + m L^2`, gravity force `-m g L`) and via the
  `M qddot + C + g = tau` equation-of-motion identity on a 2-DOF chain. C++ +
  dartpy tests.
- Phase 4 (partial) — inverse dynamics: public
  `Multibody::computeInverseDynamics(qddot)` (dartpy `compute_inverse_dynamics`)
  returning `tau = M qddot + C qdot + g` (with armature) via RNEA
  (`compute::computeMultibodyInverseDynamics`). Verified analytically and by a
  forward/inverse round-trip. C++ + dartpy tests.
- Phase 4 (partial) — generalized impulse response: public
  `Multibody::computeImpulseResponse(f)` (dartpy `compute_impulse_response`)
  returning `dqdot = M^-1 f` (M includes armature) — the joint-space primitive
  for impulse-based constraint dynamics. Verified analytically and by the
  `M dqdot = f` identity. C++ + dartpy tests. (The full constrained impulse
  dynamics still needs the constraint solve.)
- Phase 5 (partial) — link Jacobians: public `Multibody::getJacobian(link)`
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

Historical gates passed at each slice: `pixi run lint`, focused C++ tests
(`test_world`, `test_serialization`, `test_compute_graph`), and
`test_experimental_world.py`. The current re-alignment merge is in progress on
the `feature/experimental-rigid-body-dynamics` branch and still needs local
verification before pushing.

Key implementation note: each link's center of mass is assumed at the link frame
origin; the joint motion subspace is transformed by the post-joint link offset
(`Ad(transformFromParent^{-1}) * S_jointframe`) — this was essential to get
correct gravity torques. See `compute/multibody_dynamics.cpp`.

## Current Branch

Working in `/home/js/dev/dartsim/dart/task_2` on
`feature/experimental-rigid-body-dynamics` (PR #2705). `origin/main`
`b7f5380679c` has been merged locally; conflict resolution and verification are
still in progress.

## Immediate Next Step

**Blocker — land the MVP PR #2705.** Finish the local main merge and verification:
the integration convention is now #2698-style pure/persistent force inputs plus
a transient gravity force-assembly buffer for default `World::step()`. Next,
build and run the focused C++/Python gates, fix fallout, add the required
dart-gui example (Subsystem C), then push only after maintainer approval.

**All joint types are now implemented and verified** (fixed, revolute,
prismatic, screw, universal, planar, ball, free) with a floating base via a
`Free` joint, including the config-dependent-subspace `cJ` term and SO(3)/SE(3)
manifold integration. The two remaining roadmap subsystems are each a dedicated
multi-session effort with a specific architectural prerequisite, detailed below.

### Subsystem A — full constraint solver / boxed-LCP (two-sided contacts)

What works today: rigid-body-vs-rigid-body and rigid-body-vs-static contacts
(`RigidBodyContactStage`, sequential impulse + friction + restitution +
positional correction); link-vs-static-rigid-body one-sided articulated contact
(inside `simulateMultibody`). **Missing:** link-vs-dynamic-rigid-body and
link-vs-link (two-sided) contacts, and a coupled simultaneous boxed-LCP/PGS
solve over all contacts at once.

- **Pipeline ordering (partly addressed):** the default `World::step` pipeline is
  now `RigidBodyVelocityStage` → `RigidBodyContactStage` →
  `MultibodyForwardDynamicsStage` → `RigidBodyPositionStage` → `KinematicsStage`
  (rigid-body position integration moved after the multibody stage so two-sided
  link-vs-rigid impulses land the same step). A fully coupled solve still wants
  the stronger split: **(1) all velocity integration (rigid + multibody
  unconstrained `qddot`→`qdot`), (2) one unified contact/constraint solve over
  all bodies, (3) all position integration** — so rigid-rigid and link contacts
  are resolved jointly rather than in separate Gauss-Seidel passes. This touches
  the core step loop and all passing tests, so do it as its own slice with the
  suite as the guardrail (revert if it destabilizes, as was done for the first
  universal-joint attempt).
- **After reordering:** assemble every contact as a constraint row with a
  Jacobian that maps the contact impulse to each involved body's velocity —
  rigid bodies via `invMass`/`invWorldInertia` + arm (already in
  `RigidBodyContactStage`), links via `J_point` + `M^-1` (already in
  `simulateMultibody`). Stack them and run the **boxed-LCP/PGS** in
  `dart/math/lcp/` (normal `lambda_n >= 0`, friction cone `|lambda_t| <= mu
lambda_n`) instead of the two separate per-stage Gauss-Seidel loops. Add
  islands for performance later.
- **(DONE) Smaller correct increment — two-sided link-vs-dynamic-rigid-body.**
  `simulateMultibody`'s contact solve now couples a dynamic rigid-body obstacle:
  `LinkContact::otherBody` carries it, and the contact rows include its inverse
  mass/inertia + arm so the normal and friction impulses are applied to both the
  link (`M^-1 J^T`) and the rigid body (equal-and-opposite, via its
  `comps::Velocity`). An immovable obstacle leaves `otherBody` null and reduces
  to the original one-sided solve. `RigidBodyPositionStage` now runs _after_
  `MultibodyForwardDynamicsStage` in `World::step`, so these impulses reach the
  rigid body's pose in the same step (no lag; addressed a P1 review note).
  Verified by total X-momentum conservation when a prismatic striker link hits a
  free body (C++ + dartpy). **Still remaining:** link-vs-link
  contacts and the coupled simultaneous boxed-LCP (both need the pipeline
  reordering above).

### Subsystem B — model loading (URDF/SDF/MJCF) into the experimental World

`dart::io` (`dart/io/read.hpp`) parses to the **legacy** types only:
`readSkeleton` → `dynamics::SkeletonPtr`, `readWorld` → `simulation::WorldPtr`.
The first experimental-World bridge composes the legacy Skeleton parser with
the experimental importer; richer diagnostics and unsupported-feature reporting
still need the remaining translation layer:

- Walk the legacy `Skeleton`'s `BodyNode` tree; for each `BodyNode` create an
  experimental `Link` (mass, inertia) and for each parent `Joint` map the legacy
  joint type → experimental `JointType` + axis/axis2/pitch and the
  parent-joint-to-child-link transform (`transformFromParent`).
- **(DONE) Impedance mismatch — COM offset.** The experimental dynamics no
  longer requires a link's COM at the link-frame origin: `comps::MassProperties`
  carries `localCenterOfMass`, `spatialInertia` uses the full COM-coupled form
  `[[I_C - m c× c×, m c×],[-m c×, m 1]]`, and `Link::setCenterOfMass`/
  `getCenterOfMass` (dartpy `link.center_of_mass`) expose it. So a loader can map
  URDF/Skeleton COM offsets directly (no reframing). Verified by an offset-COM
  pendulum (C++ + dartpy). Child-joint origins relative to the body frame still
  map onto `transformFromParent` as today.
- **DONE first C++/dartpy bridge:** `simulation::experimental::io::addSkeleton`
  /`addWorld` and dartpy `simulation_experimental.add_skeleton()`/`add_world()`
  now translate already-parsed legacy `dynamics::Skeleton` and
  `simulation::World` objects into experimental `World`/`Multibody` handles for
  Weld/Revolute/Prismatic/Screw/Universal/Ball/Planar/Free tree joints. It maps
  names, synthetic root anchors, parent-joint transforms, axes, screw pitch,
  joint state/limits, actuator family, passive spring/damping/friction, mass,
  inertia, local COM offsets, and one centered collidable Box/Sphere/Capsule/
  Cylinder/Mesh shape per link when the legacy shape maps exactly to the
  experimental `CollisionShape` facade. URI-loading overloads accept explicit
  `dart::io::ReadOptions`, including dartpy bindings for format selection, SDF
  default root-joint selection, and URDF package directories.
  `test_skeleton_loader` verifies structure/property transfer,
  classic-vs-experimental pendulum stepping from a loaded Skeleton, higher-DOF
  joint-family coverage, basic collision-shape transfer, default Skeleton/World
  URI loading, multi-skeleton legacy-World import, and honest rejection of
  unsupported legacy-only joints or unrepresentable collision shapes; the dartpy
  regression covers the already-parsed handoff, URI-string loading, shape
  transfer, world import, and `SkeletonLoadOptions`.
- **Remaining:** remaining legacy-only joint families, collision shape source
  offsets, multiple collision shapes, visual geometry, materials, resource
  retriever bindings, diagnostics, and richer load-result ergonomics.

### Subsystem C — MVP GUI example (required for the MVP)

Today the only experimental example is **headless**
(`python/examples/experimental_rigid_body/main.py`). The MVP needs at least one
**GUI** example that steps the experimental rigid-body `World` and renders it
live.

Grounding (verified in-tree):

- DART's maintained native renderer is **`dart-gui`** (Filament + GLFW3 + Dear
  ImGui, built with `DART_BUILD_GUI=ON`). Public, renderer-neutral headers:
  `dart/gui/{scene,viewer,renderable,geometry,panel,gizmo}.hpp`
  (see `docs/onboarding/gui-rendering.md`). dartpy already exposes gui bindings
  (`python/dartpy/gui`, tests in `python/tests/unit/gui/`), so the example may be
  C++ or Python.
- **Template:** `examples/collision_sandbox/main.cpp` shows the viewer/scene/
  panel/gizmo wiring and `gui::RunOptions` lifecycle — but it drives the _legacy_
  `simulation::World`. Follow its GUI wiring, not its physics.
- **Net-new (small): an experimental→renderable bridge.** There is no
  experimental-`World`→`gui` adapter yet. Each frame, iterate the rigid bodies
  (and multibody links), read `comps::Transform` (+ the body's shape) and emit /
  update `gui::Renderable`s in a `gui::Scene`; advance with `world.step()` in the
  viewer loop. Keep it a thin read-only extraction (aligns with the SoA / batch
  intent — a future batched pose→renderable path can replace it).
- **Scope:** a small, visually obvious scene — e.g. bodies falling under gravity
  onto a static floor (exercises gravity + the contact stage), and/or a short
  articulated chain swinging. Gate the build on `DART_BUILD_GUI=ON`. Put a C++
  example under `examples/experimental_rigid_body_gui/` (mirroring
  `collision_sandbox`) or a Python one under `python/examples/`.
- Depends on the gravity decision in `convention_realignment.md` (the falling-
  body scene should actually fall).

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
   link-vs-static-rigid-body contacts (one-sided) inside `simulateMultibody`'s
   velocity phase (`LinkContact`, contact-point normal Jacobian, unilateral
   normal impulse + Baumgarte), routed by `MultibodyForwardDynamicsStage::execute`
   via `world.collide()`. **Two-tangent Coulomb friction** (accumulated-impulse,
   friction-cone bounded) and **per-contact restitution** are also implemented.
   Verified by a prismatic-leg drop-and-rest, a sliding-link friction brake, and
   a dropped-link bounce (C++ + dartpy). **Still pending:** link-vs-dynamic-body
   and link-vs-link (two-sided) contacts, and a boxed-LCP for coupled
   simultaneous contacts. The original plan, for reference:
   - In `MultibodyForwardDynamicsStage::execute`, call `world.collide()` once and
     route to each multibody the contacts whose link belongs to it and whose
     other body `isRigidBody()` and is static (`comps::StaticBodyTag`). Pass them
     into `simulateMultibody`.
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
