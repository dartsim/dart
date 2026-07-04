# Resume: Rigid-Body Dynamics Solver

## Current Reality (2026-07-04)

Landed locally on `main` (verified, not yet pushed as a PR): the **`Locked`
joint actuator mode** in the default semi-implicit articulated dynamics
(`compute/multibody_dynamics.cpp`). A locked joint is held rigidly at its
current position by the same velocity-level equality constraint the `Velocity`
actuator uses, with target velocity zero; the locked coordinate's commanded
effort and passive spring/damping are ignored (the holding-constraint reaction
absorbs them — proven independent, and covered by a regression). No new
serialized state (target velocity is a constant zero), so no binary-format
change. `ActuatorType::Locked` was already plumbed through the public C++/dartpy
API and only rejected at step time; the `default:` throw now lists Force,
Passive, Velocity, and Locked as supported.

Evidence: `test_world_contact_parity` +3 (`LockedActuatorHoldsRevoluteJoint`,
`LockedActuatorIgnoresItsOwnPassiveAndCommandedForces`,
`LockedActuatorMatchesZeroVelocityServo`) 8/8; `test_multibody_constraint` 3/3,
`test_unified_constraint` 26/26, `test_unified_constraint_stage` 2/2,
`test_world` 424/424; a dartpy test
(`test_simulation_locked_actuator_holds_joint`). Changelog entry drafted under
DART 7 → Simulation and Solvers.

Boundary: implemented in the **default semi-implicit** path only. The opt-in
variational integrator and the AVBD path special-case only `Velocity`; there a
`Locked` joint currently falls back to passive behavior (pre-existing pattern
for every non-`Force`/`Velocity` mode, documented in the `setActuatorType`
API comment). Extending `Locked` (and the other reserved modes) to those paths
is a separate follow-up.

Also landed locally (stacked on the `Locked` branch, separate PR): the
**`Servo`** joint actuator mode. A servo drives the joint to its commanded
velocity like `Velocity` but bounds the motor impulse by the effort limits
(`[effortLower·dt, effortUpper·dt]`) so it saturates like a real motor. It
reuses the velocity-actuator `J M⁻¹ Jᵀ` machinery: the pure Velocity/Locked
path keeps the unbounded SPD solve (bit-stable), and only when a servo
coordinate has a finite effort bound does the coupled solve route through the
`dart/math/lcp` Dantzig **boxed LCP** (velocity/locked rows free, servo rows
bounded). No new serialized state (reuses `commandVelocity` + effort limits).
Evidence: `test_world_contact_parity` +3 (Servo saturates at effort\*dt/massDof;
reaches target with ample effort; boxed solve ≡ unbounded solve when no bound
binds); a dartpy servo test; `test_world` regression green; variational 179/179,
serialization 55/55 unaffected. Same integrator boundary as `Locked` (default
semi-implicit only; variational/AVBD fall back to passive).

Also landed locally (stacked on the `Servo` branch, separate PR): the
**`Acceleration`** joint actuator mode. The joint realizes a commanded
acceleration exactly through the same velocity-level equality machinery with
target `q̇ + q̈·dt`, overriding gravity and applied effort. Adds a
`commandAcceleration` field to `comps::JointActuation` (initialized to
DOF-sized zeros at joint construction — a missed init site was the cause of an
early test crash) and bumps the simulation binary format to **version 28**
(clean break: v27 packets no longer round-trip the actuation record). C++
(`Joint::setCommandAcceleration`) + dartpy (`command_acceleration`) +
closed-form single/coupled tests + a serialization round-trip.

Remaining reserved actuator mode: **`Mimic`**/coupler (the last unimplemented
actuator type). The Subsystem A friction-cone / warm-start polish remains the
maintainer-stated headline deferred item (see below); it changes default contact
behavior and should land as a reviewed, release-6.\* parity-evidenced slice, not
an unreviewed autonomous one.

## Current Reality (2026-06-06)

Use `README.md`, `docs/plans/dashboard.md`, and the current code as the live
status. The older branch-local sections below record useful implementation
history, but references to `feature/experimental-model-loader`, unpublished
local commits, "DONE locally" slices, and publish/push instructions are
historical handoff notes, not current branch state. The default semi-implicit
path now routes rigid-rigid and articulated link contacts through the unified
boxed-LCP stage when multibody structures are present; remaining PLAN-080 work is
the Subsystem A polish called out in `README.md` and the dashboard: warm
starting, friction-cone iteration, scaling around the unified solve, and
separate deferred model-loading/actuator/mimic/loop-closure slices.

## B2 Gate — Rigid Open-Chain Dynamics Parity Harness (2026-06-01)

Status of the DART 7 "Rigid dynamics parity" checkable gate (PLAN-080 B2):
"Shared open-chain scenes match the classic DART 6 path within documented
tolerances for gravity, integration, drift, and controls."

The dedicated world-parity suite lives in
`tests/unit/simulation/world/test_world_parity.cpp`. It builds the
same scene in both `dart::simulation::World` (classic DART 6 path) and
`dart::simulation::World`, steps both with matched dt, and asserts
state agreement within named tolerance constants. It runs under the
`simulation-experimental` ctest label, i.e. `pixi run
test-simulation` (or focused `ctest -R test_world_parity`).

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
joint-space dynamics layer on the DART 7 `World` (Phases 0-1 complete;
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
  (sphere/box/capsule/cylinder/plane/triangular mesh) attachable to rigid bodies, public
  `Contact`, and
  `World::collide()` bridging to `dart/collision/native` via pairwise
  narrow-phase queries (returns contact point/normal/depth; the native normal is
  negated so `Contact.normal` points bodyA→bodyB). Shape-node local transforms
  are preserved by `CollisionShape::localTransform`, capsules/cylinders carry
  height, and planes carry normal/offset. C++ + dartpy + tests. New files:
  `body/collision_shape.hpp`, `body/contact.hpp`,
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
`test_experimental_world.py`. The model-loading and unified contact/constraint
follow-up line later landed on `main` as PR #2838, with follow-up allocation
polish in PR #2899 and PR #2910.

Key implementation note: each link's center of mass is assumed at the link frame
origin; the joint motion subspace is transformed by the post-joint link offset
(`Ad(transformFromParent^{-1}) * S_jointframe`) — this was essential to get
correct gravity torques. See `compute/multibody_dynamics.cpp`.

## Current Main Status

PR #2705 (the rigid-body MVP) **merged to `main` on 2026-05-25** (merge commit
`003c2d0e39`). The MVP convention re-alignment and the GUI example (Subsystem C)
are done and on `main`.

PR #2838 **merged to `main` on 2026-06-02** (merge commit `c325539ec29`) and
published the former `feature/experimental-model-loader` line: the
`dynamics::Skeleton` / `simulation::World` to experimental `Multibody` bridge,
preserved collision shape offsets, capsule/cylinder/plane/mesh-like collision
shape support, multiple collision shapes per body/link, same- and
cross-multibody link contacts, and row-islanded unified constraints. PR #2899
and PR #2910 then broadened the baked-step allocation guards and reused unified
constraint stage scratch. Future work should branch from current `main` rather
than resurrecting that retired local branch.

### Committed: pre-joint offset on `JointSpec`

`JointSpec`/`LinkOptions::transformToParent` (dartpy `transform_to_parent`) adds
a parent-side offset so the experimental joint matches legacy exactly:
`child_in_parent = transformToParent · jointMotion(q) · transformFromParent`
(legacy `A · Q(q) · C^-1`). Threaded through the forward dynamics, FK, and the
variational integrator; the motion subspace is unchanged (`S_child =
Ad(G_post^-1) · S_jointframe` is independent of the pre-offset). Default identity
is a verified no-op (full experimental suite 38/38); a new
`test_pre_joint_offset.cpp` covers FK and the fixed-base translation-invariance
of the dynamics. This unlocks branching at an offset, offset roots, and
arbitrary trees.

### Committed: loader refactored onto the pre-joint offset + branching

`buildMultibodyFromSkeleton` now maps each joint with
`transformToParent = A` (legacy `getTransformFromParentBodyNode`) and
`transformFromParent = C^-1` (inverse `getTransformFromChildBodyNode`), so each
experimental link frame coincides with its legacy body frame — axis, mass,
center of mass, and inertia map across directly with no reframing. The
synthetic-base `O_b` reframing, `outgoingJointOffset`, and the
`M`/`kAnchorTolerance` anchor check are gone. This adds **branching parents**
(sibling joints at different offsets, each carrying its own `transformToParent`)
and **offset/rotated roots**, verified by `BranchingTreeMatchesLegacyDynamics`
and `OffsetRootMatchesLegacyDynamics` (C++) and a dartpy branching parity test.
Ball/free/planar still require identity-rotation parent/child offsets
(translation is fine). Full experimental suite 39/39; Python loader suite 9/9.

**(DONE) `readWorld` / multi-skeleton.** `io::buildMultibodiesFromWorld(world,
legacyWorld, options)` (dartpy `build_multibodies_from_world`) converts every
skeleton of a legacy `simulation::World` into its own multibody, named after the
skeleton. C++ + dartpy tests. (A file-based one-call loader can wrap
`dart::io::readWorld` + this, but would pull `dart-io` into the experimental
lib's link, so it is left to the caller for now.)

**(DONE) Collision shape offsets plus capsule/cylinder/plane support.** The
loader translates the first sphere, box, capsule, cylinder, or plane collision
shape of each body (a shape node with a collision aspect) onto the link, gated by
`loadCollisionShapes`.
`CollisionShape::localTransform` preserves shape-node offsets, the binary format
serializes that transform, and `World::collide()` poses native shapes as
`worldTransform * shape.localTransform`. Capsules and cylinders carry radius
plus height, and planes carry normal plus offset, through C++, dartpy,
serialization, and the native collision query. Triangular `MeshShape` and
`ConvexMeshShape` instances carry their vertex/index buffers through the
model-loading bridge as DART 7 mesh collision shapes; `HeightmapShape`
instances are triangulated into the same carrier, and `SoftMeshShape` instances
snapshot their point-mass mesh into that carrier. Triangular meshes also
round-trip through C++, dartpy, serialization, and native collision queries.
Multiple shapes per body/link are stored as compound collision geometry,
serialized in binary format v11, exposed in C++/dartpy, imported from all
representable collision shape nodes in the model-loading bridge, and skipped for
same-entity self collision in `World::collide()`. Deformable obstacle barriers
still consume the first sphere/box shape only.

**(DONE) First collision-query filtering option.** `CollisionQueryOptions`
adds an explicit `includeSameMultibodyLinkPairs` switch (dartpy:
`include_same_multibody_link_pairs`) for `World::collide()`. The default remains
`true` so existing same-multibody self-contact queries and articulated solver
paths are unchanged; setting it to `false` filters link-vs-link pairs within the
same multibody while preserving rigid-body, link-vs-rigid-body, and
cross-multibody link contacts. Covered by C++ and dartpy tests.

**(DONE) Broad-phase pruning for `World::collide()`.** Experimental collision
queries now build the native collision world's AABB broad-phase snapshot and
only run narrow-phase contact generation for candidate pairs. Contact reporting,
same-entity compound-shape skipping, and same-multibody link filtering semantics
are preserved. Covered by the existing collision-query suite plus a sparse
far-body regression.

**(DONE) Body-type filtering for `World::collide()`.** `CollisionQueryOptions`
now includes explicit rigid-body, rigid-body/link, and link/link switches
(dartpy: `include_rigid_body_pairs`, `include_rigid_body_link_pairs`,
`include_link_pairs`) while preserving the existing same-multibody link subset
filter and the default all-pairs behavior. Covered by C++ and dartpy tests.

**(DONE) Persistent native collision world for `World::collide()`.** Collision
queries now keep a native collision world cache across calls, update cached
object transforms/AABBs for repeated queries, and rebuild only when the
experimental collision geometry revision or topology changes. Covered by the
collision-query suite plus a transform-update and shape-invalidation regression.

**Resume here — Subsystem A (the remaining headline gap):**

**Subsystem A — coupled boxed-LCP.** Reorder the `World::step` pipeline to
(1) all velocity integration, (2) one unified contact/constraint solve over all
bodies via `dart/math/lcp`, (3) all position integration. Multi-session; touches
the core step loop and every passing test — do it as its own slice with the
suite as the guardrail, revert if it destabilizes (as the first universal-joint
attempt was).

**(DONE) Safe first increment — coupled normal boxed-LCP in
`RigidBodyContactStage`** (commit `1d2340f1006`). The stage's per-contact
sequential normal impulses are replaced by a coupled boxed-LCP solve over all
rigid-rigid contacts: it assembles the contact-space inverse-mass (Delassus)
operator `A` (diagonal = the old `effectiveMass`; off-diagonal = coupling
through a body shared by two contacts, summed over each contact's two signed
ends) and solves `A lambda = b` with `b[i] = restitutionTarget_i - approach_i`,
`0 <= lambda`, under the solvers' `w = A lambda - b` convention, via the
`dart/math/lcp` Dantzig solver. A single isolated contact is a 1x1 system equal
to the previous closed form, so the 1e-9 elastic-swap guardrail is intact; a
rank-deficient set (the near-coplanar contacts of a box resting flat on a
plane) drives a non-positive pivot, so `earlyTermination` turns that into a
clean failure (no console warning) and the solve falls back to an uncoupled
diagonal projection. The friction pass (now a separate Gauss-Seidel sweep
bounded by the solved normal impulse) and the positional correction are
unchanged. Code: `world_step_stage.cpp` `RigidBodyContactStage::execute`.
Verified by the drop/rest/bounce/friction guardrails plus a new coupled
two-sphere-stack rest test (`RigidBodyContactCoupledStackRests`, non-singular
2x2 off-diagonal coupling); full experimental suite 39/39, lint clean.

**(DONE) Friction inside the LCP via `findex`.** The contact LCP now has three
rows per contact (normal + two friction tangents). The friction rows store the
coefficient `mu` in `hi`, set `lo = -mu`, and point `findex` at the contact's
normal row (global index `3*i`); the ODE Dantzig solver scales that bound by the
solved normal impulse to enforce `|lambda_t| <= mu*lambda_n`, so normal and
friction are solved jointly. The Delassus assembly generalized cleanly: the same
per-end formula with the row/column direction (normal or tangent) substituted,
giving `effectiveMass`/`tangentMass1`/`tangentMass2` on the block diagonal. The
right-hand side is `restitutionTarget - approach_n` for the normal row and
`-approach_t` for the tangent rows (drive tangential velocity to zero). On the
success path the solved normal+friction impulse is applied in one shot. The
rank-deficient fallback now solves the **coupled normal-only** LCP (the
normal-row block of the assembled operator) and only then the proven sequential
friction sweep — extracting the normal block matters because adding friction
rows makes the box-on-plane system fail the pivot more often, and the bare
diagonal projection under-supported the box (large corner lever arms) and
starved friction. Verified by the drop/rest/bounce/friction guardrails, the
stack test, the 1e-9 swap, and a new `RigidBodyContactFrictionRollsSlidingSphere`
test (single contact = coupled success path; friction drives the contact slip to
zero so the sphere rolls). Full experimental suite 39/39, `test_world` 104/104,
lint clean.

**Next (remaining Subsystem A): the full pipeline reorder.** Velocity-integrate
everything, then ONE unified boxed-LCP over rigid-rigid AND multibody-link
contacts, then position-integrate — subsuming the separate `RigidBodyContactStage`
and `simulateMultibody` contact passes so the friction cone couples across the
rigid/articulated split. **This is a multi-slice refactor of the core step loop,
not a safe increment** — design + adversarial review done (below); execute as
individually-reviewed gated PRs, suite as guardrail, revert per slice.

Validated plan (read-only design workflow + two adversarial critiques):

- **Target**: `[Velocity] RigidBodyVelocity + MultibodyVelocity` → `[Solve]
UnifiedConstraintStage (one boxed LCP)` → `[Position] RigidBodyPosition +
MultibodyPosition`, then Deformable, Kinematics. The hard part is splitting
  `simulateMultibody` (`multibody_dynamics.cpp:637`), which today fuses the qddot
  velocity solve + link-contact GS + manifold position integration into one
  per-multibody call.
- **Unification math**: both solves are the same Delassus form. Generalize the
  rigid `delassusEntry` (`world_step_stage.cpp:5181`) to "sum over shared
  bodies", where a shared _link_ contributes `J_i^T M^-1 J_j` (the existing
  `J^T M^-1 J` denominator at `multibody_dynamics.cpp:928`). Link-vs-link then
  falls out mathematically but still needs new collision routing
  (`multibody_dynamics.cpp:1282-1316` currently drops both-in-body pairs).
- **Slicing** (each green on `pixi run test-all`): (0) factor the **four**
  `World::step` pipeline builders (`world.cpp:1592/1619/1646/1665`) into one
  helper — two append a caller stage and skip the variational branch, so
  preserve that divergence; (1) extract the manifold **position-integration
  tail** (`multibody_dynamics.cpp:1042-1107`) into a free
  `integrateMultibodyPositions(...)` in a new `compute/multibody_constraint.*`
  (cleanest, purely-mechanical seam — the recommended FIRST slice); (2) add
  `MultibodyVelocityStage`/`MultibodyPositionStage` + reorder, link contacts
  still in their own stage; (3+4, **merged** — do not bisect the link-contact
  set across two solvers) unify rigid + link-static + link-dynamic-rigid into
  `UnifiedConstraintStage`; (5a) same-multibody link-vs-link routing via
  relative Jacobians; (5b) cross-multibody link-vs-link, which needs a
  multi-block row ownership/application model.
- **(DONE locally, commit `d7f3a17c98f`) Slice 1 — multibody position integration
  helper.** Added `compute/multibody_constraint.{hpp,cpp}` with
  `integrateMultibodyPositions(registry, structure, nextVelocity, timeStep)`,
  wired `simulateMultibody` to call it after velocity limits, and added
  `test_multibody_constraint` for direct spherical/floating manifold integration
  and position-limit clamping. This is behavior-preserving: it does not reorder
  `World::step`, move contact solving, or split velocity integration yet.
  Verified with `cmake --build build/default/cpp/Release --target
dart-simulation-experimental test_multibody_constraint test_world
test_skeleton_to_multibody`, focused CTest over the new/adjacent tests, and
  full `ctest --test-dir build/default/cpp/Release --output-on-failure -L
simulation-experimental` (40/40).
- **(DONE locally, commit `0684f723132`) Slice 0 — `World::step` pipeline
  builder factor.** Factored the four default/custom-stage pipeline builders in
  `world.cpp` through one stack-owned `WorldStepPipelineStages` helper. This
  preserves the current order, including the default overloads' variational
  multibody branch and the custom-stage overloads' existing semi-implicit
  multibody stage. Verified with focused build + `test_world`, full
  `ctest --test-dir build/default/cpp/Release --output-on-failure -L
simulation-experimental` (40/40), and `pixi run lint`.
- **(DONE locally) Slice 2 — semi-implicit multibody
  velocity/contact/position split.** Added `MultibodyVelocityStage`,
  `MultibodyContactStage`, and `MultibodyPositionStage`, with an internal
  transient staged-velocity component so link-contact impulses and velocity
  limits still act before positions are written back. The default
  semi-implicit pipeline is now all velocity work first, then rigid/link
  contact stages, then rigid/multibody position stages, followed by deformable
  dynamics and kinematics; the variational branch keeps its previous ordering.
  Verified with focused build + CTest for `test_multibody_constraint`,
  `test_world`, and `test_skeleton_to_multibody`, plus full
  `ctest --test-dir build/default/cpp/Release --output-on-failure -L
simulation-experimental` (40/40), and `pixi run lint`.
- **(DONE locally) Slice 3a — rigid-only LCP assembly seam and determinism
  gate.** Extracted rigid contact problem assembly into
  `compute/rigid_body_constraint.{hpp,cpp}` so `RigidBodyContactStage` now solves
  the same assembled `A,b,lo,hi,findex` returned by a focused helper. Added
  `test_rigid_body_constraint` to lock the rigid-only row ordering, normal
  coupling block, Coulomb `findex` bounds, and element-exact repeat assembly for
  a multibody-free stack scene. Verified with focused build + CTest for
  `test_rigid_body_constraint` and `test_world`, plus full
  `ctest --test-dir build/default/cpp/Release --output-on-failure -L
simulation-experimental` (41/41), and `pixi run lint`.
- **(DONE locally) Slice 3b — multibody link-contact assembly seam (the
  link-side comparator).** Extracted the link-contact row assembly (point
  Jacobians, normal/tangent directions, diagonal Delassus denominators,
  Baumgarte bias, restitution target, two-sided dynamic-rigid-obstacle coupling)
  out of `solveMultibodyLinkContacts` into a public
  `assembleMultibodyLinkContactProblem(registry, structure, nextVelocity,
  timeStep, linkContacts)` in `compute/multibody_dynamics.{hpp,cpp}`, returning
  `MultibodyLinkContactProblem{rows, inverseMass}` (the joint-space M^-1 the
  unify needs to form `J_i^T M^-1 J_j` cross-coupling). Promoted the anonymous
  `LinkContact`/`ContactRow` to public `LinkContact` / `MultibodyLinkContactRow`,
  and shared the relative-velocity math through a file-local
  `linkContactRelativeVelocity` helper used by both the restitution baseline and
  the unchanged 8-iteration Gauss-Seidel sweep. This is the link-side
  counterpart of slice 3a's `assembleRigidBodyContactProblem`: the unify (3c)
  stacks both assemblers' rows into one boxed-LCP, comparing the rigid sub-block
  against `assembleRigidBodyContactProblem` and the link sub-block against this.
  **Behavior-preserving** — no `World::step` reorder, no solve-math change; the
  assembler computes `inverseMass` once and the solve consumes it (no recompute).
  An independent code-review pass confirmed byte-for-byte equivalence against
  `HEAD`. The assembler lives in `multibody_dynamics.*` (it needs the internal
  dynamics tree / mass matrix / body Jacobians); `multibody_constraint.*` stays
  the pure position-integration home. Added `test_multibody_link_contact`
  locking the one-sided row (orthonormal contact frame, denominator
  self-consistency, Baumgarte bias, restitution target), the two-sided obstacle
  coupling (unit inverse mass / identity inverse inertia / arm, strict
  denominator increase), element-exact repeat assembly, and the
  velocity-dimension-mismatch throw. Verified with focused build + CTest for
  `test_multibody_link_contact`, `test_multibody_constraint`,
  `test_rigid_body_constraint`, `test_world`, and `test_skeleton_to_multibody`,
  plus full `ctest --test-dir build/default/cpp/Release --output-on-failure -L
simulation-experimental` (42/42), and `pixi run lint`.
- **Slice 3c — `UnifiedConstraintStage` (the unify). Validated design + 3-way
  adversarial critique** (workflow `unify-contact-solve-design`, 14 agents).
  Decomposed into 7 individually-gated sub-slices; ONLY the last (3c-ii.2) flips
  the pipeline — everything before ships net-new, dead, unit-tested code (zero
  behavior risk). Hard correctness pins from the critiques: (a) construct
  `LcpOptions` as `solver.getDefaultOptions(); o.earlyTermination = true;`
  verbatim — never default-construct (it flips `succeeded()`, hence the fallback
  decision, in exactly the rank-deficient cases bit-identity must cover);
  (b) a dynamic obstacle shared between a rigid contact and a link contact must
  have ONE inverse mass/inertia injected into the rigid diagonal, link diagonal,
  AND all cross terms (the two legacy paths normalize differently and zero
  differently on LDLT failure, which would make A asymmetric); (c) compact
  inactive link rows BEFORE computing global findex; (d) the rank-deficient
  fallback's normal-row gather enumerates in ASCENDING global-row order so the
  multibody-free case reproduces the legacy `i*3` stride byte-for-byte; (e) fill
  shared-obstacle cross terms over ALL NINE direction pairs (rigid
  `unitOrthogonal` vs link `cross` tangent frames are misaligned). Rigid
  bit-identity is guaranteed ONLY for multibody-free and disjoint-mixed worlds;
  shared-obstacle mixed scenes change at FP level (simultaneous vs sequential
  solve) and are tolerance-based. Variational path is untouched (keeps
  `RigidBodyContactStage`); the `method != Variational` gate is structural.
  - **(DONE locally) Slice 3c-i.2 — unified problem types + within-domain
    assembler.** Added `compute/unified_constraint.{hpp,cpp}` with
    `UnifiedConstraintProblem` (global `delassus/rhs/lo/hi/findex` + `RowOwner`
    provenance + verbatim `rigidConstraints` copy + per-multibody
    `UnifiedMultibodyBlock`s) and the pure function
    `assembleUnifiedConstraintProblem(rigidProblem, multibodyContacts)`. It
    copies the rigid `A,b,lo,hi,findex` VERBATIM into the leading block, compacts
    each multibody's active link rows, lays out three-row triples after the rigid
    block, fills the full dense within-multibody `J_i^T M_k^-1 J_j` coupling (the
    diagonal reproduces the stored row denominators bit-identically for
    obstacle-free contacts), and sets link `lo/hi/findex` against the COMPACTED
    global indices. Cross terms are zero (deferred to 3c-i.3). Extended
    `MultibodyLinkContactRow` with `normalRhs/tangentRhs1/tangentRhs2` (the
    pre-solve boxed-LCP targets) computed in `assembleMultibodyLinkContactProblem`
    so the unified assembler stays a pure function of the sub-problems; the GS
    solve ignores them, so 3b behavior is unchanged. NO pipeline change. Added
    `test_unified_constraint` (multibody-free reproduces
    `assembleRigidBodyContactProblem` byte-for-byte; rigid-free single-multibody
    dense block equals `J_i^T M^-1 J_j` with the diagonal == denominators and a
    non-trivial off-diagonal; findex range/self/normal-target asserts; fixed-base
    inactive-row compaction → non-singular) and extended the 3b test for the new
    rhs fields. Verified focused build + CTest for `test_unified_constraint`,
    `test_multibody_link_contact`, `test_rigid_body_constraint`,
    `test_multibody_constraint`, `test_world`, `test_skeleton_to_multibody`, full
    `ctest -L simulation-experimental` (43/43), and `pixi run lint`.
  - **(DONE locally) Slice 3c-i.3 — shared-obstacle cross terms + single-source
    inertia.** `assembleUnifiedConstraintProblem` now fills the coupling through
    a dynamic rigid body shared between contacts, reusing one `sharedBodyEntry`
    kernel that mirrors the rigid `delassusEntry` exactly: a link contact's own
    obstacle self-term (the diagonal completes to the stored denominator),
    link<->link coupling through a shared obstacle, and rigid<->link coupling
    through a body that is both a rigid-contact participant and a link obstacle —
    over ALL nine direction pairs (the rigid `unitOrthogonal` and link `cross`
    tangent frames are misaligned, so tangential coupling is generally nonzero).
    Cross terms are added only for pairs with at least one link row, so the
    verbatim rigid-rigid block is untouched. A shared body's `(invMass,
invInertia)` is reconciled to the rigid path's canonical value (the rigid and
    link assemblers normalize / handle LDLT failure differently). Still a pure
    function; NO pipeline change. Added the `CouplesSharedDynamicObstacle`
    test: an obstacle touched by both a rigid contact (on a static ground) and a
    prismatic leg, asserting the 3x3 cross block equals the analytic shared-body
    term over all nine direction pairs, global symmetry to 1e-12, the rigid block
    unchanged, reconciliation equality, and the link diagonal completing to the
    stored denominator. Verified focused build + `test_unified_constraint`
    (5/5), full `ctest -L simulation-experimental` (43/43), `pixi run lint`.
  - **(DONE locally) Slice 3c-i.4 — unified solver + impulse application.** Added
    `solveUnifiedConstraintProblem` (joint Dantzig solve with the PINNED options
    `solver.getDefaultOptions(); earlyTermination = true;` — never
    default-constructed — returning `{lambda, succeeded}`) and
    `applyUnifiedConstraintImpulses(registry, problem, lambda,
multibodyVelocities)` which applies the solved global impulses to rigid
    bodies (`applyRigidBodyContactImpulse` verbatim) and to each multibody's
    staged generalized velocity (`M_k^-1 J^T lambda`), plus the
    equal-and-opposite Newton impulse to a dynamic obstacle; a body shared
    between a rigid contact and a link obstacle accumulates both. Normal impulses
    clamped non-negative; all applied post-solve so order is irrelevant. Still no
    pipeline change. Added four tests: boxed-LCP KKT + Coulomb cone on a sliding
    box, head-on rigid contact driven to zero relative normal velocity, a
    one-sided link contact reaching the `max(bias, restitution)` target, and the
    exact equal-and-opposite Newton impulse delivered to a two-sided obstacle.
    Verified focused build + `test_unified_constraint` (9/9), full
    `ctest -L simulation-experimental` (43/43), `pixi run lint`.
  - **(DONE locally) Slice 3c-i.5 — generalized rank-deficient fallback.** Added
    `applyUnifiedConstraintFallback` (coupled NORMAL-only boxed-LCP over the
    `{findex<0}` rows gathered in ASCENDING global-row order — so a
    multibody-free set reproduces the legacy `i*3` stride — with a
    diagonal-projection last resort, then a sequential Coulomb friction sweep
    bounded by each contact's solved normal impulse over rigid contacts then link
    contacts, reading live velocities each pass: rigid friction via
    `comps::Velocity`, link friction via `M_k^-1 J^T` + the obstacle's
    `comps::Velocity`) and `resolveUnifiedConstraints` (joint solve → apply, else
    fallback; returns whether the joint solve succeeded). No pipeline change.
    Added tests: the fallback stops a head-on rigid contact identically to the
    joint solve; the fallback's friction opposes a sliding box without reversing
    it; `resolveUnifiedConstraints` takes the joint path on a full-rank contact;
    and the fallback arrests a four-coplanar box-on-plane (the canonical
    rank-deficient set, exercised directly since this configuration's joint solve
    happens to succeed). Verified focused build + `test_unified_constraint`
    (13/13), full `ctest -L simulation-experimental` (43/43), `pixi run lint`.
  - **(DONE locally) Slice 3c-ii.1 — the `UnifiedConstraintStage` class (dead,
    not wired).** Added `UnifiedConstraintStage` (declared in
    `multibody_dynamics.hpp`, defined in `multibody_dynamics.cpp` for access to
    the anonymous routing/staging helpers `collectMultibodyLinkContacts`,
    `gatherMultibodyVelocity`, `PendingMultibodyVelocity`). Its `execute`
    queries collisions once, assembles the rigid-rigid problem and each
    multibody's link problem (recomputing the dynamics tree / `M^-1` in-stage,
    never cached; `PendingMultibodyVelocity` created from `gatherMultibodyVelocity`
    when absent, mirroring `MultibodyContactStage`), stacks them via
    `assembleUnifiedConstraintProblem`, calls `resolveUnifiedConstraints` over the
    staged generalized velocities (collected in lockstep with the contacts so the
    blocks line up), writes each resolved velocity back to `PendingMultibodyVelocity`,
    then runs the rigid positional projection verbatim from
    `RigidBodyContactStage`. Defaults to 8 friction iterations (matching both
    `RigidBodyContactStage` and the link GS). NOT wired into any pipeline.
    Unit-tested via direct `execute` (`test_unified_constraint_stage`): a sphere
    overlapping a static ground has its descent arrested and is projected out of
    penetration, and a contact-free body is untouched. (The link-side
    orchestration and the bit-identity / tolerance baselines are validated by the
    full emergent suite at the flip, where the stage actually runs.) Verified
    focused build + `test_unified_constraint_stage` (2/2) + `test_world`
    (104/104), full `ctest -L simulation-experimental` (44/44), `pixi run lint`.
  - **(DONE locally) Slice 3c-ii.2 — the pipeline flip (the behavior-changing
    slice).** `appendSemiImplicitSplitStages()` (`world.cpp`) now wires the
    semi-implicit order as `RigidBodyVelocityStage`, `MultibodyVelocityStage`,
    `UnifiedConstraintStage`, `RigidBodyPositionStage`, `MultibodyPositionStage`,
    then `DeformableDynamicsStage`:
    the single `UnifiedConstraintStage` replaces the separate
    `RigidBodyContactStage` + `MultibodyContactStage` passes. Positions-last and
    the post-solve velocity-limit clamp (in `MultibodyPositionStage`) are
    preserved. The variational branch and both legacy stage classes are left
    intact (the variational path still uses `RigidBodyContactStage`). The whole
    emergent suite — rigid drop/rest/stack/friction/bounce, link-rest, two-sided
    striker momentum — now runs through the unified boxed-LCP and stays within
    its existing tolerances. Verified: full `ctest -L simulation-experimental`
    (44/44, including `test_unified_constraint_stage`), and an independent
    code-review pass APPROVED with zero behavior-changing defects (verified
    positions-last, the lockstep block/velocity write-back including the
    zero-active-row case, the byte-identical rigid positional projection running
    exactly once, the `PendingMultibodyVelocity` lifecycle, and that the single
    `collide()` call is a consistency improvement).
  - **Known environmental blocker (NOT from this work):** `pixi run test-all`
    cannot complete the **dartpy (Python bindings) build** in this environment —
    `python/dartpy/.../tri_mesh.cpp`, `hierarchical_ik.cpp`, etc. fail with
    `_POSIX_C_SOURCE`/`_XOPEN_SOURCE` redefined in `pyconfig.h` under `-Werror`
    (the system `features.h` uses `202405L`/`800`, newer than Python 3.12's
    `200809L`/`700`). Confirmed pre-existing: the dartpy build fails identically
    with this slice's change stashed. It blocks the Python half of `test-all`
    for the whole repo; address separately (e.g. include `<Python.h>` first in the
    affected binding TUs, or relax `-Werror=cpp` for dartpy).
  - **Slice 3c is complete locally** (assembler → cross terms → solver/apply →
    fallback → stage → flip, six gated commits). The default semi-implicit
    pipeline now resolves rigid-rigid and articulated link contacts in ONE
    coupled boxed-LCP. Remaining Subsystem A work is **slice 5 (link-vs-link
    two-sided contacts + the new collision routing)** and optional islands /
    warm-starting / friction-cone polish (all explicit follow-ups).
  - **(DONE locally) Slice 5a — same-multibody link-vs-link contacts.**
    `LinkContact` now carries `otherLink` for a link obstacle owned by the same
    `MultibodyStructure`. `collectMultibodyLinkContacts` accepts both-in-body
    collision pairs and orients one contact row into the primary link; the
    assembler subtracts the obstacle link's contact-point Jacobian from the
    primary link's Jacobian, so the existing unified block naturally sees
    `J_rel M^-1 J_rel^T` and the solved impulse updates both links through the
    same staged generalized velocity. Dynamic rigid `otherBody` coupling remains
    unchanged and is intentionally ignored when `otherLink` is set. Added
    assembler coverage for the `[1, -1]` relative Jacobian on two sibling
    prismatic links and an emergent world test where overlapping sibling links
    stop approaching and separate through the default `UnifiedConstraintStage`.
    Focused validation passed:
    `build-simulation-tests`, `test_multibody_link_contact`,
    selected `test_world` link-contact cases, selected `test_unified_constraint`
    cases, and `test_unified_constraint_stage`; full
    `ctest -L simulation-experimental` passed (44/44), and `pixi run lint`
    passed.
  - **(DONE locally) Slice 5b — cross-multibody link-vs-link contacts.** A
    cross-multibody link row is owned by one multibody but now carries
    `otherLink` + `otherMultibody` as a second articulated end. The stage hoists
    multibody contact collection so both the owner and the other multibody get
    solve blocks and staged velocities, completes the other side's contact-point
    Jacobians after all blocks are assembled, and routes the solved normal and
    friction impulses with opposite signs into both staged velocity vectors.
    `assembleUnifiedConstraintProblem` resolves `otherMultibodyIndex`, adds the
    cross-block articulated Delassus terms without perturbing the existing
    primary-primary block, and both the direct apply path and rank-deficient
    fallback update the second articulated end. Added assembler/apply coverage
    for the second articulated end and an emergent world test where two separate
    prismatic multibodies collide and stop approaching. Focused validation
    passed: `build-simulation-tests`, selected
    `test_unified_constraint`, selected `test_world`, and
    `test_multibody_link_contact`; full
    `ctest -L simulation-experimental` passed (44/44). Final validation passed:
    `pixi run lint`, post-lint `pixi run build-simulation-tests`,
    and post-lint `ctest -L simulation-experimental` (44/44).
  - **(DONE locally) Slice 5c — row-islanded unified LCP solve.**
    `solveUnifiedConstraintProblem` now decomposes independent row islands before
    calling Dantzig: rows are connected by nonzero Delassus entries or by
    `findex` normal/friction references, each island is solved with the same
    pinned `getDefaultOptions(); earlyTermination = true` settings, and the
    local lambdas are scattered back into the global impulse vector. Fully
    coupled systems still use the original monolithic solve, and any island
    failure returns `succeeded=false` so the existing global fallback remains the
    recovery path. Added a regression with two independent contact islands where
    the second island's friction rows must remap global `findex=3` to local row
    zero. Focused validation passed: `build-simulation-tests`,
    selected `test_unified_constraint`; full
    `ctest -L simulation-experimental` passed (44/44). Final validation passed:
    `pixi run lint`, post-lint `pixi run build-simulation-tests`,
    and post-lint `ctest -L simulation-experimental` (44/44).
- **Blockers the critiques verified (address before the relevant slice):**
  - _Positions last._ Keep the existing invariant (`world.cpp:1606` comment): no
    position stage runs until every velocity-writing stage has. A naive Slice-2
    order (`RigidBodyPosition` before `MultibodyPosition`) violates it.
  - _Per-multibody loop hoist._ The stage loops over multibodies and calls
    `simulateMultibody` per body; a single LCP must hoist that loop and index
    per-multibody solve contexts.
  - _Velocity clamp placement._ The joint velocity-limit clamp runs **after** the
    contact solve today (`multibody_dynamics.cpp:1021`); keep it post-solve, not
    in the velocity stage.
  - _Per-row restitution/Baumgarte._ Rigid uses threshold `1e-3`, no bias; link
    uses `1e-2` + Baumgarte (`multibody_dynamics.cpp:961,980`). Carry a per-row
    threshold + bias (0 for rigid rows) so the rigid `1e-9` swap stays
    bit-identical and link resting/restitution is preserved — do NOT collapse to
    one global convention.
  - _Friction `findex` stride._ The rank-deficient fallback extracts the
    normal-only block by striding `i*3` (`world_step_stage.cpp:5288-5392`); that
    breaks once dense link rows interleave. Recompute `findex` against the
    unified global row index and rework the fallback for link rows.
  - _Fallback friction over links._ The fallback's sequential Coulomb sweep only
    samples/applies rigid `comps::Velocity`; it has no link `M^-1 J^T` path, so
    it would silently skip link friction. Generalize before merging link contacts.
  - _M^-1 lifetime._ Do NOT cache `M^-1`/`DynamicsTree` in an EnTT component
    across stages (reference-invalidation hazard); recompute `M^-1` once in the
    solve stage (the project favors bit-stability over the micro-opt).
  - _Determinism gate._ The rigid-only assembly helper and
    `test_rigid_body_constraint` now lock row order and element-exact
    `A,b,lo,hi,findex` repeat assembly for a multibody-free world; the unify
    slice should compare its rigid sub-block against that helper.
  - _Out of scope initially_: the variational integrator path
    (`variational_integration.cpp:1101`) keeps its own contact handling — gate
    `UnifiedConstraintStage` on `method != Variational`. Joint limits stay
    post-integration clamps; motors stay the pre-contact equality solve.
    Warm-starting remains a deliberate non-goal until it has a separate
    evidence-backed slice.

Then: optional **friction-cone polish** — the ODE `findex` bound is frozen once
per solve (a one-shot decoupling approximation); a true cone or an outer
normal/friction iteration would tighten coupled stacking friction if a scene
needs it. Warm-starting is also available as a separate scaling polish slice.

## Immediate Next Step

**MVP done (PR #2705 merged).** The #2698 convention re-alignment shipped and
the required dart-gui example (Subsystem C) is on `main`
(`examples/demos/scenes/experimental_rigid_body_gui.cpp` plus the Python `sx_*`
demos). The blocker described in earlier sessions is resolved.

**All joint types are implemented and verified** (fixed, revolute, prismatic,
screw, universal, planar, ball, free) with a floating base via a `Free` joint,
including the config-dependent-subspace `cJ` term and SO(3)/SE(3) manifold
integration. **Subsystem B (model loading) and the Subsystem A unified contact
solve are both on `main`** — see below. The immediate next step is a smaller
follow-up slice such as visual geometry/material loading, richer load
diagnostics, or warm-starting/friction-cone polish; for the current Subsystem A
headline, prefer warm-start/friction-cone scaling polish around the unified
constraint solve.

### Subsystem A — full constraint solver / boxed-LCP (two-sided contacts)

What works today: the default semi-implicit pipeline velocity-integrates rigid
and articulated bodies, runs one `UnifiedConstraintStage` boxed-LCP over
rigid-rigid, link-vs-static-rigid, link-vs-dynamic-rigid, and same-multibody
link-vs-link and cross-multibody link-vs-link rows, then position-integrates.
Rigid positional correction still runs after the unified velocity solve.
**Missing:** remaining scaling/polish work (warm starting / friction-cone
iteration).

- **Pipeline ordering (done for semi-implicit):** the default semi-implicit
  `World::step` pipeline now runs `RigidBodyVelocityStage` →
  `MultibodyVelocityStage` → `UnifiedConstraintStage` →
  `RigidBodyPositionStage` → `MultibodyPositionStage` →
  `DeformableDynamicsStage` → `KinematicsStage`. The variational branch keeps
  its dedicated path.
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
  free body (C++ + dartpy). The later unified pipeline supersedes this
  per-multibody solve for the default semi-implicit path. **Still remaining:**
  cross-multibody link-vs-link contacts and scaling polish.

### Subsystem B — model loading (URDF/SDF/MJCF) into the DART 7 World

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
- **(DONE) C++/dartpy model-loading bridge.**
  `simulation::io::buildMultibodyFromSkeleton()` /
  `buildMultibodiesFromWorld()` provide the lower-level translation pass, while
  `addSkeleton()` / `addWorld()` and dartpy
  `simulation.add_skeleton()` / `add_world()` compose that bridge
  with `dart::io::readSkeleton()` / `readWorld()` URI loading and optional
  `ReadOptions` / `SkeletonLoadOptions`.
  - **Frame-mapping decision.** The importer places each experimental link frame
    on that link's outgoing joint frame, adds a synthetic fixed root anchor when
    needed, and computes each child joint's parent-to-joint transform with mass,
    inertia, COM, axes, and joint state re-expressed in the placed frame. For
    moving joints the residual parent-side mapping must be translation-free;
    that check rejects currently unrepresentable legacy layouts honestly.
  - **(DONE) Joint families and properties.** Weld, revolute, prismatic, screw,
    universal, ball-to-spherical, planar, and free-to-floating tree joints are
    mapped, including names, synthetic root anchors, axes, screw pitch, state,
    actuator family, revolute/prismatic limits, damping, springs, friction, mass,
    inertia, and local COM offsets. C++ and dartpy tests cover structure/property
    transfer, classic-vs-experimental pendulum stepping, higher-DOF joint-family
    parity, URI loading, already-parsed handoff, and multi-skeleton world import.
  - **(DONE) Collision shapes.** Sphere, box, capsule, cylinder, plane, and
    triangular mesh collision shapes are translated with shape-node local
    transforms preserved on `CollisionShape`. Convex meshes reuse the mesh
    carrier, heightmaps are triangulated into that carrier, and soft meshes
    snapshot their point-mass mesh into it. Multiple representable collision
    shape nodes per body/link are imported, serialized, exposed in C++/dartpy,
    and collided as compound geometry without same-entity self contacts.
  - **Remaining:** visual geometry, materials, resource-retriever bindings,
    richer diagnostics/load-result ergonomics, multi-DOF joint-property force
    application, rotated parent-offset handling for ball/free/planar coordinate
    re-expression, and deformable obstacle queries beyond the first sphere/box
    barrier.

### Subsystem C — MVP GUI example (DONE)

Landed on `main`: `examples/demos/scenes/experimental_rigid_body_gui.cpp` steps
the experimental rigid-body `World` and renders it live in `dart-demos`, and the
Python `sx_*` demos (`python/examples/demos/scenes/sx_articulated.py`,
`sx_floating_base.py`) cover it from dartpy. The historical note below is kept
for context.

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

- **Phase 4:** MIMIC actuator mode / mimic-coupler relations is the last
  remaining actuator slice — reuse the existing `J M^-1 J^T` equality machinery
  (LOCKED, SERVO, and ACCELERATION landed 2026-07-04; SERVO adds a boxed-LCP path
  for effort-bounded saturation, ACCELERATION adds a `commandAcceleration` field
  and bumps the binary format to v28).
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

The old branch-publication guidance is obsolete: the model-loading and unified
contact/constraint line is already on `main` via PR #2838. Future follow-ups
should start from current `main`, keep PRs scoped to one polish slice, and use
the normal `DART 7.0` PR milestone and validation gates.

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
- Two related dev tasks: `simulation_api_design` (facade shape) and
  this one (dynamics). Keep concerns separate.

## How To Resume

```bash
cd /home/js/dev/dartsim/dart/task_1
git status -sb && git log -3 --oneline
```

Then read, in order:

```bash
docs/dev_tasks/rigid_body_dynamics_solver/README.md
docs/dev_tasks/rigid_body_dynamics_solver/01-gap-analysis.md
docs/dev_tasks/rigid_body_dynamics_solver/02-roadmap.md
docs/design/simulation_solver_architecture.md
docs/design/simulation_cpp_api.md
```

Then continue the current phase from "Immediate Next Step".
