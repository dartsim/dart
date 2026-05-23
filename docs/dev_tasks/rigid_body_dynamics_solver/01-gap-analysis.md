# Gap Analysis: Legacy DART 6 Rigid-Body Sim vs Experimental World

Snapshot date: 2026-05-23. This is a working artifact; it will decay as the
solver lands. Code is the source of truth — re-verify before acting on any row.

## Summary

The experimental `World` (`dart/simulation/experimental/`) today provides
topology, frames, forward kinematics, ECS storage, a compute-graph executor, and
serialization. Its only "dynamics" is a single explicit/semi-implicit
integration of **free rigid bodies** from a user-set force/torque accumulator.
There is **no gravity, no articulated-body dynamics, no constraints, no
contacts, and no collision integration**. Multibodies have forward kinematics
only; `Link`/`Joint` carry no dynamics. The legacy DART 6 path
(`dart/dynamics/`, `dart/constraint/`, `dart/simulation/world.cpp`) is a full
constrained rigid-body simulator. This is the gap the first solver must close.

## Current Experimental Capability (verified)

- `RigidBodyIntegrationStage` (`compute/world_step_stage.cpp`) integrates bodies
  with `RigidBodyTag`: `v += (F/m)·dt` and angular update via world inertia;
  then integrates pose. Reads the `Force` accumulator; does not clear it; **no
  gravity term**.
- `KinematicsStage` refreshes frame transforms (open-chain FK for tree joints).
- `World::step()` = `RigidBodyIntegrationStage` then `KinematicsStage`, advances
  time/frame once. No substepping. No collision, no constraint solve.
- `MultiBody`/`Link`/`Joint`: topology, names, DOF count, generalized
  position/velocity, open-chain FK. **No forward dynamics, no joint forces,
  limits, actuators, damping, or springs in the step.**
- `LoopClosure`: topology + residual diagnostics only; projection/solve rejected
  at runtime until a compatible stage exists.
- No `World::setGravity`. No collision detector ownership. No contact buffers.
- Public rigid-body handle exposes transform, velocity, mass, inertia, force,
  torque accumulators.

## Feature Gap Matrix

Status: MISSING (absent), PARTIAL (some support), PRESENT (comparable to DART 6).

### Forward / inverse dynamics

| Capability                               | DART 6 (legacy)                                         | Experimental                                                                             | Status  |
| ---------------------------------------- | ------------------------------------------------------- | ---------------------------------------------------------------------------------------- | ------- |
| Articulated-body forward dynamics        | `Skeleton::computeForwardDynamics` (Featherstone ABA)   | RNEA-based, fixed base, revolute/prismatic (Phase 1)                                     | PARTIAL |
| Inverse dynamics (RNEA)                  | `Skeleton::computeInverseDynamics`                      | `MultiBody::computeInverseDynamics` (Phase 4)                                            | PRESENT |
| Mass matrix / inverse mass matrix        | `getMassMatrix`, `getInvMassMatrix`, augmented variants | `MultiBody::getMassMatrix`/`getInverseMassMatrix` (Phase 4)                              | PRESENT |
| Coriolis / gravity generalized forces    | `getCoriolisForces`, `getGravityForces`                 | `MultiBody::getCoriolisForces`/`getGravityForces` (Phase 4)                              | PRESENT |
| Impulse-based dynamics (for constraints) | `computeImpulseForwardDynamics`                         | joint-space impulse response `M^-1 f` (Phase 4); full constrained version pending solver | PARTIAL |
| Free single-body integration             | per-body explicit integration                           | semi-implicit Euler (Phase 0.1)                                                          | PARTIAL |
| Floating base / other joint dynamics     | all joint types, free base                              | fixed base only; ball/free/universal/planar/screw dynamics not yet                       | MISSING |

### Gravity & global forces

| Capability              | DART 6                               | Experimental                                                        | Status  |
| ----------------------- | ------------------------------------ | ------------------------------------------------------------------- | ------- |
| World gravity           | `World::setGravity` → per-skeleton   | `World::setGravity`/`getGravity` applied in integration (Phase 0.1) | PRESENT |
| Per-body external force | `BodyNode::addExtForce/addExtTorque` | world-frame force/torque accumulator (Phase 0.1)                    | PARTIAL |
| Force reset each step   | `clearExternalForces` per step       | force/torque cleared after each step (Phase 0.2)                    | PRESENT |

### Joints

| Capability                                                  | DART 6                                                                                                 | Experimental                                                  | Status                       |
| ----------------------------------------------------------- | ------------------------------------------------------------------------------------------------------ | ------------------------------------------------------------- | ---------------------------- |
| Joint types                                                 | Weld, Revolute, Prismatic, Screw, Universal, Translational2D, Euler, Planar, Translational, Ball, Free | type tag + axis (FK only)                                     | PARTIAL                      |
| Joint forward dynamics                                      | per-joint accel via ABA                                                                                | revolute/prismatic (Phase 1)                                  | PARTIAL                      |
| Position/velocity/accel/force limits                        | `GenericJoint` setters + `JointConstraint`                                                             | position/velocity/effort limits (Phase 3-4)                   | PARTIAL                      |
| Spring stiffness + rest position                            | `setSpringStiffness`/`setRestPosition`                                                                 | revolute/prismatic (Phase 4)                                  | PRESENT (1-DOF)              |
| Damping                                                     | `setDampingCoefficient`                                                                                | revolute/prismatic (Phase 4)                                  | PRESENT (1-DOF)              |
| Coulomb joint friction                                      | `JointCoulombFrictionConstraint`                                                                       | per-coordinate velocity-level Coulomb friction (Phase 4)      | PARTIAL                      |
| Actuator types (FORCE/PASSIVE/SERVO/MIMIC/ACCEL/VEL/LOCKED) | per-DOF actuator type                                                                                  | Force + Passive (Phase 4); other modes reserved               | PARTIAL                      |
| Mimic / coupler                                             | `MimicMotorConstraint`, `CouplerConstraint`                                                            | LoopClosure metadata only                                     | MISSING                      |
| Armature / rotor inertia                                    | not in DART 6 (gap to _improve_ on)                                                                    | per-coordinate armature on the mass-matrix diagonal (Phase 4) | PRESENT (improves on DART 6) |

### Constraints & contacts

| Capability                      | DART 6                                             | Experimental                                                | Status                 |
| ------------------------------- | -------------------------------------------------- | ----------------------------------------------------------- | ---------------------- |
| Contact response solver         | `BoxedLcpConstraintSolver` (island grouping)       | sequential normal impulses, free rigid bodies (Phase 3)     | PARTIAL                |
| Boxed-LCP solver                | Dantzig primary, PGS fallback (`dart/math/lcp/`)   | not wired; current solver is sequential-impulse             | MISSING                |
| Rigid contact constraints       | `ContactConstraint` (friction pyramid)             | normal + friction-pyramid impulses, free bodies (Phase 3)   | PARTIAL                |
| Friction                        | friction pyramid + slip/dir params                 | two-tangent friction-pyramid Coulomb, free bodies (Phase 3) | PARTIAL                |
| Restitution                     | restitution coeff + `ContactSurfaceParams`         | per-body coeff, combined max, applied in solver (Phase 3)   | PARTIAL                |
| ERP / CFM tuning                | static members + `ContactSurfaceParams`            | fixed projection factor; no per-contact ERP/CFM yet         | MISSING                |
| Joint limit / motor constraints | `JointLimitConstraint`, `ServoMotorConstraint`     | none                                                        | MISSING                |
| Closed-loop joint constraints   | `BallJointConstraint`, `WeldJointConstraint`, etc. | residual only                                               | MISSING                |
| Contacts on multibody links     | links participate in the constraint solve          | rigid bodies only; links not yet                            | MISSING                |
| Static-body convention          | static skeletons / fixed base                      | public `isStatic` flag; immovable, no gravity (Phase 3)     | PRESENT                |
| Penetration correction          | ERP / split impulse                                | projection-based positional correction (Phase 3)            | PARTIAL                |
| Soft contacts                   | `SoftContactConstraint`                            | none                                                        | MISSING (later domain) |

### Integration

| Capability                 | DART 6                                            | Experimental        | Status                       |
| -------------------------- | ------------------------------------------------- | ------------------- | ---------------------------- |
| Semi-implicit Euler        | hard-coded step order; manifold-aware joint integ | free-body only      | PARTIAL                      |
| Manifold-aware joint integ | `math::integratePosition` (SO3/SE3 exp map)       | none (no joint dyn) | MISSING                      |
| Higher-order / pluggable   | none (DART 6 gap)                                 | none                | MISSING (improvement target) |
| Substepping inside step    | none                                              | none                | MISSING                      |

### Collision

| Capability                 | DART 6                                                      | Experimental                                  | Status  |
| -------------------------- | ----------------------------------------------------------- | --------------------------------------------- | ------- |
| Collision query owner      | `ConstraintSolver` owns detector                            | `World::collide()` query bridge (Phase 2)     | PARTIAL |
| Backends                   | FCL, Bullet, ODE, DART native, `collision/native`           | native engine bridged via `World::collide()`  | PARTIAL |
| Shape types                | box, sphere, capsule, cylinder, mesh, plane, heightmap, ... | sphere + box on rigid bodies (Phase 2)        | PARTIAL |
| Self-collision / filtering | `setSelfCollisionCheck`, `CollisionFilter`                  | none                                          | MISSING |
| Contacts in step (solved)  | contacts feed the constraint solver                         | query only; not fed to a solver yet (Phase 3) | MISSING |

> Note: `dart/collision/native/` is a maintained native collision engine with
> standalone world/query concepts (PLAN-035/036/037). The experimental World
> needs a public owner bridge before contacts can flow into the solver.

### Rigid-body quantities & API

| Capability                     | DART 6                                           | Experimental                                                | Status               |
| ------------------------------ | ------------------------------------------------ | ----------------------------------------------------------- | -------------------- |
| Center of mass / COM jacobian  | `Skeleton::getCOM*`                              | single-body COM = origin                                    | PARTIAL              |
| Linear/angular momentum        | `BodyNode::getLinearMomentum/getAngularMomentum` | single rigid body (Phase 0.3)                               | PRESENT (rigid body) |
| Kinetic/potential energy       | `Skeleton::computeKineticEnergy/Potential`       | single rigid body (Phase 0.3)                               | PRESENT (rigid body) |
| External force application API | `addExtForce` (local/global, offset)             | world-frame force/torque accum                              | PARTIAL              |
| Jacobians                      | `BodyNode::getJacobian`, COM jacobians           | body- and world-frame link Jacobians (Phase 5); COM pending | PARTIAL              |
| Model loading into world       | `dart::io` URDF/SDF/MJCF/SKEL                    | none on experimental                                        | MISSING (deferred)   |

## Improvement Opportunities Over DART 6

Where the new solver should do better than legacy, not just match:

- **Armature / rotor inertia** as first-class joint data (DART 6 lacks it).
- **Pluggable integrator** and explicit **substepping** (DART 6 hard-codes a
  single semi-implicit Euler step).
- **True friction cone** option alongside the legacy box/pyramid model.
- **Fresh-by-default reads** with explicit work placement instead of the DART 6
  dirty-flag network spread across frames/joints/bodies.
- **Backend-neutral compute graph** so the same dynamics can run sequential,
  multi-core, or accelerated without changing the public API.
- **Model/State separation** enabling rollouts, batching, and differentiability
  that DART 6 cannot express.

## Implications For Sequencing

The dependency order falls out of the matrix:

1. Gravity + a correct single-body integrator (foundation; unblocks "drop a
   box"). **First slice.**
2. Articulated-body forward dynamics for tree multibodies (joint accel, mass
   matrix, Coriolis/gravity, joint forces, damping/springs).
3. Collision bridge: experimental shapes + native-collision query → contact
   buffers.
4. Constraint/contact solver: wire the existing boxed-LCP library into a
   contact + joint-limit solve.
5. Joint features: limits, actuators, friction, mimic/coupler.
6. Loop-closure kinematic projection and dynamic solving.
7. Quantities/diagnostics: COM, momentum, energy, jacobians.

See [`02-roadmap.md`](02-roadmap.md) for the phased plan.
