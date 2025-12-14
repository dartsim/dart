# Control Theory Primer

## Why this exists

DART makes it easy to _simulate_ articulated rigid bodies, but it does not pick
controller gains, filter noisy measurements, or guarantee that an arbitrary
control law is numerically stable for a given timestep. This page gives a short
control-theory refresher and maps common concepts onto DART APIs.

## Mapping notation to DART

In most robotics/control texts, a multibody system is written in generalized
coordinates:

- Generalized position: `q`
- Generalized velocity: `dq`
- Generalized acceleration: `ddq`
- Generalized forces/torques: `tau`

In DART, these correspond to:

- `q`: `Skeleton::getPositions()` / `skeleton.getPositions()`
- `dq`: `Skeleton::getVelocities()` / `skeleton.getVelocities()`
- `ddq`: `Skeleton::getAccelerations()` / `skeleton.getAccelerations()`
- `tau`: `Skeleton::setForces(tau)` / `skeleton.setForces(tau)`

When computing pose errors, prefer DART’s configuration-space helpers over
naively subtracting vectors:

- Position error on configuration manifolds: `Skeleton::getPositionDifferences(q2, q1)`
- Velocity error: `Skeleton::getVelocityDifferences(dq2, dq1)`

These matter for joints whose coordinates live on a manifold (e.g., `BallJoint`
and the rotational part of `FreeJoint`).

## Actuation in DART: commands vs forces

Every `Joint` has an `ActuatorType` that determines what “command” means. Two
common patterns are:

- **Direct generalized forces** (`Joint::FORCE`): you compute `tau` and apply it
  with `Skeleton::setForces(...)`. This is what most tutorial controllers use.
- **Kinematic/servo-style commands** (`Joint::VELOCITY`, `Joint::SERVO`, ...):
  you set commands with `Skeleton::setCommands(...)` / `Joint::setCommand(...)`,
  and DART converts them into forces internally.

One easy-to-miss detail: `World::step()` defaults to `_resetCommand = true`,
which clears commands/forces after each step. A controller should therefore set
its commands (or forces) every timestep.

## Proportional-derivative (PD) feedback

PD control is the baseline for stabilizing a pose:

```text
tau = Kp * (q_des - q) + Kd * (dq_des - dq)
```

For floating-base systems, some DOFs are unactuated (typically the first six).
Set their gains (or the corresponding entries in `tau`) to zero.

The DART biped tutorial shows a full PD controller and discusses gain choices
for floating-base models.

## Feedforward + inverse dynamics (“computed torque”)

For better tracking than pure PD, add a feedforward term using the rigid-body
dynamics model:

```text
tau = M(q) * ddq_des + C(q, dq) + g(q) + Kp * e_q + Kd * e_dq
```

Where DART exposes:

- `M(q)`: `Skeleton::getMassMatrix()`
- `C(q, dq)`: `Skeleton::getCoriolisForces()` (or `getCoriolisAndGravityForces()`)
- `g(q)`: `Skeleton::getGravityForces()`

This style of controller is sensitive to modeling errors, contact, and time
discretization, so it is common to keep the PD terms as a stabilizing “outer
loop” even when using inverse dynamics.

## Impedance, stiffness, and damping

DART joints support implicit springs and damping (see the multi-pendulum
tutorial). This is often a convenient way to get “soft” behavior without
explicitly writing a controller:

- Rest pose: `DegreeOfFreedom::setRestPosition(...)`
- Spring stiffness: `DegreeOfFreedom::setSpringStiffness(...)`
- Damping: `DegreeOfFreedom::setDampingCoefficient(...)`

Because these are handled with implicit terms, they tend to be more numerically
stable than explicit high-gain PD for the same timestep.

```{eval-rst}
See also:

- :doc:`../tutorials/biped` (PD control in a full example)
- :doc:`../tutorials/multi-pendulum` (implicit spring + damping)
```
