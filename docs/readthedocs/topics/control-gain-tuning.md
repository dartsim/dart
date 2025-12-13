# Control Gain Tuning (Discrete-Time)

## Why discrete time matters

Most controller equations are derived in continuous time, but simulation runs
in discrete timesteps. If you increase gains without considering the timestep
(`dt`), an otherwise “reasonable” PD controller can become unstable purely due
to discretization.

This page gives practical gain-tuning guidance for DART’s default
semi-implicit Euler stepping (`World::step()`).

## A simple 1-DOF mental model

For a single actuated joint with inertia `I`, the dynamics are:

```text
I * ddq = tau
```

A PD controller is:

```text
tau = Kp * (q_des - q) + Kd * (dq_des - dq)
```

In continuous time, it is common to choose gains via a desired natural
frequency `omega` (rad/s) and damping ratio `zeta`:

```text
Kp = I * omega^2
Kd = 2 * zeta * I * omega
```

In simulation, stability depends strongly on `omega * dt`. As a rule of thumb,
pick `omega` such that `omega * dt` is comfortably below 1 (often closer to
0.1 for stiff/high-accuracy tracking).

## Practical tuning workflow in DART

1. Start with a conservative `dt` and verify the model behaves plausibly with
   *no controller* (gravity, contacts, joint limits).
2. Implement PD with modest `omega` and `zeta ≈ 1` (critical-ish damping).
3. Increase `omega` gradually until you hit a stability/quality limit, then
   either:
   - reduce `dt`, or
   - back off gains / add compliance (implicit springs/damping).

If you change `dt`, expect to re-tune gains. Keeping `omega * dt` roughly
constant is usually more predictive than keeping `Kp` or `Kd` constant.

## Use configuration-space differences

For joints with non-Euclidean coordinates (e.g., rotations), subtracting
position vectors can produce the wrong error direction/magnitude. Prefer:

- `Skeleton::getPositionDifferences(q_des, q)`
- `Skeleton::getVelocityDifferences(dq_des, dq)`

These compute errors in the appropriate configuration/tangent spaces.

## Floating-base models: unactuated DOFs

Many skeletons have an unactuated floating base (typically the first 6 DOFs).
Your controller should not apply torques there:

- Set the corresponding entries in `tau` to zero, or
- Use joint-level APIs and skip those DOFs explicitly.

## Saturation and safety clamps

Real actuators saturate, and so should most simulation controllers. Clamping
`tau` (or commands) can prevent numerical blow-ups during transients, bad
initial conditions, or contact events.

## Derivative noise and filtering

The “D” term is sensitive to noisy velocities (especially if you compute
numerical derivatives of positions). Prefer measured/true `dq` from the
skeleton, and if needed apply simple low-pass filtering to `dq` or to the
computed `tau`.

```{eval-rst}
See also:

- :doc:`../topics/control-theory` (PD and inverse-dynamics overview)
- :doc:`../tutorials/biped` (full PD controller example)
- :doc:`../tutorials/multi-pendulum` (implicit stiffness/damping as a stable alternative to huge gains)
```

