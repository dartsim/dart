# Simulation Stability Checklist

## Symptoms and common causes

When a simulation looks “wrong”, it is often a numerical stability problem.
Typical symptoms and likely causes:

- **Explodes / velocities go to infinity**: timestep too large for stiffness
  (gains, springs), bad inertias/scales, or aggressive contact events.
- **High-frequency jitter** (especially in contact stacks): timestep too large,
  contact/friction edge cases, or insufficient damping/compliance.
- **Slow drift**: discretization error, marginally stable feedback, or energy
  injection from contacts/constraints.

## First-line stabilizers (in order)

1. **Reduce `dt`** via `World::setTimeStep(dt)`.
2. **Reduce stiffness**:
   - lower PD gains, or
   - use implicit joint stiffness/damping where appropriate.
3. **Add damping** (controller `Kd`, joint damping coefficients, or modest
   velocity-dependent damping forces).
4. **Clamp commands/forces** to avoid unrealistic transients.

These changes resolve most “it blows up” reports without needing deeper solver
work.

## Controller gotchas in DART

- `World::step()` defaults to `_resetCommand = true`, which clears commands and
  forces after each step. A controller should therefore set commands/forces
  every timestep.
- Ensure your `Joint::ActuatorType` matches the quantity you are setting
  (forces vs velocity/servo commands).

## Contacts and constraints

DART enforces contacts/constraints at the velocity level each step (impulses).
This is generally robust, but still sensitive to `dt` and model details.

If you see contact instability:

- Reduce `dt` first.
- Verify collision geometry and inertial parameters (mass/inertia scaling).
- Experiment with collision detector backends and constraint solver settings
  through `World::getConstraintSolver()`.

## Prefer compliance over huge gains

If you are pushing gains high just to “stiffen” a joint, consider modeling that
stiffness directly:

- Joint spring stiffness (`DegreeOfFreedom::setSpringStiffness`)
- Joint damping (`DegreeOfFreedom::setDampingCoefficient`)

These are handled with implicit terms and tend to remain stable at larger `dt`
than explicit high-gain feedback.

```{eval-rst}
See also:

- :doc:`../topics/numerical-methods` (time stepping + constraints overview)
- :doc:`../tutorials/collisions` (collision/contact setup patterns)
- :doc:`../tutorials/multi-pendulum` (implicit spring + damping behavior)
```

