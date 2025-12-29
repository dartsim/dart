# Numerical Methods Primer

## Why this exists

If a simulation “explodes”, jitters, or drifts, the root cause is often
numerical—not a bug in your controller logic. DART exposes enough low-level
dynamics data to implement sophisticated methods, but it still helps to know
what the default timestepper and constraint solver are doing.

This page summarizes the numerical methods most users run into first and links
to the DART tutorials where they show up in practice.

## Time stepping in DART

At the top level, simulation usually looks like:

```text
world->setTimeStep(dt);
while (...) {
  // compute/set controls for this dt
  world->step();  // advances by dt
}
```

The default `World::step()` integrates velocities, solves constraints, and then
integrates positions. In other words, DART uses a **semi-implicit Euler**
scheme for the unconstrained dynamics:

1. Compute forward dynamics to obtain `ddq`
2. Integrate velocities: `dq <- dq + dt * ddq`
3. Solve constraints and apply impulse-based velocity changes
4. Integrate positions: `q <- integrate(q, dq, dt)`

On Euclidean coordinates, the position integration is `q <- q + dt * dq`. For
rotational configuration spaces, DART uses exponential-map updates so the state
stays on the manifold.

## Choosing a timestep (`dt`)

Smaller timesteps generally improve stability and contact behavior, but increase
runtime. Large timesteps are more likely to cause:

- Instability with stiff springs, damping, or high-gain controllers
- Missed collisions (tunneling) for fast-moving bodies
- Noisy or “sticky” contact responses due to coarse discretization

When debugging instability, treat `dt` as a first-class parameter: many control
gains that are stable at `1/1000 s` will not be stable at `1/60 s`.

## Constraints, contacts, and implicit impulses

DART handles contacts and other constraints using an implicit, velocity-level
solve each step (see `ConstraintSolver::solve()` as invoked from `World::step()`).
This is why you can often get reasonable contact behavior even with relatively
simple integrators: the hard non-penetration constraints are enforced via
impulses rather than by explicitly integrating extremely stiff penalty forces.

However, contact problems are still numerically delicate. If you observe
jittering stacks or friction artifacts, experiment with:

- Smaller `dt`
- Different collision backends (FCL / Bullet / ODE) depending on your setup
- Constraint solver settings (through `World::getConstraintSolver()`)

## Stiffness and implicit joint springs

DART joints can model spring and damping terms that are handled implicitly,
which can be much more stable than explicit high-gain feedback at the same
timestep. If you are tempted to crank PD gains until things “look right”, first
consider whether joint stiffness/damping is a better fit for what you want.

```{eval-rst}
See also:

- :doc:`../tutorials/multi-pendulum` (implicit joint springs and damping)
- :doc:`../tutorials/collisions` (contact setup and collision handling)
```

## Implementing custom integrators

If you need higher-order explicit methods, adaptive timestepping, or custom
filtering, you can build your own loop around DART’s exposed dynamics quantities
(`getMassMatrix()`, `getCoriolisForces()`, `getGravityForces()`, etc.) instead
of relying on `World::step()`. Doing so means you also take responsibility for
handling constraints/contacts appropriately.
