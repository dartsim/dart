# `free_joint_cases`

Visualizes multiple `dart::dynamics::FreeJoint` single-body scenarios in a
zero-gravity world.

Each case shows:

- An actively simulated body (colored box)
- A kinematic "ground truth" reference body (slightly larger, transparent box)
  whose pose is computed from the initial world-frame velocities `(ω, v)` as:
  `R(t) = Exp(ω t) R0` and `p(t) = p0 + v t`

Run from a build tree:

```sh
./free_joint_cases --gui-scale 1.0
```

Controls are available in the ImGui window for toggling simulation, resetting the cases, and running numeric checks (analytic vs. finite-difference Jacobians).
