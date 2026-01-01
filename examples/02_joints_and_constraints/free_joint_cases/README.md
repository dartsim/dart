# Free Joint Cases Example

## Summary

- Goal: compare free-joint integration against reference models in zero gravity.
- Concepts/APIs: `dynamics::FreeJoint`, Jacobian checks, ImGui viewer.
- Expected output: multiple colored bodies with transparent reference boxes.
- Controls: use the ImGui panel; CLI flags tune the reference model.

## Details

Visualizes multiple `dart::dynamics::FreeJoint` single-body scenarios in a
zero-gravity world.

Each case shows:

- An actively simulated body (colored box)
- A kinematic reference body (slightly larger, transparent box)

The reference pose can be computed using either:

- **Torque-free rigid-body dynamics** (default): integrates Euler's equations
  for the current inertia (RK4) while using `p(t)=p0+v0 t` for translation
- **Constant world twist**: `R(t) = Exp(ω t) R0` and `p(t) = p0 + v t` (exact
  when the world-frame twist `(ω, v)` stays constant, e.g., spherical inertia)

Run from a build tree:

```sh
./free_joint_cases --gui-scale 1.0
# Use spherical inertia so the constant-twist reference is exact:
./free_joint_cases --ground-truth constant --spherical-inertia
```

Controls are available in the ImGui window for toggling simulation, resetting the cases, and running numeric checks (analytic vs. finite-difference Jacobians).
