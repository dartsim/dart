# Free Joint Cases Example

## Summary

- Goal: compare `FreeJoint` integration against reference models in zero
  gravity.
- Concepts/APIs: `dart::dynamics::FreeJoint`, world Jacobians, finite
  difference checks, transparent reference bodies, custom `dart::gui` panels,
  per-step callbacks, and example-specific command-line flags.
- Expected output: five colored single-body free-joint cases with larger
  transparent reference boxes.
- Controls: Space starts or pauses simulation. The panel can pause, step, reset
  cases, recompute numeric checks, toggle reference visibility, switch
  spherical inertia, switch constant-world-twist reference mode, and adjust
  torque-free substeps.

## Run

From the source tree:

```bash
pixi run ex free_joint_cases
```

The promoted `dart::gui` runner supports headless image capture:

```bash
pixi run ex free_joint_cases --headless --frames 2 --screenshot /tmp/free_joint_cases.ppm
```

Example-specific options:

```bash
pixi run ex free_joint_cases --ground-truth constant --spherical-inertia
pixi run ex free_joint_cases --numeric-dt 1e-6 --dt 0.001 --ground-truth-substeps 20
```
