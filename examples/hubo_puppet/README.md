# Hubo Puppet Example

## Summary

- Goal: teleoperate the Hubo model with kinematic IK targets in the promoted
  `dart::gui` application runner.
- Concepts/APIs: whole-body IK targets, support geometry,
  `dart::gui::ApplicationOptions`, renderer-neutral keyboard actions, promoted
  `dart::gui::Gizmo` target affordances, panel callbacks, and source-owned
  support debug line geometry.
- Expected output: a kinematic Hubo model with toggleable hand, foot, and peg IK
  target gizmos plus an always-visible support-polygon and COM validity overlay.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, press `1`-`6`
  to toggle/select targets, left-drag active gizmo handles, arrow/PageUp/PageDown
  nudges selected targets, and Escape exits.

## Notes

- This example runs in kinematic mode. The world keeps gravity for support
  polygon computation, while the Hubo body nodes ignore gravity and collision so
  target manipulation is stable.
- Press `1`-`6` to toggle the hand, foot, and peg IK targets. Active targets are
  rendered as public gizmos and solved each step; inactive targets are hidden
  and do not constrain the robot.
- WASD moves the root, Q/E yaws the root, and F/Z changes root height. Hold
  Shift with those keys for the historical 2x movement amplification.
- X/C toggles left/right foot support, hold R optimizes whole-body posture and
  balance, P prints the current DOF positions, and T resets the relaxed posture
  while preserving the root x/y/yaw placement.
- Whole-body IK continuously resolves active targets with the source-owned
  relaxed-posture objective and public balance constraint.
- The hand, foot, and peg targets use the historical source-owned analytical
  arm and leg IK methods, including post-analytical extra DOF use on hand
  targets.
- The support overlay is source-owned DART line geometry. The polygon and
  centroid are green; the COM marker is blue when its support projection is
  inside the active support polygon and red otherwise.
- The historical custom camera up vector is restored through public
  `dart::gui::OrbitCamera`.

## Run In Tree

From the repository root:

```bash
pixi run ex hubo_puppet --gui-scale 2
```

## Build

From this directory:

```bash
mkdir build
cd build
cmake ..
make
```

## Run

From the build directory:

```bash
./hubo_puppet
```
