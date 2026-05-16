# Hubo Puppet Example

## Summary

- Goal: teleoperate the Hubo model with kinematic IK targets in the promoted
  `dart::gui` application runner.
- Concepts/APIs: whole-body IK targets, support geometry,
  `dart::gui::ApplicationOptions`, renderer-neutral keyboard actions, promoted
  panel callbacks, and source-owned debug line geometry.
- Expected output: a kinematic Hubo model with toggleable hand, foot, and peg IK
  targets plus an always-visible support-polygon overlay.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, click
  selects renderables, Ctrl-left drag or arrow/PageUp/PageDown nudges selected
  items, Ctrl-Shift-left drag rotates selected target handles, and Escape
  exits.

## Notes

- This example runs in kinematic mode. The world keeps gravity for support
  polygon computation, while the Hubo body nodes ignore gravity and collision so
  target manipulation is stable.
- Press `1`-`6` to toggle the hand, foot, and peg IK targets. Active targets are
  added to the world and solved each step; inactive targets are hidden and do
  not constrain the robot.
- WASD moves the root, Q/E yaws the root, and F/Z changes root height.
- X/C toggles left/right foot support, P prints the current DOF positions, and T
  resets the relaxed posture while preserving the root x/y/yaw placement.
- The historical OpenSceneGraph example also supported Shift movement
  amplification, hold/release R balance optimization, exact COM marker colors,
  and a custom camera up vector. Those remain promoted API or solver follow-ups.

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
