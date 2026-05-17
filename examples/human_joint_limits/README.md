# Human Joint Limits Example

## Summary

- Goal: run the Kima human skeleton with DART joint-limit enforcement in the
  promoted `dart::gui` viewer.
- Concepts/APIs: `dart::io::readWorld`, `Joint::setLimitEnforcement`,
  `ConstraintSolver`, source-owned neural arm/leg custom constraints,
  renderer-neutral panels, and `dart::gui::ApplicationOptions`.
- Expected output: a live humanoid skeleton and ground loaded from
  `kima_human_edited.skel`.
- Controls: Space toggles simulation, `n` steps once while paused, and Escape
  exits.

## Run In Tree

From the repository root:

```bash
pixi run ex human_joint_limits
```

The default window size is 640x480. Command-line `--width`, `--height`,
`--headless`, `--screenshot <path>`, and `--out <dir>` options are handled by
the promoted `dart::gui` runner.

For a bounded headless capture:

```bash
pixi run ex human_joint_limits --headless --frames 2 --width 640 --height 480 --screenshot /tmp/dart_human_joint_limits.ppm
```

## Notes

- The historical OSG example installed neural
  `HumanArmJointLimitConstraint` and `HumanLegJointLimitConstraint` objects.
  The promoted source restores those constraints with an example-local reader
  for the checked-in TinyDNN-serialized model files, without adding TinyDNN
  back as a package dependency.
- The live `dart::gui` version still loads the original SKEL world, preserves
  its timestep and gravity, keeps the ground static, and enables DART
  joint-limit enforcement on every human joint.
