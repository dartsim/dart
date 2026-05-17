# Atlas Puppet Example

## Summary

- Goal: teleoperate the Atlas model with kinematic whole-body IK targets.
- Concepts/APIs: `dart::gui::ApplicationOptions`, `InverseKinematicsHandle`,
  `dart::gui::Gizmo`, keyboard actions, `SimpleFrame` target handles, and foot
  support geometry.
- Expected output: a DART GUI viewer displaying Atlas with selectable hand and
  foot IK target handles plus transform gizmo axes at those targets.
- Controls: press 1-4 or click a target handle to select it; left-drag a gizmo
  axis arrow or plane handle to move it, or a rotation ring to orient it;
  Ctrl-left-drag a selected target handle to move it; keyboard nudges also move
  the selected handle; WASD moves the root; Q/E yaw; F/Z adjusts height; space
  toggles simulation.

## Notes

- This example runs in kinematic mode and continuously resolves the promoted IK
  handles.
- The transform gizmos currently support X/Y/Z axis-arrow dragging and rotation
  ring dragging plus XY/YZ/XZ plane handles. Hovered and actively dragged
  handles use the configured highlight color.
- The default launch size is 1280x960, matching the historical standalone
  viewer.
- Remaining strict-parity gaps from the historical OSG source include target
  activation/deactivation.
- Other remaining gaps are X/C support toggles, R/T posture recovery,
  whole-body balance recovery, and default support-polygon/COM visualization.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

From the DART repository root:

```bash
pixi run ex atlas_puppet
```

For an automated smoke run:

```bash
pixi run ex atlas_puppet --headless --frames 2 --screenshot atlas_puppet.ppm
```
