# Atlas Puppet Example

## Summary

- Goal: teleoperate the Atlas model with kinematic whole-body IK targets.
- Concepts/APIs: `dart::gui::ApplicationOptions`, `InverseKinematicsHandle`,
  `dart::gui::Gizmo`, keyboard actions, `SimpleFrame` IK targets, source-owned
  support debug geometry, and foot support geometry.
- Expected output: a DART GUI viewer displaying Atlas with toggleable hand and
  foot transform gizmos plus an always-visible support-polygon and COM validity
  overlay.
- Controls: press 1-4 to toggle/select a target for keyboard nudges; left-drag
  a gizmo axis arrow or plane handle to move it, or a rotation ring to orient
  it; WASD moves the root; Q/E yaw; F/Z adjusts height; X/C toggles left/right
  foot support; hold R to optimize whole-body posture and balance; P prints
  DOFs; T resets the relaxed posture; space toggles simulation.

## Notes

- This example runs in kinematic mode and continuously resolves promoted
  whole-body IK with the source-owned relaxed-posture objective and balance
  constraint.
- The transform gizmos support X/Y/Z axis-arrow dragging and rotation ring
  dragging plus XY/YZ/XZ plane handles. Hovered and actively dragged handles use
  the configured highlight color.
- Only active targets solve each simulation step. Activating a target adds its
  visible frame at the current end-effector transform; deactivating it removes
  the frame.
- The support overlay is source-owned DART line geometry. The polygon and
  centroid are green; the COM marker is blue when its support projection is
  inside the active support polygon and red otherwise.
- The default launch size is 1280x960, matching the historical standalone
  viewer.

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
