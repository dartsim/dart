# WAM IKFast Example

## Summary

- Goal: interactively solve a WAM arm end-effector target using IKFast.
- Concepts/APIs: analytic IK, source-owned target frames, public `dart::gui`
  keyboard actions, and kinematic target manipulation.
- Expected output: a WAM arm above a blue ground plane. Press `1` to show the
  blue target handle, then move it to solve the active IK target.
- Controls:
  - `1`: toggle the end-effector target
  - `P`: print current joint values
  - `T`: reset the robot to its relaxed posture
  - Ctrl-left drag: translate the selected target handle
  - Ctrl+Shift-left drag: rotate the selected target handle
  - Arrow keys / PageUp / PageDown: nudge the selected target handle
  - X / Y / Z while Ctrl-dragging: constrain movement to one axis

## Notes

- This example runs as a kinematic target scene. The promoted `dart::gui` runner
  does not expose the old OSG `allowSimulation(false)` switch yet, so the world
  uses zero gravity and solves the active target from the pre-step callback.
- The old OSG Alt/Ctrl/Shift drag modes are superseded by the promoted
  selection tool. Exact parent-joint-only manipulation remains a public
  manipulation API gap.

## Run

From the repository root, use the in-tree runner:

```bash
pixi run ex wam_ikfast
```

The example defaults to a 1280x960 window. Shared GUI options such as
`--width`, `--height`, `--gui-scale`, `--screenshot`, and `--out` are supported:

```bash
pixi run ex wam_ikfast --headless --frames 5 --screenshot /tmp/wam_ikfast.ppm
pixi run ex wam_ikfast --headless --frames 5 --out /tmp/wam_ikfast_frames
```

## Build Instructions

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

From this directory:

```bash
mkdir build
cd build
cmake ..
make
```

## Execute Instructions

From the build directory above:

```bash
./{generated_executable}
```

Follow the instructions detailed in the console.
