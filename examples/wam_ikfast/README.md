# WAM IKFast Example

## Summary

- Goal: interactively solve IK targets for a WAM arm using IKFast.
- Concepts/APIs: analytic IK, drag-and-drop targets, kinematic mode.
- Expected output: a WAM arm that follows interactive targets.
- Controls: Alt/Ctrl/Shift click for drag modes; 1 toggles target; P prints
  joints; T resets posture.

## Notes

- This example runs in kinematic mode (no physics simulation).

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

Inside the source tree, `pixi run ex wam_ikfast` routes to
`examples/filament_gui --scene wam-ikfast` so the in-tree runner uses the
maintained Filament visual path. This standalone source remains comparison
material for the IKFast solver, drag modes, keyboard shortcuts, and posture
reset workflow.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

Launch the executable from the build directory above:

    $ ./{generated_executable}

Follow the instructions detailed in the console.
