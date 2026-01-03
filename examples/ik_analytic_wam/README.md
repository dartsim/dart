# WAM Analytic IK Example

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
