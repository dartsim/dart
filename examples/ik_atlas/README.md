# Atlas IK Example

## Summary

- Goal: teleoperate the Atlas model with inverse kinematics targets.
- Concepts/APIs: whole-body IK, interactive frames, support polygon visual.
- Expected output: a kinematic Atlas model you can reposition with input.
- Controls: mouse modifiers for drag modes; WASD/QE/FZ for movement.

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
