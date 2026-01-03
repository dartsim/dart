# IK Humanoid Example

## Summary

- Goal: demonstrate whole-body inverse kinematics on humanoid models.
- Concepts/APIs: `InverseKinematics`, interactive frames, support polygons.
- Expected output: a kinematic humanoid you can reposition with input.
- Controls: mouse modifiers for drag modes; optional keyboard toggles per model.

## Notes

- Default robot: Atlas (`--robot atlas`).
- Supported robots: `atlas`, `g1`, `hubo`.
- G1 loads a remote package by default; use `--package-uri` and `--robot-uri`
  to override.
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

    $ ./{generated_executable} --robot atlas

Pass `--help` to see available options.
