# Hubo Puppet Example

## Summary

- Goal: teleoperate the Hubo model with kinematic IK targets.
- Concepts/APIs: whole-body IK, interactive frames, support polygon visual.
- Expected output: a kinematic Hubo model you can reposition with input.
- Controls: mouse modifiers for drag modes; WASD/QE/FZ for movement.

## Notes

- This example runs in kinematic mode (no physics simulation).

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

Inside the source tree, `pixi run ex hubo_puppet` routes to
`examples/filament_gui --scene hubo-puppet` so the in-tree runner uses the
maintained Filament visual path. This standalone source remains comparison
material for the OSG teleoperation widget, support-polygon visual, and
keyboard controls.

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
