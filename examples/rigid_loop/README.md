# Rigid Loop Example

## Summary

- Goal: form a closed-loop chain using a ball joint constraint.
- Concepts/APIs: `constraint::BallJointConstraint`, `dart::io::readWorld`.
- Expected output: an OSG viewer with a chain where two red links close the loop.
- Controls: space toggles simulation; ESC exits.

## Details

The red body nodes are connected by a ball joint constraint to close the
kinematic loop while the chain moves under gravity and damping.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

Launch the executable from the build directory above:

    $ ./rigid_loop

## Controls

- Space: play/pause simulation.
- ESC: exit application.
- Mouse: rotate, zoom, and pan the camera view.
