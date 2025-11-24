# Rigid Loop Example

This example demonstrates a closed-loop chain simulation using DART with OSG for visualization.

## Description

This example shows:

- Loading a chain skeleton from a SKEL file
- Creating ball joint constraints to form a closed loop
- Applying damping forces during simulation
- Using the OSG viewer for 3D visualization

The red-colored body nodes are connected by a ball joint constraint to close the kinematic loop, creating interesting dynamics as the chain moves under gravity and damping forces.

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

- **Space bar**: Play/pause simulation
- **ESC**: Exit application
- **Mouse**: Rotate, zoom, and pan the camera view
