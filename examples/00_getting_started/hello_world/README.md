# Hello World Example

## Summary

- Goal: create a single rigid body and ground plane, then visualize the world.
- Concepts/APIs: `dynamics::Skeleton`, `simulation::World`, `gui::Viewer`.
- Expected output: a blue box falls onto a gray ground plane in an OSG viewer.
- Controls: press space to start the simulation.

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
