# Boxes Example

## Summary

- Goal: spawn a grid of rigid boxes over a ground plane and visualize them.
- Concepts/APIs: `dynamics::Skeleton`, `simulation::World`,
  `collision::CollisionDetector`, `gui::Viewer`.
- Expected output: an OSG viewer with colored boxes falling onto a gray ground.
- Controls: press space to toggle simulation.

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
