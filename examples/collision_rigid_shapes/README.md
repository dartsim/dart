# Collision Rigid Shapes Example

## Summary

- Goal: spawn primitive shapes and observe collisions/contact points.
- Concepts/APIs: `dart::io::readWorld`, collision detectors, viewer event handling.
- Expected output: an interactive OSG viewer with key bindings to spawn shapes.
- Controls: see the console instructions for spawn/toggle keys.

## Notes

- CLI options include `--collision-detector`, `--max-contacts`, and
  `--ground-thickness`.

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
