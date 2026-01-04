# Joint Chain Example

## Summary

- Goal: simulate a multi-link chain with per-joint damping.
- Concepts/APIs: `dart::io::readWorld`, `gui::RealTimeWorldNode`, joint damping.
- Expected output: an OSG viewer with a falling chain that settles under damping.
- Controls: space toggles simulation.

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
