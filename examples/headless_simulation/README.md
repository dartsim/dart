# Headless Simulation Example

## Summary

- Goal: run deterministic, headless simulation steps for batch workflows.
- Concepts/APIs: `dart::io::readWorld`, `dart::simulation::World::step`.
- Expected output: timing and simulation progress printed to the console.
- Controls: CLI flags for world URI, step count, time step, and random seed.

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

Pass `--help` to see available options.
