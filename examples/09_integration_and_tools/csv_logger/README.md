# CSV Logger Example

## Summary

- Goal: export simulation state to a CSV file for external tooling.
- Concepts/APIs: `dart::io::readWorld`, `dart::simulation::World::step`.
- Expected output: a CSV log with time, COM, and body pose samples.
- Controls: CLI flags for world URI, output path, step count, and target names.

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
