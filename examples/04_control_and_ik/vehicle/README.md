# Vehicle Example

## Summary

- Goal: drive a simple vehicle model with steering and wheel velocity control.
- Concepts/APIs: PD control, joint targets, keyboard input handling.
- Expected output: a car model that responds to steering and throttle keys.
- Controls: w/s/x move; a/d steer; space toggles simulation.

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
