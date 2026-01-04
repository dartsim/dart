# Control Actuator Modes Example

## Summary

- Goal: mix passive, velocity-controlled, and locked joints in one model.
- Concepts/APIs: joint actuator types, scripted joint motion, harness toggle.
- Expected output: a humanoid with scripted arm/leg motion.
- Controls: h toggles harness; space toggles simulation.

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
