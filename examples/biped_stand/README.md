# Biped Stand Example

## Summary

- Goal: keep a biped standing with SPD control while applying perturbations.
- Concepts/APIs: SPD tracking, custom `RealTimeWorldNode`, external forces.
- Expected output: a biped that tries to balance under scripted pushes.
- Controls: space starts simulation; 1-4 apply directional pushes.

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
