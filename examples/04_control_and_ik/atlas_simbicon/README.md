# Atlas Simbicon Example

## Summary

- Goal: run a Simbicon-style controller on the Atlas model.
- Concepts/APIs: gait controller, ImGui viewer controls.
- Expected output: an Atlas model walking in an ImGui-driven OSG viewer.
- Controls: use the ImGui panel; `--gui-scale` adjusts widget scaling.

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
