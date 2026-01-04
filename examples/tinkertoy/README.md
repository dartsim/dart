# Tinkertoy Builder Example

## Summary

- Goal: interactively assemble simple jointed structures.
- Concepts/APIs: `gui::InteractiveFrame`, ImGui widgets, mouse picking.
- Expected output: an ImGui-driven builder for adding joint blocks.
- Controls: 1/2/3 add joints; backspace/delete removes; `--gui-scale` adjusts UI.

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
