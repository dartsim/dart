# Tinkertoy Builder Example

## Summary

- Goal: interactively assemble simple jointed structures.
- Concepts/APIs: `gui::InteractiveFrame`, ImGui widgets, mouse picking.
- Expected output: an ImGui-driven builder for adding joint blocks.
- Controls: 1/2/3 add joints; backspace/delete removes; `--gui-scale` adjusts UI.

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

Inside the source tree, `pixi run ex tinkertoy` routes to
`examples/filament_gui --scene tinkertoy` so the in-tree runner uses the
maintained Filament visual path. This standalone source remains comparison
material for the ImGui panel, mouse picking, and block-add/delete controls.

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
