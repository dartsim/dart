# Collision Heightmap Example

## Summary

- Goal: interactively sculpt a heightmap and observe contact alignment.
- Concepts/APIs: `dynamics::HeightmapShape`, ImGui viewer controls.
- Expected output: an ImGui-driven OSG viewer with a heightmap surface.
- Controls: use the ImGui panel; optional `--demo alignment` mode.

## Notes

- Use `--gui-scale` to scale the ImGui widgets.
- Use `--demo alignment` to compare heightmap alignment with a reference box.

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
