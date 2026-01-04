# ImGui Viewer Example

## Summary

- Goal: embed ImGui widgets into a DART OSG viewer.
- Concepts/APIs: `gui::ImGuiViewer`, custom ImGui widgets.
- Expected output: an ImGui-driven viewer with toggles for world options.
- Controls: use the ImGui panel; `--gui-scale` adjusts widget sizing.

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
