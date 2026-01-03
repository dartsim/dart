# Mimic Pendulums Example

## Summary

- Goal: visualize mimic joint behavior across multiple pendulum rigs.
- Concepts/APIs: mimic joints, constraint solver selection, ImGui viewer.
- Expected output: an OSG viewer with pendulum rigs and an ImGui overlay that
  reports mimic errors and base drift.
- Controls: space toggles simulation; other settings live in the ImGui panel.

## Notes

- Use `--gui-scale` to scale the ImGui widgets.

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

    $ ./{generated_executable} --gui-scale 1.0

Follow the instructions detailed in the console.
