# Simulation Event Handler Example

## Summary

- Goal: provide a reusable event handler for simulation control and forces.
- Concepts/APIs: `gui::Viewer` input handling, force/torque visualization.
- Expected output: a viewer with controllable rigid bodies and force arrows.
- Controls: space toggles sim; S steps; R resets; Tab cycles bodies; arrows/QWE
  apply forces/torques; V toggles force arrows.

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
