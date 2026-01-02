# Operational Space Control Example

## Summary

- Goal: drive an end effector toward a draggable target in task space.
- Concepts/APIs: operational-space control, Jacobians, drag-and-drop targets.
- Expected output: a robot arm tracking a red target sphere.
- Controls: drag the red ball; hold 1/2/3 to constrain motion axes.

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
