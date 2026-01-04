# Rigid Hardcoded Design Example

## Summary

- Goal: build a skeleton in code without loading a model file.
- Concepts/APIs: `dynamics::Skeleton`, joint creation, manual transforms.
- Expected output: a simple multi-joint chain rendered in wireframe.
- Controls: 1/2/3 move joints; '-' flips direction; mouse navigates.

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

    $ ./rigid_hardcoded_design

Follow the instructions detailed in the console.
