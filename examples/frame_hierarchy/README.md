# Frame Hierarchy Example

## Summary

- Goal: show how to build and render a hierarchy of `SimpleFrame` objects.
- Concepts/APIs: `SimpleFrame`, shape attachments, frame transforms.
- Expected output: an OSG viewer with boxes, ellipsoids, and an arrow shape.
- Controls: none.

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

This example demonstrates how to create and visualize a frame hierarchy using
the OSG renderer in DART. It shows various geometric shapes (boxes, ellipsoids,
arrows)
attached to different coordinate frames with different transformations applied.

Key features demonstrated:

- Creating SimpleFrame objects with different transformations
- Attaching shapes to frames
- Frame hierarchy and relative transformations
- OSG-based visualization
