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

This example demonstrates how to create and visualize simple frames using the OSG
renderer in DART. It shows various geometric shapes (boxes, ellipsoids, arrows)
attached to different coordinate frames with different transformations applied.

Key features demonstrated:
- Creating SimpleFrame objects with different transformations
- Attaching shapes to frames
- Frame hierarchy and relative transformations
- OSG-based visualization
