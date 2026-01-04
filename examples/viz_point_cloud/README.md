# Point Cloud Visualization Example

## Summary

- Goal: sample points from a robot mesh and visualize a voxel grid.
- Concepts/APIs: `PointCloudShape`, `VoxelGridShape`, ImGui controls.
- Expected output: an ImGui viewer with a point cloud and voxel occupancy.
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
