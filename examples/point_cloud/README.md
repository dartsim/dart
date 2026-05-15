# Point Cloud Example

## Summary

- Goal: sample points from a robot mesh and visualize a voxel grid.
- Concepts/APIs: `PointCloudShape`, `VoxelGridShape`, and the experimental
  Filament GUI scene.
- Expected output: a Filament viewer with a colored point cloud and voxel
  occupancy fixture.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, click
  selects renderables, Ctrl-left drag or arrow/PageUp/PageDown nudges selected
  frames, and Escape exits.

## Notes

- The recommended in-tree runner uses the Filament scene in
  `examples/filament_gui`, which renders a point-cloud and voxel-grid fixture
  through backend-hidden DART descriptors.
- The standalone source in this directory remains as the legacy OSG/ImGui
  version for robot-mesh sampling controls until the promoted Filament GUI API
  replaces the old viewer path.

## Run In Tree

From the repository root:

```bash
pixi run ex point_cloud
```

This builds and runs `examples/filament_gui --scene point-cloud`, so the
recommended visual path no longer depends on the legacy OSG viewer. On Linux
without a display, the runner automatically uses headless defaults.

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

This launches the legacy OSG/ImGui viewer. Follow the instructions detailed in
the console and panel to use the robot-sampling controls.
