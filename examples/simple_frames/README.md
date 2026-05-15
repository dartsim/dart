# Simple Frames Example

## Summary

- Goal: show how to build and render a hierarchy of `SimpleFrame` objects.
- Concepts/APIs: `SimpleFrame`, shape attachments, frame transforms, and the
  experimental Filament GUI scene.
- Expected output: a Filament viewer with boxes, ellipsoids, and an arrow
  marker.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, click
  selects renderables, Ctrl-left drag or arrow/PageUp/PageDown nudges selected
  frames, and Escape exits.

## Notes

- The recommended in-tree runner uses the Filament scene in
  `examples/filament_gui`, which carries this example's frame hierarchy,
  marker ellipsoids, and arrow marker through backend-hidden DART descriptors.
- The standalone source in this directory remains as the legacy OSG version for
  comparison until the promoted Filament GUI API replaces the old viewer path.

## Run In Tree

From the repository root:

```bash
pixi run ex simple_frames
```

This builds and runs `examples/filament_gui --scene simple-frames`, so the
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

This example demonstrates how to create and visualize simple frames using the
legacy OSG renderer in DART. It shows various geometric shapes (boxes,
ellipsoids, arrows) attached to different coordinate frames with different
transformations applied.

Key features demonstrated:

- Creating SimpleFrame objects with different transformations
- Attaching shapes to frames
- Frame hierarchy and relative transformations
- OSG-based visualization
