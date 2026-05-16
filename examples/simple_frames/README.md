# Simple Frames Example

## Summary

- Goal: show how to build and render a hierarchy of `SimpleFrame` objects.
- Concepts/APIs: `dart::gui` application setup, `SimpleFrame`, shape
  attachments, frame transforms, marker ellipsoids, and `ArrowShape` markers.
- Expected output: three colored box frames, three marker ellipsoids, and a
  pink arrow marker.
- Controls: Space starts or pauses simulation. Click selection and keyboard
  nudging are provided by the promoted `dart::gui` viewer.

## Run

From the source tree:

```bash
pixi run ex simple_frames
```

Headless capture is provided by the promoted `dart::gui` runner:

```bash
pixi run ex simple_frames --headless --frames 2 --screenshot /tmp/simple_frames.ppm
```

## Build Instructions

From this directory:

```bash
mkdir build
cd build
cmake ..
make
```

## Execute Instructions

Launch the standalone executable from the build directory above:

```bash
./simple_frames
```

This example demonstrates how to create and visualize simple frames. It shows
boxes, ellipsoids, and arrows attached to different coordinate frames with
different transformations applied.

Key features demonstrated:

- Creating `SimpleFrame` objects with different transformations.
- Attaching shapes to frames.
- Frame hierarchy and relative transformations.
- Visualization through the promoted `dart::gui` renderer.
