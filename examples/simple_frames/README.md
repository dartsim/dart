# Simple Frames Example

## Summary

- Goal: show how to build and render a hierarchy of `SimpleFrame` objects.
- Concepts/APIs: `dart::gui` application setup, `SimpleFrame`, shape
  attachments, frame transforms, marker ellipsoids, and line-segment markers.
- Expected output: three colored box frames, three marker ellipsoids, and a
  line-based arrow marker.
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
