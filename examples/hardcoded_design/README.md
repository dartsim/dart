# Hardcoded Design Example

## Summary

- Goal: build a three-link revolute skeleton directly in C++ without loading a
  model file.
- Concepts/APIs: `dart::dynamics::Skeleton`, `RevoluteJoint`, manual shape and
  body setup, `dart::gui::ApplicationOptions`, keyboard actions, and camera
  defaults.
- Expected output: a hand-built three-link skeleton displayed through the
  official `dart::gui` renderer.
- Controls: `1`, `2`, and `3` increment the LHY, LHR, and LHP degrees of
  freedom. `-` toggles the increment direction.

## Run

From the source tree:

```bash
pixi run ex hardcoded_design
```

Headless capture is also supported:

```bash
pixi run ex hardcoded_design --headless --frames 2 --screenshot /tmp/hardcoded_design.ppm
```

The historical OpenSceneGraph example also forced wireframe rendering. That
render style still needs a DART-owned public render-style or debug-mode API
before it can be restored without exposing backend details.
