# Hardcoded Design Example

## Summary

- Goal: build a three-link revolute skeleton directly in C++ without loading a
  model file.
- Concepts/APIs: `dart::dynamics::Skeleton`, `RevoluteJoint`, manual shape and
  body setup, `dart::gui::ApplicationOptions`, keyboard actions, and camera
  defaults.
- Expected output: a hand-built three-link skeleton with dark wireframe link
  edges displayed through the official `dart::gui` renderer.
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

The historical OpenSceneGraph example forced wireframe rendering. The promoted
example restores that appearance with source-owned DART line geometry attached
to each link instead of backend renderer state.
