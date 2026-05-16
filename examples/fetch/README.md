# Fetch MJCF Example

## Summary

- Goal: load an MJCF pick-and-place task and drive the Fetch end effector.
- Concepts/APIs: `dart::io::readWorld`, Bullet collision when available,
  `dart::gui::ApplicationOptions`, custom panels, draggable simple frames, and
  per-step callbacks.
- Expected output: a Fetch robot, object, work-area grid, and draggable
  transparent green target bars.
- Controls: select the green target bars, then Ctrl-left drag them or move
  them with the
  keyboard selection controls. `--gui-scale` adjusts the GUI scale, and
  `--width`/`--height` override the default 1280x960 window size.

## Run

From the source tree:

```bash
pixi run ex fetch
```

Headless capture is also supported through the promoted `dart::gui` runner:

```bash
pixi run ex fetch --headless --frames 2 --screenshot /tmp/fetch.ppm
```
