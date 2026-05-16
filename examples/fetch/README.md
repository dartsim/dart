# Fetch MJCF Example

## Summary

- Goal: load an MJCF pick-and-place task and drive the Fetch end effector.
- Concepts/APIs: `dart::io::readWorld`, Bullet collision when available,
  `dart::gui::ApplicationOptions`, custom panels, draggable simple frames, and
  per-step callbacks.
- Expected output: a Fetch robot, object, work-area grid, and draggable
  transparent green target handle.
- Controls: select the green target handle, then Ctrl-left drag it or move it
  with the keyboard selection controls. Hold X/Y/Z while Ctrl-dragging to
  constrain motion to one world axis. Use u/j, i/k, and o/l to rotate the
  target about its local X/Y/Z axes, and r to reset the target pose.
  `--gui-scale` adjusts the GUI scale, and `--width`/`--height` override the
  default 1280x960 window size.
- Remaining gap: the historical mouse rotation rings need a public
  renderer-neutral manipulation API. This example restores orientation control
  through keyboard actions until that API exists.

## Run

From the source tree:

```bash
pixi run ex fetch
```

Headless capture is also supported through the promoted `dart::gui` runner:

```bash
pixi run ex fetch --headless --frames 2 --screenshot /tmp/fetch.ppm
```
