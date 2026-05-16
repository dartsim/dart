# Fetch MJCF Example

## Summary

- Goal: load an MJCF pick-and-place task and drive the Fetch end effector.
- Concepts/APIs: `dart::io::readWorld`, Bullet collision when available,
  `dart::gui::ApplicationOptions`, custom panels, draggable simple frames, and
  per-step callbacks.
- Expected output: a Fetch robot, object, work-area grid, and draggable
  transparent green target handle with local-axis arrowheads, planar translation
  guides, rotation rings, and a restored Fetch control panel with menu and
  collapsible help/simulation sections.
- Controls: select the green target handle, then Ctrl-left drag it or move it
  with the keyboard selection controls. Hold X/Y/Z while Ctrl-dragging to
  constrain motion to one world axis. Ctrl-Shift-left drag rotates the selected
  target; hold X/Y/Z while rotating to use a local target axis. Use u/j, i/k,
  and o/l to rotate the target about its local X/Y/Z axes, and r to reset the
  target pose.
  `--gui-scale` adjusts the GUI scale, and `--width`/`--height` override the
  default 1280x960 window size.
- Mouse manipulation uses the promoted renderer-neutral selection API for
  translation and rotation of the target frame.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

From the source tree:

```bash
pixi run ex fetch
```

From a standalone build directory:

```bash
./fetch
```

Headless capture is also supported through the promoted `dart::gui` runner:

```bash
pixi run ex fetch --headless --frames 2 --screenshot /tmp/fetch.ppm
```
