# Fetch MJCF Example

## Summary

- Goal: load an MJCF pick-and-place task and drive the Fetch end effector.
- Concepts/APIs: `dart::io::readWorld`, Bullet collision when available,
  `dart::gui::ApplicationOptions`, public `dart::gui::Gizmo` target
  affordances, custom panels, and per-step callbacks.
- Expected output: a Fetch robot, object, work-area grid, a target at the cross
  of two transparent green bars with public transform gizmo handles, plus a
  restored Fetch control panel with menu and collapsible help/simulation
  sections.
- Controls: left-drag the target gizmo arrows/planes/rings at the green cross
  to move or rotate the target. Arrow/PageUp/PageDown keys nudge the selected
  target. Use u/j, i/k, and o/l to rotate the target about its local X/Y/Z
  axes, and r to reset the target pose.
  `--gui-scale` adjusts the GUI scale, and `--width`/`--height` override the
  default 1280x960 window size.
- Mouse manipulation uses the public target gizmo while the transparent green
  bars preserve the historical Fetch target cue.

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
pixi run ex fetch --headless --frames 2 --out /tmp/fetch_frames
```
