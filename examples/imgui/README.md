# Panel Extension Example

## Summary

- Goal: demonstrate the promoted `dart::gui` panel-extension surface with a
  selectable target frame and source-owned controls.
- Concepts/APIs: `dart::gui::ApplicationOptions`, custom panels, keyboard
  actions, simple-frame selection/dragging, per-step callbacks, run defaults,
  and camera defaults.
- Expected output: an empty world with a yellow target handle and a
  `Tinkertoy Control` panel.
- Controls: select the yellow target, then Ctrl-left drag it or use the
  keyboard selection controls. Press q, Left, or Right to exercise the promoted
  keydown callbacks. `--gui-scale` adjusts the GUI scale, and `--width` /
  `--height` override the default 640x480 window size.
- Remaining gaps: headlight toggles, camera eye/center/up readouts,
  key-release callbacks, and pre/post render or post-step hooks need public
  renderer-neutral APIs.

## Run

From the source tree:

```bash
pixi run ex imgui
```

Headless capture is also supported through the promoted `dart::gui` runner:

```bash
pixi run ex imgui --headless --frames 2 --screenshot /tmp/panel_extension.ppm
```
