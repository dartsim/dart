# Panel Extension Example

## Summary

- Goal: demonstrate the promoted `dart::gui` panel-extension surface with a
  selectable target frame and source-owned controls.
- Concepts/APIs: `dart::gui::ApplicationOptions`, custom panels, keyboard
  actions, public `dart::gui::Gizmo` target affordances, per-step callbacks,
  run defaults, and camera defaults.
- Expected output: an empty world with a target transform gizmo and a
  `Tinkertoy Control` panel.
- Controls: left-drag the target gizmo arrows/planes/rings. Press and release q,
  Q, Left, or Right to exercise the promoted keyboard callbacks. `--gui-scale`
  adjusts the GUI scale, and `--width` / `--height` override the default 640x480
  window size.
- Remaining gaps: pre/post render or post-step hooks need public
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
