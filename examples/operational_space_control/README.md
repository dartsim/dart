# Operational Space Control Example

## Summary

- Goal: control the KR5 end effector by moving an operational-space target
  gizmo.
- Concepts/APIs: URDF loading, Jacobian-based operational-space control,
  per-step callbacks, `SimpleFrame` targets, public `dart::gui::Gizmo`
  affordances, renderer-neutral drag and nudge controls, and public
  `dart::gui` panels.
- Expected output: the KR5 robot over the KR5 ground with a public transform
  gizmo near the end effector.
- Controls: left-drag the target gizmo arrows/planes/rings to move the
  operational-space target. Press `1` to select the target for
  arrow/PageUp/PageDown nudges. Press `s` or `S` to toggle shadows. Space
  toggles simulation.

## Run

From the source tree:

```bash
pixi run ex operational_space_control
```

Headless capture is provided by the promoted `dart::gui` runner:

```bash
pixi run ex operational_space_control --headless --frames 2 --screenshot /tmp/operational_space_control.ppm
```

Image-sequence capture uses `--out`:

```bash
pixi run ex operational_space_control --headless --frames 3 --out /tmp/operational_space_control_frames
```

## Notes

- The historical camera home, custom up vector, and shadow toggle are restored
  through public `dart::gui` APIs.
