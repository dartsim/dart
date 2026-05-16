# Operational Space Control Example

## Summary

- Goal: control the KR5 end effector by moving a red operational-space target.
- Concepts/APIs: URDF loading, Jacobian-based operational-space control,
  per-step callbacks, selectable `SimpleFrame` targets, renderer-neutral drag
  and nudge controls, and public `dart::gui` panels.
- Expected output: the KR5 robot over the KR5 ground with a red target ball near
  the end effector.
- Controls: click the red target ball, then Ctrl-left drag it or use
  arrow/PageUp/PageDown keys to move it. Hold `1`, `2`, or `3` while
  Ctrl-dragging to constrain movement to X, Y, or Z; `X`, `Y`, and `Z` are also
  supported by the promoted viewer. Space toggles simulation.

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

- The historical OSG shadow toggle is tracked as a public renderer-neutral
  shadow-control API gap.
- The historical camera home used a custom up vector; the promoted camera
  restores the same eye/target framing, while exact roll/up-vector control is a
  public camera API gap.
