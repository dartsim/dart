# Drag and Drop Example

## Summary

- Goal: demonstrate promoted `dart::gui` selection and frame movement with a
  source-owned DART world.
- Concepts/APIs: `SimpleFrame`, public `dart::gui::Gizmo` transform
  affordances, renderer-neutral panels, camera defaults, and
  `dart::gui::ApplicationOptions`.
- Expected output: a public transform gizmo on the interactive frame, a child
  red box, and X/Y/Z marker boxes in the Filament viewer.
- Controls: click selects renderables, left-drag the public gizmo
  arrows/planes/rings to move or rotate the interactive frame,
  arrow/PageUp/PageDown keys nudge the selected frame, mouse wheel zooms,
  right/middle drag pans, and Escape exits.

## Run In Tree

From the repository root:

```bash
pixi run ex drag_and_drop
```

The default window size is 640x480. Command-line `--width`, `--height`,
`--headless`, `--screenshot <path>`, and `--out <dir>` options are handled by
the promoted `dart::gui` runner.

For a bounded headless capture:

```bash
pixi run ex drag_and_drop --headless --frames 2 --width 640 --height 480 --screenshot /tmp/dart_drag_and_drop.ppm
```

## Notes

- The historical OSG example used `gui::InteractiveFrame` and printed
  renderer-owned manipulation instructions.
- The promoted runner currently supports renderer-neutral click selection,
  public `dart::gui::Gizmo` axis-arrow, plane, and rotation-ring dragging, and
  keyboard nudging.
- The example does not include OSG or private backend hooks.
