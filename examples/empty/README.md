# Empty Viewer Example

## Summary

- Goal: show a minimal source-owned `dart::gui` viewer scaffold with public
  frame selection, drag/nudge controls, panel controls, and keyboard actions.
- Concepts/APIs: `dart::gui::ApplicationOptions`, `SimpleFrame`,
  renderer-neutral panels, lifecycle/render callbacks, `KeyboardAction`, run
  defaults, and camera defaults.
- Expected output: a selectable anchor, draggable child frame, and axis markers
  in the Filament-backed `dart::gui` viewer.

## Run In Tree

From the repository root:

```bash
pixi run ex empty
```

Useful promoted runner options:

```bash
pixi run ex empty --headless --frames 2 --screenshot /tmp/empty.ppm
pixi run ex empty --width 960 --height 540
```

The example defaults to the historical 640x480 viewer and preserves
command-line overrides handled by the shared `dart::gui` runner.

## Controls

- The panel exposes pause/resume and single-step controls.
- Click a frame, then use Ctrl-left drag or arrow/PageUp/PageDown keys to move
  selected items through the promoted viewer controls.
- `q`, `Q`, Left, and Right print the historical keydown and key-release
  messages through
  renderer-neutral `KeyboardAction` callbacks.
- The panel reports pre-step, post-step, pre-render, and post-render callback
  counts through the promoted application callback hooks.

The example does not include backend event-handler types.
