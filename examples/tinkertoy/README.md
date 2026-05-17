# Tinkertoy Builder Example

## Summary

- Goal: interactively assemble simple jointed structures and pull on selected
  blocks.
- Concepts/APIs: DART dynamics shapes, weld/revolute/ball joints,
  renderer-neutral selection hit data, public `dart::gui::Gizmo` target
  affordances, public `dart::gui` panels, keyboard actions, and headless
  capture.
- Expected output: the historical two-assembly tinkertoy scene with an ImGui
  control panel and a movable public transform gizmo force target.
- Controls: `1`/`2`/`3` attach weld, revolute, and ball-joint blocks;
  Backspace clears the selected block; Delete removes the selected block
  subtree while paused; Up/Down adjusts pulling strength; `` ` `` resets the
  target orientation; left-drag target gizmo arrows/planes/rings to move the
  force target; Tab resets the camera. Space toggles simulation and `n` steps
  once while paused.

Runtime Enter-key recording and viewer headlight toggles remain public
`dart::gui` API gaps. Use the promoted capture flags below for maintained image
sequence output.

## Run

From the repository root:

```bash
pixi run ex tinkertoy
```

The shared runner accepts GUI options such as `--gui-scale`:

```bash
pixi run ex tinkertoy --gui-scale 1.25
```

Headless capture is provided by the promoted `dart::gui` runner:

```bash
pixi run ex tinkertoy --headless --frames 2 --screenshot /tmp/tinkertoy.ppm
```

Image-sequence capture uses `--out`:

```bash
pixi run ex tinkertoy --headless --frames 3 --out /tmp/tinkertoy_frames
```

## Build Instructions

From this directory:

```bash
mkdir build
cd build
cmake ..
make
```

## Execute Instructions

Launch the executable from the build directory above:

```bash
./tinkertoy
```

The standalone executable uses the same promoted `dart::gui` viewer and the
same builder controls.
