# Drag and Drop Example

## Summary

- Goal: demonstrate drag-and-drop interaction with frames and shapes.
- Concepts/APIs: `gui::InteractiveFrame`, `SimpleFrame`, and the experimental
  Filament GUI scene.
- Expected output: a selectable frame and box with axis markers in the
  Filament viewer.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, click
  selects renderables, Ctrl-left drag or arrow/PageUp/PageDown nudges selected
  items, and Escape exits.

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

## Notes

- The recommended in-tree runner uses the Filament scene in
  `examples/filament_gui`, which carries this example's frame, child box, and
  axis-marker layout through the backend-hidden manipulation path.
- The standalone source in this directory remains as the legacy OSG version for
  comparison until the promoted Filament GUI API replaces the old viewer path.

## Run In Tree

From the repository root:

```bash
pixi run ex drag_and_drop
```

This builds and runs `examples/filament_gui --scene drag-and-drop`, so it uses
the same Filament renderer path as the other experimental GUI fixtures. On
Linux without a display, the runner automatically uses headless defaults.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

Launch the executable from the build directory above:

    $ ./{generated_executable}

Follow the instructions detailed in the console.
