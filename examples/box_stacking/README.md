# Box Stacking Example

## Summary

- Goal: inspect dynamic box stacking in the maintained Filament path.
- Concepts/APIs: Filament GUI dynamic body descriptors,
  `constraint::ConstraintSolver`, and legacy ImGui viewer controls.
- Expected output: a Filament viewer with colored boxes falling onto a ground
  plane.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, Space
  pauses/resumes, `n` steps once while paused, click selects renderables,
  Ctrl-left drag or arrow/PageUp/PageDown keys move selectable items, and
  Escape exits.

## Notes

- Use `--gui-scale` to scale the ImGui widgets.
- The recommended in-tree runner uses the Filament `--scene boxes` fixture in
  `examples/filament_gui`, which keeps renderer setup, resources, input,
  overlays, capture, and dynamic body visualization behind the DART GUI
  implementation boundary.
- The standalone source in this directory remains as the legacy OSG/ImGui
  comparison example for solver selection, gravity controls, and custom key
  callbacks.

## Run In Tree

From the repository root:

```bash
pixi run ex box_stacking
```

This builds and runs `examples/filament_gui --scene boxes`, so the recommended
visual path no longer depends on the legacy OSG viewer. On Linux without a
display, the runner automatically uses headless defaults.

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

Launch the executable from the build directory above:

    $ ./{generated_executable}

This launches the legacy OSG/ImGui viewer. Follow the instructions detailed in
the console and panel for its solver and gravity controls.
