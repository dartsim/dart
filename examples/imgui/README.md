# ImGui Viewer Example

## Summary

- Goal: inspect DART's current built-in viewer panel path.
- Concepts/APIs: the experimental Filament GUI scene and legacy
  `gui::ImGuiViewer` custom widgets.
- Expected output: a Filament viewer with the built-in status panel.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, Space
  pauses/resumes, `n` steps once while paused, click selects renderables,
  `--gui-scale` adjusts panel sizing, and Escape exits.

## Notes

- The recommended in-tree runner uses the Filament scene in
  `examples/filament_gui`, which renders the built-in status panel through the
  private Filament ImGui overlay.
- The standalone source in this directory remains as the legacy OSG/ImGui
  custom-widget example until the promoted Filament GUI API defines any needed
  DART-owned panel/tool extension points.

## Run In Tree

From the repository root:

```bash
pixi run ex imgui
```

This builds and runs `examples/filament_gui`, so the recommended visual path no
longer depends on the legacy OSG viewer. On Linux without a display, the runner
automatically uses headless defaults.

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
the console to use its custom widget controls.
