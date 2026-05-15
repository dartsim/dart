# Mimic Pendulums Example

## Summary

- Goal: visualize mimic joint behavior across multiple pendulum rigs.
- Concepts/APIs: mimic joints, constraint solver selection, ImGui viewer.
- Expected output: `pixi run ex mimic_pendulums` opens the experimental
  Filament view with the SDF-loaded pendulum rigs and ground. The standalone
  legacy source shows an OSG viewer with an ImGui overlay that reports mimic
  errors and base drift.
- Controls: in the legacy standalone source, space toggles simulation and
  other settings live in the ImGui panel.

## In-Tree Runner

Inside the DART checkout:

```bash
pixi run ex mimic_pendulums
```

The in-tree runner builds the experimental Filament GUI and launches the
`--scene mimic-pendulums` fixture. The source in this directory remains legacy
OSG comparison material for the solver controls and debug table until the
promoted `dart::gui` API has panel/tool support.

## Notes

- Use `--gui-scale` to scale the ImGui widgets.

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

    $ ./{generated_executable} --gui-scale 1.0

Follow the instructions detailed in the console.
