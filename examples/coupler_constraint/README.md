# Coupler Constraint Example

## Summary

- Goal: compare coupler constraints with mimic motor constraints on paired joints.
- Concepts/APIs: mimic joints, `constraint::CouplerConstraint`,
  `constraint::MimicMotorConstraint`, ImGui viewer.
- Expected output: a Filament viewer with two side-by-side mimic/coupler rigs.
- Controls: space toggles simulation; r resets both rigs.

## Notes

- Use `--gui-scale` to scale the ImGui widgets.

The recommended in-tree runner now opens the shared Filament viewer:

```bash
pixi run ex coupler_constraint
```

The standalone source in this directory remains legacy OSG/ImGui comparison
material for the status overlay and reset controls until the promoted
`dart::gui` API replaces the remaining viewer-specific example code.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

From the DART repository root:

```bash
pixi run ex coupler_constraint
```

Or launch the legacy standalone executable from the build directory above:

    $ ./{generated_executable} --gui-scale 1.0

Follow the instructions detailed in the console.
