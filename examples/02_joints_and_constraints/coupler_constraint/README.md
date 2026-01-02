# Coupler Constraint Example

## Summary

- Goal: compare coupler constraints with mimic motor constraints on paired joints.
- Concepts/APIs: mimic joints, `constraint::CouplerConstraint`,
  `constraint::MimicMotorConstraint`, ImGui viewer.
- Expected output: two side-by-side rigs with an ImGui overlay showing errors
  and limit engagement.
- Controls: space toggles simulation; r resets both rigs.

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
