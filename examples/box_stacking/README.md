# Box Stacking Example

## Summary

- Goal: stack boxes and explore solver choices and gravity settings.
- Concepts/APIs: `constraint::ConstraintSolver`, ImGui viewer controls.
- Expected output: an ImGui-driven OSG viewer with stacked boxes on a ground
  plane.
- Controls: use the ImGui panel; q/Q and arrow keys log test messages.

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

    $ ./{generated_executable}

Follow the instructions detailed in the console.
