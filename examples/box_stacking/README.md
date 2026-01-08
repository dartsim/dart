# Box Stacking Example

## Summary

- Goal: stack boxes and explore solver choices and gravity settings.
- Concepts/APIs: `constraint::ConstraintSolver`, ImGui viewer controls, `ContactManifoldCache`.
- Expected output: an ImGui-driven OSG viewer with two stacked boxes side-by-side.
  - Left stack: Legacy collision handling (may jitter).
  - Right stack: Persistent contact manifold (more stable).
- Controls: use the ImGui panel to toggle manifolds, adjust stack height, and reset.

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
