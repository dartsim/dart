# GUI Scene Diagnostics Example

## Summary

- Goal: inspect the renderer-independent GUI descriptors without opening a
  window or adding another renderer.
- Concepts/APIs: `dart::gui::extractRenderables`, `extractDebugLines`,
  `makeSelectionDebugLines`, `RunOptions`, `OrbitCamera`, picking helpers, and
  `SimpleFrame` visual descriptors.
- Expected output: descriptor counts, debug/selection line counts, camera
  information, and center-pick diagnostics printed to the console.
- Controls: CLI flags for frame count and diagnostic viewport size.

This example is intended for API diagnostics. The maintained GUI application is
`apps/dartsim`.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

Launch the executable from the build directory above:

    $ ./{generated_executable}

Pass `--help` to see available options.
