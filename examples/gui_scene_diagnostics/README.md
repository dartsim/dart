# GUI Scene Diagnostics Example

## Summary

- Goal: inspect the renderer-independent GUI descriptors and exercise the
  built-in debug overlay through `ApplicationOptions::debugProvider`, without
  opening a window (the bounded run is headless by default).
- Concepts/APIs: `dart::gui::extractRenderables`, `extractDebugLines`,
  `makeSelectionDebugLines`, `ApplicationOptions::debugProvider` /
  `DebugScene`, `RunOptions`, `OrbitCamera`, picking helpers, and
  `SimpleFrame` visual descriptors.
- Expected output: descriptor counts, debug line counts, camera information,
  center-pick diagnostics, and per-frame debug-provider statistics printed to
  the console.
- Controls: the standard application CLI (`--frames`, `--width`, `--height`,
  `--screenshot`, ...); see `--help`.

This example is intended for API diagnostics. The maintained GUI application is
`dartsim`.

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
