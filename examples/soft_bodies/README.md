# Soft Bodies Example

## Summary

- Goal: demonstrate soft-body simulation with recorded playback.
- Concepts/APIs: `dart::io::readWorld`, soft-body SKEL loading, and the
  experimental Filament GUI scene.
- Expected output: a Filament viewer with the soft-body SKEL fixture.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, click
  selects renderables, Ctrl-left drag or arrow/PageUp/PageDown nudges selected
  frames, and Escape exits.

## Notes

- The recommended in-tree runner uses the Filament scene in
  `examples/filament_gui`, which loads the existing `softBodies.skel` fixture
  and renders its soft meshes through backend-hidden DART descriptors.
- The standalone source in this directory remains as the legacy OSG version for
  recorded-playback comparison until the promoted Filament GUI API replaces the
  old viewer path.

## Run In Tree

From the repository root:

```bash
pixi run ex soft_bodies
```

This builds and runs `examples/filament_gui --scene soft-bodies`, so the
recommended visual path no longer depends on the legacy OSG viewer. On Linux
without a display, the runner automatically uses headless defaults.

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

This launches the legacy OSG viewer. Follow the instructions detailed in the
console to use the recorded-playback controls.
