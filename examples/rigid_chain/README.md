# Rigid Chain Example

## Summary

- Goal: simulate a multi-link chain with per-joint damping.
- Concepts/APIs: `dart::io::readWorld`, Filament GUI scene descriptors,
  `gui::RealTimeWorldNode`, and joint damping.
- Expected output: a Filament viewer with the SKEL-loaded rigid chain.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, Space
  pauses/resumes, `n` steps once while paused, click selects renderables,
  Ctrl-left drag or arrow/PageUp/PageDown keys move selectable items, and
  Escape exits.

## Notes

- The recommended in-tree runner uses the Filament `--scene rigid-chain`
  fixture in `examples/filament_gui`, which keeps viewer setup, resources,
  capture, input, and scene synchronization behind the DART GUI implementation
  boundary.
- The standalone source in this directory remains as the legacy OSG comparison
  scaffold for the custom per-step damping hook.

## Run In Tree

From the repository root:

```bash
pixi run ex rigid_chain
```

This builds and runs `examples/filament_gui --scene rigid-chain`, so the
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
console for its custom damping behavior.
