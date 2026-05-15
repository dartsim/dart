# Rigid Shapes Example

## Summary

- Goal: inspect broad primitive and mesh visual coverage in the maintained
  Filament path.
- Concepts/APIs: Filament GUI shape descriptors, `dart::io::readWorld`,
  collision detectors, and legacy viewer event handling.
- Expected output: a Filament viewer with primitive, mesh, point-cloud,
  heightmap, soft-mesh, and robot visual fixtures.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, Space
  pauses/resumes, `n` steps once while paused, click selects renderables,
  Ctrl-left drag or arrow/PageUp/PageDown keys move selectable items, and
  Escape exits.

## Notes

- CLI options include `--collision-detector`, `--max-contacts`, and
  `--ground-thickness` when launching the standalone legacy executable.
- The recommended in-tree runner uses the default MVP scene in
  `examples/filament_gui`, which keeps renderer setup, resources, input,
  overlays, capture, and fixture logic encapsulated by the DART GUI
  implementation boundary.
- The standalone source in this directory remains as the legacy OSG comparison
  example for shape spawning, contact toggles, and collision-detector controls.

## Run In Tree

From the repository root:

```bash
pixi run ex rigid_shapes
```

This builds and runs `examples/filament_gui --scene mvp`, so the recommended
visual path no longer depends on the legacy OSG viewer. On Linux without a
display, the runner automatically uses headless defaults.

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
console for its shape spawning and contact-toggle controls.
