# Hardcoded Design Example

## Summary

- Goal: build a skeleton in code without loading a model file.
- Concepts/APIs: `dynamics::Skeleton`, joint creation, manual transforms, and
  Filament GUI scene descriptors.
- Expected output: a Filament viewer with a simple three-link hand-built
  skeleton.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, Space
  pauses/resumes, `n` steps once while paused, click selects renderables,
  Ctrl-left drag or arrow/PageUp/PageDown keys move selectable items, and
  Escape exits.

## Notes

- The recommended in-tree runner uses the Filament `--scene hardcoded-design`
  fixture in `examples/filament_gui`, which keeps viewer setup, resources,
  capture, input, and scene synchronization behind the DART GUI implementation
  boundary.
- The standalone source in this directory remains as the legacy OSG comparison
  scaffold for wireframe rendering and direct key-controlled joint motion.

## Run In Tree

From the repository root:

```bash
pixi run ex hardcoded_design
```

This builds and runs `examples/filament_gui --scene hardcoded-design`, so the
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

    $ ./hardcoded_design

This launches the legacy OSG viewer. Follow the instructions detailed in the
console for its wireframe and key-controlled joint callbacks.
