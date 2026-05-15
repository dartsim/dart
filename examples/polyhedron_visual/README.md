# Polyhedron Visual Example

## Summary

- Goal: render a convex polyhedron from a vertex list.
- Concepts/APIs: `gui::PolyhedronVisual`, `gui::GridVisual`, and the
  experimental Filament GUI scene.
- Expected output: a translucent convex hull and wireframe displayed in the
  Filament viewer.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, and Escape
  exits.

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

## Notes

- The recommended in-tree runner uses the Filament scene in
  `examples/filament_gui`, which carries this example's convex hull and
  wireframe through DART-owned shape descriptors.
- The standalone source in this directory remains as the legacy OSG version for
  comparison until the promoted Filament GUI API replaces the old viewer path.

## Run In Tree

From the repository root:

```bash
pixi run ex polyhedron_visual
```

This builds and runs `examples/filament_gui --scene polyhedron`, so it uses the
same Filament renderer path as the other experimental GUI fixtures. On Linux
without a display, the runner automatically uses headless defaults.

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
