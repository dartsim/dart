# Hello World Example

## Summary

- Goal: create a single rigid body and ground plane, then visualize the world.
- Concepts/APIs: `dynamics::Skeleton`, `simulation::World`, and the
  experimental Filament GUI scene.
- Expected output: a blue box falls onto a gray ground plane in a Filament
  viewer.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, Space
  pauses/resumes, `n` steps once while paused, and Escape exits.

## Notes

- The recommended in-tree runner uses the Filament scene in
  `examples/filament_gui`, which carries this example's single dynamic box and
  ground plane through backend-hidden DART descriptors.
- The standalone source in this directory remains as the legacy OSG version for
  comparison until the promoted Filament GUI API replaces the old viewer path.

## Run In Tree

From the repository root:

```bash
pixi run ex hello_world
```

This builds and runs `examples/filament_gui --scene hello-world`, so the
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

Follow the instructions detailed in the console.
