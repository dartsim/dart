# Capsule Ground Contact Example

## Summary

- Goal: observe capsule-plane contact behavior with persistent manifolds.
- Concepts/APIs: `collision::OdeCollisionDetector`, `dynamics::CapsuleShape`,
  and the experimental Filament GUI scene.
- Expected output: a Filament viewer with a capsule resting on a visible
  ground plane.
- Controls: left drag orbits, right/middle drag pans, wheel zooms, click
  selects renderables, Ctrl-left drag or arrow/PageUp/PageDown nudges selected
  bodies, and Escape exits.

## Notes

- The recommended in-tree runner uses the Filament scene in
  `examples/filament_gui`, which renders the capsule and ground through
  backend-hidden DART descriptors and uses ODE collision when available.
- The standalone source in this directory remains as the legacy OSG version for
  pose reset and persistent-manifold controls until the promoted Filament GUI
  API replaces the old viewer path.

## Run In Tree

From the repository root:

```bash
pixi run ex capsule_ground_contact
```

This builds and runs `examples/filament_gui --scene capsule-ground-contact`, so
the recommended visual path no longer depends on the legacy OSG viewer. On
Linux without a display, the runner automatically uses headless defaults.

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
console to use the pose reset controls.
