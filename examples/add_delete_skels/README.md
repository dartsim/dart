# Add/Delete Skeletons Example

## Summary

- Goal: spawn and remove skeletons at runtime via keyboard input.
- Concepts/APIs: `simulation::World`, event handlers, dynamic skeleton creation.
- Expected output: a Filament viewer with a deterministic set of cube skeletons.
- Controls: q spawns a cube; w deletes the most recent cube; space toggles sim.

The recommended in-tree runner now opens the shared Filament viewer:

```bash
pixi run ex add_delete_skels
```

The standalone source in this directory remains legacy OSG comparison material
for the live q/w add-delete controls until the promoted `dart::gui` API
replaces the remaining viewer-specific example code.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

From the DART repository root:

```bash
pixi run ex add_delete_skels
```

Or launch the legacy standalone executable from the build directory above:

    $ ./{generated_executable}

Follow the instructions detailed in the console.
