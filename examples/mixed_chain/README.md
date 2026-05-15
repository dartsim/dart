# Mixed Chain Example

## Summary

- Goal: simulate a chain that mixes rigid and soft bodies.
- Concepts/APIs: soft body nodes, `dart::io::readWorld`, external forces.
- Expected output: a Filament viewer with mixed rigid and soft chain segments.
- Controls: q/w, e/r, t/y apply forces; space toggles simulation.

The recommended in-tree runner now opens the shared Filament viewer:

```bash
pixi run ex mixed_chain
```

The standalone source in this directory remains legacy OSG comparison material
for keyboard-applied external forces until the promoted `dart::gui` API
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
pixi run ex mixed_chain
```

Or launch the legacy standalone executable from the build directory above:

    $ ./mixed_chain

Follow the instructions detailed in the console.
