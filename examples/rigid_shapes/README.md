# Rigid Shapes Example

## Summary

- Goal: spawn primitive shapes and observe collisions/contact points.
- Concepts/APIs: `dart::io::readWorld`, the built-in collision detector,
  viewer event handling.
- Expected output: an interactive OSG viewer with key bindings to spawn shapes.
- Controls: see the console instructions for spawn/toggle keys.

## Notes

- CLI options include `--collision-detector`, `--max-contacts`, and
  `--ground-thickness`. The detector option accepts `dart`; retained legacy
  aliases map to the built-in detector.

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
