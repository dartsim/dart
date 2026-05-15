# Vehicle Example

## Summary

- Goal: drive a simple vehicle model with steering and wheel velocity control.
- Concepts/APIs: PD control, joint targets, keyboard input handling.
- Expected output: a Filament-rendered car model with four wheels, ground, and
  obstacle boxes.
- Controls: the recommended in-tree runner is visual-only for now. The
  standalone legacy source keeps the original w/s/x throttle, a/d steering, and
  space simulation toggle controls as comparison material until those controls
  are promoted into `dart::gui`.

This project is dependent on DART. Please make sure a proper version of DART is
installed before building this project.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

From the repository root, prefer the in-tree Filament runner:

    $ pixi run ex vehicle

The legacy standalone executable can still be launched from the build directory
above:

    $ ./{generated_executable}

Follow the instructions detailed in the console.
