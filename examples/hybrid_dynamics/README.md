# Hybrid Dynamics Example

## Summary

- Goal: mix passive, velocity-controlled, and locked joints in one model.
- Concepts/APIs: joint actuator types, scripted joint motion, harness toggle.
- Expected output: a Filament-rendered posed humanoid and ground plane.
- Controls: the recommended in-tree runner is visual-only for now. The
  standalone legacy source keeps the original h harness toggle, scripted
  arm/leg motion, and space simulation toggle as comparison material until
  those controls are promoted into `dart::gui`.

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

    $ pixi run ex hybrid_dynamics

The legacy standalone executable can still be launched from the build directory
above:

    $ ./{generated_executable}

Follow the instructions detailed in the console.
