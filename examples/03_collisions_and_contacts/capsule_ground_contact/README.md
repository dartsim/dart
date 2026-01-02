# Capsule Ground Contact Example

## Summary

- Goal: observe capsule-plane contact behavior with persistent manifolds.
- Concepts/APIs: `collision::OdeCollisionDetector`, `dynamics::CapsuleShape`.
- Expected output: a capsule resting on an infinite plane with a visible ground.
- Controls: h/v reset pose; space clears velocities.

## Notes

- Requires DART built with ODE collision (`DART_BUILD_COLLISION_ODE=ON`).

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
