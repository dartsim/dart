# Add/Delete Skeletons Example

## Summary

- Goal: spawn and remove skeletons at runtime via keyboard input.
- Concepts/APIs: `simulation::World`, event handlers, dynamic skeleton creation.
- Expected output: a viewer where you can add/remove cubes on demand.
- Controls: q spawns a cube; w deletes the most recent cube; space toggles sim.

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
