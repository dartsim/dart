# Hybrid Mixed Chain Example

## Summary

- Goal: simulate a chain that mixes rigid and soft bodies.
- Concepts/APIs: soft body nodes, `dart::io::readWorld`, external forces.
- Expected output: a mixed chain where you can push soft segments.
- Controls: q/w, e/r, t/y apply forces; space toggles simulation.

This project is dependent on DART with OSG support. Please make sure a proper
version of DART is installed with the gui component before building this
project.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

Launch the executable from the build directory above:

    $ ./hybrid_mixed_chain

Follow the instructions detailed in the console.
