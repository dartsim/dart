# Soft Bodies Example

## Summary

- Goal: demonstrate soft-body simulation with recorded playback.
- Concepts/APIs: `dart::io::readWorld`, custom `RealTimeWorldNode`, event handling.
- Expected output: soft bodies simulated in an OSG viewer with playback keys.
- Controls: use bracket keys to step through recorded frames (see console output).

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
