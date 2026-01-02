# Empty Viewer Example

## Summary

- Goal: provide a minimal viewer scaffold with custom world hooks and input handling.
- Concepts/APIs: `gui::RealTimeWorldNode`, `gui::InteractiveFrame`, event handlers.
- Expected output: an OSG viewer with a draggable target frame and key events.
- Controls: press Q or arrow keys to see event callbacks in the console.

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
