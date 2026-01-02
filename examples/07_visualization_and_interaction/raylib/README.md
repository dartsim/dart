# Raylib Backend Example

## Summary

- Goal: exercise the experimental Raylib backend window.
- Concepts/APIs: Raylib integration, basic render loop.
- Expected output: a Raylib window showing DART and Raylib version strings.
- Controls: close the window to exit; `--frames` limits frame count.

## Notes

- Requires DART built with the Raylib GUI option.

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
