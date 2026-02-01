# Raylib Backend Example

## Summary

- Goal: exercise the experimental Raylib backend with physics scene.
- Concepts/APIs: Raylib integration, SceneViewer, orbit camera, picking.
- Expected output: a Raylib window showing falling shapes on a ground plane.
- Controls:
  - **Right-drag**: orbit camera
  - **Middle-drag**: pan camera
  - **Scroll**: zoom in/out
  - **Left-click**: select object (yellow wireframe highlight)
  - **Escape**: deselect
  - **Space**: pause/unpause simulation
  - **Enter**: single-step (while paused)
  - `--frames N`: run for N frames then exit

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
