# Atlas Puppet Example

## Summary

- Goal: teleoperate the Atlas model with kinematic IK targets.
- Concepts/APIs: whole-body IK, interactive frames, support polygon visual.
- Expected output: by default, the shared Filament viewer displays Atlas with
  selectable hand and foot IK target frames. The legacy standalone source still
  contains the OSG teleoperation widget for comparison.
- Controls: in the Filament route, press 1-4 or click an IK target, then move
  it with keyboard nudges or Ctrl-left drag. In the legacy standalone source,
  mouse modifiers and WASD/QE/FZ drive the original OSG interaction.

## Notes

- This example runs in kinematic mode (no physics simulation).
- `pixi run ex atlas_puppet` routes to `examples/filament_gui --scene
atlas-puppet` so the in-tree runner uses the Filament path.

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
