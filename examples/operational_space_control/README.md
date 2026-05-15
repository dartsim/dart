# Operational Space Control Example

## Summary

- Goal: drive an end effector toward a draggable target in task space.
- Concepts/APIs: operational-space control, Jacobians, drag-and-drop targets.
- Expected output: by default, the shared Filament viewer displays a WAM arm
  tracking a selectable red target sphere. The legacy standalone source still
  contains the OSG drag-and-drop axis-constraint controls for comparison.
- Controls: in the Filament route, click the red target sphere, then move it
  with keyboard nudges or Ctrl-left drag. In the legacy standalone source, drag
  the red ball and hold 1/2/3 to constrain motion axes.

## Notes

- `pixi run ex operational_space_control` routes to
  `examples/filament_gui --scene operational-space-control` so the in-tree
  runner uses the Filament path.

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
