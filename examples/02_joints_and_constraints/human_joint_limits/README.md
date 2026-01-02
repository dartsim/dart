# Human Joint Limits Example

## Summary

- Goal: apply custom joint-limit constraints to a humanoid skeleton.
- Concepts/APIs: `constraint::ConstraintSolver`, joint limit enforcement,
  custom joint-limit constraint classes.
- Expected output: an OSG viewer with a human model constrained at the arms
  and legs.
- Controls: space toggles simulation.

## Notes

- Requires TinyDNN and DART built with both Bullet and ODE collision backends.

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
