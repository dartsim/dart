# Joint Constraints Example

## Summary

- Goal: apply SPD tracking control and toggle a harness constraint on a humanoid.
- Concepts/APIs: `constraint::WeldJointConstraint`, custom controller,
  `gui::RealTimeWorldNode`.
- Expected output: a humanoid model that balances while you apply impulses.
- Controls: 1/2/3/4 apply pushes, h toggles the harness, space toggles simulation.

## Details

The controller computes torques to maintain desired joint configurations while
the character responds to external forces. The harness feature welds the pelvis
to the world frame to simplify debugging.

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
