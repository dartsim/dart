# Joint Constraints Example

## Summary

- Goal: apply SPD tracking control and toggle a harness constraint on a humanoid.
- Concepts/APIs: `dart::io::readWorld`,
  `dart::constraint::WeldJointConstraint`,
  `dart::gui::ApplicationOptions::preStep`, renderer-neutral keyboard actions,
  and promoted panels.
- Expected output: a `fullbody1.skel` humanoid and ground plane at the
  historical 640x480 launch size.
- Controls: press `1`/`2`/`3`/`4` for programmed perturbations, press `h` to
  toggle the pelvis harness, use the panel buttons for the same actions, and
  press space to pause or resume simulation.

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

From the repository root:

```bash
pixi run ex joint_constraints
```

From a standalone build directory:

```bash
./joint_constraints
```

Headless capture is also supported through the promoted `dart::gui` runner:

```bash
pixi run ex joint_constraints --headless --frames 2 --screenshot /tmp/joint_constraints.ppm
```
