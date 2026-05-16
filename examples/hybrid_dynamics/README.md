# Hybrid Dynamics Example

## Summary

- Goal: mix passive, velocity-controlled, and locked joints in one model.
- Concepts/APIs: `dart::io::readWorld`, joint actuator types,
  `dart::gui::ApplicationOptions::preStep`, renderer-neutral keyboard actions,
  and promoted panels.
- Expected output: a `fullbody1.skel` humanoid and ground plane at the
  historical 640x480 launch size.
- Controls: press `h` to toggle the pelvis harness lock, use the panel harness
  button for the same action, and press space to pause or resume simulation.

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
pixi run ex hybrid_dynamics
```

From a standalone build directory:

```bash
./hybrid_dynamics
```

Headless capture is also supported through the promoted `dart::gui` runner:

```bash
pixi run ex hybrid_dynamics --headless --frames 2 --screenshot /tmp/hybrid_dynamics.ppm
```
