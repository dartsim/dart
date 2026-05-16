# Biped Stand Example

## Summary

- Goal: keep a biped standing with SPD control while applying perturbations.
- Concepts/APIs: `dart::io::readWorld`, SPD tracking, external forces,
  `dart::gui::ApplicationOptions`, custom panels, per-step callbacks, and
  keyboard actions.
- Expected output: a posed fullbody biped balancing on the ground.
- Controls: Space starts or pauses simulation. Keys `1` and `2` push the biped
  along `+X` and `-X`; keys `3` and `4` push along `+Z` and `-Z`. The panel
  exposes the same perturbations.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

From the source tree:

```bash
pixi run ex biped_stand
```

From a standalone build directory:

```bash
./biped_stand
```

Headless capture is also supported through the promoted `dart::gui` runner:

```bash
pixi run ex biped_stand --headless --frames 2 --screenshot /tmp/biped_stand.ppm
```
