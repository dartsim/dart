# Add/Delete Skeletons Example

## Summary

- Goal: spawn and remove skeletons at runtime through promoted `dart::gui`
  controls.
- Concepts/APIs: `simulation::World`, dynamic skeleton creation,
  `ApplicationOptions::keyboardActions`, and `PanelBuilder`.
- Expected output: a DART GUI viewer with the ground world loaded; press `q` or
  the panel button to spawn cube skeletons.
- Controls: `q` spawns a cube; `w` deletes the most recent cube; space toggles
  simulation.

## Build Instructions

From this directory:

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make

## Execute Instructions

From the DART repository root:

```bash
pixi run ex add_delete_skels
```

For an automated smoke run:

```bash
pixi run ex add_delete_skels --headless --frames 2 --screenshot add_delete_skels.ppm
```
