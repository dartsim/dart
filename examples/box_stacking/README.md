# Box Stacking Example

## Summary

- Goal: inspect dynamic rigid-body contact in a small stack of boxes.
- Concepts/APIs: `dart::gui` application setup, custom panels,
  `constraint::ConstraintSolver`, Dantzig/PGS LCP solver selection, and split
  impulse contact correction.
- Expected output: five colored boxes falling onto a light gray floor.
- Controls: Space starts or pauses simulation. The panel can pause, step,
  toggle gravity, switch between Dantzig and PGS, and toggle split impulse.

## Run

From the source tree:

```bash
pixi run ex box_stacking
```

Select the initial LCP solver at launch:

```bash
pixi run ex box_stacking --solver pgs
```

Headless capture is provided by the promoted `dart::gui` runner:

```bash
pixi run ex box_stacking --headless --frames 2 --screenshot /tmp/box_stacking.ppm
```
