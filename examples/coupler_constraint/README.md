# Coupler Constraint Example

## Summary

- Goal: compare a bilateral coupler constraint against a legacy mimic motor
  when the follower joint saturates.
- Concepts/APIs: mimic joints, `Joint::setUseCouplerConstraint`, per-step
  callbacks, keyboard actions, custom panels, and DART-owned line visuals.
- Expected output: two paired two-link rigs, limit guides, live coupling-error
  links, and an XY reference grid.
- Controls: Space toggles simulation, `r` resets both rigs to their initial
  pose, and `--width`/`--height` override the default 960x720 window size.

## Run

From the source tree:

```bash
pixi run ex coupler_constraint
```

Headless capture is also supported through the promoted `dart::gui` runner:

```bash
pixi run ex coupler_constraint --headless --frames 2 --screenshot /tmp/coupler_constraint.ppm
```
