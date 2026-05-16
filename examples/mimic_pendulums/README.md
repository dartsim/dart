# Mimic Pendulums Example

## Summary

- Goal: visualize and inspect mimic joint behavior across multiple pendulum
  rigs.
- Concepts/APIs: SDF mimic joints, retargeted mimic references, LCP solver
  selection, collision detector selection, custom `dart::gui` panels, and
  per-frame mimic diagnostics.
- Expected output: an uncoupled gray baseline pendulum plus red and blue mimic
  pendulum rigs.
- Controls: Space starts or pauses simulation. The panel can pause, step, and
  displays mimic position error, mimic velocity error, and base drift for each
  retargeted pair.

## Run

From the source tree:

```bash
pixi run ex mimic_pendulums
```

Select solver or collision settings at launch:

```bash
pixi run ex mimic_pendulums --solver pgs --collision dart
```

Headless capture is provided by the promoted `dart::gui` runner:

```bash
pixi run ex mimic_pendulums --headless --frames 2 --screenshot /tmp/mimic_pendulums.ppm
```
