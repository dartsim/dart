# Boxes Example

## Summary

- Goal: spawn a grid of rigid boxes over a ground plane and visualize contact
  dynamics.
- Concepts/APIs: `dart::gui` application setup, `dynamics::Skeleton`,
  `simulation::World`, Bullet collision preference, and headless capture.
- Expected output: a 5x5x5 grid of colored boxes falling onto a gray ground
  plane.
- Controls: Space starts or pauses simulation.

## Run

From the source tree:

```bash
pixi run ex boxes
```

Headless capture is provided by the promoted `dart::gui` runner:

```bash
pixi run ex boxes --headless --frames 2 --screenshot /tmp/boxes.ppm
```
