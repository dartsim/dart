# Mimic Pendulums Example

## Summary

- Goal: visualize and inspect mimic joint behavior across multiple pendulum
  rigs.
- Concepts/APIs: SDF mimic joints, retargeted mimic references, LCP solver
  selection, collision detector selection, custom `dart::gui` panels, and
  per-frame mimic diagnostics.
- Expected output: an uncoupled gray baseline pendulum plus red and blue mimic
  pendulum rigs over an XY reference grid.
- Controls: Space starts or pauses simulation. The panel can pause, step, and
  reset the world, toggle ODE collision, toggle the PGS solver, and display
  reference angle, follower angle, mimic position error, mimic velocity error,
  and base drift for each retargeted pair.

## Run

From the source tree:

```bash
pixi run ex mimic_pendulums
```

Select solver or collision settings at launch:

```bash
pixi run ex mimic_pendulums --solver pgs --collision dart
```

## Command-Line Options

```text
--solver <name>     Solver to use (default: dantzig)
--collision <name>  Collision detector preference: file, dart, fcl, bullet, ode
--headless          Run without display window
--frames <n>        Number of frames
--out <dir>         Output directory for image-sequence capture
--screenshot <p>    Output path for a single screenshot
--width <n>         Viewport width (default: 1280)
--height <n>        Viewport height (default: 720)
--gui-scale <n>     GUI scale factor (default: 1.0)
```

Headless capture is provided by the promoted `dart::gui` runner:

```bash
pixi run ex mimic_pendulums --headless --frames 2 --screenshot /tmp/mimic_pendulums.ppm
```

Image-sequence capture uses `--out`:

```bash
pixi run ex mimic_pendulums --headless --frames 3 --out /tmp/mimic_pendulums_frames
```
