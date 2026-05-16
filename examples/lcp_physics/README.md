# LCP Physics Example

## Summary

- Goal: exercise contact-heavy rigid-body scenarios with selectable LCP
  solvers.
- Concepts/APIs: `dart::constraint::ConstraintSolver`, Dantzig and PGS LCP
  solvers, rigid contact dynamics, custom `dart::gui` panels, and
  example-specific command-line flags.
- Expected output: one selected benchmark scene rendered with the promoted DART
  GUI.
- Controls: Space starts or pauses simulation. The panel can pause, step,
  reset, switch scenarios, switch solvers, tune timestep/gravity, and show
  timing/contact diagnostics.

## Scenarios

| Scenario         | Description                           | Challenge                                  |
| ---------------- | ------------------------------------- | ------------------------------------------ |
| `mass_ratio`     | Heavy box (1000kg) on light box (1kg) | Solver stability with 1000:1 mass ratio    |
| `box_stack`      | Pyramid of stacked boxes              | Shock propagation (Guendelman et al. 2003) |
| `ball_drop`      | 75 balls dropping                     | Many simultaneous contacts                 |
| `dominos`        | Domino chain reaction                 | Sequential impulse propagation             |
| `inclined_plane` | Block sliding on ramp                 | Friction model accuracy                    |

## Run

From the source tree:

```bash
pixi run ex lcp_physics
```

List scenarios and solvers:

```bash
pixi run ex lcp_physics --list
```

Select a scenario and solver:

```bash
pixi run ex lcp_physics --scenario box_stack --solver pgs
```

## Command-Line Options

```text
--scenario <name>  Scenario to run (default: mass_ratio)
--solver <name>    Solver to use (default: dantzig)
--list             List available scenarios and solvers
--headless         Run without display window
--frames <n>       Number of frames
--out <dir>        Output directory for image-sequence capture
--screenshot <p>   Output path for a single screenshot
--width <n>        Viewport width (default: 1280)
--height <n>       Viewport height (default: 720)
--gui-scale <n>    GUI scale factor (default: 1.0)
```

Headless capture is provided by the promoted `dart::gui` runner:

```bash
pixi run ex lcp_physics --scenario dominos --headless --frames 2 --screenshot /tmp/lcp_physics.ppm
```

Image-sequence capture uses `--out`:

```bash
pixi run ex lcp_physics --headless --scenario mass_ratio --frames 3 --out /tmp/lcp_physics_frames
```

## References

- Guendelman et al. (2003), "Nonconvex Rigid Bodies with Stacking"
- SimBenchmark, physics-engine benchmark scenes
- Stewart and Trinkle (1996), implicit time-stepping with friction
